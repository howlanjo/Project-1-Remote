#include <msp432.h>
#include <string.h>
#include "stdint.h"
#include "driverlib.h"
#include "support.h"
#include "msprf24.h"
#include "nrf_userconfig.h"

#define	ADDRESS_WIDTH					22


int InitFunction(void);
int SendDataToBase(char dataType, int16_t dataIn);
int Refresh = 0;
int16_t calData[11];
int16_t lux, humidity, temp1;
int32_t temperature = 99, pressure = 99;
int8_t lightIndex = 6;

void main()
{
	int err = 0;
	char addr[5], i = '0';
	char buf[32], data[64];
	uint8_t status;

	err = InitFunction();
	err |= ADC_InitFunction();
	err |= I2C_InitFunc(calData);
	err |= InitOneWire();

	rf_crc = RF24_CRCO;
	rf_addr_width      = (uint8_t)ADDRESS_WIDTH;
	rf_speed_power     = RF24_SPEED_MIN | RF24_POWER_MAX;
	rf_channel         = 120;
	msprf24_init();  // All RX pipes closed by default
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst enabled for receiving Auto-ACKs
	msprf24_set_pipe_packetsize(0, (uint8_t)ADDRESS_WIDTH);  // Dynamic payload length enabled (size=0)

	// Transmit to 'rad01' (0x72 0x61 0x64 0x30 0x31)
	msprf24_standby();
	memcpy(addr, "\xDE\xAD\xBE\xEF\x01", 5);
//	addr[0] = 'r'; addr[1] = 'a'; addr[2] = 'd'; addr[3] = '0'; addr[4] = '2';
	w_tx_addr(addr);
	w_rx_addr(0, addr);  // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
						 // needs to listen to the TX addr on pipe#0 to receive them.
	buf[0] = '1';
	buf[1] = '\0';
//	w_tx_payload(1, buf);
	w_tx_payload_noack(1, buf);
	msprf24_activate_tx();

	if (rf_irq & RF24_IRQ_FLAGGED)
	{
			msprf24_get_irq_reason();  // this updates rf_irq
			if (rf_irq & RF24_IRQ_TX)
					status = 1;
			if (rf_irq & RF24_IRQ_TXFAILED)
					status = 0;
			msprf24_irq_clear(RF24_IRQ_MASK);  // Clear any/all of them
	}
	memset(buf, 0, sizeof(buf));

	MAP_Interrupt_enableMaster();
//	MAP_Interrupt_setPriority(INT_TA0_0, 0x00);

	while(1)
	{
		while(Refresh == 0)
		{
		}
		Refresh = 0;

	    GetLightValue(&lux, &lightIndex);
	    GetBaroTemp(calData, &temperature, &pressure);

	    __delay_cycles(100);
	    dht_start_read();
	    int t = dht_get_temp();
	    int h = dht_get_rh();
		__delay_cycles(100);

		printf("Temperature: %d\n", temperature);
		printf("Pressure: %d\n", pressure);
		printf("Lux: %d\n", lux);
		printf("Humidity: %d\n", h);

		sprintf(buf, "<T%003dP%00005dH%003dL%00005d>", temperature, pressure, h, lux);

		GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
		w_tx_payload((uint8_t)ADDRESS_WIDTH, buf);
		msprf24_activate_tx();
		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

//		__delay_cycles(10000000);
		MAP_ADC14_toggleConversionTrigger();

	}
}
//------------------------------------------------------------------------------
// GPIO ISR
void gpio_isr(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, status);

    msprf24_get_irq_reason();  // this updates rf_irq
	if (rf_irq & RF24_IRQ_TX)
	{
		status = 1;
		GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
		GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
	}
	if (rf_irq & RF24_IRQ_TXFAILED)
	{
		status = 0;
		GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
		GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
	}
	msprf24_irq_clear(RF24_IRQ_MASK);  // Clear any/all of them
}
//------------------------------------------------------------------------------
void timer0_a0_isr(void)
{
  TA0CCTL0 &= ~CCIFG;
  // This handles only TA0CCR0 interrupts
  switch (dht_current_state)
  {
  case DHT_IDLE:
    break; // Shouldn't be here
  case DHT_TRIGGERING:
    // 1ms has passed since setting the pin low
    // Let P7.0 go high and set Compare input on T1
    P7DIR &= ~BIT3; // input
    P7SEL0 |= BIT3; // Timer0_A0.CCI0A input
    TA0CTL = TACLR;
    TA0CTL = TASSEL_2 | ID_0 | MC_2;
    TA0CCTL0 = CM_2 | CCIS_0 | CAP | CCIE; // Capture on falling edge
    dht_current_state = DHT_WAITING_ACK;
    break;
  case DHT_WAITING_ACK:
    // I don't care about timings here...
    P7DIR &= ~BIT3; // input
    TA0CTL = TACLR;
    TA0CTL = TASSEL_2 | ID_0 | MC_2;
    TA0CCTL0 = CM_1 | CCIS_0 | CAP | CCIE; // Capture on rising edge
    dht_current_state = DHT_ACK_LOW;
    break;
  case DHT_ACK_LOW:
    // I don't care about timings here either...
    TA0CTL = TACLR;
    TA0CTL = TASSEL_2 | ID_0 | MC_2;
    TA0CCTL0 = CM_2 | CCIS_0 | CAP | CCIE; // Capture on falling edge
    dht_current_state = DHT_ACK_HIGH;
    dht_data_byte = dht_data_bit = 0;
    break;
  case DHT_ACK_HIGH:
    TA0CTL = TACLR;
    TA0CTL = TASSEL_2 | ID_0 | MC_2;
    TA0CCTL0 = CM_1 | CCIS_0 | CAP | CCIE; // Capture on rising edge
    dht_current_state = DHT_IN_BIT_LOW;
    break;
  case DHT_IN_BIT_LOW:
    TA0CTL = TACLR;
    TA0CTL = TASSEL_2 | ID_0 | MC_2;
    TA0CCTL0 = CM_2 | CCIS_0 | CAP | CCIE; // Capture on falling edge
    dht_current_state = DHT_IN_BIT_HIGH;
    break;
  case DHT_IN_BIT_HIGH:
    // OK now I need to measure the time since last time
    dht_data.bytes[dht_data_byte] <<= 1;
//    if (TA0CCR0 > 750) {  // > 47 us with 16 MHz SMCLK
    if (TA0CCR0 > 564) {  // > 47 us with 12 MHz SMCLK
      // Long pulse: 1
      dht_data.bytes[dht_data_byte] |= 1;
    }
    if (++dht_data_bit >= 8)
    {
      dht_data_bit = 0;
      dht_data_byte++;
    }
    if (dht_data_byte >= 5)
    {
      // I'm done, bye
      // TODO: check CRC
      TA0CTL = TACLR;
      dht_current_state = DHT_IDLE;
    }
    else
    {
      TA0CTL = TACLR;
      TA0CTL = TASSEL_2 | ID_0 | MC_2;
      TA0CCTL0 = CM_1 | CCIS_0 | CAP | CCIE; // Capture on rising edge
      dht_current_state = DHT_IN_BIT_LOW;
    }
    break;
  }
}
//------------------------------------------------------------------------------
void timer_b_0_isr(void)
{
    MAP_Timer_A_clearInterruptFlag(TIMER_A1_MODULE);
    Refresh = 1;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
