#include <msp432.h>
#include <string.h>
#include "stdint.h"
#include "driverlib.h"
#include "support.h"
#include "msprf24.h"
#include "nrf_userconfig.h"
#include "ff.h"
#include "diskio.h"
#include "spiDriver.h"

#define	ADDRESS_WIDTH					22


int InitFunction(void);
int SendDataToBase(char dataType, int16_t dataIn);
int SD_CardInit(void);

// The following are data structures used by FatFs.
static FATFS g_sFatFs;
static DIR g_sDirObject;
static FILINFO g_sFileInfo;
static FIL g_sFileObject;

int Refresh = 0, Use_SD_CARD = 0;
int16_t calData[11];
int16_t lux, humidity, temp1;
int32_t temperature = 99, pressure = 99;
int8_t lightIndex = 6;

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig SDspiConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,             // ACLK Clock Source
        12000000,                                   // ACLK 128khz
        12000000,                                    // SPICLK = 128khz
        EUSCI_B_SPI_MSB_FIRST,                     // MSB First
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // Phase
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH, // High polarity
        EUSCI_B_SPI_3PIN                           // 3Wire SPI Mode
};

void main()
{
	int err = 0, count = 0, bytesWritten = 0;
	char addr[5], i = '0';
	char buf[32], data[64];
	uint8_t status;
	uint32_t ACLKfreq;
	BYTE temp[100];
	UINT bw = 0, btw;
	FRESULT iFResult;
	DWORD sizeBuf = 0;

//Initialization Functions
	err = InitFunction();
	err |= ADC_InitFunction();
	err |= I2C_InitFunc(calData);
	err |= InitOneWire();

//Initialize the SD card
	Use_SD_CARD = YES;
	if(SD_CardInit())
	{
		Use_SD_CARD = NO;
	}

	//Initialize the nRF wireless module
	ACLKfreq = MAP_CS_getACLK();  // get ACLK value to verify it was set correctly

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

	//Entering the main loop
	while(1)
	{
		while(Refresh == 0)
		{	//Keep the process locked here until 'Refresh' is set high.
		}
		MAP_ADC14_toggleConversionTrigger();
		Refresh = 0;

		//Get light value
	    GetLightValue(&lux, &lightIndex);

	    //Get temperature and pressure
	    GetBaroTemp(calData, &temperature, &pressure);

	    //Get temperature and humidity
	    __delay_cycles(100);
	    dht_start_read();
	    int t = dht_get_temp();
	    int h = dht_get_rh();
		__delay_cycles(100);

		//For debug purposes
		printf("Temperature: %d\n", temperature);
		printf("Pressure: %d\n", pressure);
		printf("Lux: %d\n", lux);
		printf("Humidity: %d\n", h);

		//Form the string to send to base station
		sprintf(buf, "<T%003dP%000006dH%003dL%00005d>", temperature, pressure, h, lux);

		//transmit data
		GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
		w_tx_payload((uint8_t)ADDRESS_WIDTH, buf);
		msprf24_activate_tx();
		GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

		//open SD card
		if(Use_SD_CARD == YES)
		{
			iFResult = f_open(&g_sFileObject, "data.txt", FA_OPEN_EXISTING | FA_WRITE);
			count = 0;
			//if the first open fails, unmount and try again for 5 times
			while ((iFResult != FR_OK) && (count < 5))
			{
				iFResult = f_mount(0, 0);
				count++;
				__delay_cycles(10000);
				iFResult = f_mount(0, &g_sFatFs);
				__delay_cycles(10000);
				iFResult |= f_open(&g_sFileObject, "data.txt", FA_WRITE | FA_CREATE_NEW);
			}
//			sizeBuf = strlen(temp) + 1;
			sizeBuf = sizeBuf + bw;

			sprintf(temp, "\n%d, %d, %d, %d ", temperature, pressure, lux, h);
			btw = strlen(temp);
			if(count < 5)
			{
				//Find the end of the file and write new data here
				iFResult = f_lseek(&g_sFileObject, sizeBuf);
				iFResult = f_write(&g_sFileObject, temp, btw, &bw);
				//close the file
				iFResult = f_close(&g_sFileObject);
			}
		}
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
////------------------------------------------------------------------------------
Fd_t spi_Open(void)
{
    /* Selecting P9.5 P9.6 and P9.7 in SPI mode */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
			GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

	/* CS setup. */
	GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);

    /* Configuring SPI in 3wire master mode */
	SPI_initMaster(EUSCI_B1_MODULE, &SDspiConfig);

	/* Enable SPI module */
	SPI_enableModule(EUSCI_B1_MODULE);

    return 0;//NONOS_RET_OK;
}
//------------------------------------------------------------------------------
int SD_CardInit(void)
{
	int err = 0;

	int8_t lucNStatus = 0;
	FRESULT iFResult;

	spi_Open();

	// Mount the file system, using logical disk 0.
	iFResult = f_mount(0, &g_sFatFs);
	//iFResult = f_mount(&g_sFatFs, "", 0);
	if (iFResult != FR_OK)
	{
		err = -1;
//		printf("f_mount error: %s\n", StringFromFResult(iFResult));
	}

	return err;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
