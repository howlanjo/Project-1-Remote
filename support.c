#include <driverlib.h>
#include "support.h"
#include "msprf24.h"
#include "nrf_userconfig.h"
#include "I2C0.h"


volatile uint32_t HSMfreq, MCLKfreq, SMCLKfreq, ACLKfreq;
volatile uint16_t rtnVal;
volatile uint16_t rtnVal8;  // for the I2C_Recv() function only
volatile uint8_t uncompTempBytes[2];
volatile uint16_t uncompTemp;
volatile uint8_t uncompPresBytes[3];
volatile uint32_t uncompPressure;
volatile uint16_t calDataBytes[BMP180_PROM_DATA__LEN];
//volatile int16_t calData[BMP180_PROM_DATA__LEN>>1];

//------------------------------------------------------------------------------
const Timer_A_ContinuousModeConfig continuousModeConfigB =
{
        TIMER_A_CLOCKSOURCE_ACLK,           // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_4,      // ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_ENABLE,      // Enable Overflow ISR
        TIMER_A_DO_CLEAR                    // Clear Counter
};
//------------------------------------------------------------------------------
const Timer_A_UpModeConfig upConfig =
{
		TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source = 12 MHz
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 16 MHz
        TIMER_PERIOD,                           // 12000 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};
//------------------------------------------------------------------------------
int ADC_InitFunction(void)
{
	int err = 0;
	// Initializing ADC (MCLK/1/4)
	MAP_ADC14_enableModule();
	MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, 0);
	// Configuring GPIOs (5.5 A0)
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
	// Configuring ADC Memory
	MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
	MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);
//	MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, false);
	// Configuring Sample Timer
	MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
	// Enabling/Toggling Conversion
	MAP_ADC14_enableConversion();
	MAP_ADC14_toggleConversionTrigger();

	return err;
}
//------------------------------------------------------------------------------
int I2C_InitFunc(int16_t calData[11])
{
	int err = 0, i;
	uint8_t deviceID;
	CS_setExternalClockSourceFrequency(32000,48000000);

	/* Starting HFXT in non-bypass mode without a timeout. Before we start
	 * we have to change VCORE to 1 to support the 48MHz frequency */
	MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
	MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
	MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

	MAP_CS_setDCOFrequency(48000000);  // set DCO to 48 MHz (use this until we figure out crystal freq problem)

	HSMfreq=MAP_CS_getHSMCLK();  // get HSMCLK value to see if crystal defaulted (it did, so use DCO frequency)
	printf("HSMfreq=%d\r\n",HSMfreq);

	MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4);  // set SMCLK to 12 MHz
	SMCLKfreq = MAP_CS_getSMCLK();  // get SMCLK value to verify it was set correctly
	printf("SMCLKfreq=%d\r\n",SMCLKfreq);

	I2C_Init();  // initializes I2C hardware registers

	I2C_Send1(0x77, 0xD0);               // use command Send1 to set pointer to chip ID
										 // read from register 0xD0 to get data
	deviceID=I2C_Recv(0x77);
	if(deviceID != 0x55) {
//		printf("*** error, cannot connect to BMP180, error code= %d\r\n", deviceID);
		while(1);  // trap here for analysis
	} else {
//		printf("BMP180 Device ID= 0x%x\n\r", deviceID);
	}
	// use command Send1 to set pointer to beginning of calibration data EEPROM
	rtnVal= I2C_Send1(0x77, 0xAA);
	if(rtnVal != 0) {
//		printf("*** error *** code= %d\r\n", rtnVal);
		while(1); // trap here for analysis
	}
	// read from EEPROM to get calibration data (you may want to put this in a loop)
	for(i = 0; i < 11; i++)
	{
		calData[i]=I2C_Recv2(0x77);
		if(calData[i]==0xFFFF) {
//			printf("error in receiving data\r\n");
		} else {
//			printf("Calibration coefficient %d   =   %d\n", (i+1), calData[i]);
		}
	}

	return err;
}
//------------------------------------------------------------------------------
int InitFunction(void)
{
	int err = 0;

	MAP_WDT_A_holdTimer();

	MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
	GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
	GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

	//Set P6.1 to be the pin interupt
	MAP_GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN1);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, GPIO_PIN1);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN1);
	MAP_Interrupt_enableInterrupt(INT_PORT6);

	/* Starting and enabling ACLK (32kHz) */
	MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
	MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_4);
//	ACLKfreq = MAP_CS_getACLK();  // get ACLK value to verify it was set correctly

	/* Configuring Continuous Mode */
	MAP_Timer_A_configureContinuousMode(TIMER_A1_MODULE, &continuousModeConfigB);
	/* Enabling interrupts and going to sleep */
	MAP_Interrupt_enableSleepOnIsrExit();
	MAP_Interrupt_enableInterrupt(INT_TA1_N);
	/* Starting the Timer_A0 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A1_MODULE, TIMER_A_CONTINUOUS_MODE);

	return err;
}
//------------------------------------------------------------------------------
int InitOneWire(void)
{
	int err = 0;

	MAP_WDT_A_holdTimer();

//	MAP_CS_setDCOFrequency(12000000);

	/* Configuring Timer_A0 for Up Mode */
	MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

	// enable timer interrupts (more configuration done in dht22.c)
	MAP_Timer_A_enableInterrupt(TIMER_A0_BASE);

	return err;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
