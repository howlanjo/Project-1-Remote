#include "driverlib.h"
/* Standard Defines */
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "support.h"
#include "I2C0.h"


int RequestData(uint8_t controlByte, uint8_t DataRequestType, int byteCount, uint16_t *dataBack);

uint16_t tempData[5];
int16_t ADC1 = 0, ADC2 = 0, ADC3 = 0, B1 = 0, B2 = 0, MB = 0, MC = 0, MD = 0;
uint16_t ADC4 = 0, ADC5 = 0, ADC6 = 0;
uint32_t B4 = 0, B7 = 0;
int32_t X1 = 0, X2 = 0, X3 = 0, B3 = 0, B5 = 0, B6 = 0, T = 0, P = 0, UT = 0, UP = 0.0;
int oss = 0;

//------------------------------------------------------------------------------------
int GetBaroTemp(int16_t calData[11], int32_t *temp, int32_t *pressure)
{
	uint16_t rtnVal;

    ADC1 = calData[0];
    ADC2 = calData[1];
    ADC3 = calData[2];
    ADC4 = (unsigned short)calData[3];
    ADC5 = (unsigned short)calData[4];
    ADC6 = (unsigned short)calData[5];
    B1 = calData[6];
    B2 = calData[7];
    MB = calData[8];
    MC = calData[9];
    MD = calData[10];

	rtnVal = RequestData(BMP180_CTRL_MEAS_REG, BMP180_T_MEASURE, 1, &tempData);
	UT = tempData[0];

	rtnVal = RequestData((BMP180_CTRL_MEAS_REG + (oss << 6)), BMP180_P_MEASURE, 2, &tempData);
	UP = (tempData[0] << 8) + (tempData[0] & 0xFF00) >> (8 - oss);

	X1 = (UT - ADC6) * ADC5/(pow(2,15));
	X2 = MC * (pow(2,11))/(X1 + MD);
	B5 = X1 + X2;
	T = (B5 + 8)/(pow(2,4));

	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 / pow(2,12)))/pow(2,11);
	X2 = ADC2 * B6 / pow(2,11);
	X3 = X1 + X2;
	B3 = (((ADC1 * 4 + X3) << oss) + 2) / 4;
	X1 = ADC3 * B6 / (pow(2,13));
	X2 = (B1 * (B6 * B6 / pow(2,12)))/pow(2,15);
	X3 = ((X1 + X2) + 2)/(pow(2,2));
	B4 = ADC4 * (unsigned long)(X3 + 32768) / (pow(2,15));
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);

	if(B7 < 0x80000000)
	{
		P = (B7 * 2) / B4;
	}
	else
	{
		P = (B7 / B4) * 2;
	}

	X1 = (P / (pow(2,8))) * (P / (pow(2,8)));
	X1 = (X1 * 3038) / (pow(2,15));
	X2 = (-7357 * P) / (pow(2,15));
	P = P + (X1 + X2 + 3791) / (pow(2,4));

	*temp = T;
	*pressure = P;

}
//------------------------------------------------------------------------------------
int RequestData(uint8_t controlByte, uint8_t DataRequestType, int byteCount, uint16_t *dataBack)
{
	int rtnVal = 0, i = 0;

	// get uncompensated temperature
	// send command for temperature conversion
	rtnVal =  I2C_Send2(0x77, controlByte, DataRequestType);
	if(rtnVal != 0)
	{
//		printf("*** error *** code= %d\r\n", rtnVal);
		while(1); // trap here for analysis
	}

	__delay_cycles(60000); // wait 5 ms for result
	__delay_cycles(30000); // wait 5 ms for result
	rtnVal= I2C_Send1(0x77, BMP180_ADC_OUT_MSB_REG);

	for (i = 0; i < byteCount; i++)
	{
		dataBack[i] = I2C_Recv2(0x77);
		if(dataBack[i] == 0xFFFF)
		{
//			printf("error in receiving data\r\n");
		} else {
//			printf("Received data value = %d\n", dataBack[i]);
		}
	}

	return rtnVal;
}
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
