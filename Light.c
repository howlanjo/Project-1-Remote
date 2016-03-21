#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <driverlib.h>
#include "support.h"


void calculateLighting(uint16_t value, char stringOUT[30], int8_t *index);
int DoubleToString(uint16_t numberIn, char *stringOut);
int Debouncer(uint16_t input[2]);


uint16_t Light[6], temp[5][2];
//int8_t flag;
uint16_t resultsBuffer, photoRAW, tempRAW, tempCON_C[2], tempCON_F[2], index[2], photoCON[2];
char lightString[30], numberString[30];

void GetLightValue(uint16_t *luxValue,int8_t *lightIndex)
{
	int err = 0;
	int16_t photoGain = 1.0, photoOffset = -1450.0;

	photoGain = 1.0;
	photoOffset = -1450.0;

	memset(resultsBuffer, 0, sizeof(resultsBuffer));
	resultsBuffer = MAP_ADC14_getResult(ADC_MEM0);
	//MAP_ADC14_getMultiSequenceResult(resultsBuffer);

	photoRAW = resultsBuffer;
	photoCON[0] = (photoGain*((3.3*1000)/((photoRAW)*3.3/16384)) * 1.3) + photoOffset;
	calculateLighting(photoCON[0], lightString, &index[0]);

	*luxValue = photoCON[0];
	*lightIndex = index[0];

	return err;
}
//-----------------------------------------------------------------------
void calculateLighting(uint16_t value, char stringOUT[30], int8_t *index)
{
	memset(stringOUT, 0, sizeof(stringOUT));

	Light[0] = 100.0;
	Light[1] = 800.0;
	Light[2] = 10000.0;
	Light[3] = 16000.0;
	Light[4] = 24000.0;
	Light[5] = 36000.0;

	if(value < Light[0])
	{
		sprintf(stringOUT, "Very Dark");
		*index = 0;
	}
	else if (value > Light[0] && value < Light[1])
	{
		sprintf(stringOUT, "Full Moon");
		*index = 1;
	}
	else if (value > Light[1] && value < Light[2])
	{
		sprintf(stringOUT, "Dusk");
		*index = 2;
	}
	else if (value > Light[2] && value < Light[3])
	{
		sprintf(stringOUT, "Overcast");
		*index = 3;
	}
	else if (value > Light[3] && value < Light[4])
	{
		sprintf(stringOUT, "Lil' Sun");
		*index = 4;
	}
	else if (value > Light[4] && value < Light[5])
	{
		sprintf(stringOUT, "SUNNY");
		*index = 5;
	}
	else
	{
		sprintf(stringOUT, "ERROR");
		*index = 6;
	}
}
//-----------------------------------------------------------------------
int DoubleToString(uint16_t numberIn, char *stringOut)
{
	int err = 0;
	char str[10], str1[10];

	sprintf(str, "%.1d", numberIn);
	sscanf(str, "%s", str1);
	strcpy(stringOut, str1);

	return err;
}
//-----------------------------------------------------------------------
int Debouncer(uint16_t input[2])
{
	int err = 0, i;

	for(i = 5; i >= 0; i--)
	{
		if(i == 0)
		{
			temp[0][0] = input[0];
			temp[0][1] = input[1];
		}
		else
		{
			temp[i][0] = temp[i-1][0];
			temp[i][1] = temp[i-1][1];
		}
	}

	for(i = 0; i < 4; i++)
	{
		if((temp[i][0] - temp[i+1][0]) > 150 || (temp[i][1] - temp[i+1][1]) > 150)
			err = -1;
	}

	return err;
}
