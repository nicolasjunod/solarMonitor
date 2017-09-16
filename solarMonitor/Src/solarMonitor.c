/*
 * solarMonitor.c
 *
 *  Created on: 16 sept. 2017
 *      Author: Nicolas
 */

#include "solarMonitor.h"
#include "main.h"
#include "stm32l0xx_hal.h"
#include "flash.h"

extern ADC_HandleTypeDef hadc;
//extern DMA_HandleTypeDef hdma_adc;

//extern LPTIM_HandleTypeDef hlptim1;

//extern RTC_HandleTypeDef hrtc;

//extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;


uint32_t adcTable [100];
uint8_t voltage1 [nbData];
uint8_t voltage2 [nbData];
uint8_t voltage3 [nbData];
uint8_t uartBuffer[10];

static uint32_t i;

void takeMes()
{
	HAL_ADC_Start_DMA(&hadc, &adcTable[0], 3);
	voltage1[i] = adcTable[0]>>4;
	voltage2[i] = adcTable[1]>>4;
	voltage3[i] = adcTable[2]>>4;

	uartBuffer[0] = voltage1[i];
	uartBuffer[1] = voltage2[i];
	uartBuffer[2] = voltage3[i];

	HAL_UART_Transmit(&huart1, &uartBuffer[0], 3, 500);

	//flash_readID(&uartBuffer[0]);

	i++;
	if(i==nbData)
		i=0;
}

void initTables()
{
	for(i=0 ; i< nbData ; i++)
	{
		voltage1[i]=0;
		voltage2[i]=0;
		voltage3[i]=0;
	}
	i=0;
}
