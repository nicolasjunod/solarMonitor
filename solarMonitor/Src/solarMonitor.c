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
#include "uart.h"

extern ADC_HandleTypeDef hadc;
//extern DMA_HandleTypeDef hdma_adc;

//extern LPTIM_HandleTypeDef hlptim1;

//extern RTC_HandleTypeDef hrtc;

//extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;


uint32_t adcTable [10];
//uint8_t voltage1 [nbData];
//uint8_t voltage2 [nbData];
//uint8_t voltage3 [nbData];
uint8_t voltage1;
uint8_t voltage2;
uint8_t voltage3;
uint8_t uartBuffer[10];

//static uint32_t i;

void takeMes()
{
	uint32_t i;
	uint32_t mesID;
	uint8_t mesID_unit;
	uint32_t mesID_address;
	uint8_t flashDataByte;
	static uint8_t flashData4kB[4096];
	uint8_t mesID_blockID;

	i=0;

	HAL_ADC_Start_DMA(&hadc, &adcTable[0], 3);
	voltage1 = adcTable[0]>>4;
	voltage2 = adcTable[1]>>4;
	voltage3 = adcTable[2]>>4;

	for(i=0 ; i<16 ; i++)	//4096 = 0x1000
		flash_readMemory(COUNTER_ADDRESS_ORIGIN + i*256, 256, &flashData4kB[i*256]);

	i=0;
	do
	{
		flashDataByte = flashData4kB[i];
		uartBuffer[0]=flashDataByte;
		HAL_UART_Transmit(&huart1, &uartBuffer[0], 1, 500);


		i++;
	}while((flashDataByte == 10) && (i<0x20000)); // max 128kB --> 0x20000 bytes

	mesID = 10*(i-1) + flashDataByte;	// each counter byte is 10 before going to the next one.

	uartBuffer[0]=(uint8_t)mesID;
	HAL_UART_Transmit(&huart1, &uartBuffer[0], 1, 500);

	mesID_blockID = i/4096;	//which 4kB block must be written?


	mesID++;

	mesID_unit = mesID%10;
	mesID_address = mesID/10;
	if(mesID_unit == 0)
	{
		mesID_unit = 10;
		mesID_address--;
	}


	flashData4kB[mesID_address]=mesID_unit;   // il manque de mesBlock par ici...

	flash_writeEnable();
	flash_erase4kB(0x1000*mesID_blockID + COUNTER_ADDRESS_ORIGIN);

	for(i=0 ; i<16 ; i++)	//4096 = 0x1000
	{
		flash_writeEnable();
		flash_pageProgram(0x1000*mesID_blockID + COUNTER_ADDRESS_ORIGIN + i*256, 256, &flashData4kB[i*256]);
	}

	i++;

}
/*
void initTables()
{
	uint32_t i;
	for(i=0 ; i< nbData ; i++)
	{
		voltage1[i]=0;
		voltage2[i]=0;
		voltage3[i]=0;
	}
	i=0;
}*/

void downloadData(void)
{
	uint32_t j;
	uint8_t uartData[256];
	for (j=0; j<64; j++)
	//for (j=0; j<512; j++)
	{
		flash_readMemory(COUNTER_ADDRESS_ORIGIN + j*0x100, 256, &uartData[0]);
		HAL_UART_Transmit(&huart1, &uartData[0], 256, 1500);
	}
	uartData[0] = 'D';
	uartData[1] = 'o';
	uartData[2] = 'n';
	uartData[3] = 'e';
	HAL_UART_Transmit(&huart1, &uartData[0], 4, 1500);
}

void initFlashCounter(void)
{
	uint32_t j, k;
	static uint8_t flashData[256];
	uint8_t charData;
	//flash_writeEnable();
	//flash_chipErase();


	flash_globalBlockProtectionUnlock();

	for (j=0 ; j<256 ; j++)
		flashData[j]=0;

//	for(j=0 ; j < 32 ; j++) // 32 blocks of 4 kB --> 128 kB (size of counter memory)
	for(j=0 ; j < 4 ; j++) // 32 blocks of 4 kB --> 128 kB (size of counter memory)
	{
		flash_writeEnable();
		flash_erase4kB(COUNTER_ADDRESS_ORIGIN + j*0x1000);	//0x1000 = 4096

		for(k=0 ; k < 16 ; k++)	// 16 blocks of 256 Bytes --> 4kB
		{
			flash_writeEnable();
			flash_pageProgram(COUNTER_ADDRESS_ORIGIN + j*0x1000 + k*0x100, 256, &flashData[0]);

		}
		charData = j;
		HAL_UART_Transmit(&huart1, &charData, 1, 1500);
	}

	/*flashData[0]=0;

	flash_writeEnable();
	flash_pageProgram(COUNTER_ADDRESS_ORIGIN, 1, &flashData[0]);*/
}

void testFlash(void)
{
	uint32_t j;
	uint8_t flashData[256];
	uint8_t charData;

	flash_globalBlockProtectionUnlock();

	for (j=0 ; j<256 ; j++)
		flashData[j]=j*10;


	j=0;
	flash_writeEnable();
	//flash_erase4kB(COUNTER_ADDRESS_ORIGIN + j*0x1000);	//0x1000 = 4096
	flash_erase4kB(COUNTER_ADDRESS_ORIGIN);	//0x1000 = 4096

	flash_writeEnable();
	//flash_pageProgram(COUNTER_ADDRESS_ORIGIN + j*0x1000 + k*0x100, 1, &flashData[0]);
	flash_pageProgram(COUNTER_ADDRESS_ORIGIN, 10, &flashData[0]);

	flash_readMemory(COUNTER_ADDRESS_ORIGIN, 1, &flashData[1]);

	charData = flashData[1];

	HAL_UART_Transmit(&huart1, &charData, 1, 1500);

}
