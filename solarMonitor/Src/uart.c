/*
 * uart.c
 *
 *  Created on: 17 sept. 2017
 *      Author: Nicolas
 */

#include "main.h"
#include "stm32l0xx_hal.h"
#include "uart.h"
#include "flash.h"
#include "solarMonitor.h"

extern UART_HandleTypeDef huart1;
extern LPTIM_HandleTypeDef hlptim1;

uint8_t uartData;

void enableUartInterrupt(void)
{
	HAL_UART_Receive_IT(&huart1, &uartData, 1);
}


void uartReceived(void)
{
	HAL_LPTIM_Counter_Stop_IT(&hlptim1); //disable measurements during uart communication

	switch (uartData)
	{
	case 'c' :	// Chip Erase
	case 'C' :
		flash_writeEnable();
		flash_chipErase();
		uartData='A';
		HAL_UART_Transmit(&huart1, &uartData, 1, 500);	// Send 'A' as acknowledge
		break;

	case 'd' :	// Download Data
	case 'D' :
		downloadData();
		break;

	case 'i' :	// Reinit data
	case 'I' :
		initFlashCounter();
		uartData='A';
		HAL_UART_Transmit(&huart1, &uartData, 1, 1500);	// Send 'A' as acknowledge
		break;

	case 't' :	// for test
	case 'T' :	// for test
		testFlash();
		uartData='A';
		HAL_UART_Transmit(&huart1, &uartData, 1, 1500);	// Send 'A' as acknowledge
		break;

	default:
		uartData='E';
		HAL_UART_Transmit(&huart1, &uartData, 1, 500);	// Send 'E' as Error
		break;
	}
	//HAL_LPTIM_Counter_Start_IT(&hlptim1, adcPeriod*32768);
	//must read adcPeriod in the flash;
   HAL_LPTIM_Counter_Start_IT(&hlptim1, 3*2048);

	enableUartInterrupt();
}
