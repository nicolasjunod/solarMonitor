/*
 * uart.c
 *
 *  Created on: 17 sept. 2017
 *      Author: Nicolas
 */

#include "main.h"
#include "stm32l0xx_hal.h"
#include "uart.h"

extern UART_HandleTypeDef huart1;

uint8_t uartData;

void enableUartInterrupt(void)
{
	HAL_UART_Receive_IT(&huart1, &uartData, 1);
}


void uartReceived(void)
{
	uartData=0;
	enableUartInterrupt();
}
