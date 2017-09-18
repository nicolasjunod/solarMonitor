/*
 * flash.h
 *
 *  Created on: 16 sept. 2017
 *      Author: Nicolas
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "main.h"
#include "stm32l0xx_hal.h"

#define flash_timeout	100

#define flash_CE_Port 	GPIOC
#define flash_WP_Port	GPIOC
#define flash_HOLD_Port	GPIOC

#define COUNTER_ADDRESS_ORIGIN	0x010000
#define DATA_ADDRESS_ORIGIN		0x080000

void flash_CE_low(void);
void flash_CE_high(void);
void flash_readID(uint8_t* flashID);
void flash_initGPIO(void);

void flash_readStatus(uint8_t* flashStatus);
void flash_readConfig(uint8_t* flashConfig);
void flash_readMemory(uint32_t address, uint32_t size, uint8_t* flashData);

void flash_writeEnable(void);
void flash_writeDisable(void);

void flash_erase4kB(uint32_t address);
void flash_chipErase(void);

void flash_pageProgram(uint32_t address, uint32_t size, uint8_t* flashData);

void flash_waitBusy(void);
void flash_dummyPeriod(uint32_t nbPeriods);

void flash_globalBlockProtectionUnlock(void);
void flash_readBlockProtection(uint8_t* flash_BlockProtect);
//void flash_writeBlockProtection(void);


#endif /* FLASH_H_ */
