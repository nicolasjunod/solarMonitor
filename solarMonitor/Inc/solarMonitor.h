/*
 * solarMonitor.h
 *
 *  Created on: 16 sept. 2017
 *      Author: Nicolas
 */

#ifndef SOLARMONITOR_H_
#define SOLARMONITOR_H_

#define nbData 1024

void takeMes(void);
//void initTables(void);
void downloadData(void);
void initFlashCounter(void);
void testFlash(void);

#endif /* SOLARMONITOR_H_ */
