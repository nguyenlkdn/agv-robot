/*
 * rfid.h
 *
 *  Created on: Sep 26, 2017
 *      Author: sdev
 */

#ifndef RFID_H_
#define RFID_H_
#include <stdint.h>
#include <stdbool.h>

#define RFID_BUFFER_SIZE 100
void rfidInit(void);
void rfidSendCMD(char* data);
void rfidGet(void);
void rfidInBufClear(void);
void rfidOutBufClear(void);
int32_t rfidPoll(int length);
extern char RFID_ID[];

uint32_t g_ui32SysClock;

#endif /* RFID_H_ */
