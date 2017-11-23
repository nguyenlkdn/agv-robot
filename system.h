/*
 * systemctl.h
 *
 *  Created on: Jan 4, 2014
 *      Author: TDNC
 */

#ifndef SYSTEMCTL_H_
#define SYSTEMCTL_H_
#include <stdint.h>
#include <stdbool.h>
//*****************************************************************************
//
// Function System
//
//*****************************************************************************
void cfg_uart(void);
void cfg_clock(void);
void cfg_timer(void);
void cfg_interrupt(void);
void cfg_inout(void);
void cfg_peripheral(void);
void cfg_pwm(void);
void cfg_systick(void);
void cfg_qei(void);
void cfg_wdt(void);
void cfg_dma(void);
void init(void);
void* MemoryAllocation(void *fp, uint32_t size);

// ERROR MAPPING
#define ERRORPORT	GPIO_PORTN_BASE
#define FRTSEN		GPIO_PIN_0
#define FWDSEN		GPIO_PIN_1

// NOTICE MAPPING
#define NOTICEPORT	GPIO_PORTN_BASE
#define SPK			GPIO_PIN_2
#define FRTLED		GPIO_PIN_3
#define FWDLED		GPIO_PIN_4

// BUTTON MAPPING
#define BUTTONPORTN	GPIO_PORTN_BASE
#define FRTBUTTON	GPIO_PIN_5

#define BUTTONPORTH	GPIO_PORTH_BASE
#define FWDBUTTON	GPIO_PIN_0
#define STRBUTTON	GPIO_PIN_1

// ZIGBEE MAPPING
#define ZIGBEEPORT	GPIO_PORTB_BASE
#define ZIGBEETX	GPIO_PIN_1
#define ZIGBEERZ	GPIO_PIN_0

// OUTPUT MAPPING
#define OUTPUTPORT	GPIO_PORTQ_BASE
#define OUT1		GPIO_PIN_0
#define OUT2		GPIO_PIN_1
#define OUT3		GPIO_PIN_2
#define OUT4		GPIO_PIN_3
#define OUT5		GPIO_PIN_4

// RFID MAPPING
#define	RFIDPORT	GPIO_PORTA_BASE
#define RFIDFSS		GPIO_PIN_3
#define RFIDFCL		GPIO_PIN_2
#define RFIDMOSI	GPIO_PIN_5
#define RFIDMISO	GPIO_PIN_4
#define RFIDIQR		GPIO_PIN_6
#define RFIDRST		GPIO_PIN_7

// LCD1 MAPPING
#define LCD1PORT1	GPIO_PORTB_BASE
#define LCD1CLK		GPIO_PIN_5
#define LCD1MOSI	GPIO_PIN_4

#define LCD1PORT2	GPIO_PORTE_BASE
#define LCD1CS		GPIO_PIN_4
#define	LCD1DC		GPIO_PIN_5

// LCD2 MAPPING
#define LCD2DATAPORT    GPIO_PORTL_BASE
#define LCD2CTLPORT     GPIO_PORTL_BASE
void cfg_uartext(uint32_t uartbase, uint32_t baurate, uint32_t mode);

#endif /* SYSTEMCTL_H_ */
