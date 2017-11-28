/*
 * config.h
 *
 *  Created on: Nov 5, 2017
 *      Author: TDNC
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
//#define MEMORY_DEBUG
//#define MODBUS_DEBUG
//#define ZIGBEE_DEBUG

//#define RFID_DEBUG

#define MOSBUS_TIMEOUT           (g_ui32SysClock/50)
#define MODBUS_RESEND            (MOSBUS_TIMEOUT/2)
#define STATION1MASK             (1<<0)
#define STATION2MASK             (1<<1)
#define STATION3MASK             (1<<2)
#define STATION4MASK             (1<<3)
#define STATION5MASK             (1<<4)
#define STATION6MASK             (1<<5)
#define STATION7MASK             (1<<6)
#define GLCD_INITED              (1<<0)
#define ZIGBEE_INITED            (1<<0)
#define MODBUS_INITED            (1<<0)
#define SYSTICKS_PER_SECOND      (100)
/*
 * DMA Defination
 */
#define MEM_BUFFER_SIZE          1024
#define UARTRX_BUFFER_SIZE       50
#define UARTTX_BUFFER_SIZE       50
#define STATION1IDM1            ":6E34D62"
#define STATION2IDM2            ":9174D72"
#define STATION3IDM3            ":11B3D62"
#define STATION4IDM4            ":A473D72"
#define STATION5IDM5            ":E5A6D62"

/*
 * RFID Definaion
 */
#define UART2RX_BUFFER_SIZE       41
extern uint8_t g_UART2RX1[UART2RX_BUFFER_SIZE];
extern uint8_t g_UART2RX2[UART2RX_BUFFER_SIZE];
extern uint32_t rfid_timeout;
/*
 * SLAVER Defination
 */
#define SLAVER
#ifdef SLAVER
#define ROBOT_RXBUFFER_SIZE        25
#define ROBOT_TXBUFFER_SIZE        25
#define ROBTO_TX_MAX_SIZE          10
#define ROBTO_RX_MAX_SIZE          10
extern uint16_t ROBOTRX_Buffer[ROBTO_TX_MAX_SIZE];
extern uint16_t ROBOTTX_Buffer[ROBTO_RX_MAX_SIZE];
#endif
/*
 *
 */

extern uint16_t numofStation;
static uint8_t STATE;
extern uint32_t robotmax;
extern uint32_t stationmax;
extern uint8_t lcdinit;
extern uint32_t g_ui32uDMAErrCount;
extern uint32_t g_ui32CPUUsage;
extern uint32_t g_ui32Seconds;

extern uint32_t g_ui32RxBufACount;
extern uint32_t g_ui32RxBufBCount;
extern volatile uint8_t g_bFeedWatchdog;
//*****************************************************************************
//
// The source and destination buffers used for memory transfers.
//
//*****************************************************************************
extern uint32_t g_ui32SrcBuf[MEM_BUFFER_SIZE];
extern uint32_t g_ui32DstBuf[MEM_BUFFER_SIZE];

extern uint8_t g_UART3RX1[UARTRX_BUFFER_SIZE];
extern uint8_t g_UART3RX2[UARTRX_BUFFER_SIZE];
//extern uint32_t MMPUnit[100];
typedef struct {
    uint32_t *mm[100];
    uint32_t idex;
} MMU;
extern MMU MMUnit;
extern int den;
extern int binh;
/*
 * Slaver
 */
extern uint32_t rfid_location;
#endif /* CONFIG_H_ */

