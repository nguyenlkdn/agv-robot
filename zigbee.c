/*
 * zigbee.c
 *
 *  Created on: Oct 4, 2017
 *      Author: sdev
 */
#include "zigbee.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
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
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "modbus-rtu.h"
#include "system.h"
#include "config.h"
/*
 *
 */
uint8_t g_UART3RX1[UARTRX_BUFFER_SIZE];
uint8_t g_UART3RX2[UARTRX_BUFFER_SIZE];
uint8_t g_UART3TX[UARTTX_BUFFER_SIZE];
bool timer_processing=false;
uint8_t bufferid=0;

/*
 *
 *
 */
#define         ZIGBEE_FUNC16       16
#define         ZIGBEE_FUNC03       3
#define         ZIGBEE_BUFFER_SIZE  50
#define         SLAVER_REG          10
#define         MATCH_ADDR          0x00000001
#define         MATCH_FUNC          0x00000010
uint32_t        ZIGBEE_MASK;

/*
 *
 */
uint32_t g_ui32SysClock;
volatile uint8_t g_bFeedWatchdog;

//int tram0;

void zigbeeInit(uint8_t addr, uint32_t uart)
{
    g_bFeedWatchdog = false;
    cfg_uartext(uart, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    int i;
    for(i=0;i<ROBOT_RXBUFFER_SIZE;i++)
    {
        ROBOTRX_Buffer[i] = 0;
        ROBOTTX_Buffer[i] = 0;
    }
    // Timer 4
    TimerClockSourceSet(TIMER4_BASE, TIMER_CLOCK_PIOSC);
    ROM_TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER4_BASE, TIMER_A, g_ui32SysClock/100);
    ROM_IntEnable(INT_TIMER4A);
    g_bFeedWatchdog = true;
    ROM_TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER4_BASE, TIMER_A);

}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
uint32_t dma_previos_size = 0;
void Timer4IntHandler(void)
{
    //
    ROM_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    uint32_t dma_current_size = uDMAChannelSizeGet(UDMA_CH16_UART3RX);
#ifdef ZIGBEE_DEBUG
    UARTprintf("DMA in processing %d %d\n", dma_current_size, dma_previos_size);
#endif
    if (dma_previos_size != 0 && dma_previos_size != dma_current_size || dma_current_size < UARTRX_BUFFER_SIZE)
    {
        modbusarrayProcessing(g_UART3RX1, sizeof(g_UART3RX1), 1);
        ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(UART3_BASE + UART_O_DR),
                                   g_UART3RX1, sizeof(g_UART3RX1));
        MAP_uDMAChannelEnable(UDMA_CH16_UART3RX);
        dma_previos_size = 0;
    }
    if (timer_processing == true)
    {
        timer_processing = false;
        switch (bufferid)
        {
        case 1:
            modbusarrayProcessing(g_UART3RX1, sizeof(g_UART3RX1), 1);
            break;

        case 2:
            modbusarrayProcessing(g_UART3RX2, sizeof(g_UART3RX2), 1);
            break;
        default:
            break;
        }
    }
    dma_previos_size = dma_current_size;
}
// Vector Rx/Tx UART2 from RFID
//void UART3IntHandler(void)
//{
//        UARTprintf("UART3 Interrupt !!! \n");
////    uint32_t status;
////    uint32_t mode;
////    status = MAP_UARTIntStatus(UART3_BASE, true);
////    MAP_UARTIntClear(UART3_BASE, status);
////    modbusarrayProcessing(g_UART3RX1, sizeof(g_UART3RX1), 1);
//
//
////    uint32_t channel;
////
//    uint32_t status;
//    uint32_t mode;
//
//    status = MAP_UARTIntStatus(UART3_BASE, true);
//    MAP_UARTIntClear(UART3_BASE, status);
//
//    mode = MAP_uDMAChannelModeGet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT);
//
//    //channel = MAP_uDMAIntStatus();
//
//    if (mode == UDMA_MODE_STOP && MAP_uDMAChannelIsEnabled(UDMA_CH16_UART3RX))
//    {
//        //MAP_UARTIntDisable(UART3_BASE, UART_INT_DMARX);
//        modbusarrayProcessing(g_UART3RX1, sizeof(g_UART3RX1), 1);
//        ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT,
//        UDMA_MODE_PINGPONG,
//                                   (void *) (UART3_BASE + UART_O_DR),
//                                   g_UART3RX1, sizeof(g_UART3RX1));
//       // MAP_uDMAChannelEnable(UDMA_CH16_UART3RX);
//
////        MAP_UARTDMADisable(UART3_BASE, UART_DMA_RX);
//    }
//
//}
void
UART3IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;

    //
    // Read the interrupt status of the UART.
    //
    ui32Status = ROM_UARTIntStatus(UART3_BASE, 1);

    //
    // Clear any pending status, even though there should be none since no UART
    // interrupts were enabled.  If UART error interrupts were enabled, then
    // those interrupts could occur here and should be handled.  Since uDMA is
    // used for both the RX and TX, then neither of those interrupts should be
    // enabled.
    //
    ROM_UARTIntClear(UART3_BASE, ui32Status);

    //
    // Check the DMA control table to see if the ping-pong "A" transfer is
    // complete.  The "A" transfer uses receive buffer "A", and the primary
    // control structure.
    //
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT);

    //
    // If the primary control structure indicates stop, that means the "A"
    // receive buffer is done.  The uDMA controller should still be receiving
    // data into the "B" buffer.
    //
    bufferid=1;
    if(ui32Mode == UDMA_MODE_STOP)
    {
#ifdef ZIGBEE_DEBUG_LV1
        UARTprintf("DMA UDMA_PRI_SELECT STOP\n");
        int i;
        UARTprintf("UDMA_PRI_SELECT DATA: ");
        for (i = 0; i < UARTRX_BUFFER_SIZE; i++)
        {
            UARTprintf("%x ", g_UART3RX1[i]);
        }
        UARTprintf("\n");
#endif
        //modbusarrayProcessing(g_UART3RX1, sizeof(g_UART3RX1), 1);
        //
        // Increment a counter to indicate data was received into buffer A.  In
        // a real application this would be used to signal the main thread that
        // data was received so the main thread can process the data.
        //
        //g_ui32RxBufACount++;

        //
        // Set up the next transfer for the "A" buffer, using the primary
        // control structure.  When the ongoing receive into the "B" buffer is
        // done, the uDMA controller will switch back to this one.  This
        // example re-uses buffer A, but a more sophisticated application could
        // use a rotating set of buffers to increase the amount of time that
        // the main thread has to process the data in the buffer before it is
        // reused.
        //
        timer_processing = true;

        ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(UART3_BASE + UART_O_DR),
                                   g_UART3RX1, sizeof(g_UART3RX1));
        MAP_uDMAChannelEnable(UDMA_CH16_UART3RX);

    }

    //
    // Check the DMA control table to see if the ping-pong "B" transfer is
    // complete.  The "B" transfer uses receive buffer "B", and the alternate
    // control structure.
    //
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CH16_UART3RX | UDMA_ALT_SELECT);
    bufferid = 2;
    //
    // If the alternate control structure indicates stop, that means the "B"
    // receive buffer is done.  The uDMA controller should still be receiving
    // data into the "A" buffer.
    //
    if(ui32Mode == UDMA_MODE_STOP)
    {
#ifdef ZIGBEE_DEBUG_LV1
        UARTprintf("DMA UDMA_ALT_SELECT STOP\n");
        int i;
        UARTprintf("UDMA_ALT_SELECT DATA: ");
        for (i = 0; i < UARTRX_BUFFER_SIZE; i++)
        {
            UARTprintf("%x ", g_UART3RX2[i]);
        }
        UARTprintf("\n");
#endif
        //modbusarrayProcessing(g_UART3RX2, sizeof(g_UART3RX2), 1);
        //
        // Increment a counter to indicate data was received into buffer A.  In
        // a real application this would be used to signal the main thread that
        // data was received so the main thread can process the data.
        //
       // g_ui32RxBufBCount++;

        //
        // Set up the next transfer for the "B" buffer, using the alternate
        // control structure.  When the ongoing receive into the "A" buffer is
        // done, the uDMA controller will switch back to this one.  This
        // example re-uses buffer B, but a more sophisticated application could
        // use a rotating set of buffers to increase the amount of time that
        // the main thread has to process the data in the buffer before it is
        // reused.
        //
        ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(UART3_BASE + UART_O_DR),
                                   g_UART3RX2, sizeof(g_UART3RX2));
        MAP_uDMAChannelEnable(UDMA_CH16_UART3RX);

    }

    //
    // If the UART1 DMA TX channel is disabled, that means the TX DMA transfer
    // is done.
    //
//    if(!ROM_uDMAChannelIsEnabled(UDMA_CH17_UART3TX))
//    {
//        UARTprintf("DMA UDMA_CH17_UART3TX STOP\n");
//
//        //
//        // Start another DMA transfer to UART1 TX.
//        //
//        ROM_uDMAChannelTransferSet(UDMA_CH17_UART3TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_BASIC, g_UART3TX,
//                                   (void *)(UART3_BASE + UART_O_DR),
//                                   sizeof(g_UART3TX));
//
//        //
//        // The uDMA TX channel must be re-enabled.
//        //
//        ROM_uDMAChannelEnable(UDMA_CH17_UART3TX);
//    }
}
