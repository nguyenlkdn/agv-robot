/*
 * rfid.c
 *
 *  Created on: Sep 26, 2017
 *      Author: sdev
 */
#include <stdint.h>
#include <stdbool.h>
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
#include <string.h>
#include "rfid.h"
#include "system.h"
#include "config.h"
char rfid_in_buffer[RFID_BUFFER_SIZE];
uint32_t rfid_in_counter    = 0;
uint32_t rfid_out_counter   = 0;
char rfid_out_buffer[RFID_BUFFER_SIZE];
char RFID_ID[9];
uint16_t data[5];
uint8_t g_UART2RX1[UART2RX_BUFFER_SIZE];
uint8_t g_UART2RX2[UART2RX_BUFFER_SIZE];
uint32_t rfid_timeout=0;

void rfidInBufClear(void)
{
    int i;
    for(i=0;i<RFID_BUFFER_SIZE;i++)
    {
        rfid_in_buffer[i]=0;
    }
    while(UARTCharsAvail(UART2_BASE)){
        UARTCharGetNonBlocking(UART2_BASE);
    }
}
void rfidOutBufClear(void)
{

    int i;
    for(i=0;i<RFID_BUFFER_SIZE;i++)
    {
        rfid_out_buffer[i]=0;
    }
}
void rfidMemPrintf(void)
{
//    uint32_t i;
//    for(i=0;i<strlen(rfid_in_counter);i++)
//    {
//        if(rfid_in_buffer[i] != '\r' && rfid_in_buffer[i] != '\n')
//        {
//            UARTCharPut(UART0_BASE, rfid_in_buffer[i]);
//        }
//    }
}
void rfidInit(void)
{
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
    IntEnable(INT_GPIOA);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3,
                                              GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);

    // Timer 2
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_PIOSC);
    ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock/100);
    ROM_IntEnable(INT_TIMER2A);
    ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER2_BASE, TIMER_A);

    // Interrupt PORTF.0 & PORTF.4
//    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_RISING_EDGE);
//    IntEnable(INT_GPIOJ);
//    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

     ROM_GPIOPinConfigure(GPIO_PA6_U2RX);
     ROM_GPIOPinConfigure(GPIO_PA7_U2TX);
     ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
     UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);
     ROM_UARTConfigSetExpClk(UART2_BASE, g_ui32SysClock, 9600,
          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
          UART_CONFIG_PAR_NONE));
     ROM_UARTDMAEnable(UART2_BASE, UART_DMA_RX);
     MAP_uDMAChannelAssign(UDMA_CH12_UART2RX);

     ROM_uDMAChannelAttributeDisable(UDMA_CH12_UART2RX,
                                     UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                     UDMA_ATTR_HIGH_PRIORITY |
                                     UDMA_ATTR_REQMASK);

     ROM_uDMAChannelControlSet(UDMA_CH12_UART2RX | UDMA_PRI_SELECT,
                               UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                               UDMA_ARB_4);

//     ROM_uDMAChannelControlSet(UDMA_CH12_UART2RX | UDMA_ALT_SELECT,
//                               UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
//                               UDMA_ARB_4);

     ROM_uDMAChannelTransferSet(UDMA_CH12_UART2RX | UDMA_PRI_SELECT,
                                UDMA_MODE_BASIC,
                                (void *)(UART2_BASE + UART_O_DR),
                                g_UART2RX1, sizeof(g_UART2RX1));

//     ROM_uDMAChannelTransferSet(UDMA_CH12_UART2RX | UDMA_ALT_SELECT,
//                                UDMA_MODE_BASIC,
//                                (void *)(UART2_BASE + UART_O_DR),
//                                g_UART2RX2, sizeof(g_UART2RX2));
     //ROM_uDMAChannelAttributeEnable(UDMA_CH16_UART3RX, UDMA_ATTR_ALL);
     ROM_uDMAChannelEnable(UDMA_CH12_UART2RX);
     //ROM_uDMAChannelRequest(UDMA_CH16_UART3RX);
     //ROM_UARTIntEnable(UART2_BASE, UART_INT_DMARX);
     //ROM_IntEnable(INT_UART2);

     //ROM_UARTFIFODisable(UART2_BASE);
}

void rfidHandler(void)
{

}

void rfidGet(void)
{
    SysCtlDelay(g_ui32SysClock/1000);

    while(UARTCharsAvail(UART2_BASE) || rfid_in_counter < 14){
        if(rfid_in_counter < RFID_BUFFER_SIZE)
        {
            rfid_in_buffer[rfid_in_counter] = UARTCharGet(UART2_BASE);
            rfid_in_counter++;
        }
        else
        {
            UARTprintf("ERROR: RFID Overload buffers: %d/%d\n", rfid_in_counter, RFID_BUFFER_SIZE);
        }
    }
    UARTprintf("Data: %s", rfid_in_buffer);
}
void rfidSendCMD(char* inbuf)
{
    int length=strlen(inbuf);
    int i;

    if(length < 2)
    {
        UARTprintf("ERROR: RFID Invalid CMD => %s\n", inbuf);
    }
    for (i=0;i<length;i++)
    {
        UARTCharPut(UART2_BASE, inbuf[i]);
        //SysCtlDelay(g_ui32SysClock/1000);
    }
    UARTCharPut(UART2_BASE, '\r');
    UARTCharPut(UART2_BASE, '\n');
}
int32_t rfidPoll(int length)
{
    int i=0;
    for(i=0;i<=length;i++)
    {
        rfid_in_buffer[i]=UARTCharGet(UART2_BASE);
        if(rfid_in_buffer[i] != 13 && rfid_in_buffer[i] != 10)
        {
            //UARTprintf("Getting %d => %c\n", i, rfid_in_buffer[i]);
        }
        else
        {
            //UARTprintf("Getting %d => cr/cn\n", i);
        }
    }
    while(UARTCharsAvail(UART2_BASE))
    {
        UARTCharGet(UART2_BASE);
    }
    RFID_ID[0] = rfid_in_buffer[13];
    RFID_ID[1] = rfid_in_buffer[14];
    RFID_ID[2] = rfid_in_buffer[15];
    RFID_ID[3] = rfid_in_buffer[16];
    RFID_ID[4] = rfid_in_buffer[17];
    RFID_ID[5] = rfid_in_buffer[18];
    RFID_ID[6] = rfid_in_buffer[19];
    RFID_ID[7] = rfid_in_buffer[20];
    RFID_ID[8] = 0;
    return i;
}
// Vector Rx/Tx UART2 from RFID
//void UART2IntHandler(void){
//    UARTIntClear(UART2_BASE, UART_INT_RX);
//    while(UARTCharsAvail(UART2_BASE)){
//        if(rfid_in_counter < RFID_BUFFER_SIZE)
//        {
//            rfid_in_buffer[rfid_in_counter] = UARTCharGet(UART2_BASE);
//            rfid_in_counter++;
//        }
//        else
//        {
//            UARTprintf("Interrupt ERROR: RFID Overload buffers: %d/%d\n", rfid_in_counter, RFID_BUFFER_SIZE);
//        }
//    }
//    UARTprintf("Data %d: %s", rfid_in_counter, rfid_in_buffer);
//}
//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void Timer2IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //

    ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
#ifdef RFID_DEBUG
    UARTprintf("Timer2 Interrupt!!!\n");
#endif
    uint32_t ui32Mode = ROM_uDMAChannelModeGet(UDMA_CH12_UART2RX | UDMA_PRI_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
#ifdef RFID_DEBUG
        UARTprintf("DMA was stopped with data:\n");
        int i;
        for(i=0;i<UART2RX_BUFFER_SIZE;i++)
        {
            UARTprintf("%c", g_UART2RX1[i]);
        }
        UARTprintf("\n");
#endif
        ROM_uDMAChannelTransferSet(UDMA_CH12_UART2RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(UART2_BASE + UART_O_DR),
                                   g_UART2RX1, sizeof(g_UART2RX1));
        MAP_uDMAChannelEnable(UDMA_CH12_UART2RX);
        //rfid_timeout=0;
        /*
         *
         */
        int index;
        for(index=0;index<UART2RX_BUFFER_SIZE-5;index++)
        {
            if(g_UART2RX1[index] == 'I' && g_UART2RX1[index+1] == 'D')
            {
#ifdef RFID_DEBUG
                UARTprintf("Detected RFID Card: ");
#endif
                int rfdicount;
                for(rfdicount=0;rfdicount<8;rfdicount++)
                {
#ifdef RFID_DEBUG
                    UARTprintf("%c", g_UART2RX1[index+2+rfdicount]);
#endif
                    RFID_ID[rfdicount] = g_UART2RX1[index+2+rfdicount];
                    if(g_UART2RX1[index] == '\r')
                        break;
                }
                RFID_ID[rfdicount] = 0;
#ifdef RFID_DEBUG
                UARTprintf("\n");
#endif
            }
        }
    }
    else
    {
        uint32_t remain = uDMAChannelSizeGet(UDMA_CH12_UART2RX);
        if(remain < UART2RX_BUFFER_SIZE)
        {
            if(++rfid_timeout == 20)
            {
                UARTprintf("[WARNING] RFID Timeout !!!\n Try to re-start\n");
                MAP_uDMAChannelDisable(UDMA_CH12_UART2RX);
                ROM_uDMAChannelTransferSet(UDMA_CH12_UART2RX | UDMA_PRI_SELECT,
                                           UDMA_MODE_BASIC,
                                           (void *)(UART2_BASE + UART_O_DR),
                                           g_UART2RX1, sizeof(g_UART2RX1));
                MAP_uDMAChannelEnable(UDMA_CH12_UART2RX);
                //rfid_timeout=0;
            }
        }
#ifdef RFID_DEBUG
        UARTprintf("DMA is processing: %d %d\n", remain, rfid_timeout);
#endif
    }
}

void
UART2IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;

    //
    // Read the interrupt status of the UART.
    //
    ui32Status = ROM_UARTIntStatus(UART2_BASE, 1);

    //
    // Clear any pending status, even though there should be none since no UART
    // interrupts were enabled.  If UART error interrupts were enabled, then
    // those interrupts could occur here and should be handled.  Since uDMA is
    // used for both the RX and TX, then neither of those interrupts should be
    // enabled.
    //
    ROM_UARTIntClear(UART2_BASE, ui32Status);

    //
    // Check the DMA control table to see if the ping-pong "A" transfer is
    // complete.  The "A" transfer uses receive buffer "A", and the primary
    // control structure.
    //
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CH12_UART2RX | UDMA_PRI_SELECT);

    //
    // If the primary control structure indicates stop, that means the "A"
    // receive buffer is done.  The uDMA controller should still be receiving
    // data into the "B" buffer.
    //

    if(ui32Mode == UDMA_MODE_STOP)
    {
#ifdef RFID_DEBUG
        UARTprintf("DMA UDMA_PRI_SELECT STOP\n");
        int i;
        UARTprintf("UDMA_PRI_SELECT DATA: ");
        for (i = 0; i < UART2RX_BUFFER_SIZE; i++)
        {
            UARTprintf("%c", g_UART2RX1[i]);
        }
        UARTprintf("\n");
#endif
        int index;
        for(index=0;index<UART2RX_BUFFER_SIZE-5;index++)
        {
            if(g_UART2RX1[index] == 'I' && g_UART2RX1[index+1] == 'D')
            {
#ifdef RFID_DEBUG
                UARTprintf("Detected RFID Card: ");
#endif
                int rfdicount;
                for(rfdicount=0;rfdicount<8;rfdicount++)
                {
#ifdef RFID_DEBUG
                    UARTprintf("%c", g_UART2RX1[index+2+rfdicount]);
#endif
                    RFID_ID[rfdicount] = g_UART2RX1[index+2+rfdicount];
                    if(g_UART2RX1[index] == '\r')
                        break;
                }
                RFID_ID[rfdicount] = 0;
#ifdef RFID_DEBUG
                UARTprintf("\n");
#endif
            }
        }
        ROM_uDMAChannelTransferSet(UDMA_CH12_UART2RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(UART2_BASE + UART_O_DR),
                                   g_UART2RX1, sizeof(g_UART2RX1));
        MAP_uDMAChannelEnable(UDMA_CH12_UART2RX);

    }

//    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CH12_UART2RX | UDMA_ALT_SELECT);
//
//    if(ui32Mode == UDMA_MODE_STOP)
//    {
//#ifdef RFID_DEBUG
//        UARTprintf("DMA UDMA_ALT_SELECT STOP\n");
//        int i;
//        UARTprintf("UDMA_ALT_SELECT DATA: ");
//        for (i = 0; i < UART2RX_BUFFER_SIZE; i++)
//        {
//            UARTprintf("%c", g_UART2RX2[i]);
//        }
//        UARTprintf("\n");
//#endif
//        int index;
//        for(index=0;index<UART2RX_BUFFER_SIZE-5;index++)
//        {
//            if(g_UART2RX2[index] == 'I' && g_UART2RX2[index+1] == 'D')
//            {
//#ifdef RFID_DEBUG
//                UARTprintf("Detected RFID Card: ");
//#endif
//                int rfdicount;
//                for(rfdicount=0;rfdicount<8;rfdicount++)
//                {
//#ifdef RFID_DEBUG
//                    UARTprintf("%c", g_UART2RX2[index+2+rfdicount]);
//#endif
//                    RFID_ID[rfdicount] = g_UART2RX2[index+2+rfdicount];
//                    if(g_UART2RX2[index] == '\r')
//                        break;
//                }
//#ifdef RFID_DEBUG
//                UARTprintf("\n");
//#endif
//            }
//        }
//        ROM_uDMAChannelTransferSet(UDMA_CH12_UART2RX | UDMA_ALT_SELECT,
//                                   UDMA_MODE_BASIC,
//                                   (void *)(UART2_BASE + UART_O_DR),
//                                   g_UART2RX2, sizeof(g_UART2RX2));
//        MAP_uDMAChannelEnable(UDMA_CH12_UART2RX);

//    }

}
void
PORTAIntHandler(void)
{
    uint32_t PortAmask = GPIOIntStatus(GPIO_PORTA_BASE, GPIO_PIN_3);
    MAP_uDMAChannelEnable(UDMA_CH12_UART2RX);
    rfidSendCMD("AT+ID");
    if(PortAmask & GPIO_PIN_3){
        /////////////////////////////////////////////////////////////////////////////////////////////
#ifdef RFID_DEBUG
        UARTprintf("RFID Interrupt !\n");
#endif
        /////////////////////////////////////////////////////////////////////////////////////////////
        GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
    }
}
