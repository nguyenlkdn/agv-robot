/*
 * modbus-rtu.c
 *
 *  Created on: Oct 21, 2017
 *      Author: TDNC
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
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

#include "system.h"
#include "config.h"
#include "modbus-rtu.h"
#include "t6963c.h"
#include "config.h"
uint32_t g_ui32SysClock;
modbusbuffer receivedbuf;
extern modbus sentpack;
extern modbus receiedpack;
volatile uint8_t g_bFeedWatchdog;
modbus *modbusrunning;
/*
 *
 */
#ifdef SLAVER
uint16_t ROBOTRX_Buffer[ROBTO_TX_MAX_SIZE];
uint16_t ROBOTTX_Buffer[ROBTO_RX_MAX_SIZE];
#endif
/*
 * General Internal APIs
 */
//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
#ifdef MATER_
void Timer5IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    g_bFeedWatchdog = false;
    //sentpack.uartbase = UART3_BASE;
    if (modbusrunning == NULL)
    {
        UARTprintf("[ERROR] CANNOT detect any UART Ports!\n");
        return;
    }
    else
    {
#ifdef MODBUS_DEBUG_LV1
        UARTprintf("Modbus Hanlder \n");
#endif
        int32_t incoming = 0;
        while (UARTCharsAvail(modbusrunning->uartbase))
            while (UARTCharsAvail(modbusrunning->uartbase))
            {
                //incoming = UARTCharGet(sentpack.uartbase);
                incoming = UARTCharGet(modbusrunning->uartbase);

                //UARTCharPut(UART3_BASE, incoming);

                if (incoming == '.')
                {
                    continue;
                }
                modbusrunning->readingbuffer.buffer[modbusrunning->readingbuffer.index] = incoming;
                if(modbusrunning->readingbuffer.index >= 50)
                {
                    UARTprintf("[WARN] Modbus buffer was fulled\n");
                    modbusrunning->readingbuffer.index = 0;
                }
#ifdef MODBUS_DEBUG
                UARTprintf("UART3 INCOMING %d: %x\n", modbusrunning->readingbuffer.index, (uint8_t)incoming);
#endif
                modbusrunning->readingbuffer.index++;
//                receivedbuf.buffer[receivedbuf.index] = incoming;
//                if (receivedbuf.index == 50)
//                {
//                    receivedbuf.index = 0;
//                }
//                receivedbuf.index++;
            }
    }
    g_bFeedWatchdog = true;
}

void UART1IntHandler(void)
{
    UARTIntClear(UART1_BASE, UART_INT_RX);
    UARTprintf("[WARNING] Dropped modbus buffer due to low performance\n");
//    int32_t incoming = 0;
//    while (UARTCharsAvail(UART1_BASE))
//    {
//        incoming = UARTCharGet(UART1_BASE);
//        UARTprintf("INCOMING: %x\n", incoming);
//        switch ((char) incoming)
//        {
//        case '.':
//
//            break;
//        default:
//
//            break;
//        }
//    }
}
#endif
uint8_t getLOWbyte(uint16_t input)
{
    return input & 0xFF;
}

/* Get HIGH byte of int */
uint8_t getHIGHbyte(uint16_t input)
{
    return (input >> 8) & 0xFF;
}
uint16_t swap16bits(uint16_t input)
{
    uint16_t tmp=input&0xFF00;
    input = ((input<<8) | (tmp>>8));
    return input;
}
void modbusInitProtocol(modbus *modbuspack)
{
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);
    if(modbuspack->uartbase == UART2_BASE)
    {
        ROM_GPIOPinConfigure(GPIO_PA6_U2RX);
        ROM_GPIOPinConfigure(GPIO_PA7_U2TX);
        ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
        UARTClockSourceSet(modbuspack->uartbase, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(modbuspack->uartbase, g_ui32SysClock, 4800,
             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
        //GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7,
        //GPIO_STRENGTH_12MA,
         //                GPIO_PIN_TYPE_STD_WPU);
        //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6 | GPIO_PIN_7);
        UARTFIFOLevelSet(modbuspack->uartbase, UART_FIFO_RX1_8, UART_FIFO_RX1_8);
    }
    else if (modbuspack->uartbase == UART1_BASE)
    {
        UARTprintf("Modbus for UART1_BASE\n");
        ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
        ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
        ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTClockSourceSet(modbuspack->uartbase, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(modbuspack->uartbase, g_ui32SysClock, 4800,
             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_EVEN));
        GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1,
        GPIO_STRENGTH_12MA,
                         GPIO_PIN_TYPE_STD_WPU);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
        UARTFIFOLevelSet(modbuspack->uartbase, UART_FIFO_RX1_8, UART_FIFO_RX1_8);
    }
    //UARTFIFODisable(UART1_BASE);
    //UARTIntEnable(modbuspack->uartbase, UART_INT_RX);
    //UARTCharGetNonBlocking(modbuspack->uartbase);
    //IntEnable(INT_UART2);
    // Timer 5
    TimerClockSourceSet(TIMER5_BASE, TIMER_CLOCK_PIOSC);
    ROM_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, g_ui32SysClock/200);
    ROM_IntEnable(INT_TIMER5A);
    //ROM_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    //ROM_TimerEnable(TIMER5_BASE, TIMER_A);
}
uint16_t crc16Get(modbus *modbuspack)
{
    uint16_t ret=modbuspack->crc;
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Get CRC from package: %x\n", ret);
#endif
    return ret;
}

uint16_t startaddrGet(modbus *modbuspack)
{
    uint16_t ret = modbuspack->start;
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Get Starting addr from package: %x\n", ret);
#endif
    return ret;
}

void startaddrSet(modbus *modbuspack, uint16_t start)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Set Starting addr from package: %x\n", start);
#endif
    modbuspack->start = start;
}

uint16_t numofregGet(modbus *modbuspack)
{
    uint16_t ret = modbuspack->nofreg;
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Get number of register to package: %d\n", ret);
#endif
    return ret;
}

void numofregSet(modbus *modbuspack, uint16_t numofreg)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Set number of register to package: %d\n", numofreg);
#endif
    modbuspack->nofreg = numofreg;
}
uint8_t modbusarrayProcessing(uint8_t *array, uint8_t length, uint8_t addr)
{
#ifdef MODBUS_DEBUG_LV1
    int idex1;
    UARTprintf("Processing Buffer: ");
    for(idex1=0;idex1<ROBOT_RXBUFFER_SIZE;idex1++)
    {
        UARTprintf("%x ", array[idex1]);
    }
    UARTprintf("\n");
#endif
    int index, i;
    uint16_t crcgen, crcrec;
    int numofbytes;
    for(index=0;index<length/2;index++)
    {
        if(array[index] == addr)
        {
#ifdef MODBUS_DEBUG_LV1
            UARTprintf("[SUCESS] Corrected Address %x!\n", array[index]);
#endif
            uint8_t f03processbuffer[8];
            switch(array[index+1])
            {
            case 0x03:
#ifdef MODBUS_DEBUG_LV1
                UARTprintf("Accepted Function 3  !!!\n");
                UARTprintf("Received Buffer: ");
#endif
                for(i=0;i<8;i++)
                {
                    f03processbuffer[i] = array[index+i];
#ifdef MODBUS_DEBUG_LV1
                    UARTprintf("%x ", f03processbuffer[i]);
#endif
                }
#ifdef MODBUS_DEBUG
                UARTprintf("\n");
#endif
                crcgen = crc16Gen(f03processbuffer, 6);
                crcrec = combineBytes(f03processbuffer[6], f03processbuffer[7]);
                if(crcgen == crcrec)
                {
#ifdef MODBUS_DEBUG_LV1
                    UARTprintf("[SUCESS] Modbus Package was accepted: %4x/%3x!!! \n", crcgen, crcrec);
#endif
                    modbus receivedpack;
                    arrayf03tomodbus(&receivedpack, f03processbuffer, 8);
                    receivedpack.uartbase = UART3_BASE;
                    modbusRespondF03(&receivedpack);
                    free(receivedpack.data);
                    return 0;
#ifdef MODBUS_DEBUG_LV1
                modbusExport(&receivedpack);
#endif
                }
                else
                {
                    UARTprintf("[WARNING] Modbus Package was ignored: %4x/%3x!!! \n", crcgen, crcrec);
                    continue;
                }
            case 0x10:
#ifdef MODBUS_DEBUG
                UARTprintf("Processing on BUFFER: ");
                for(i=0;i<ROBOT_RXBUFFER_SIZE;i++)
                {
                    UARTprintf("%x ", array[i]);
                }
                UARTprintf("\n");
#endif
                numofbytes = array[index+6];
#ifdef MODBUS_DEBUG
                UARTprintf("Accepted Function 16 !!!\n");
                UARTprintf("Prepare to write: %d\n", numofbytes);
#endif
                if((length - index) < numofbytes+9)
                {
#ifdef MODBUS_DEBUG
                    UARTprintf("No enough DATA !!!\n");
#endif
                    return 1;
                }
                else
                {
                    crcgen = crc16Gen(array+index, numofbytes+7);
                    crcrec = combineBytes(array[index+numofbytes+7], array[index+numofbytes+8]);
#ifdef MODBUS_DEBUG
                    UARTprintf("Generated CRC16: %x / %x\n", crcgen, crcrec);
#endif
                    if(crcgen == crcrec)
                    {
                        uint16_t startaddr = combineBytes(array[index+2], array[index+3]);
                        uint16_t numofreg = combineBytes(array[index+4], array[index+5]);
#ifdef MODBUS_DEBUG
                        UARTprintf("Starting Addr: %x, %x => %d, Number of Register: %x, %x => %d\n", array[REGADDR_POS], array[REGADDR_POS+1], startaddr, array[REGNO_POS], array[REGNO_POS+1], numofreg);
#endif
                        if(startaddr + numofreg > ROBOT_RXBUFFER_SIZE)
                        {
                            UARTprintf("[ERROR] Invalid range!!!\n");
                        }
                        else
                        {
                            int i;
                            for(i=startaddr;i<startaddr+numofreg;i++)
                            {
                                ROBOTRX_Buffer[i] = combineBytes(array[index+7+i*2], array[index+8+i*2]);
#ifdef MODBUS_DEBUG_LV1
                                UARTprintf("Assigned: %2x | %2x -> %3d to Robot buffer %5d\n", array[index+7+i*2], array[index+8+i*2], ROBOTTX_Buffer[i], i);
#endif
                            }
                            uint8_t respond[8];
                            for(i=0;i<6;i++)
                            {
                                respond[i] = array[index+i];
                            }
                            crcgen = crc16Gen(respond, 6);
#ifdef MODBUS_DEBUG_LV1
                            UARTprintf("CRC Respond: %x \n", crcgen);
#endif
                            respond[6] = getHIGHbyte(crcgen);
                            respond[7] = getLOWbyte(crcgen);
                            modbussendstring(UART3_BASE, respond, 8);
                            return 0;
                        }
                    }
                    else
                    {
                        continue;
                    }
                }

                break;
            default:
                continue;
            }
        }
    }
//    ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT,
//                               UDMA_MODE_PINGPONG,
//                               (void *)(UART3_BASE + UART_O_DR),
//                               g_UART3RX1, sizeof(g_UART3RX1));
//    ROM_uDMAChannelEnable(UDMA_CH16_UART3RX);
    //ROM_uDMAChannelEnable(UDMA_CH16_UART3RX);
    //ROM_uDMAChannelRequest(UDMA_CH16_UART3RX);
    return 0;
}
uint8_t modbussendstring(uint32_t ui32Base, uint8_t *array, uint16_t length)
{
    int i;
    for(i=0;i<length;i++)
    {
#ifdef MODBUS_DEBUG_LV1
        UARTprintf("Sending %d: %x \n", i, array[i]);
#endif
        UARTCharPut(ui32Base, array[i]);
    }
    return i;
}

void crc16Set(modbus *modbuspack, uint16_t crc16)
{
//    uint8_t array[ADDR_LEN + FUNC_LEN + modbuspack->length];
//    modbustoarray(modbuspack, array);
//    uint16_t crc16 = crc16Gen(array, ADDR_LEN + FUNC_LEN + modbuspack->length);
    modbuspack->crc = crc16;
}

void addrSet(modbus *modbuspack, uint16_t addr)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Setting ADDR to package: %x\n", addr);
#endif
    modbuspack->addr = addr;
}

uint16_t addrGet(modbus *modbuspack)
{
    uint16_t ret=modbuspack->addr;
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Get ADDR from package: %x\n", ret);
#endif
    return ret;
}

void dataLenghSet(modbus *modbuspack, uint8_t lengh)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Setting Data Length to package: %d\n", lengh*2);
#endif
    modbuspack->nofreg = lengh*2;
}

void funcSet(modbus *modbuspack, uint8_t function)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Setting function to package: %x\n", function);
#endif
    modbuspack->func = function;
}
uint8_t funcGet(modbus *modbuspack)
{
    uint8_t ret = modbuspack->func;
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Get FUNC from package: %x\n", ret);
#endif
    return ret;
}

uint8_t dataAttach(modbus *modbuspack, uint16_t data)
{
    uint8_t ret=0;
    if(modbuspack->index >= modbuspack->nofreg*2 || modbuspack->status == 0)
    {
        UARTprintf("[ERROR] CANNOT Attach %x to package due to full buffer\n", data);
        ret = 1;
    }
    else
    {
        modbuspack->data[modbuspack->index++] = getHIGHbyte(data);
        modbuspack->data[modbuspack->index++] = getLOWbyte(data);
    }
    return ret;
}
uint16_t dataDetach(modbus *modbuspack, uint16_t idex)
{
    return combineBytes(modbuspack->data[idex*2], modbuspack->data[idex*2+1]);
}

uint8_t dataAttachIndex(modbus *modbuspack, uint16_t data, uint8_t index)
{
    uint8_t ret=0;
    if(index >= modbuspack->nofreg*2 || modbuspack->status == 0)
    {
        UARTprintf("[ERROR] CANNOT Attach %x to package due to full buffer\n", data);
        ret = 1;
    }
    else
    {
#ifdef MODBUS_DEBUG_LV1
        UARTprintf("Attached %d at location: %d\n", data, index);
#endif
        modbuspack->data[index] = getHIGHbyte(data);
        modbuspack->data[index+1] = getLOWbyte(data);
    }
    return ret;
}
void dataSet(modbus *modbuspack, uint8_t *data)
{
#ifdef MODBUS_DEBUG_LV1
    int i;
    UARTprintf("Setting Data to package: ");
    for(i=0;i<modbuspack->length;i++)
    {
        UARTprintf("%x ", data[i]);
    }
    UARTprintf("\n");
#endif

    modbuspack->data = data;
}

uint8_t *dataGet(modbus *modbuspack)
{
    uint8_t *datafp=NULL;
    datafp = modbuspack->data;
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Get addr from package: %x\n", datafp);
#endif
    return datafp;
}

/* Combine HIGH and LOW bytes */
uint16_t combineBytes(uint8_t high, uint8_t low)
{
    return (uint16_t) (high << 8) + low;
}

/* CRC algorithm */
uint16_t CRC16(uint16_t crc, uint8_t data)
{
    const uint16_t Poly16=0xA001;
    uint16_t LSB, i;
    crc = ((crc^data) | 0xFF00) & (crc | 0x00FF);
    for (i=0; i<8; i++) {
        LSB=(crc & 0x0001);
        crc=crc/2;
        if (LSB)
        {
            crc=crc^Poly16;
        }
    }
    return(crc);
}

/* Generate CRC */
uint16_t crc16Gen(uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    int i;
    for (i = 0; i < length; i++) {
        //UARTprintf("Checksum %d: %x\n", i, data[i]);
        crc = CRC16 (crc, data[i]);
    }
    return swap16bits(crc);
}

void modbusInit(modbus *modbuspack, uint16_t addr, uint16_t func,
                uint16_t start, uint16_t numofreg, uint32_t uartbase)
{
    modbuspack->addr = addr;
    modbuspack->func = func;
    modbuspack->start = start;
    modbuspack->nofreg = numofreg;
    modbuspack->uartbase = uartbase;
    modbuspack->index = 0;
    modbuspack->data = NULL;
    modbuspack->status = 0;
    modbuspack->crc = 0x0000;
    modbusReInit(modbuspack);

    modbusInitProtocol(modbuspack);
    GLCDPrintfNormal(0, 2, " > Modbus Initialed.");
    SysCtlDelay(g_ui32SysClock/1000);
}
void modbusReInit(modbus *modbuspack)
{
    if(modbuspack->data != NULL)
    {
        free(modbuspack->data);
    }
//    modbuspack->data = (uint8_t*) calloc(dataLenghGet(modbuspack),
//                                         sizeof(uint8_t));
    modbuspack->data = MemoryAllocation(modbuspack->data, dataLenghGet(modbuspack));
    if (modbuspack->data == NULL)
    {
        UARTprintf("[ERROR] modbusReInit CANNOT allocate data buffer: %d!\n",
                   dataLenghGet(modbuspack) * sizeof(uint8_t));
    }
    else
    {
        modbuspack->status = 1;
#ifdef MODBUS_DEBUG
        UARTprintf("[SUCESS] modbusReInit Allocated %d at %x\n",
                   dataLenghGet(modbuspack) * sizeof(uint8_t),
                   modbuspack->data);
#endif
    }
}

void modbusDeInit(modbus *modbuspack)
{
    modbuspack->addr = 0x00;
    modbuspack->crc = 0x0000;
    modbuspack->func = 0;
    modbuspack->index = 0;
    modbuspack->nofreg = 0;
    modbuspack->start = 0;
    modbuspack->status = 0;
    modbuspack->uartbase = 0;
    free(modbuspack->data);
    modbuspack->data = NULL;
}

uint8_t dataLenghGet(modbus *modbuspack)
{
    uint8_t ret = modbuspack->nofreg*2;
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Get Length from package: %d\n", ret);
#endif
    return ret;
}
uint8_t modbustoarraywithoutdata(modbus *modbuspack, uint8_t *array, uint8_t length)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Export Converted Package before send: \n");
    modbusExport(modbuspack);
#endif
    uint8_t ret=0;
#ifdef ADDR16
    array[ADDR_POS] = getHIGHbyte(modbuspack->addr);
    array[ADDR_POS+1] = getLOWbyte(modbuspack->addr);
#else
    array[ADDR_POS] = (uint8_t)modbuspack->addr&0x00FF;
#endif
    array[FUNC_POS] = modbuspack->func;

    array[REGADDR_POS] = getHIGHbyte(modbuspack->start);
    array[REGADDR_POS+1] = getLOWbyte(modbuspack->start);

    array[REGNO_POS] = getHIGHbyte(modbuspack->nofreg);
    array[REGNO_POS+1] = getLOWbyte(modbuspack->nofreg);
    uint16_t crc16;
    crc16 = crc16Gen(array, length-2);
    array[REGNO_POS+2] = getHIGHbyte(crc16);
    array[REGNO_POS+3] = getLOWbyte(crc16);
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("CRC16: %x\n", crc16);
    UARTprintf("Package Contents: ");
    int index=0;
    for(index=0;index<length;index++)
    {
        UARTprintf("%x ", array[index]);
    }
    UARTprintf("\n");
#endif
    return ret;
}
uint16_t arraytomodbus(modbus *modbuspack, uint8_t *array, uint8_t length)
{
    uint16_t ret=crc16Gen(array, length-2);
#ifdef MODBUS_DEBUG_LV1
    int i;
    UARTprintf("Converted buffer: ");
    for(i=0;i<length;i++)
    {
        UARTprintf("%x ", array[i]);
    }
    UARTprintf("\n");
#endif
#ifdef ADDR16
    modbuspack->addr = array[0] | array[1]<<8;
#else
    modbuspack->addr = array[0];
#endif
    modbuspack->func = array[FUNC_POS];
    modbuspack->start = combineBytes(array[REGADDR_POS], array[REGADDR_POS+1]);
    modbuspack->nofreg = combineBytes(array[REGNO_POS], array[REGNO_POS+1]);
    modbuspack->crc = combineBytes(array[REGNO_POS+2], array[REGNO_POS+3]);
    return ret;
}

uint8_t dataAllocate(modbus *modbuspack, uint8_t size)
{
    if(modbuspack->data != NULL)
    {
#ifdef MODBUS_DEBUG_LV1
        UARTprintf("[WARNING] Buffer was allocated at %x, so it needs to reallocate\n", modbuspack->data);
#endif
        free(modbuspack->data);
    }

    //modbuspack->data = (uint8_t*) calloc(size, sizeof(uint8_t));
    modbuspack->data = MemoryAllocation(modbuspack->data, size);
    if(modbuspack->data == NULL)
    {
        UARTprintf("[ERROR] dataAllocate CANNOT Allocate buffer\n");
        return 1;
    }
    else
    {
#ifdef MODBUS_DEBUG
        UARTprintf("[SUCESS] Buffer was allocated at %x\n", modbuspack->data);
#endif
        return 0;
    }
}
uint16_t masterGetLocation(modbus *modbuspack)
{
    return combineBytes(modbuspack->data[REG_LOCATION], modbuspack->data[REG_LOCATION+1]);
}
uint16_t arrayf03tomodbus(modbus *modbuspack, uint8_t *array, uint8_t length)
{
    uint16_t ret=crc16Gen(array, length-2);
#ifdef MODBUS_DEBUG_LV1
    int i;
    UARTprintf("Converted buffer: ");
    for(i=0;i<length;i++)
    {
        UARTprintf("%x ", array[i]);
    }
    UARTprintf("\n");
#endif
#ifdef ADDR16
    modbuspack->addr    = array[0] | array[1]<<8;
#else
    modbuspack->addr    = array[0];
#endif
    modbuspack->func    = array[FUNC_POS];
    modbuspack->start   = combineBytes(array[REGADDR_POS], array[REGADDR_POS+1]);
    modbuspack->nofreg  = combineBytes(array[REGNO_POS], array[REGNO_POS+1]);
    modbuspack->data    = MemoryAllocation(modbuspack->data, modbuspack->nofreg*2);
    modbuspack->index   = 0;
    modbuspack->status  = 1;
    int idex;
    for(idex=0;idex<modbuspack->nofreg;idex++)
    {
        dataAttach(modbuspack, ROBOTTX_Buffer[modbuspack->start + idex]);
    }
    //uint16_t crcgen     = crc16Gen(array, length);
    modbuspack->crc     = combineBytes(array[CRC_POS], array[CRC_POS+1]);
    //modbuspack->crc     = crc16Gen(array, length-2);
    return ret;
}
uint8_t modbustostring(modbus *modbuspack, uint8_t *array, uint8_t length)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Export Converted Package before send: \n");
    modbusExport(modbuspack);
#endif
    uint8_t ret=0;
#ifdef ADDR16
    array[ADDR_POS] = getHIGHbyte(modbuspack->addr);
    array[ADDR_POS+1] = getLOWbyte(modbuspack->addr);
#else
    array[ADDR_POS] = (uint8_t)modbuspack->addr&0x00FF;
#endif
    array[FUNC_POS] = modbuspack->func;
    array[FUNC_POS+1] = dataLenghGet(modbuspack);
    int i;
    for(i=0;i<array[FUNC_POS+1];i++)
    {
        array[FUNC_POS+2+i] = modbuspack->data[i];
    }
    uint16_t crc16 = crc16Gen(array, length-2);
    array[FUNC_POS+2+i]     = getHIGHbyte(crc16);
    array[FUNC_POS+2+i+1]   = getLOWbyte(crc16);
#ifdef MODBUS_DEBUG
    UARTprintf("CRC16 Generated: %x\n", crc16);
    UARTprintf("Package Contents: ");
    int index=0;
    for(index=0;index<length;index++)
    {
        UARTprintf("%x ", array[index]);
    }
    UARTprintf("\n");
#endif
    return ret;
}
uint8_t modbustoarray(modbus *modbuspack, uint8_t *array, uint8_t length)
{
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Export Converted Package before send: \n");
    modbusExport(modbuspack);
#endif
    uint8_t ret=0;
#ifdef ADDR16
    array[ADDR_POS] = getHIGHbyte(modbuspack->addr);
    array[ADDR_POS+1] = getLOWbyte(modbuspack->addr);
#else
    array[ADDR_POS] = (uint8_t)modbuspack->addr&0x00FF;
#endif
    array[FUNC_POS] = modbuspack->func;

    array[REGADDR_POS] = getHIGHbyte(modbuspack->start);
    array[REGADDR_POS+1] = getLOWbyte(modbuspack->start);

    array[REGNO_POS] = getHIGHbyte(modbuspack->nofreg);
    array[REGNO_POS+1] = getLOWbyte(modbuspack->nofreg);
    array[SENT_POS] = dataLenghGet(modbuspack);
    uint16_t crc16;
    int i;
    for(i=0;i<modbuspack->nofreg*2;i++)
    {
        array[DATA_POS+i] = modbuspack->data[i];
    }
    crc16 = crc16Gen(array, length-2);
    array[length-2] = getHIGHbyte(crc16);
    array[length-1] = getLOWbyte(crc16);
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("CRC16 Generated: %x\n", crc16);
#endif
#ifdef MODBUS_DEBUG_LV1
    UARTprintf("Package Contents: ");
    int index=0;
    for(index=0;index<length;index++)
    {
        UARTprintf("%x ", array[index]);
    }
    UARTprintf("\n");
#endif
    return ret;
}
uint8_t modbusSendF16(modbus *modbuspack)
{
    funcSet(modbuspack, 0x10);
    uint8_t sent = dataLenghGet(modbuspack) + MODBUS_SIZE;
    //uint8_t *sendingbuffer = (uint8_t*) calloc(sent, sizeof(uint8_t));
    uint8_t *sendingbuffer=NULL;
    sendingbuffer = MemoryAllocation(sendingbuffer, sent);
    if (sendingbuffer == NULL)
    {
        UARTprintf("[ERROR] CANNOT allocate data buffer: %d!\n", sent);
    }
    else
    {
#ifdef MODBUS_DEBUG
        UARTprintf("[SUCESS] modbusSendF16 Allocated %d at %x\n", sent, sendingbuffer);
#endif
    }
    modbustoarray(modbusrunning, sendingbuffer, sent);
#ifdef MODBUS_DEBUG_LV1
    int index;
    UARTprintf("Sending buffer: ");
    for(index=0;index<sent;index++)
    {
        UARTprintf("%d ", sendingbuffer[index]);
    }
    UARTprintf("\n");
#endif
    int i;

    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4 | GPIO_PIN_5);
    for(i=0;i<sent;i++)
    {
        //SysCtlDelay(g_ui32SysClock/1000);
        UARTCharPut(modbusrunning->uartbase, sendingbuffer[i]);
    }
    SysCtlDelay(g_ui32SysClock/10);
    for(;i<sent;i++)
    {
        UARTCharPut(modbusrunning->uartbase, sendingbuffer[i]);
        //SysCtlDelay(g_ui32SysClock/1000);
    }
//    for(i=0;i<sent;i++)
//    {
//        UARTCharPut(modbusrunning->uartbase, sendingbuffer[i]);
//        UARTprintf("Sending buffer: %x\n", sendingbuffer[i]);
//        SysCtlDelay(g_ui32SysClock/100);
////        if(i == sent)
////        {
//////            break;
////            SysCtlDelay(g_ui32SysClock/100);
////
////        }
////        else
////        {
////            SysCtlDelay(g_ui32SysClock/100);
////        }
//
//    }

    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);
    free(sendingbuffer);
    return sent;
}
uint8_t modbusSendF03(modbus *modbuspack)
{
    modbusrunning = modbuspack;
    funcSet(modbuspack, 0x03);
    uint8_t sent = MODBUS_SIZE - SENT_NUM;
    //uint8_t *sendingbuffer = (uint8_t*) calloc(sent, sizeof(uint8_t));
    uint8_t *sendingbuffer=NULL;
    sendingbuffer = MemoryAllocation(sendingbuffer, sent);
    if (sendingbuffer == NULL)
    {
        UARTprintf("[ERROR] modbusSendF03 CANNOT allocate data buffer: %d!\n", sent);
        return 1;
    }
    else
    {
#ifdef MODBUS_DEBUG_LV1
        UARTprintf("[SUCESS] modbusSendF03 Allocated %d at %x\n", sent, sendingbuffer);
#endif
    }
    modbustoarraywithoutdata(modbuspack, sendingbuffer, sent);
#ifdef MODBUS_DEBUG
    int index;
    UARTprintf("Sending buffer: ");
    for(index=0;index<sent;index++)
    {
        UARTprintf("%x ", sendingbuffer[index]);
    }
    UARTprintf("\n");
#endif

    int i;
    modbusrunning->readingbuffer.index = 0;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4 | GPIO_PIN_5);
    for(i=0;i<sent;i++)
    {
        UARTCharPut(modbusrunning->uartbase, sendingbuffer[i]);
    }
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);
    free(sendingbuffer);
    return sent;
}
uint8_t modbusWriteSingle(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t data)
{
    uint8_t ret=0;
    modbuspack->respond = 0;
    funcSet(modbuspack, 0x10);
    addrSet(modbuspack, addr);
    startaddrSet(modbuspack, reg);
    numofregSet(modbuspack, 1);
    dataAttachIndex(modbuspack, data, 0);
    receivedbuf.index = 0;
    modbusSendF16(modbuspack);
    return ret;
}

uint8_t modbusRespondF03(modbus *modbuspack)
{
    uint8_t sent = dataLenghGet(modbuspack) + MODBUS_SIZE - 4;
    uint8_t *sendingbuffer = NULL;
    sendingbuffer = MemoryAllocation(sendingbuffer, sent);
    if (sendingbuffer == NULL)
    {
        UARTprintf("[ERROR] CANNOT allocate data buffer: %d!\n", sent);
    }
    else
    {
#ifdef MODBUS_DEBUG_LV1
        UARTprintf("[SUCESS] modbusRespondF03 Allocated %d at %x\n", sent, sendingbuffer);
#endif
    }
    modbustostring(modbuspack, sendingbuffer, sent);
#ifdef MODBUS_DEBUG_LV1
    int index;
    UARTprintf("Sending buffer: ");
    for(index=0;index<sent;index++)
    {
        UARTprintf("%x ", sendingbuffer[index]);
    }
    UARTprintf("\n");
#endif
    int i;
    for(i=0;i<sent;i++)
    {
        UARTCharPut(UART3_BASE, sendingbuffer[i]);
        //SysCtlDelay(g_ui32SysClock/100);
    }
    free(sendingbuffer);
    return sent;
}
uint8_t modbusF03Respond(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t *data)
{
    uint8_t ret=0;
    modbuspack->respond = 0;
    funcSet(modbuspack, 0x03);
    addrSet(modbuspack, addr);
    startaddrSet(modbuspack, reg);
    numofregSet(modbuspack, 1);
    for(ret=0;ret<reg*2;ret++)
    {
        dataAttach(modbuspack, data[ret]);
    }
    receivedbuf.index = 0;
    modbusRespondF03(modbuspack);
    return ret;
}

uint8_t modbusWriteMulti(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t *data, uint32_t leng)
{
    uint8_t ret=0;
    if(0)
    {
        UARTprintf("[ERROR] CANNOT attach %d for slaver!!!\n", leng);
        ret=1;
    }
    else
    {
        numofregSet(modbuspack, leng);
        modbuspack->respond = 0;
        modbuspack->index=0;
        funcSet(modbuspack, 0x10);
        addrSet(modbuspack, addr);
        startaddrSet(modbuspack, reg);
        numofregSet(modbuspack, leng);
        int i;
        for(i=0;i<leng;i++)
        {
#ifdef MODBUS_DEBUG_LV1
            UARTprintf("Attaching: %d\n", i);
#endif
            dataAttach(modbuspack, data[i]);
            //dataAttach(modbuspack, 0);
        }
        modbuspack->readingbuffer.index = 0;
        modbusSendF16(modbuspack);
    }

    return ret;
}
uint8_t modbusWriteMultiBlocking(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t *data, uint32_t leng, uint32_t timeout)
{
    uint32_t resend = 0;
    modbusrunning = modbuspack;
    modbusrunning->readingbuffer.buffer = calloc(8, sizeof(uint16_t));
    if(modbusrunning->readingbuffer.buffer == NULL)
    {
        UARTprintf("[ERROR] CANNOT Allocate buffer for modbus\n");
        return 0;
    }
    else
    {
#ifdef MODBUS_DEBUG
        UARTprintf("[SUCESS] modbusWriteMultiBlocking Allocated: %x\n", modbusrunning->readingbuffer.buffer);
#endif
    }
    ROM_TimerEnable(TIMER5_BASE, TIMER_A);
    while (timeout > 0)
    {
        if(resend == 0)
        {
            resend++;
            modbusWriteMulti(modbusrunning, addr, reg, data, leng);
        }
        else
        {
            if(modbusrunning->readingbuffer.index == 8)
            {
                if(modbusrunning->readingbuffer.buffer[FUNC_POS] == modbusrunning->func && modbusrunning->readingbuffer.buffer[ADDR_POS] == modbusrunning->addr)
                {
                    uint16_t crcrevied = arraytomodbus(modbusrunning, modbusrunning->readingbuffer.buffer, modbusrunning->readingbuffer.index);
                    if(crc16Check(modbusrunning, crcrevied))
                    {
#ifdef MODBUS_DEBUG_LV1
                        UARTprintf("[SUCESS] Sent is OK!\n");
                        modbusExport(modbusrunning);
#endif
                        return 0;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                if(++resend == 1000000)
                {
                    resend = 0;
                }
            }
        }

        timeout--;
    }
    ROM_TimerDisable(TIMER5_BASE, TIMER_A);

    //UARTprintf("[ERROR] Writing TIMTOUT\n\n");
    return ERROR_TIMEOUT;
}
uint8_t crc16Check(modbus *modbuspack, uint16_t crc)
{
    uint16_t crc16 = crc16Get(modbuspack);
    if( crc16 == crc)
    {
#ifdef MODBUS_DEBUG_LV1
        UARTprintf("[SUCESS] CRC Matched!!!\n");
#endif
        return 1;
    }
    else
    {
        UARTprintf("[ERROR] CRC Mismatch %x/%x\n", crc16, crc);
        return 0;
    }
}
uint8_t CRCCompare(modbus *pack1, modbus *pack2)
{
    return pack1->crc == pack2->crc;
}

uint8_t modbusWriteSingleBlocking(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t data, uint32_t timeout)
{
    uint32_t resend = 0;
    while (timeout > 0)
    {
        if(resend == 0)
        {
            resend++;
            modbusWriteSingle(modbuspack, addr, reg, data);
        }
        else
        {
            if(receivedbuf.index == 8)
            {
                if(receivedbuf.buffer[FUNC_POS] == sentpack.func && receivedbuf.buffer[ADDR_POS] == sentpack.addr)
                {
                    uint16_t crcrevied = arraytomodbus(&receiedpack, receivedbuf.buffer, receivedbuf.index);
                    if(crc16Check(&receiedpack, crcrevied))
                    {
#ifdef MODBUS_DEBUG_LV1
                        UARTprintf("[SUCESS] Sent is OK!\n");
#endif
                        return 0;
                    }
                }
            }
            else
            {
                if(++resend == MODBUS_RESEND)
                {
                    resend = 0;
                }
            }
        }

        timeout--;
    }
    UARTprintf("[ERROR] Writing TIMTOUT\n\n");
    return ERROR_TIMEOUT;
}

uint8_t modbusReadSingle(modbus *modbuspack, uint16_t addr, uint16_t reg)
{
    uint8_t ret=0;
    modbuspack->respond = 0;
    funcSet(modbuspack, 0x03);
    addrSet(modbuspack, addr);
    startaddrSet(modbuspack, reg);
    numofregSet(modbuspack, 1);
    modbusSendF03(modbuspack);
    return ret;
}
uint8_t modbusReadMulti(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t numofreg)
{
    uint8_t ret=0;
    modbuspack->respond = 0;
    funcSet(modbuspack, 0x03);
    addrSet(modbuspack, addr);
    startaddrSet(modbuspack, reg);
    numofregSet(modbuspack, numofreg);
    modbusSendF03(modbuspack);
    return ret;
}
uint8_t modbusReadSingleBlocking(modbus *modbuspack, uint16_t addr,
                                 uint16_t reg, uint16_t *data, uint32_t timeout)
{
    uint32_t resend = 0;
    while (timeout > 0)
    {
        if (resend == 0)
        {
            resend++;
            modbusReadSingle(modbuspack, addr, reg);
        }
        else
        {
            if (receivedbuf.index == 7)
            {
                if (receivedbuf.buffer[FUNC_POS] == sentpack.func && receivedbuf.buffer[ADDR_POS] == sentpack.addr)
                {
                    uint16_t crcrevied = arrayf03tomodbus(&receiedpack,
                                                          receivedbuf.buffer, 7);
#ifdef MODBUS_DEBUG_LV1
                    modbusExport(&receiedpack);
#endif
                    if (crc16Check(&receiedpack, crcrevied))
                    {
                        *data = combineBytes(receiedpack.data[0], receiedpack.data[1]);
                        //modbusDeInit(&receiedpack);
#ifdef MODBUS_DEBUG_LV1
                        UARTprintf("[SUCESS] Sent is OK %d!\n", *data);
#endif
                        return 0;
                    }
                    receivedbuf.index=0;
                }
            }
            else
            {
                if (++resend == MODBUS_RESEND)
                {
                    resend = 0;
                }
            }
        }

        timeout--;
    }
    UARTprintf("[ERROR] Requesting TIMEOUT!!!\n");
    return ERROR_TIMEOUT;
}
uint8_t modbusReadMultiBlocking(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t numofreg, uint32_t timeout)
{
    uint32_t resend = 0;
    uint32_t expected = 5+2*numofreg;
    modbusrunning = modbuspack;
//    if(modbusrunning->readingbuffer.buffer != NULL)
//    {
//        free(modbusrunning->readingbuffer.buffer);
//    }
    modbusrunning->readingbuffer.buffer = MemoryAllocation(modbusrunning->readingbuffer.buffer, expected);
    ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(UART3_BASE + UART_O_DR),
                               g_UART3RX1, sizeof(g_UART3RX1));
    if(modbusrunning->readingbuffer.buffer == NULL)
    {
        UARTprintf("[ERROR] CANNOT Allocate buffer for modbus\n");
        return 0;
    }
    else
    {
#ifdef MODBUS_DEBUG
        UARTprintf("[SUCESS] modbusReadMultiBlocking Allocated: %x\n", modbusrunning->readingbuffer.buffer);
#endif
    }
    ROM_TimerEnable(TIMER5_BASE, TIMER_A);

#ifdef MODBUS_DEBUG_LV1
            UARTprintf("Expected Respond: %d Bytes\n", expected);
#endif
    while (timeout > 0)
    {
        if (resend == 0)
        {
            resend++;
            modbusReadMulti(modbusrunning, addr, reg, numofreg);
        }
        else
        {
//            int i;
//            UARTprintf("DATA %3d: ", timeout);
//            for(i=0;i<expected;i++)
//            {
//                UARTprintf("%x ", modbusrunning->readingbuffer.buffer[i]);
//            }
//            UARTprintf("\n");
            if (1)
            {
#ifdef MODBUS_DEBUG_LV1
                UARTprintf("[SUCESS] Processing Buffer\n");
#endif
                if (g_UART3RX1[FUNC_POS]
                        == modbusrunning->func
                        && g_UART3RX1[ADDR_POS]
                                == modbusrunning->addr)
                {
#ifdef MODBUS_DEBUG
                    UARTprintf("Accepted Addr/Function: %2d/%2d\n", g_UART3RX1[ADDR_POS], g_UART3RX1[FUNC_POS]);
#endif
                    uint16_t crcrevied = arrayf03tomodbus(modbusrunning,
                                                          g_UART3RX1, expected);
#ifdef MODBUS_DEBUG
                    UARTprintf("[SUCESS] Sent is OK!\n");
                    modbusExport(modbusrunning);
#endif
                    if (crc16Check(modbusrunning, crcrevied))
                    {
                        //data = receiedpack.data;
                        //combineBytes(receiedpack.data[0], receiedpack.data[1]);
                        //modbusDeInit(&receiedpack);
                        modbusrunning->respond = 1;
                       // modbusrunning->readingbuffer.index = 0;
                        break;
                    }
                    else
                    {
                        break;
                    }
                }
//                else
//                {
//                    UARTprintf("Don't Accept buffer: ");
//                    int i;
//                    for(i=0;i<expected;i++)
//                    {
//                        //UARTprintf("%x ", modbusrunning->readingbuffer.buffer[i]);
//                        UARTprintf("%x ", g_UART3RX1[i]);
//                    }
//                    UARTprintf("\n");
//                }
            }
            else
            {
#ifdef MODBUS_DEBUG_LV1
                UARTprintf("Waiting Repsond: %d/%d: %d\n", modbusrunning->readingbuffer.index, expected, timeout);
#endif
                if (++resend == MODBUS_RESEND)
                {
                    resend = 0;
                }
            }
        }
        timeout--;
    }
    ROM_TimerDisable(TIMER5_BASE, TIMER_A);
    if(timeout > 0)
    {
        return 0;
    }
    else
    {
        UARTprintf("[ERROR] modbusReadMultiBlocking %d TIMEOUT!!!\n", addr);
        return ERROR_TIMEOUT;
    }
}

void modbusExport(modbus *modbuspack)
{
    UARTprintf("Modbus Created Package Included:\n");
    UARTprintf("\tTo Addr: %d\n", addrGet(modbuspack));
    UARTprintf("\tFunction Code: %d\n", funcGet(modbuspack));
    UARTprintf("\tStarting Addr: %d\n", startaddrGet(modbuspack));
    UARTprintf("\tNumber of Register: %d\n", numofregGet(modbuspack));
    UARTprintf("\tData Addr %x, length %d\n", modbuspack->data, dataLenghGet(modbuspack));
    if(modbuspack->data == NULL)
    {
        UARTprintf("\tData Contents: NULL\n");
    }
    else
    {
        UARTprintf("\tData Contents: ");
        int i;
        for(i=0;i<modbuspack->nofreg*DATA_SIZE;i++)
        {
            UARTprintf("%x ", modbuspack->data[i]);
        }
        UARTprintf("\n");
    }

    UARTprintf("\tCRC 16: %x\n", modbuspack->crc);

}
