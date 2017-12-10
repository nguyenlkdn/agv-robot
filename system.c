/*
 * systemctl.c
 *

 */
#include "system.h"

#include <stdint.h>
#include <stdbool.h>
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
#include "driverlib/udma.h"
#include "utils/cpu_usage.h"
#include "config.h"
//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif
// #include "drivers/pinout.h"
uint32_t g_ui32SysClock;
uint32_t g_ui32uDMAErrCount;
volatile uint8_t g_bFeedWatchdog;
uint32_t g_ui32CPUUsage;
uint32_t g_ui32MemXferCount=0;
uint32_t g_ui32BadISR=0;
uint32_t g_ui32SrcBuf[MEM_BUFFER_SIZE];
uint32_t g_ui32DstBuf[MEM_BUFFER_SIZE];

MMU MMUnit;
//*****************************************************************************
//
// System Function Details
//
//*****************************************************************************
void init(void){
    cfg_clock();
    //SysCtlDelay(g_ui32SysClock/10);
    cfg_wdt();
    cfg_peripheral();
    cfg_inout();
    cfg_uart();
    cfg_timer();
    cfg_pwm();
    cfg_interrupt();
    cfg_systick();
    cfg_dma();

    //
    // Initialize the CPU usage measurement routine.
    //
    CPUUsageInit(g_ui32SysClock, SYSTICKS_PER_SECOND, 2);
    ROM_FPULazyStackingEnable();
    UARTprintf("System Was Initialized !!!\n");
}
void cfg_dma(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
    //
    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    //
    ROM_IntEnable(INT_UDMAERR);

    //
    // Enable the uDMA controller.
    //
    ROM_uDMAEnable();
    //
    // Point at the control table to use for channel control structures.
    //
    ROM_uDMAControlBaseSet(pui8ControlTable);

    ROM_IntEnable(INT_UDMA);
    //
    // Fill the source memory buffer with a simple incrementing pattern.
    //
    int ui16Idx;
    for(ui16Idx = 0; ui16Idx < MEM_BUFFER_SIZE; ui16Idx++)
    {
        g_ui32SrcBuf[ui16Idx] = ui16Idx;
    }

    //
    // Enable interrupts from the uDMA software channel.
    //
    //
    // Put the attributes in a known state for the uDMA software channel.
    // These should already be disabled by default.
    //
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SW,
                                    UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                    (UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK));

    //
    // Configure the control parameters for the SW channel.  The SW channel
    // will be used to transfer between two memory buffers, 32 bits at a time.
    // Therefore the data size is 32 bits, and the address increment is 32 bits
    // for both source and destination.  The arbitration size will be set to 8,
    // which causes the uDMA controller to rearbitrate after 8 items are
    // transferred.  This keeps this channel from hogging the uDMA controller
    // once the transfer is started, and allows other channels cycles if they
    // are higher priority.
    //
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
                              UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_32 |
                              UDMA_ARB_8);

    //
    // Set up the transfer parameters for the software channel.  This will
    // configure the transfer buffers and the transfer size.  Auto mode must be
    // used for software transfers.
    //
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
                               UDMA_MODE_AUTO, g_ui32SrcBuf, g_ui32DstBuf,
                               MEM_BUFFER_SIZE);

    //
    // Now the software channel is primed to start a transfer.  The channel
    // must be enabled.  For software based transfers, a request must be
    // issued.  After this, the uDMA memory transfer begins.
    //
    ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
    //ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);
    //ROM_IntDisable(INT_UDMA);
}

void cfg_wdt(void)
{
    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    //
    // Enable the watchdog interrupt.
    //
    ROM_IntEnable(INT_WATCHDOG);

    //
    // Set the period of the watchdog timer.
    //
    ROM_WatchdogReloadSet(WATCHDOG0_BASE, g_ui32SysClock/2);

    //
    // Enable reset generation from the watchdog timer.
    //
    ROM_WatchdogResetEnable(WATCHDOG0_BASE);

    //
    // Enable the watchdog timer.
    //
    ROM_WatchdogEnable(WATCHDOG0_BASE);
}
void* MemoryAllocation(void *fp, uint32_t size)
{
    g_bFeedWatchdog = false;
    if(fp == NULL)
    {

    }
    else
    {
#ifdef MEMORY_DEBUG
        UARTprintf("[WARNING] not free pointer: %x\n", fp);
#endif
        free(fp);
    }
    free(fp);
    fp = calloc(size, sizeof(uint8_t));
    if(fp == NULL)
    {
        UARTprintf("[ERROR] Cannot allocate memory: %d", size);
        g_bFeedWatchdog = true;
        return NULL;
    }
    else
    {
#ifdef MEMORY_DEBUG
        UARTprintf("[SUCESS] Allocated %d at %x\n", size, fp);
#endif
        MMUnit.mm[MMUnit.idex] = fp;
#ifdef MEMORY_DEBUG
        UARTprintf("[SUCESS] Recorded: %x\n", MMUnit.mm[MMUnit.idex]);
#endif
        g_bFeedWatchdog = true;
        return fp;
    }

}
void cfg_pwm(void){
    g_bFeedWatchdog = false;
    // Wait for the PWM0 module to be ready.
    //
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinConfigure(GPIO_PK5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Configure the PWM generator for count down mode with immediate updates
    // to the parameters.
    //
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
    // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
    // Use this value to set the period.
    //
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 24000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 24000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 24000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 24000);
    //
    // Set the pulse width of PWM0 for a 25% duty cycle.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 3);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 3);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 3);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 3);
    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT), true);
//    PWM_duty = SysCtlClockGet()/2000;
//    // PWM Max value < PWM_duty (-100)
//    PWM_Limit_Min = PWM_duty-100;

//    // Gioi han out put cho PWM (%)
//    PWM_Limit_Per = 50;
//    UARTprintf("PWM_duty: %d\n", PWM_duty);
//    UARTprintf("PWM_Limit_Min: %d\n", PWM_Limit_Min);
//    UARTprintf("PWM_Limit_Per %d %%\n", PWM_Limit_Per);

    //Configure PWM Clock to match system
    // GPIOPinConfigure(GPIO_PB6_M0PWM0);
    // GPIOPinConfigure(GPIO_PE4_M0PWM4);

    // GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    // GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

    // PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
 //    //Set the Period (expressed in clock ticks)
 //    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, SysCtlClockGet()/2000);
 //    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, SysCtlClockGet()/2000);
 //    //Set PWM duty-50% (Period /2)
 //    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
 //    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);

 //    // Enable the PWM generator
 //    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
 //    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

 //    // Turn on the Output pins
    //PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_4_BIT, true);
        //Configure PWM Clock to match system
    // SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // // GPIOPinConfigure(GPIO_PB6_M0PWM0);
    // // GPIOPinConfigure(GPIO_PB7_M0PWM1);
    // // GPIOPinConfigure(GPIO_PB4_M0PWM2);
    // // GPIOPinConfigure(GPIO_PB5_M0PWM3);
    // // GPIOPinConfigure(GPIO_PE4_M0PWM4);
    // // GPIOPinConfigure(GPIO_PE5_M0PWM5);
    // GPIOPinConfigure(GPIO_PD0_M0PWM6);
    // GPIOPinConfigure(GPIO_PD1_M0PWM7);


    // // GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // // GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    // GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // //Set the Period
    // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 15625);
    // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 15625);
    // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 15625);
    // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 15625);

    // //Set PWM duty
    // // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ServoMin);
    // // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ServoMin);
    // // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ServoMin);
    // // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ServoMin);
    // // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ServoMin);
    // // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, ServoMin);
    // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 1100);


    // // Enable the PWM generator
    // PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    // PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    // PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    // PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    // // Turn on the Output pins
    // PWMOutputState(PWM0_BASE,
    //         PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT
    //                 | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT
    //                 | PWM_OUT_7_BIT, true);
}
void cfg_interrupt(void){
//  GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5);
//  GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);
//  IntEnable(INT_GPIOC);
//  GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
}
void cfg_inout(void) {
    // ERROR MAPPING
    ROM_GPIOPinTypeGPIOInput(ERRORPORT, FRTSEN | FWDSEN);

    // NOTICE MAPPING
    ROM_GPIOPinTypeGPIOOutput(NOTICEPORT, SPK | FRTLED | FWDLED);

    // BUTTON MAPPING
    ROM_GPIOPinTypeGPIOInput(BUTTONPORTN, FRTBUTTON);
    ROM_GPIOPinTypeGPIOInput(BUTTONPORTH, FWDBUTTON | STRBUTTON);

    // ZIGBEE MAPPING
    // UART Config

    // OUTPUT MAPPING
    ROM_GPIOPinTypeGPIOOutput(OUTPUTPORT, OUT1 | OUT2 | OUT3 | OUT4 | OUT5);

    // RFID MAPPING
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // LCD1 MAPPING
    ROM_GPIOPinTypeGPIOInput(LCD1PORT1, GPIO_PIN_4 | GPIO_PIN_5);
    ROM_GPIOPinTypeGPIOInput(LCD1PORT2, GPIO_PIN_4 | GPIO_PIN_5);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    //GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
     HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
     HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
     HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

     HWREG(GPIO_PORTG_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
     HWREG(GPIO_PORTG_BASE + GPIO_O_CR) |= GPIO_PIN_0 | GPIO_PIN_1;
     HWREG(GPIO_PORTG_BASE + GPIO_O_LOCK) = 0;

    // ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
    //         GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    // ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);

    // // Resistor Pull up for PORTF.0 & PORTF.4 with 8mA
    // ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4,
    //         GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);

    // // Resistor Pull up for PORTA.2 & PORTF.3 with 8mA
 /*   GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1,
            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_RISING_EDGE  );
    IntEnable(INT_GPIOJ);
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // // Resistor Pull up for PORTA.2 & PORTF.3 with 8mA
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3,
            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
  //  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2 | GPIO_PIN_3);
    */

    // vd /////////////////////////////

    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 ,
              GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);

      GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 );
      GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 , GPIO_BOTH_EDGES  );
      //IntEnable(INT_GPIOJ);
      //GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 );
      // // Resistor Pull up for PORTA.2 & PORTF.3 with 8mA
     // GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3,




}
void cfg_uart(void){
    g_bFeedWatchdog = false;
    // UART 0
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTStdioConfig(0, 115200, g_ui32SysClock);
    UARTFIFODisable(UART0_BASE);
    UARTIntEnable(UART0_BASE, UART_INT_RX);
    IntEnable(INT_UART0);
    //
    UARTprintf("\n***Thong Tin He Thong*******************");
    UARTprintf("\n***        CPU Clock %3d (Mhz)       ***", g_ui32SysClock/1000000);
    UARTprintf("\n***        Timer 0 Enable            ***");
    UARTprintf("\n***        Timer 1 Enable            ***");
    UARTprintf("\n***        Timer 2 Enable            ***");
    UARTprintf("\n***        Timer 3 Enable            ***");
    UARTprintf("\n***        Timer 4 Enable            ***");
    UARTprintf("\n***        Timer 5 Enable            ***");
    UARTprintf("\n****************************************\n");
    g_bFeedWatchdog = true;
}
void cfg_peripheral(void){
    g_bFeedWatchdog = false;
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    g_bFeedWatchdog = true;
}
void cfg_clock(void){
    //
    // Set the clocking to run directly from the crystal at 120MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
}
void cfg_timer(void){
    g_bFeedWatchdog = false;
    // Timer 0
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_PIOSC);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock/100);
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    // Timer 1
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_PIOSC);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock/50);
    ROM_IntEnable(INT_TIMER1A);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);


    // Timer 3
    TimerClockSourceSet(TIMER3_BASE, TIMER_CLOCK_PIOSC);
    ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, g_ui32SysClock/100);
    ROM_IntEnable(INT_TIMER3A);
    ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER3_BASE, TIMER_A);

    // Timer 4
    TimerClockSourceSet(TIMER4_BASE, TIMER_CLOCK_PIOSC);
    ROM_TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER4_BASE, TIMER_A, g_ui32SysClock/100);
    ROM_IntEnable(INT_TIMER4A);
    ROM_TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    //ROM_TimerEnable(TIMER4_BASE, TIMER_A);
    g_bFeedWatchdog = true;
}
void cfg_systick(void){
    SysTickPeriodSet(g_ui32SysClock/SYSTICKS_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();
}
void cfg_uartext(uint32_t uartbase, uint32_t baurate, uint32_t mode)
{
    if(uartbase == UART1_BASE)
    {
        GPIOPinConfigure(GPIO_PB0_U1RX);
        GPIOPinConfigure(GPIO_PB1_U1TX);
        GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                 GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
        UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(UART1_BASE, g_ui32SysClock, baurate, mode);
        //(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
    }
    else if(uartbase == UART2_BASE)
    {
        ROM_GPIOPinConfigure(GPIO_PA6_U2RX);
        ROM_GPIOPinConfigure(GPIO_PA7_U2TX);
        ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7,
                 GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6 | GPIO_PIN_7);
        UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(UART2_BASE, g_ui32SysClock, baurate, mode);
        UARTFIFOLevelSet(UART2_BASE, UART_FIFO_RX7_8, UART_FIFO_RX7_8);
        UARTFIFODisable(UART2_BASE);
        UARTIntEnable(UART2_BASE, UART_INT_RX);
        UARTCharGetNonBlocking(UART2_BASE);
        IntEnable(INT_UART2);
    }
    else if(uartbase == UART3_BASE)
    {
        ROM_GPIOPinConfigure(GPIO_PA4_U3RX);
        ROM_GPIOPinConfigure(GPIO_PA5_U3TX);
        ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);
        UARTClockSourceSet(UART3_BASE, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(UART3_BASE, g_ui32SysClock, 115200,
             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
             UART_CONFIG_PAR_NONE));
        //UARTFIFODisable(UART3_BASE);
        ROM_UARTDMAEnable(UART3_BASE, UART_DMA_RX);
        MAP_uDMAChannelAssign(UDMA_CH16_UART3RX);

        ROM_uDMAChannelAttributeDisable(UDMA_CH16_UART3RX,
                                        UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                        UDMA_ATTR_HIGH_PRIORITY |
                                        UDMA_ATTR_REQMASK);

        ROM_uDMAChannelControlSet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT,
                                  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                                  UDMA_ARB_4);

        ROM_uDMAChannelControlSet(UDMA_CH16_UART3RX | UDMA_ALT_SELECT,
                                  UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                                  UDMA_ARB_4);

        ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(UART3_BASE + UART_O_DR),
                                   g_UART3RX1, sizeof(g_UART3RX1));

        ROM_uDMAChannelTransferSet(UDMA_CH16_UART3RX | UDMA_ALT_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(UART1_BASE + UART_O_DR),
                                   g_UART3RX2, sizeof(g_UART3RX2));
        //ROM_uDMAChannelAttributeEnable(UDMA_CH16_UART3RX, UDMA_ATTR_ALL);
        ROM_uDMAChannelEnable(UDMA_CH16_UART3RX);
        ROM_uDMAChannelRequest(UDMA_CH16_UART3RX);
        //ROM_UARTIntEnable(UART3_BASE, UART_INT_DMARX);
        //ROM_IntEnable(INT_UART3);
        //UARTFIFOLevelSet(UART3_BASE, UART_FIFO_RX2_8, UART_FIFO_RX2_8);

        //UARTFIFODisable(UART3_BASE);
        //UARTIntEnable(UART3_BASE, UART_INT_RX);
    }
    else if(uartbase == UART4_BASE)
    {
        ROM_GPIOPinConfigure(GPIO_PK0_U4RX);
        ROM_GPIOPinConfigure(GPIO_PK1_U4TX);
        ROM_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                 GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
        UARTClockSourceSet(UART4_BASE, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(UART4_BASE, g_ui32SysClock, baurate, mode);
    }else if(uartbase == UART5_BASE)
    {
        ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
        ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
        ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(UART1_BASE, g_ui32SysClock, baurate, mode);
    }
    else if(uartbase == UART6_BASE)
    {
        ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
        ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
        ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
        ROM_UARTConfigSetExpClk(UART1_BASE, g_ui32SysClock, baurate, mode);
    }
    else
    {
        UARTprintf("[ERROR] CANNOT support for UART[%d]", uartbase);
    }
}

//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    UARTprintf("uDMAErrorHandler !!!\n");
    uint32_t ui32Status;
    //
    // Check for uDMA error bit
    //
    ui32Status = ROM_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }
}
//*****************************************************************************
//
// The interrupt handler for uDMA interrupts from the memory channel.  This
// interrupt will increment a counter, and then restart another memory
// transfer.
//
//*****************************************************************************
void
uDMAIntHandler(void)
{
    UARTprintf("uDMAIntHandler !!!\n");
    uint32_t ui32Mode;

    //
    // Check for the primary control structure to indicate complete.
    //
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        //
        // Increment the count of completed transfers.
        //
        g_ui32MemXferCount++;

        //
        // Configure it for another transfer.
        //
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW, UDMA_MODE_AUTO,
                                     g_ui32SrcBuf, g_ui32DstBuf,
                                     MEM_BUFFER_SIZE);

        //
        // Initiate another transfer.
        //
        ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
        ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);
    }

    //
    // If the channel is not stopped, then something is wrong.
    //
    else
    {
        g_ui32BadISR++;
    }
}
int dem = 0;
void
SysTickHandler(void)
{
    static uint32_t ui32TickCount = 0;

    //
    // Increment the tick counter.
    //
    ui32TickCount++;
    dem ++;
     if(dem == 20){
    		if (den == 1) {
    			GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,
    					~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1));
    		} else {
    			GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
    		}
       dem = 0;
     }

    //
    // If the number of ticks per second has occurred, then increment the
    // seconds counter.
    //
    if(!(ui32TickCount % SYSTICKS_PER_SECOND))
    {
        g_ui32Seconds++;
        binh = 1;
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0));
    }

    //
    // Call the CPU usage tick function.  This function will compute the amount
    // of cycles used by the CPU since the last call and return the result in
    // percent in fixed point 16.16 format.
    //
    g_ui32CPUUsage = CPUUsageTick();
    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1));
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0));
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4));

}

void
WatchdogIntHandler(void)
{
    //
    // If we have been told to stop feeding the watchdog, return immediately
    // without clearing the interrupt.  This will cause the system to reset
    // next time the watchdog interrupt fires.
    //
    if(!g_bFeedWatchdog)
    {
        return;
    }

    //
    // Clear the watchdog interrupt.
    //
    ROM_WatchdogIntClear(WATCHDOG0_BASE);

    //
    // Invert the GPIO PF3 value.
    //
    ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2,
                     (ROM_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_2) ^
                                     GPIO_PIN_2));
}
