/*
 * batterysensor.c
 *

 */

#include "batterysensor.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
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
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "config.h"
uint32_t g_ui32SysClock;
void ADCInit()
{
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(10);

    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_EIGHTH, 30);

        // Choose Sequencer 2 and set it at the highest Priority.
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 0);

        // Choose Step 0 in Sequencer 2 as Data Buffer, set it as last Step and enable Interrupts
    ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH10 | ADC_CTL_IE | ADC_CTL_END);
    //ADCSequenceStepConfigure(ADC0_BASE,0,1, ADC_CTL_CH1 | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 0);
}

uint32_t oldbatterypercent = 0;
uint32_t ADCGet(uint32_t *adcValues)
{
    uint32_t batterypercent = 0;
    ADCProcessorTrigger(ADC0_BASE,0);
    // Wait until the sample sequence has completed.
    uint32_t timeout = g_ui32SysClock/10;
    while(!ADCIntStatus(ADC0_BASE, 0, false)) {
        if(--timeout == 0)
        {
            break;
        }
    }
    // Read the value from the ADC.
    ADCSequenceDataGet(ADC0_BASE, 0, adcValues);
    uint32_t volt = (26 - (adcValues[0]/124));
    batterypercent = (100-((volt*100)/3));
    batterypercent = batterypercent*0.3 + oldbatterypercent*0.7;
    oldbatterypercent = batterypercent;
    ROBOTTX_Buffer[3] = batterypercent;
    return batterypercent;
}


