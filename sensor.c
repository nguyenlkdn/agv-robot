/*
 * sensor.c
 *
 *  Created on: Sep 16, 2017
 *      Author: sdev
 */
#include "sensor.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"

uint8_t LineSensorGet(int8_t sensorname)
{
    if (sensorname == 1)
    {
        return GPIOPinRead(SEN1PORT, OSEN11 | OSEN12 | OSEN13 | OSEN14 | OSEN15 | OSEN16 | OSEN17 | OSEN18);
    }
    else if (sensorname == 2)
    {
        return GPIOPinRead(SEN2PORT, OSEN21 | OSEN22 | OSEN23 | OSEN24 | OSEN25 | OSEN26 | OSEN27 | OSEN28);
    }
    else
    {
        UARTprintf("ERR: Sensor name invalid: %d", sensorname);
        return 0;
    }
}

void LineSensorInit()
{
    ///ROM_SysCtlPeripheralEnable(SEN1PERH);
    ROM_SysCtlPeripheralEnable(SEN2PERH);

    // SEN1 MAPPING
    //ROM_GPIOPinTypeGPIOInput(SEN1PORT, OSEN11 | OSEN12 | OSEN13 | OSEN14 | OSEN15 | OSEN16 | OSEN17 | OSEN18);
//    GPIOPadConfigSet(SEN1PORT, 0xFF,
//            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
    //GPIOPinWrite(SEN1PORT, 0xFF, 0xFF);

    // SEN2 MAPPING
    ROM_GPIOPinTypeGPIOInput(SEN2PORT, OSEN21 | OSEN22 | OSEN23 | OSEN24 | OSEN25 | OSEN26 | OSEN27 | OSEN28);
    GPIOPadConfigSet(SEN2PORT, 0xFF,
            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(SEN2PORT, 0xFF, 0xFF);
}

void GPDSensorInit()
{
//    // GPD1 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD1PORT, PSEN1OUT);
//
//    // GPD2 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD2PORT, PSEN2OUT);
//
//    // GPD3 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD3PORT, PSEN3OUT);
//
//    // GPD4 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD4PORT, PSEN4OUT);
//
//    // GPD5 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD5PORT, PSEN5OUT);
//
//    // GPD6 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD6PORT, PSEN6OUT);
//      // GPD6 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD7PORT, PSEN7OUT);
//
//    // GPD4 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD8PORT, PSEN8OUT);
//
//    // GPD5 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD9PORT, PSEN9OUT);
//
//    // GPD6 MAPPING
//    ROM_GPIOPinTypeGPIOInput(GPD10PORT, PSEN10OUT);







//    GPIOPadConfigSet(GPD1PORT, PSEN1OUT | PSEN2OUT,
//            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
//    GPIOPinWrite(GPD1PORT, PSEN1OUT | PSEN2OUT, true);
//
//    GPIOPadConfigSet(GPD3PORT, PSEN3OUT | PSEN4OUT | PSEN5OUT | PSEN6OUT,
//            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
//    GPIOPinWrite(GPD1PORT, PSEN1OUT | PSEN2OUT, true);
//    GPIOPinWrite(GPD3PORT, PSEN3OUT | PSEN4OUT | PSEN5OUT | PSEN6OUT, true);
//  // sen so H
//     GPIOPadConfigSet(GPIO_PORTH_BASE, PSEN7OUT | PSEN8OUT | PSEN9OUT | PSEN10OUT,
//            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
//
//    GPIOPinWrite(GPIO_PORTH_BASE, PSEN7OUT | PSEN8OUT | PSEN9OUT | PSEN10OUT, true);


    // Interrupt PORTB.2 & PORTB.3
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3,  GPIO_BOTH_EDGES  );
    IntEnable(INT_GPIOB);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
   // GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, true);
     GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3,
            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);


//    // Interrupt PORTC.4 & PORTC.5 & PORTC.6 & PORTC.6
    GPIOIntEnable(GPIO_PORTC_BASE,  GPIO_PIN_6 | GPIO_PIN_7);
    GPIOIntTypeSet(GPIO_PORTC_BASE,  GPIO_PIN_6 | GPIO_PIN_7,  GPIO_BOTH_EDGES   );
    //IntEnable(INT_GPIOC);
    GPIOIntClear(GPIO_PORTC_BASE,  GPIO_PIN_6 | GPIO_PIN_7);
    // GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6 | GPIO_PIN_7, true);
     GPIOPadConfigSet(GPIO_PORTC_BASE,  GPIO_PIN_6 | GPIO_PIN_7,
                GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
//
//      // Interrupt PORTH.0 & PORTH1 & PORTH2 & PORTH3
    GPIOIntEnable(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOIntTypeSet(GPIO_PORTH_BASE,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,  GPIO_BOTH_EDGES   );
   // IntEnable(INT_GPIOH);
    GPIOIntClear(GPIO_PORTH_BASE,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
  //  GPIOPinWrite(GPIO_PORTH_BASE,  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, true);
    GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                 GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);

}
