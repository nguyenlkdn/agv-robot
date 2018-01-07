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
#include "config.h"
uint32_t midleftGet(uint32_t data)
{
    uint32_t ret = ((data>>4)&0x000f);
    return ret;
}
uint32_t midrightGet(uint32_t data)
{
    uint32_t ret=0;
    uint32_t readmask=0x0001;
    uint32_t writemask=0x0008;
    data &= 0x000f;
    int i;
    for(i=0;i<4;i++)
    {
        if(data & readmask)
        {
            ret |= writemask;
        }
        readmask<<=1;
        writemask>>=1;
    }
    return ret;
}
int32_t LineSensorGet(void)
{
    uint32_t sensor = GPIOPinRead(GPIO_PORTM_BASE, 0xff)&0xff;
#ifdef SENSOR_DEBUG
    UARTprintf("Line Value  : %d%d%d%d%d%d%d%d (%3d)", (sensor & 0x80)>>7,
                     (sensor & 0x40)>>6, (sensor & 0x20)>>5, (sensor & 0x10)>>4, (sensor & 0x08)>>3,
                     (sensor & 0x04)>>2, (sensor & 0x02)>>1, (sensor & 0x01), sensor);
#endif
    if(sensor == 0)
    {
        return 256;
    }
    int invalid = 0;
    int mask = 0x80;
    int i;
    int sensor1[8];
    for (i = 0; i < 8; i++)
    {
        if (sensor & mask)
        {
            sensor1[i] = 1;
            invalid++;
        }
        else
        {
            sensor1[i] = 0;
        }
        mask >>= 1;
    }

    for (i = 0; i < 5; i++)
    {
        if ((sensor1[i] == 1) && (sensor1[i + 1] == 0) && (sensor1[i + 2] == 1))
        {
            invalid = 9;
        }
    }
    if(invalid >= 3)
    {
        return 255;
    }
    else
    {
        return midleftGet(sensor)-midrightGet(sensor);
    }
}

int32_t SensorValueGet(void)
{
    int32_t ret=-1;
    int32_t sensor = GPIOPinRead(GPIO_PORTM_BASE, 0xFF);
#ifdef SENSOR_DEBUG
    UARTprintf("Line Value  : %d%d%d%d%d%d%d%d (%3d)", (sensor & 0x80)>>7,
                     (sensor & 0x40)>>6, (sensor & 0x20)>>5, (sensor & 0x10)>>4, (sensor & 0x08)>>3,
                     (sensor & 0x04)>>2, (sensor & 0x02)>>1, (sensor & 0x01), sensor);
#endif
    switch ( sensor )
    {
        case 38:
            ret = 0;
            break;
        //////////////////////////////////////////////////////////////////
        //case 34:        // 0010 0010
        case 102:        // 0110 0100
            ret = 1;
            break;
        //case 50:        // 0011 0010
        case 98:          // 0110 0010
            ret = 2;
            break;
        //case 18:        // 0001 0010
        case 99:        // 0110 0011
            ret = 3;
            break;
        //case 19:        // 0001 0011
        case 67:        // 0001 0011
            ret = 4;
            break;
        //case 17:        // 0001 0001
        case 65:        // 0001 0001
            ret = 5;
            break;
        case 1:
            ret = 10;
            break;
        //////////////////////////////////////////////////////////////////
        case 54:        // 0010 0100
            ret = -1;
            break;
        //case 100:       // 0110 0100
        case 52:       // 0110 0100
            ret = -2;
            break;
        //case 68:        // 0100 0100
        case 60:        // 0100 0100
            ret = -3;
            break;
        //case 76:        // 0100 1100
        case 28:        // 0100 1100
            ret = -4;
            break;
        //case 72:        // 0100 1000
        case 24:        // 0100 1000
            ret = -5;
            break;
        case 8:
            ret = -10;
            break;
        ///////////////////////////////////////////////////////////////////
        case 0:
            ret = 128;
            break;
    }

    return ret;
}
void LineSensorInit()
{
    ROM_SysCtlPeripheralEnable(GPIO_PORTM_BASE);

    // SEN2 MAPPING
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, OSEN21 | OSEN22 | OSEN23 | OSEN24 | OSEN25 | OSEN26 | OSEN27 | OSEN28);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    GPIOPadConfigSet(GPIO_PORTM_BASE, 0xFF,
            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
            GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinWrite(GPIO_PORTM_BASE, 0xFF, 0xFF);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
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
    //IntEnable(INT_GPIOB);
    //GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
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
