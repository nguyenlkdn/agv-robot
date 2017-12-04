/*
 * src_motor.c
 *
 *  Created on: Sep 16, 2017
 *      Author: sdev
 */

#include "motor.h"

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
int32_t leftm, rightm;

void MotorController(int32_t spd_l, int32_t spd_r) {
    if (spd_l < 0) {
        spd_l *= -1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 2*spd_l+3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 3);
    }
    else
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 2*spd_l+3);
    }

    if (spd_r < 0){
        spd_r *= -1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 2*spd_r+3);
    }
    else
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 2*spd_r+3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 3);
    }
    leftm = spd_l;
    rightm = spd_r;
}

void MotorInit()
{

}
