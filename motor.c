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
#include "sensor.h"
int32_t leftm, rightm;
uint32_t minspeed;
uint32_t maxspeed;

// Right Motor, Left Motor
void MotorController(int32_t spd_l, int32_t spd_r) {
    if (spd_l < 0) {
        spd_l *= -1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 2*spd_l);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 3);
    }
    else
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 2*spd_l);
    }

    if (spd_r < 0){
        spd_r *= -1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 3);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 2*spd_r);
    }
    else
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 2*spd_r);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 3);
    }
    leftm = spd_l;
    rightm = spd_r;
}
int32_t Err, pre_err, pPart_l, dPart_l, iPart_l, result_l;
int32_t kp_l = 100;
int32_t ki_l = 0;
int32_t kd_l = 0;
int32_t pPart_r, dPart_r, iPart_r, result_r;
int32_t kp_r = 100;
int32_t ki_r = 0;
int32_t kd_r = 0;
int32_t sampling_time = 5;
int32_t inv_sampling_time = 50;
int32_t MotorControllerPID(int32_t spd_l, int32_t spd_r)
{
    Err         =   LineSensorGet();

    pPart_l         =   kp_l*Err;
    dPart_l         =   kd_l*(Err-pre_err)*inv_sampling_time;
    iPart_l         +=  (ki_l*sampling_time*Err)/1000;
    result_l        +=  pPart_l + dPart_l + iPart_l;
    if(result_l < minspeed)
    {
        result_l = minspeed;
    }

    if(result_l > maxspeed)
    {
        result_l = maxspeed;
    }

    pPart_r         =   kp_r*Err*-1;
    dPart_r         =   kd_r*(Err-pre_err)*inv_sampling_time;
    iPart_r         +=  (ki_r*sampling_time*Err)/1000;
    result_r        +=  pPart_r + dPart_r + iPart_r;
    if(result_r < minspeed)
    {
        result_r = minspeed;
    }

    if(result_r > maxspeed)
    {
        result_r = maxspeed;
    }
    pre_err     =   Err;
    UARTprintf("Error: %d, result_l: %d = result_l: %d\n", Err, result_l, result_r);
    MotorController(result_l, result_r);
    return Err;
}
