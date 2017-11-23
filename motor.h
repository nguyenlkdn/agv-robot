/*
 * hdl_motor.h
 *
 *  Created on: Sep 16, 2017
 *      Author: sdev
 */

#ifndef MOTOR_H_
#define MOTOR_H_
#include <stdint.h>
#include <stdbool.h>
void MotorController(int32_t spd_l, int32_t spd_r);
void MotorInit();




#endif /* MOTOR_H_ */
