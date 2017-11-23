/*
 * BatterySensor.h
 *
 *  Created on: Nov 4, 2017
 *      Author: TDNC
 */

#ifndef BATTERYSENSOR_H_
#define BATTERYSENSOR_H_
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
void ADCInit();
uint32_t ADCGet(uint32_t *adcValues);



#endif /* BATTERYSENSOR_H_ */
