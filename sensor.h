/*
 * sensor.h
 *
 *  Created on: Sep 16, 2017
 *      Author: sdev
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

// SEN1 MAPPING
#define SEN1PERH    SYSCTL_PERIPH_GPIOD
#define SEN1PORT    GPIO_PORTD_BASE
#define OSEN11      GPIO_PIN_0
#define OSEN12      GPIO_PIN_1
#define OSEN13      GPIO_PIN_2
#define OSEN14      GPIO_PIN_3
#define OSEN15      GPIO_PIN_4
#define OSEN16      GPIO_PIN_5
#define OSEN17      GPIO_PIN_6
#define OSEN18      GPIO_PIN_7

// SEN2 MAPPING
#define SEN2PERH    SYSCTL_PERIPH_GPIOM
#define SEN2PORT    GPIO_PORTM_BASE
#define OSEN21      GPIO_PIN_0
#define OSEN22      GPIO_PIN_1
#define OSEN23      GPIO_PIN_2
#define OSEN24      GPIO_PIN_3
#define OSEN25      GPIO_PIN_4
#define OSEN26      GPIO_PIN_5
#define OSEN27      GPIO_PIN_6
#define OSEN28      GPIO_PIN_7

// GPD1 MAPPING
#define GPD1PORT    GPIO_PORTB_BASE
#define PSEN1OUT    GPIO_PIN_2

// GPD2 MAPPING
#define GPD2PORT    GPIO_PORTB_BASE
#define PSEN2OUT    GPIO_PIN_3

// GPD3 MAPPING
#define GPD3PORT    GPIO_PORTC_BASE
#define PSEN3OUT    GPIO_PIN_7

// GPD4 MAPPING
#define GPD4PORT    GPIO_PORTC_BASE
#define PSEN4OUT    GPIO_PIN_6

// GPD5 MAPPING
#define GPD5PORT    GPIO_PORTC_BASE
#define PSEN5OUT    GPIO_PIN_5

// GPD6 MAPPING
#define GPD6PORT    GPIO_PORTC_BASE
#define PSEN6OUT    GPIO_PIN_4

// GPD3 MAPPING
#define GPD7PORT    GPIO_PORTH_BASE
#define PSEN7OUT    GPIO_PIN_0

// GPD4 MAPPING
#define GPD8PORT    GPIO_PORTH_BASE
#define PSEN8OUT    GPIO_PIN_1

// GPD5 MAPPING
#define GPD9PORT    GPIO_PORTH_BASE
#define PSEN9OUT    GPIO_PIN_2

// GPD6 MAPPING
#define GPD10PORT    GPIO_PORTH_BASE
#define PSEN10OUT    GPIO_PIN_3
uint8_t LineSensorGet(int8_t sensorname);
void LineSensorInit();
void GPDSensorInit();
#endif /* SENSOR_H_ */
