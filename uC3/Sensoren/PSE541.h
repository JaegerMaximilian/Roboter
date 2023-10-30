/*
 * PSE541.h
 *
 * Created: 10.12.2019 14:17:44
 *  Author: P20087
 */ 


#ifndef PSE541_H_
#define PSE541_H_

#include <avr/io.h>

/* intern/extern switch */
#ifndef _PSE541_EXTERN
	#define _PSE541_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
typedef struct
{
	uint16_t pressure;					/*!< measured pressure */
	uint8_t pin;						/*!< ADC pin -> 0 ... 7 */
	uint16_t (*adc)(uint8_t,uint8_t);	/*!< read ADC function */
} PSE541_t;

/* pressure sensors (front left, front right) */
_PSE541_EXTERN PSE541_t pressureRearLeft, pressureRearRight;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void PSE541_init();
void processPSE541(PSE541_t *sensor);



#endif /* PSE541_H_ */