/*
 * anaPos.h
 *
 * Created: 13.12.2019 15:43:23
 *  Author: P20087
 */ 


#ifndef ANAPOS_H_
#define ANAPOS_H_

#include <avr/io.h>

/* intern/extern switch */
#ifndef _ANAPOS_EXTERN
	#define _ANAPOS_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
typedef struct
{
	float pos;							/*!< measured possition [m] */
	uint8_t pin;						/*!< ADC pin -> 0 ... 7 */
	uint16_t (*adc)(uint8_t,uint8_t);	/*!< read ADC function */
	float k;							/*!< gradient (y = k*x + d) */
	float d;							/*!< offset (y = k*x + d) */
} anaPos_t;

/* pressure sensors (front left, front right) */
_ANAPOS_EXTERN anaPos_t posRear;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void anaPos_init();
void processAnaPos(anaPos_t *sensor);




#endif /* ANAPOS_H_ */