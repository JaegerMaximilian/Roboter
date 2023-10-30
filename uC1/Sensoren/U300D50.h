/*
 * U300D50.h
 *
 * Created: 17.12.2019 15:07:56
 *  Author: P20087
 */ 


#ifndef U300D50_H_
#define U300D50_H_

#include <avr/io.h>

/* intern/extern switch */
#ifndef _U300D50_EXTERN
	#define _U300D50_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
typedef struct
{
	int16_t distance;					/*!< measured distance [cm] */
	uint8_t pin;						/*!< ADC pin -> 0 ... 7 */
	uint16_t (*adc)(uint8_t,uint8_t);	/*!< read ADC function */
} U300D50_t;

/* pressure sensors (front left, front right) */
_U300D50_EXTERN U300D50_t usLV, usLH, usRV, usRH;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void U300D50_init();
void processU300D50(U300D50_t *sensor);

#endif /* U300D50_H_ */