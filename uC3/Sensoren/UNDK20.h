/*
 * UNDK20.h
 *
 * Created: 17.12.2019 15:07:56
 *  Author: P20087
 */ 


#ifndef UNDK20_H_
#define UNDK20_H_

#include <avr/io.h>

/* intern/extern switch */
#ifndef _UNDK20_EXTERN
	#define _UNDK20_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
typedef struct
{
	int8_t distance;					/*!< measured distance [cm] */
	uint8_t pin;						/*!< ADC pin -> 0 ... 7 */
	uint16_t (*adc)(uint8_t,uint8_t);	/*!< read ADC function */
} UNDK20_t;

/* pressure sensors (front left, front right) */
_UNDK20_EXTERN UNDK20_t us1, us2, us3, us4, us5;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void UNDK20_init();
void processUNDK20(UNDK20_t *sensor);






#endif /* UNDK20_H_ */