/*
 * anaPos.c
 *
 * Created: 13.12.2019 15:43:08
 *  Author: P20087
 */ 

#define _ANAPOS_EXTERN

#include <avr/io.h>
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "adc_driver.h"
#include "anaPos.h"


/* ************************************************************** */
/*! \brief initialize function for analog position measurement.
 *
 *  initialize the analog position measuremet (ADC, pin, ...).
 *  
*/
/* ************************************************************** */
void anaPos_init()
{
	/* ********** */
	/* FRONT LEFT */
	/* ********** */
	/* set ADC to PORTA */
	posFrontLeft.adc = adca_read;
	/* set PIN to 7 */
	posFrontLeft.pin = 7;
	/* set gradient */
	posFrontLeft.k = -0.0595;	// !!!!!!!
	/* set offset */
	posFrontLeft.d = 198.735;	// !!!!!!!
			
	/* *********** */
	/* FRONT RIGHT */
	/* *********** */
	/* set ADC to PORTA */
	posFrontRight.adc = adca_read;
	/* set PIN to 6 */
	posFrontRight.pin = 6;	
	/* set gradient */
	posFrontRight.k = -0.0632;	// !!!!!!!
	/* set offset */
	posFrontRight.d = 222.464;	// !!!!!!!
}


/* ************************************************************** */
/*! \brief read function for position measurement.
 *
 *  reads position [m].
 *
 *  \param sensor pointer to position sensor.
 *  
*/
/* ************************************************************** */
void processAnaPos(anaPos_t *poti)
{
	/* read sensor value and recalculate position value [m] */
	poti->pos = (float)(poti->adc(ADC_CH_0, poti->pin)) * poti->k + poti->d;
}