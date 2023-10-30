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
	/* ****** */
	/*  REAR  */
	/* ****** */
	/* set ADC to PORTA */
	posRear.adc = adca_read;
	/* set PIN to 6 */
	posRear.pin = 7;	
	/* set gradient */
	posRear.k = 1.0;//-0.145161;	// !!!!!!!
	/* set offset */
	posRear.d = 0.0;//379.032;	// !!!!!!!
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
	char text[20];
	
	/* read sensor value and recalculate position value [m] */
	poti->pos = (float)(poti->adc(ADC_CH_0, poti->pin)) * poti->k + poti->d;
	
// 	sprintf(text, "%.0f\r",poti->pos);
// 	debugMsg(text);
	

}