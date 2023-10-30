/*
 * PSE541.c
 *
 * Created: 10.12.2019 14:17:17
 *  Author: P20087
 */ 


#define _PSE541_EXTERN

#include <avr/io.h>
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "adc_driver.h"
#include "PSE541.h"


/* ************************************************************** */
/*! \brief initialize function for PSE541 pressure sensor.
 *
 *  initialize the PSE541 sensor (ADC, pin, ...).
 *  
*/
/* ************************************************************** */
void PSE541_init()
{
	/* ********** */
	/* FRONT LEFT */
	/* ********** */
	/* set ADC to PORTB */
	pressureRearLeft.adc = adcb_read;
	/* set PIN to 5 */
	pressureRearLeft.pin = 5;
	
	/* *********** */
	/* FRONT RIGHT */
	/* *********** */
	/* set ADC to PORTB */
	pressureRearRight.adc = adcb_read;
	/* set PIN to 7 */
	pressureRearRight.pin = 7;	
}


/* ************************************************************** */
/*! \brief read function for PSE541 pressure sensor.
 *
 *  reads the PSE541 sensor [mbar].
 *
 *  \param sensor pointer to PSE541 sensor.
 *  
*/
/* ************************************************************** */
void processPSE541(PSE541_t *sensor)
{
	/* read sensor value and recalculate pressure value (0 ... 1000 mbar) */
	sensor->pressure = 1000 - (signed int)((sensor->adc(ADC_CH_0, sensor->pin) - 780) * 10 / 33);
	/* limit pressure value to 0 mbar */
	sensor->pressure = ((sensor->pressure < 0) ? 0 : sensor->pressure); 
	/* limit pressure value to 1000 mbar */
	sensor->pressure = ((sensor->pressure > 1000) ? 1000 : sensor->pressure);
}