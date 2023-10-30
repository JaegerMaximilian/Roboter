/*
 * U300D50.c
 *
 * Created: 17.12.2019 15:07:43
 *  Author: P20087
 */ 

#define _U300D50_EXTERN

#include <avr/io.h>
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "adc_driver.h"
#include "U300D50.h"


/* ************************************************************** */
/*! \brief initialize function for U300D50 ultrasonic sensor.
 *
 *  initialize the U300D50 sensor (ADC, pin, ...).
 *  
*/
/* ************************************************************** */
void U300D50_init()
{
	/* *********** */
	/* US SENSOR 0 */
	/* *********** */
	/* set ADC to PORTA */
	usLV.adc = adca_read;
	/* set PIN to 0 */
	usLV.pin = 0;
	
	/* *********** */
	/* US SENSOR 1 */
	/* *********** */
	/* set ADC to PORTA */
	usLH.adc = adca_read;
	/* set PIN to 1 */
	usLH.pin = 1;
	
	/* *********** */
	/* US SENSOR 2 */
	/* *********** */
	/* set ADC to PORTA */
	usRV.adc = adca_read;
	/* set PIN to 2 */
	usRV.pin = 2;

	/* *********** */
	/* US SENSOR 3 */
	/* *********** */
	/* set ADC to PORTA */
	usRH.adc = adca_read;
	/* set PIN to 3 */
	usRH.pin = 3;
}


/* ************************************************************** */
/*! \brief read function for U300D50 ultrasonic sensor.
 *
 *  reads the U300D50 sensor [cm].
 *
 *  \param sensor pointer to U300D50 sensor.
 *  
*/
/* ************************************************************** */
void processU300D50(U300D50_t *sensor)
{
	/* read sensor value and recalculate distance value (1 ... 50 cm) */
	sensor->distance = (int16_t)((sensor->adc(ADC_CH_0, sensor->pin)) / 80 + 1);
 	/* limit distance value to 0 cm */
	sensor->distance = ((sensor->distance < 0) ? 0 : sensor->distance); 
	/* limit distance value to 50 cm */
 	sensor->distance = ((sensor->distance > 50) ? 50 : sensor->distance);
}