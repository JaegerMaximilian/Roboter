/*
 * UNDK20.c
 *
 * Created: 17.12.2019 15:07:43
 *  Author: P20087
 */ 

#define _UNDK20_EXTERN

#include <avr/io.h>
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "adc_driver.h"
#include "UNDK20.h"


/* ************************************************************** */
/*! \brief initialize function for UNDK20 pressure sensor.
 *
 *  initialize the UNDK20 sensor (ADC, pin, ...).
 *  
*/
/* ************************************************************** */
void UNDK20_init()
{
	/* *********** */
	/* US SENSOR 4 */
	/* *********** */
	/* set ADC to PORTA */
	usVL.adc = adca_read;
	/* set PIN to 4 */
	usVL.pin = 4;
	
	/* *********** */
	/* US SENSOR 5 */
	/* *********** */
	/* set ADC to PORTA */
	usVR.adc = adca_read;
	/* set PIN to 5 */
	usVR.pin = 5;
	
	/* *********** */
	/* US SENSOR 6 */
	/* *********** */
	/* set ADC to PORTA */
	usHL.adc = adca_read;
	/* set PIN to 6 */
	usHL.pin = 6;

	/* *********** */
	/* US SENSOR 7 */
	/* *********** */
	/* set ADC to PORTA */
	usHR.adc = adca_read;
	/* set PIN to 7 */
	usHR.pin = 7;
}


/* ************************************************************** */
/*! \brief read function for UNDK20 ultrasonic sensor.
 *
 *  reads the UNDK20 sensor [cm].
 *
 *  \param sensor pointer to UNDK20 sensor.
 *  
*/
/* ************************************************************** */
void processUNDK20(UNDK20_t *sensor)
{
	/* read sensor value and recalculate distance value (0 ... 100 cm) */
	sensor->distance = (int8_t)((sensor->adc(ADC_CH_0, sensor->pin)) / 42 + 9);
	/* limit distance value to 0 cm */
	sensor->distance = ((sensor->distance < 0) ? 0 : sensor->distance); 
	/* limit distance value to 100 cm */
	sensor->distance = ((sensor->distance > 100) ? 100 : sensor->distance);
}