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
	/* US SENSOR 1 */
	/* *********** */
	/* set ADC to PORTA */
	us1.adc = adca_read;
	/* set PIN to 0 */
	us1.pin = 0;
	
	/* *********** */
	/* US SENSOR 2 */
	/* *********** */
	/* set ADC to PORTA */
	us2.adc = adca_read;
	/* set PIN to 1 */
	us2.pin = 1;
	
	/* *********** */
	/* US SENSOR 3 */
	/* *********** */
	/* set ADC to PORTA */
	us3.adc = adca_read;
	/* set PIN to 1 */
	us3.pin = 2;

	/* *********** */
	/* US SENSOR 4 */
	/* *********** */
	/* set ADC to PORTA */
	us4.adc = adca_read;
	/* set PIN to 3 */
	us4.pin = 3;

	/* *********** */
	/* US SENSOR 5 */
	/* *********** */
	/* set ADC to PORTA */
	us5.adc = adca_read;
	/* set PIN to 1 */
	us5.pin = 4;

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
	/* read sensor value and recalculate pressure value (0 ... 1000 mbar) */
	sensor->distance = (int8_t)((sensor->adc(ADC_CH_0, sensor->pin)) / 42 + 9);
	/* limit pressure value to 0 cm */
	sensor->distance = ((sensor->distance < 0) ? 0 : sensor->distance); 
	/* limit pressure value to 100 cm */
	sensor->distance = ((sensor->distance > 100) ? 100 : sensor->distance);
}