/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      SENSOR.c
Version :  V 1.0
Date    :  13.12.2010
Author  :  ZAUNER MICHAEL

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : XMega256A3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024                

               Copyright (c) 2010 by FH-Wels                          
                   All Rights Reserved.
****************************************************************/

#define _SENSOR_EXTERN


#include <avr/io.h>   
#include "multitask.h"
#include "sensor.h"
#include "ports.h"
#include "define.h" 
#include "rrt_usart_driver.h"
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "global.h"
#include "timer.h"
#include "adc_driver.h"
#include "rrt_transmittingtask.h"
#include <util/delay.h>
#include "UNDK20.h"
#include "PSE541.h"
#include "adc_driver.h"
#include "motor.h"


/**************************************************************************
***   FUNKTIONNAME: InitSensor                                          ***
***   FUNKTION: initialisiert die Sensorbehandlung                      ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitSensor(void)
{
   // zyklischer Task - Zykluszeit: 5 ms
   SET_CYCLE(SENSOR_TASKNBR, 50);
   SET_TASK(SENSOR_TASKNBR, CYCLE);
   SET_TASK_HANDLE(SENSOR_TASKNBR, SensorTask); 
}

/**************************************************************************
***   FUNCTIONNAME:        SensorTask                                   ***
***   FUNCTION:            Servoansteuerung                             ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char SensorTask(void)
{
	SET_CYCLE(SENSOR_TASKNBR, 50);
	SET_TASK(SENSOR_TASKNBR, CYCLE);
   
	/* ********************************** */
	/* process pressure sensors (in mbar) */
	/* ********************************** */
	processPSE541(&pressureRearLeft);
	processPSE541(&pressureRearRight);
	
	/* ************************************* */
	/* Resitance measurement galery */
	/* ************************************* */
	Gal_V = adca_read(ADC_CH_0, 3);
	Gal_H = adca_read(ADC_CH_0, 4);
	
	/* ************************************* */
	/* process ultrasonic measurment (in cm) */
	/* ************************************* */
// 	processUNDK20(&us1);
// 	processUNDK20(&us2);
// 	processUNDK20(&us3);
// 	processUNDK20(&us4);
// 	processUNDK20(&us5);
		
	
	/* ******************************* */
	/* digital sensors PORTB and PORTC */
	/* ******************************* */
 	uC3sensorenPortF = PORTF.IN;
// 	uC3sensorenPortC = PORTC.IN;
// 		
 	sendSensors(pressureRearLeft.pressure, pressureRearRight.pressure, liftHL.Odo.Dis, liftHR.Odo.Dis, Gal_H, Gal_V);
		
	return(CYCLE);
}

