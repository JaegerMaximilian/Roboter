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
#include "anaPos.h"
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
   
	
	/* *********************************** */
	/* process position measurement (in m) */
	/* *********************************** */
	//processAnaPos(&posRear);	
	
	/* ********************* */
	/* digital sensors PORTB */
	/* ********************* */
	//uC2sensorenPortF= (PORTF.IN & 0xF0);
	sendSensors((uint8_t)(liftRear.Odo.Dis * 1000.0));
	
		
	return(CYCLE);
}

