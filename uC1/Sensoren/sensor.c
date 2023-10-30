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
#include "PSE541.h"
#include "anaPos.h"
#include "UNDK20.h"
#include "U300D50.h"

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
	SET_CYCLE(SENSOR_TASKNBR, 10);
	SET_TASK(SENSOR_TASKNBR, CYCLE);
   
	/* ********************************** */
	/* process pressure sensors (in mbar) */
	/* ********************************** */
// 	processPSE541(&pressureFrontLeft); 
// 	processPSE541(&pressureFrontRight);
	
	/* *********************************** */
	/* process position measurement (in mm) */
	/* *********************************** */
// 	processUNDK20(&usVL);
// 	usFrontLeft = (uint16_t)(usVL.distance)*10;
// 	processUNDK20(&usVR);
// 	usFrontRight = (uint16_t)(usVR.distance)*10;
// 	processUNDK20(&usHL);
// 	usRearLeft = (uint16_t)(usHL.distance)*10;
// 	processUNDK20(&usHR);	
// 	usRearRight = (uint16_t)(usHR.distance)*10;
// 	
// 	processU300D50(&usLV);	
// 	usLeftFront = (uint16_t)(usLV.distance)*10;
// 	processU300D50(&usRV);
// 	usRightFront = (uint16_t)(usRV.distance)*10;
// 	processU300D50(&usLH);
// 	usLeftRear = (uint16_t)(usLH.distance)*10;
// 	processU300D50(&usRH);
// 	usRightRear = (uint16_t)(usRH.distance)*10;	
	
	/* ******************************* */
	/* digital sensors PORTA and PORTF */
	/* ******************************* */
	uC1sensorenPortA = PORTA.IN;
	//uC1sensorenPortF = PORTF.IN;	
	
	
	irVL1 = IR_VL1;
	irVL2 = IR_VL2;
	irVL3 = IR_VL3;
	irVR1 = IR_VR1;
	irVR2 = IR_VR2;
	irVR3 = IR_VR3;
	refVL = END_VL;
	refVR = END_VR;
		
	return(CYCLE);
}

