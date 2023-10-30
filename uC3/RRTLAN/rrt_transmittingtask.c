
/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      DEBUG.c
Version :  V 1.0
Date    :  28.02.2011
Author  :  MUCKENHUMER BERNHARD

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : ATXmega256a3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024                

               Copyright (c) 2008 by FH-Wels                       	
                   All Rights Reserved.
****************************************************************/

#define RRTLAN_TRANSMITTINGTASK_EXTERN

#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "multitask.h"
#include "rrt_transmittingtask.h"
#include "rrt_serialconfig.h"
#include "rrt_applicationLayer.h"
#include "ports.h"
#include "define.h"
#include "global.h"
#include "sensor.h"

/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitTransmit(void)
{
   //SET_CYCLE(TRANSMITTING_TASKNBR_1, 50);
   //SET_TASK(TRANSMITTING_TASKNBR_1, CYCLE);
   //SET_TASK_HANDLE(TRANSMITTING_TASKNBR_1, TransmitTask1); 
   //
   //SET_CYCLE(TRANSMITTING_TASKNBR_2, 10);
   //SET_TASK(TRANSMITTING_TASKNBR_2, CYCLE);
   //SET_TASK_HANDLE(TRANSMITTING_TASKNBR_2, TransmitTask2); 
}

/**************************************************************************
***   FUNCTIONNAME:        debugMsg                                     ***
***   FUNCTION:            sends debug message                          ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   text ... nullterminated string               ***
**************************************************************************/
void debugMsg(char *text)
{
	char sendArray[200];
	uint8_t i = 0;
	
	do
	{
		sendArray[i] = text[i];
	}
	while(text[i++] != 0);
	
	Send_Application_Data(&MCU1, DEBUG_MSG_PORTNBR, sendArray, i);
}

/**************************************************************************
***   FUNCTIONNAME:        sendSensors                                  ***
***   FUNCTION:            gibt Sensoren aus                            ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   status ... Status                            ***
**************************************************************************/
void sendSensors(uint16_t pressureleft, uint16_t pressureright, float posleft, float posright, uint16_t galh, uint16_t galv)
{
	uint8_t sendArray[20];
	convData_t d;
	
	d.uint16[0] = pressureleft;
	sendArray[0] = d.uint8[0];
	sendArray[1] = d.uint8[1];
	
	d.uint16[0] = pressureright;
	sendArray[2] = d.uint8[0];
	sendArray[3] = d.uint8[1];
	
	d.f = posleft;
	sendArray[4] = d.uint8[0];
	sendArray[5] = d.uint8[1];
	sendArray[6] = d.uint8[2];
	sendArray[7] = d.uint8[3];
	
	d.f = posright;
	sendArray[8] = d.uint8[0];
	sendArray[9] = d.uint8[1];
	sendArray[10] = d.uint8[2];
	sendArray[11] = d.uint8[3];
	
	d.uint16[0] = galh;
	sendArray[12] = d.uint8[0];
	sendArray[13] = d.uint8[1];
	
	d.uint16[0] = galv;
	sendArray[14] = d.uint8[0];
	sendArray[15] = d.uint8[1];
	
	Send_Application_Data(&MCU1, SENSOR_PORTNBR, sendArray, 16);
}

/**************************************************************************
***   FUNCTIONNAME:        sendPosDataToEnemyDetect_RRLAN                                  ***
***   FUNCTION:            Sendet dem uC1 die Pos Daten                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   x0 ... X-Position (in mm)                    ***
***                        y0 ... Y-Position (in mm)                    ***
***                        phi0 ... Winkel (in °)                       ***
***                        enemy ... Daten der Gegner		            ***
**************************************************************************/
void sendEnemyData_to_Pathplaner_RRTLAN()
{
	uint8_t sendArray[50];

	sendArray[0] = (uint8_t)objectcenter.count;
	
	
	for (uint8_t i = 0 ; i < 3; i++)
	{
		sendArray[1+(i*4)] = (uint8_t)(objectcenter.center[i][0] >> 8);
		sendArray[2+(i*4)] = (uint8_t)(objectcenter.center[i][0]);

		sendArray[3+(i*4)] = (uint8_t)(objectcenter.center[i][1] >> 8);
		sendArray[4+(i*4)] = (uint8_t)(objectcenter.center[i][1]);
	}
	
	
	Send_Application_Data(&MCU1, ENEMYDATA_TO_PATHPLANER_PORTNBR, sendArray, 21);
}


/**************************************************************************
***   FUNCTIONNAME:        sendPosDataToEnemyDetect_RRLAN                                  ***
***   FUNCTION:            Sendet dem uC1 die Pos Daten                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   x0 ... X-Position (in mm)                    ***
***                        y0 ... Y-Position (in mm)                    ***
***                        phi0 ... Winkel (in °)                       ***
***                        enemy ... Daten der Gegner		            ***
**************************************************************************/
void sendEnemyDataRaw_to_WIFI_RRTLAN()
{
	uint8_t sendArray[50];

	sendArray[0] = (uint8_t)outputdetection.count;
	
	
	for (uint8_t i = 0 ; i < 3; i++)
	{
		sendArray[1+(i*6)] = (uint8_t)(outputdetection.center[i][0] >> 8);
		sendArray[2+(i*6)] = (uint8_t)(outputdetection.center[i][0]);

		sendArray[3+(i*6)] = (uint8_t)(outputdetection.center[i][1] >> 8);
		sendArray[4+(i*6)] = (uint8_t)(outputdetection.center[i][1]);
		
		sendArray[5+(i*6)] = (uint8_t)(outputdetection.variance[i] >> 8);
		sendArray[6+(i*6)] = (uint8_t)(outputdetection.variance[i]);
	}
	
	
	Send_Application_Data(&MCU1, ENEMYDATA_RAW_TO_WIFI_PORTNBR, sendArray, 37);
}