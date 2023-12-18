
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
void sendSensors(uint8_t posRear)
{
	uint8_t sendArray[1];
	
	sendArray[0] = posRear;
	
	Send_Application_Data(&MCU1, SENSOR_PORTNBR, sendArray, 1);
}





/**************************************************************************
***   FUNCTIONNAME:        posAntrieb                                   ***
***   FUNCTION:            gibt die Position des Roboters aus           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   xPos ... X-Position in 0,1 mm                ***
***                        yPos ... Y-Position in 0,1 mm                ***
***                        phi ... Winkel in 0,1 Grad                   ***
***                        v ... Geschwindigkeit in mm/s                ***
**************************************************************************/
void posAntrieb(int16_t xPos, int16_t yPos, int16_t phi, int16_t v)
{
	uint8_t sendArray[10];
	convData_t d;
	
	sendArray[0] = (uint8_t)(xPos >> 8);
	sendArray[1] = (uint8_t)(xPos);
	sendArray[2] = (uint8_t)(yPos >> 8);
	sendArray[3] = (uint8_t)(yPos);
	sendArray[4] = (uint8_t)(phi >> 8);
	sendArray[5] = (uint8_t)(phi);
	d.int16[0] = v;
	sendArray[6] = d.uint8[1];
	sendArray[7] = d.uint8[0];
	
	Send_Application_Data(&MCU1, POS_PORTNBR, sendArray, 8);
}

/**************************************************************************
***   FUNCTIONNAME:        statusAntrieb                                ***
***   FUNCTION:            Setzt Antrieb Status                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   status ... Status                            ***
**************************************************************************/
void statusAntrieb(uint8_t status)
{
	uint8_t sendArray;
	
	sendArray = status;
		
	Send_Application_Data(&MCU1, ANTRIEB_PORTNBR, &sendArray, 1);
}



