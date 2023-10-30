
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
#include "obstacle.h"



/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitTransmit(void)
{
	SET_CYCLE(TRANSMITTING_TASKNBR_1, 50);
	SET_TASK(TRANSMITTING_TASKNBR_1, CYCLE);
	SET_TASK_HANDLE(TRANSMITTING_TASKNBR_1, TransmitTask1);
}

/**************************************************************************
***   FUNCTIONNAME:        debugMsg                                     ***
***   FUNCTION:            sends debug message                          ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   text ... nullterminated string               ***
**************************************************************************/
void enemyPosMsg()
{
	char sendArray[100];
	uint8_t j = 1;
	
	for (uint8_t i = 0; i < OBSTACLE_LIST_LENGTH; i++)
	{
		if (obstacleList[i].nbrObstacle == 1)
		{
			sendArray[j++] = (uint8_t)obstacleList[i].centerX[0];
			sendArray[j++] = (uint8_t)(obstacleList[i].centerX[0] / 256);
			sendArray[j++] = (uint8_t)obstacleList[i].centerY[0];
			sendArray[j++] = (uint8_t)(obstacleList[i].centerY[0] / 256);
		}
	}
	
	sendArray[0] = (j-1) / 4;
	
	if (j > 1)
	{
		Send_Application_Data(&MCU1, POS_MSG_GEGNER_PORTNBR, sendArray, j);
	}
	

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
	
	Send_Application_Data(&MCU1, DEBUG_MSG_GEGNER_PORTNBR, sendArray, i);
}


/* ************************************************************** */
/*! \brief TransmitTask1-task.
*
*  Here the data is sent to the master
*
*  \version 17.03.2020
*
*/
/* ************************************************************** */
uint8_t TransmitTask1(void)
{
	SET_CYCLE(TRANSMITTING_TASKNBR_1, 50);
	
	
	return (CYCLE);
}






