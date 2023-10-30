
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

#define RRTLAN_RECEIVEDATA_EXTERN
 
#include "multitask.h"
#include "rrt_receivedata.h"
#include "rrt_serialconfig.h"
#include "rrt_applicationLayer.h"
#include "rrt_transportLayer.h"
#include "ports.h"
#include "global.h"
#include "define.h" 
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include <avr/eeprom.h>



/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitReceiveData(void)
{
 	Port_App_allocation(ROBO_POS_MSG_GEGNER_PORTNBR, RRTLAN_ROBO_POS_TASKNBR);
  
  SET_TASK_HANDLE(RRTLAN_ROBO_POS_TASKNBR, rrtlanRoboPos_Task);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanServo_Task                             ***
***   FUNCTION:            empfängt Daten von uC1 -> Servodaten         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanRoboPos_Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	uint16_t speed;
	
	convData_t d;
 	
  	if(Received_AppData_Available(&MCU1, ROBO_POS_MSG_GEGNER_PORTNBR, &nbr_of_bytes))
  	{
 	 	Receive_Application_Data(&MCU1, ROBO_POS_MSG_GEGNER_PORTNBR, receiveArray);
 		
 		/* set playing points */
		d.uint8[0] = receiveArray[0];
		d.uint8[1] = receiveArray[1];
 		xPos = d.int16[0];
		d.uint8[0] = receiveArray[2];
		d.uint8[1] = receiveArray[3];
		yPos = d.int16[0];
		d.uint8[0] = receiveArray[4];
		d.uint8[1] = receiveArray[5];
		phiPos = d.int16[0];
		
  	}
	
	return(DISABLE);
}





