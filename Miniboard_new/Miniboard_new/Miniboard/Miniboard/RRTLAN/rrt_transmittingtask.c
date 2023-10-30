
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
#include "motor.h"


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
***   FUNCTIONNAME:        MotorStatus                                  ***
***   FUNCTION:            send veleocity and position                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   controller ... dedicated controller          ***
***                        vel ... velocity [m/s]                       ***
***                        pos ... position [m]                         ***
**************************************************************************/
void MotorStatus(USART_data_t *controller, float vel, float pos)
{
	uint8_t sendArray[6];
	int16_t vel_i16;
	int32_t pos_i32;
	
	vel_i16 = (int16_t)(vel * 1000.0);
	pos_i32 = (int32_t)(pos * 1000.0);
	
	sendArray[0] = (uint8_t)((uint16_t)(vel_i16) >> 8);
	sendArray[1] = (uint8_t)((uint16_t)(vel_i16));
	sendArray[2] = (uint8_t)((uint32_t)(pos_i32) >> 24);
	sendArray[3] = (uint8_t)((uint32_t)(pos_i32) >> 16);
	sendArray[4] = (uint8_t)((uint32_t)(pos_i32) >> 8);
	sendArray[5] = (uint8_t)((uint32_t)(pos_i32));
	
	Send_Application_Data(controller, MOTOR_STATUS_MSG_PORT, sendArray, 6);
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
	
	Send_Application_Data(&MCU1, MOTOR_DEBUG_MSG_PORT, sendArray, i);
}



/**************************************************************************
***   FUNCTIONNAME:        TestSendeTasks                               ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char TransmitTask1(void)
{ 
   SET_CYCLE(TRANSMITTING_TASKNBR_1,50);  

	MotorStatus(&MCU1, motor.Odo.Vel, motor.Odo.Dis);
   
   return(CYCLE);                          
}










unsigned char TransmitTask2(void)
{ 
   uint8_t i;
   uint8_t TransmitArray[5];
   
   for(i = 0; i < 5; i++)
    {
      TransmitArray[i] = 2;
    }
   
   Send_Application_Data(&USART_data_C0, VEL_MSG_PORT, &TransmitArray[0], 5); 
   
   SET_CYCLE(TRANSMITTING_TASKNBR_2,25);
   
   return(CYCLE);                          
}

