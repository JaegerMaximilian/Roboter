
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
#include "motor.h"
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "LEDcontrol.h"
#include "usart.h"
#include <avr/eeprom.h>


/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitReceiveData(void)
{
   SET_CYCLE(VEL_MSG_TASKNBR, 10);
   SET_TASK_HANDLE(VEL_MSG_TASKNBR, VelMsgTask); 
   
   SET_CYCLE(POS_MSG_TASKNBR, 10);
   SET_TASK_HANDLE(POS_MSG_TASKNBR, PosMsgTask); 

   SET_CYCLE(PARAMETER_MSG_TASKNBR, 10);
   SET_TASK_HANDLE(PARAMETER_MSG_TASKNBR, ParameterMsgTask);
   
   Port_App_allocation(VEL_MSG_PORT, VEL_MSG_TASKNBR);
   Port_App_allocation(POS_MSG_PORT, POS_MSG_TASKNBR);
   Port_App_allocation(SET_MOTOR_PARAMETER_MSG_PORT, PARAMETER_MSG_TASKNBR);
}



/**************************************************************************
***   FUNCTIONNAME:        VelMsgTask                                   ***
***   FUNCTION:            receive velocity command [in mm/s]           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t VelMsgTask(void)
{ 
	uint8_t nbr_of_bytes = 0;
	uint8_t i, text[100];
	uint8_t receiveArray[10]; 
	float vel;   
	int16_t vel_int;
	
	
	if(Received_AppData_Available(&USART_data_C0, VEL_MSG_PORT, &nbr_of_bytes))
	{   
		Receive_Application_Data(&USART_data_C0 , VEL_MSG_PORT, receiveArray);
		
		// velocity control -> on
		motor.Vel.controlOn = 1;
		// velocity control -> off
		motor.Pos.controlOn = 0;
		
		vel = (float)((int16_t)((uint16_t)(receiveArray[0]) * 256 + (uint16_t)(receiveArray[1]))) / 1000.0;
		
		setVelocity(&motor, vel);

	}
    
	return(DISABLE);                          
}



/**************************************************************************
***   FUNCTIONNAME:        PosMsgTask                                   ***
***   FUNCTION:            receive motion command [in mm/s; mm]         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t PosMsgTask(void)
{ 
	uint8_t nbr_of_bytes = 0;// text[40];
	uint8_t i;
	uint8_t receiveArray[5];  
	float vel, s0;  

	if(Received_AppData_Available(&USART_data_C0, POS_MSG_PORT, &nbr_of_bytes))
	{
		Receive_Application_Data(&USART_data_C0 , POS_MSG_PORT, receiveArray);

		// velocity control -> on
		motor.Vel.controlOn = 1;
		// velocity control -> on
		motor.Pos.controlOn = 1;

		vel = (float)((int16_t)((uint16_t)(receiveArray[0]) * 256 + (uint16_t)(receiveArray[1]))) / 1000.0;
		s0 = (float)((int16_t)((uint16_t)(receiveArray[2]) * 256 + (uint16_t)(receiveArray[3]))) / 1000.0;
		
 		//sprintf(text,"v:%4.3f  s:%4.3f \r", vel, s0);
 		////		sprintf(text,"%4.3f;%4.3f \r", motor.Traj.Xs, motor.Odo.Dis);
 		////sprintf(text,"%2.1f \r", motor.Vel.Y);
 		//writeString_usart(&usartC1, text);
		//
		setMotion(&motor, vel, s0);
		TOGGLE_LED3;	
	}
  
	return(DISABLE);                                
}


/**************************************************************************
***   FUNCTIONNAME:        ParameterMsgTask                             ***
***   FUNCTION:            receive set parameter command                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t ParameterMsgTask(void)
{
	uint8_t nbr_of_bytes = 0;// text[40];
	uint8_t i;
	uint8_t receiveArray[12];
	float a_us, a_ds, vel_kp, vel_ki, pos_kp, pos_ki;

	if(Received_AppData_Available(&USART_data_C0, SET_MOTOR_PARAMETER_MSG_PORT, &nbr_of_bytes))
	{
		Receive_Application_Data(&USART_data_C0 , SET_MOTOR_PARAMETER_MSG_PORT, receiveArray);

		/* read out data */
		a_us = (float)((int16_t)((uint16_t)(receiveArray[0]) * 256 + (uint16_t)(receiveArray[1]))) / 10.0;
		a_ds = (float)((int16_t)((uint16_t)(receiveArray[2]) * 256 + (uint16_t)(receiveArray[3]))) / 10.0;
		vel_kp = (float)((int16_t)((uint16_t)(receiveArray[4]) * 256 + (uint16_t)(receiveArray[5]))) / 10.0;
		vel_ki = (float)((int16_t)((uint16_t)(receiveArray[6]) * 256 + (uint16_t)(receiveArray[7]))) / 10.0;
		pos_kp = (float)((int16_t)((uint16_t)(receiveArray[8]) * 256 + (uint16_t)(receiveArray[9]))) / 10.0;
		pos_ki = (float)((int16_t)((uint16_t)(receiveArray[10]) * 256 + (uint16_t)(receiveArray[11]))) / 10.0;

		/* set data to RAM */
		motor.Traj.AmaxUs = a_us;										
		motor.Traj.AmaxDs = a_ds;
		motor.Vel.Kp = vel_kp;
		motor.Vel.Ki = vel_ki;										
		motor.Pos.Kp = pos_kp;
		motor.Pos.Ki = pos_ki;

		/* store data to EEPROM */
		eeprom_write_float(&AmaxUs_EEPROM, a_us);
		eeprom_write_float(&AmaxDs_EEPROM, a_ds);
		eeprom_write_float(&VelKp_EEPROM, vel_kp);
		eeprom_write_float(&VelKi_EEPROM, vel_ki);
		eeprom_write_float(&PosKp_EEPROM, pos_kp);
		eeprom_write_float(&PosKi_EEPROM, pos_ki);

		
		
	}
	
	return(DISABLE);
}
