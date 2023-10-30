
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
#include "servo.h"
#include "rrt_serialconfig.h"
#include "dynamixel.h"
#include "motor.h"
#include "liftMotor.h"
#include "Arm.h"


/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitReceiveData(void)
{
	Port_App_allocation(SERVO_PORTNBR, RRTLAN_SERVO_TASKNBR);
	Port_App_allocation(DIGIOUT_PORTNBR, RRTLAN_DIGIOUT_TASKNBR);
	Port_App_allocation(ENEMYDATA_RAW_FROM_WIFI_PORTNBR, RRTLAN_POSENEMY_TASKNBR);
	Port_App_allocation(LIFT_VEL_PORTNBR, RRTLAN_LIFT_VEL_TASKNBR);
	Port_App_allocation(START_LIFT_INIT_PORTNBR, RRTLAN_LIFT_INIT_TASKNBR);
	Port_App_allocation(MOT_POS_PORTNBR, RRTLAN_MOT_POS_TASKNBR);

	SET_TASK_HANDLE(RRTLAN_SERVO_TASKNBR, rrtlanServo_Task);
	SET_TASK_HANDLE(RRTLAN_DIGIOUT_TASKNBR, rrtlanDigiOut_Task);
	SET_TASK_HANDLE(RRTLAN_POSENEMY_TASKNBR, rrtlanEnemyDataRaw_from_WIFI_Task);
	SET_TASK_HANDLE(RRTLAN_LIFT_VEL_TASKNBR, rrtlanLift_Task);
	SET_TASK_HANDLE(RRTLAN_LIFT_INIT_TASKNBR, rrtlanInitLift_Task);
	SET_TASK_HANDLE(RRTLAN_MOT_POS_TASKNBR, rrtlanMotorPosition_Task);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanInitLift_Task                          ***
***   FUNCTION:            Start Lift Init						        ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t rrtlanInitLift_Task()
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	
	if(Received_AppData_Available(&MCU1, START_LIFT_INIT_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, START_LIFT_INIT_PORTNBR, receiveArray);
		
		WaitForInit = receiveArray[0];
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanLift_Task                             ***
***   FUNCTION:            empfängt Daten von uC1 -> Motordaten         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/

uint8_t rrtlanLift_Task()
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	float velocity;
	char text[100];
	
	convData_t d;
	
	if(Received_AppData_Available(&MCU1, LIFT_VEL_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, LIFT_VEL_PORTNBR, receiveArray);
		
		sprintf(text,"%d\n", receiveArray[0]);
		debugMsg(text);
		
		d.uint8[0] = receiveArray[2];
		d.uint8[1] = receiveArray[1];
		velocity = ((float)d.int16[0])/1000.0;

		// *************************
		// Liftmotor hinten links
		// *************************
		if(receiveArray[0] == LIFT_REAR_LEFT_NBR)
		{
			setVelocity(&liftHL, velocity);
		}
		
		// *************************
		// Liftmotor hinten rechts
		// *************************
		else if(receiveArray[0] == LIFT_REAR_RIGHT_NBR)
		{
			setVelocity(&liftHR, velocity);
		}
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanServo_Task                             ***
***   FUNCTION:            empfängt Daten von uC1 -> Servodaten         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t rrtlanServo_Task()
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	uint16_t speed = 180;
	int16_t angle;
	char text[100];
	
	convData_t d;
	
	if(Received_AppData_Available(&MCU1, SERVO_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, SERVO_PORTNBR, receiveArray);
		
		TOGGLE_PIN(LED_PORT, LED2);
		
		sprintf(text,"%d\n", receiveArray[0]);
		debugMsg(text);
		
		d.uint8[0] = receiveArray[1];
		d.uint8[1] = receiveArray[2];
		angle = d.int16[0];
		//speed = (uint16_t)(receiveArray[2]) * 256 + (uint16_t) (receiveArray[3]);

		// *************************
		// Servo -> Dynamixl 2
		// *************************
		if(receiveArray[0] == SE_ARM_FRONTLEFT_NBR)
		{
			AX_setTorqueControl(&(ax_servo[1]),1);
			AX_setPositionSpeed_direct(&(ax_servo[1]), DEG2RAD(angle),DEG2RAD(speed));
		}
		
		// *************************
		// Servo -> Dynamixl 1
		// *************************
		else if(receiveArray[0] == SE_ARM_FRONTRIGHT_NBR)
		{
			AX_setTorqueControl(&(ax_servo[0]),1);
			AX_setPositionSpeed_direct(&(ax_servo[0]), DEG2RAD(angle),DEG2RAD(speed));
		}
		
		// *************************
		// Servo -> Dynamixl 4
		// *************************
		else if(receiveArray[0] == SE_ARM_REARLEFT_NBR)
		{
			AX_setTorqueControl(&(ax_servo[3]),1);
			AX_setPositionSpeed_direct(&(ax_servo[3]), DEG2RAD(angle),DEG2RAD(speed));
		}

		// *************************
		// Servo -> Dynamixl 3
		// *************************
		else if(receiveArray[0] == SE_ARM_REARRIGHT_NBR)
		{
			AX_setTorqueControl(&(ax_servo[2]),1);
			AX_setPositionSpeed_direct(&(ax_servo[2]), DEG2RAD(angle),DEG2RAD(speed));
		}

// 		// *************************
// 		// Servo -> Pince Nez 5
// 		// *************************
// 		else if(receiveArray[0] == SE_PINCE_NEZ5_NBR)
// 		{
// 			se_pince_nez5.Phi = (float) receiveArray[1];
// 		}
		
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanDigiOut_Task                           ***
***   FUNCTION:            empfängt Daten von uC1 -> digitial out       ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t rrtlanDigiOut_Task()
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	
	if(Received_AppData_Available(&MCU1, DIGIOUT_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, DIGIOUT_PORTNBR, receiveArray);
		
		// *************************
		// LED1
		// *************************
		if(receiveArray[0] == LED1_DIGIOUT)
		{
			if (receiveArray[1] == 1)
			{
				SET_PIN(LED_PORT, LED1);
			}
			else
			{
				CLR_PIN(LED_PORT, LED1);
			}
		}
		
		// *************************
		// LED2
		// *************************
		if(receiveArray[0] == LED2_DIGIOUT)
		{
			if (receiveArray[1] == 1)
			{
				SET_PIN(LED_PORT, LED2);
			}
			else
			{
				CLR_PIN(LED_PORT, LED2);
			}
		}
		
		// *************************
		// LED3
		// *************************
		if(receiveArray[0] == LED3_DIGIOUT)
		{
			if (receiveArray[1] == 1)
			{
				SET_PIN(LED_PORT, LED3);
			}
			else
			{
				CLR_PIN(LED_PORT, LED3);
			}
		}
		
		// *************************
		// Pumpe Links
		// *************************
		if(receiveArray[0] == VACUUM_LEFT)
		{
			if (receiveArray[1] == 1)
			{
				SET_VACUUM_PUMP_LEFT;
				SET_MV_LEFT;
			}
			else
			{
				CLR_VACUUM_PUMP_LEFT;
				CLR_MV_LEFT;
			}
		}
		
		// *************************
		// Pumpe Rechts
		// *************************
		if(receiveArray[0] == VACUUM_RIGHT)
		{
			if (receiveArray[1] == 1)
			{
				SET_VACUUM_PUMP_RIGHT;
				SET_MV_RIGHT;
			}
			else
			{
				CLR_VACUUM_PUMP_RIGHT;
				CLR_MV_RIGHT;
			}
		}
		
		// *************************
		// MV Links
		// *************************
		if(receiveArray[0] == MV_LEFT)
		{
			if (receiveArray[1] == 1)
			{
				SET_MV_LEFT;
			}
			else
			{
				CLR_MV_LEFT;
			}
		}
		
		// *************************
		// MV Rechts
		// *************************
		if(receiveArray[0] == MV_RIGHT)
		{
			if (receiveArray[1] == 1)
			{
				SET_MV_RIGHT;
			}
			else
			{
				CLR_MV_RIGHT;
			}
		}
	}
	
	return(DISABLE);
}



/**************************************************************************
***   FUNCTIONNAME:        rrtlanEnemyDataRaw_Task						***
***   FUNCTION:            empfängt Daten von uC1 -> für Gegnererkennung ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t rrtlanEnemyDataRaw_from_WIFI_Task()
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[50];
	volatile uint16_t x,y,phi;
	
	if(Received_AppData_Available(&MCU1, ENEMYDATA_RAW_FROM_WIFI_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, ENEMYDATA_RAW_FROM_WIFI_PORTNBR, receiveArray);


		// Eigene Position des Roboters
		x = ((uint16_t)receiveArray[0]) * 256 + ((uint16_t)receiveArray[1]);
		y = ((uint16_t)receiveArray[2]) * 256 + ((uint16_t)receiveArray[3]);
		phi = ((uint16_t)receiveArray[4]) * 256 + ((uint16_t)receiveArray[5]);
		
		
		Robx = x;
		Roby = y;
		Roba = phi;
		detectionotherrobot.count = receiveArray[6];
		
		for (uint8_t i = 0 ; i < 3; i++)
		{
			detectionotherrobot.center[i][0] = ((uint16_t)receiveArray[7+i*6]) * 256 + ((uint16_t)receiveArray[8+i*6]);
			detectionotherrobot.center[i][1] = ((uint16_t)receiveArray[9+i*6]) * 256 + ((uint16_t)receiveArray[10+i*6]);
			
			detectionotherrobot.variance[i] = ((uint16_t)receiveArray[11+i*6]) * 256 + ((uint16_t)receiveArray[12+i*6]);
		}
	}
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanDigiOut_Task                           ***
***   FUNCTION:            empfängt Daten von uC1 -> Digital Output     ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanMotorPosition_Task(void)
{
	char text[100];
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	float Vmax, S0;
	int16_t Vmax_i16, S0_i16;
	convData_t d;
	uint8_t nbr;
	
	if(Received_AppData_Available(&MCU1, MOT_POS_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, MOT_POS_PORTNBR, receiveArray);
		
		nbr = receiveArray[4];
		d.uint8[0] = receiveArray[1];
		d.uint8[1] = receiveArray[0];
		Vmax_i16 = d.int16[0];

		d.uint8[0] = receiveArray[3];
		d.uint8[1] = receiveArray[2];
		S0_i16 = d.int16[0];
		
		Vmax = ((float)(Vmax_i16)) / 1000.0;
		S0 = ((float)(S0_i16)) / 1000.0;
		
		if(nbr == LIFT_REAR_RIGHT_NBR)
		{
			setMotion(&liftHR, Vmax, S0);
		}
		else if(nbr == LIFT_REAR_LEFT_NBR)
		{
			setMotion(&liftHL, Vmax, S0);
		}		
	}
	
	return(DISABLE);
}