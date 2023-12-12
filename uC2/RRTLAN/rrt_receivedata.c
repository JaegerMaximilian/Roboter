
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
#include "genTrajAntrieb.h"
#include "servo.h"
#include "antrieb.h"
#include "path_math.h"
#include "motor.h"

uint16_t msgAntriebCount = 0;
uint16_t msgAntriebCountClothoid = 0;

/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitReceiveData(void)
{
	Port_App_allocation(SERVO_PORTNBR, RRTLAN_SERVO_TASKNBR);
	Port_App_allocation(ANTRIEB_PORTNBR, RRTLAN_ANTRIEB_TASKNBR);
	Port_App_allocation(ANTRIEB_CLOTHOID_PORTNBR, RRTLAN_ANTRIEB_CLOTHOID_TASKNBR);
	Port_App_allocation(POS_PORTNBR, RRTLAN_POS_TASKNBR);
	Port_App_allocation(DIGIOUT_PORTNBR, RRTLAN_DIGOUT_TASKNBR);
	Port_App_allocation(MOT_POS_PORTNBR, RRTLAN_MOT_POS_TASKNBR);
	Port_App_allocation(SET_SCHLEPP_PORTNBR, RRTLAN_SET_SCHLEPPFEHLER_TASKNBR);
	Port_App_allocation(SET_ACC_PORTNBR, RRTLAN_SET_ACC_TASKNBR);

	SET_TASK_HANDLE(RRTLAN_SERVO_TASKNBR, rrtlanServo_Task);
	SET_TASK_HANDLE(RRTLAN_ANTRIEB_TASKNBR, rrtlanAntrieb_Task);
	SET_TASK_HANDLE(RRTLAN_ANTRIEB_CLOTHOID_TASKNBR, rrtlanAntrieb_Clothoid_Task);
	SET_TASK_HANDLE(RRTLAN_POS_TASKNBR, rrtlanPosition_Task);
	SET_TASK_HANDLE(RRTLAN_DIGOUT_TASKNBR, rrtlanDigiOut_Task);
	SET_TASK_HANDLE(RRTLAN_MOT_POS_TASKNBR, rrtlanMotorPosition_Task);
	SET_TASK_HANDLE(RRTLAN_SET_SCHLEPPFEHLER_TASKNBR, rrtlanSetSchleppfehler_Task);
	SET_TASK_HANDLE(RRTLAN_SET_ACC_TASKNBR, rrtlanSetACC_Task);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanServo_Task                             ***
***   FUNCTION:            empfängt Daten von uC1 -> Servodaten         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanServo_Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	uint16_t speed;
	
	if(Received_AppData_Available(&MCU1, SERVO_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, SERVO_PORTNBR, receiveArray);
		
		//		speed = (uint16_t)(receiveArray[2]) * 256 + (uint16_t) (receiveArray[3]);

		// *************************
		// Servo -> Arm
		// *************************
		if(receiveArray[0] == SE_PINCE_NEZ1_NBR)
		{
			se_pince_nez1.Phi = (float) receiveArray[1];
		}
		
		// *************************
		// Servo -> Flag 1
		// *************************
		else if(receiveArray[0] == SE_FLAG1_NBR)
		{
			se_flag1.Phi = (float) receiveArray[1];
		}
		
		// *************************
		// Servo -> Flag 2
		// *************************
		else if(receiveArray[0] == SE_FLAG2_NBR)
		{
			se_flag2.Phi = (float) receiveArray[1];
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
unsigned char rrtlanDigiOut_Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];
	uint16_t speed;
	
	if(Received_AppData_Available(&MCU1, DIGIOUT_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, DIGIOUT_PORTNBR, receiveArray);
		
		// *************************
		// LED1
		// *************************
		if(receiveArray[0] == DO_LED1_NBR)
		{
			if (receiveArray[1] == 1)
			{
				SET_PIN(DO_PORT, DO_LED1_NBR);
			}
			else
			{
				CLR_PIN(DO_PORT, DO_LED1_NBR);
			}
			
		}
		
		// *************************
		// LED2
		// *************************
		if(receiveArray[0] == DO_LED2_NBR)
		{
			if (receiveArray[1] == 1)
			{
				SET_PIN(DO_PORT, DO_LED2_NBR);
			}
			else
			{
				CLR_PIN(DO_PORT, DO_LED2_NBR);
			}
			
		}

		// *************************
		// LED3
		// *************************
		if(receiveArray[0] == DO_LED3_NBR)
		{
			if (receiveArray[1] == 1)
			{
				SET_PIN(DO_PORT, DO_LED3_NBR);
			}
			else
			{
				CLR_PIN(DO_PORT, DO_LED3_NBR);
			}
			
		}
		
		// *************************
		// POLWENDER
		// *************************
		if(receiveArray[0] == DO_POLWENDER_NBR)
		{
			if (receiveArray[1] == 1)
			{
				SET_PIN(DO_PORT, DO_POLWENDER_NBR);
			}
			else
			{
				CLR_PIN(DO_PORT, DO_POLWENDER_NBR);
			}
			
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
	
	if(Received_AppData_Available(&MCU1, MOT_POS_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, MOT_POS_PORTNBR, receiveArray);
		
		d.uint8[0] = receiveArray[1];
		d.uint8[1] = receiveArray[0];
		Vmax_i16 = d.int16[0];

		d.uint8[0] = receiveArray[3];
		d.uint8[1] = receiveArray[2];
		S0_i16 = d.int16[0];
		
		Vmax = ((float)(Vmax_i16)) / 1000.0;
		S0 = ((float)(S0_i16)) / 1000.0;

		setMotion(&liftRear, Vmax, S0);
		
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanAntrieb_Task                           ***
***   FUNCTION:            empfängt Daten von uC1 -> Antriebdaten       ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanAntrieb_Task(void)
{
	//char text[200];
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[18];
	int16_t vStart, vEnd, vMax, phiSoll;
	uint16_t rSoll, xSoll, ySoll, sSoll;
	uint8_t type, gegner;
	uint16_t count;

	if(Received_AppData_Available(&MCU1, ANTRIEB_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, ANTRIEB_PORTNBR, receiveArray);
		
		// *************************
		// Werte auslesen
		// *************************
		count = ((uint16_t)(receiveArray[18]) << 8) + (uint16_t)(receiveArray[19]);
		
		if(count > msgAntriebCount)
		{
			msgAntriebCount = count;
			
			vStart = (int16_t)(((uint16_t)(receiveArray[0]) << 8) + (uint16_t)(receiveArray[1]));
			vEnd = (int16_t)(((uint16_t)(receiveArray[2]) << 8) + (uint16_t)(receiveArray[3]));
			vMax = (int16_t)(((uint16_t)(receiveArray[4]) << 8) + (uint16_t)(receiveArray[5]));
			phiSoll = (int16_t)(((uint16_t)(receiveArray[6]) << 8) + (uint16_t)(receiveArray[7]));
			rSoll = ((uint16_t)(receiveArray[8]) << 8) + (uint16_t)(receiveArray[9]);
			xSoll = ((uint16_t)(receiveArray[10]) << 8) + (uint16_t)(receiveArray[11]);
			ySoll = ((uint16_t)(receiveArray[12]) << 8) + (uint16_t)(receiveArray[13]);
			sSoll = ((uint16_t)(receiveArray[14]) << 8) + (uint16_t)(receiveArray[15]);
			type = receiveArray[16];
			gegner = receiveArray[17];
			
			//sprintf(text,"S: type: %d v:%d\r", type, vMax);
			//debugMsg(text);
			
			// *************************
			// MOTION INTERRUPT
			// *************************
			if(type == _MOTION_INTERRUPT_)
			{
				motionIR_Received = 1;
				paramComAntrieb.ucState = _MOTION_INTERRUPT_;
			}

			// *************************
			// Bewegung setzen
			// *************************
			else
			{
				paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
				paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
				paramInputAntrieb[indexInputInAntrieb].fVstart = (float)(vStart) / 1000.0;
				paramInputAntrieb[indexInputInAntrieb].fVend = (float)(vEnd) / 1000.0;
				if((type == _TURN_REL) || (type == _TURN_ABS))
					paramInputAntrieb[indexInputInAntrieb].fVmax = (float)(vMax);
				else
					paramInputAntrieb[indexInputInAntrieb].fVmax = (float)(vMax) / 1000.0;
				paramInputAntrieb[indexInputInAntrieb].fPhiSoll = (float)(phiSoll);
				paramInputAntrieb[indexInputInAntrieb].fRsoll = (float)(rSoll) / 1000.0;
				paramInputAntrieb[indexInputInAntrieb].fXsoll = (float)(xSoll) / 1000.0;
				paramInputAntrieb[indexInputInAntrieb].fYsoll = (float)(ySoll) / 1000.0;
				paramInputAntrieb[indexInputInAntrieb].fSsoll = (float)(sSoll) / 1000.0;
				paramInputAntrieb[indexInputInAntrieb].ucType = type;
				paramInputAntrieb[indexInputInAntrieb].gegnerErkennung = gegner;
				
				indexInputInAntrieb++;
				if(indexInputInAntrieb >= MAX_IN_ANTRIEB)
				indexInputInAntrieb = 0;
				indexInputAntrieb++;
			}
		}
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanAntrieb_Clothoid_Task                  ***
***   FUNCTION:            empfängt Daten von uC1 -> Antriebdaten       ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanAntrieb_Clothoid_Task(void)
{
	//char text[200];
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[70];
	uint8_t i;
	int16_t destinationVmax;
	uint16_t count;
	
	float vectorA[2];
	float vectorB[2];
	float alpha;

	if(Received_AppData_Available(&MCU1, ANTRIEB_CLOTHOID_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, ANTRIEB_CLOTHOID_PORTNBR, receiveArray);
		
		// *************************
		// Werte auslesen
		// *************************
		count = ((uint16_t)(receiveArray[2]) << 8) + (uint16_t)(receiveArray[3]);
		
		if(count > msgAntriebCountClothoid)
		{
			msgAntriebCountClothoid = count;

			destinationVmax = (int16_t)(((uint16_t)(receiveArray[0]) << 8) + (uint16_t)(receiveArray[1]));
			paramInputAntrieb[indexInputInAntrieb].destinationPointsCount = (nbr_of_bytes - 2) / 4;
			
			// *************************
			// Bewegung setzen
			// *************************
			paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
			paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
			paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
			paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
			paramInputAntrieb[indexInputInAntrieb].fVmax = (float)(destinationVmax) / 1000.0;
			paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
			paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
			paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
			paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
			paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
			paramInputAntrieb[indexInputInAntrieb].ucType = _CLOTHOID;
			
			for (i = 0; i < paramInputAntrieb[indexInputInAntrieb].destinationPointsCount; i++)
			{
				trayPoints[i].x = (float)((int16_t)(((uint16_t)(receiveArray[(i*4)+4]) << 8) + (uint16_t)(receiveArray[(i*4)+5]))) / 1000.0;
				trayPoints[i].y = (float)((int16_t)(((uint16_t)(receiveArray[(i*4)+6]) << 8) + (uint16_t)(receiveArray[(i*4)+7]))) / 1000.0;
			}
			
			//sprintf(text,"I: (%1.3f-%1.3f)(%1.3f-%1.3f)(%1.3f-%1.3f)(%1.3f-%1.3f) %d\r\n",
			//trayPoints[0].x,
			//trayPoints[0].y,
			//trayPoints[1].x,
			//trayPoints[1].y,
			//trayPoints[2].x,
			//trayPoints[2].y,
			//trayPoints[3].x,
			//trayPoints[3].y,
			//paramInputAntrieb[indexInputInAntrieb].destinationPointsCount
			//);
			//debugMsg(text);
			
			if(paramInputAntrieb[indexInputInAntrieb].destinationPointsCount >= 2)
			{
				for (i = 0; i < (paramInputAntrieb[indexInputInAntrieb].destinationPointsCount - 1); i++)
				{
					if (i == 0)
					{
						vectorA[X_POS] = paramOutputAntrieb.fX - trayPoints[i].x;	// startPoint[X] - supportPoint[X];
						vectorA[Y_POS] = paramOutputAntrieb.fY - trayPoints[i].y;	// startPoint[Y] - supportPoint[Y];
						vectorB[X_POS] = trayPoints[i+1].x - trayPoints[i].x;		// finalPoint[X] - supportPoint[X];
						vectorB[Y_POS] = trayPoints[i+1].y - trayPoints[i].y;		// finalPoint[Y] - supportPoint[Y];
					}
					// all between
					else
					{
						vectorA[X_POS] = trayPoints[i-1].x - trayPoints[i].x;		// startPoint[X] - supportPoint[X];
						vectorA[Y_POS] = trayPoints[i-1].y - trayPoints[i].y;		// startPoint[Y] - supportPoint[Y];
						vectorB[X_POS] = trayPoints[i+1].x - trayPoints[i].x;		// finalPoint[X] - supportPoint[X];
						vectorB[Y_POS] = trayPoints[i+1].y - trayPoints[i].y;		// finalPoint[Y] - supportPoint[Y];
					}
					
					// Calculates the angle alpha of the clothoid
					alpha = Angle2D(vectorA, vectorB);
					
					// Correction of the angle
					if (alpha > M_PI)	alpha = (2 * M_PI) - alpha;
					
					//sprintf(text,"%1.3f %d (%1.3f-%1.3f)\r\n", alpha * 180.0 / M_PI, i, paramOutputAntrieb.fX, paramOutputAntrieb.fY);
					//debugMsg(text);
					
					if((fabs(alpha) < (5.0 * M_PI / 180.0)) ||(fabs(alpha - M_PI) < (1.0 * M_PI / 180.0)))
					{
						for (int j = i; j < (paramInputAntrieb[indexInputInAntrieb].destinationPointsCount - 1); j++)
						{
							trayPoints[j] = trayPoints[j+1];
						}
						
						i--;
						(paramInputAntrieb[indexInputInAntrieb].destinationPointsCount)--;
					}
				}
			}
			
			//sprintf(text,"C: type: %d v:%d\r", paramInputAntrieb[indexInputInAntrieb].ucType, destinationVmax);
			//debugMsg(text);
			
			// Test: Destination
			//sprintf(text,"II:(%1.3f-%1.3f)(%1.3f-%1.3f)(%1.3f-%1.3f)(%1.3f-%1.3f) %d\r\n",
			//trayPoints[0].x,
			//trayPoints[0].y,
			//trayPoints[1].x,
			//trayPoints[1].y,
			//trayPoints[2].x,
			//trayPoints[2].y,
			//trayPoints[3].x,
			//trayPoints[3].y,
			//paramInputAntrieb[indexInputInAntrieb].destinationPointsCount
			//);
			//debugMsg(text);

			indexInputInAntrieb++;
			if(indexInputInAntrieb >= MAX_IN_ANTRIEB)
			indexInputInAntrieb = 0;
			indexInputAntrieb++;
		}
	}
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanPosition_Task                          ***
***   FUNCTION:            empfängt Daten von uC1 -> Position           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanPosition_Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[18];
	uint16_t x0, y0, phi0;

	if(Received_AppData_Available(&MCU1, POS_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, POS_PORTNBR, receiveArray);
		
		// *************************
		// Werte auslesen
		// *************************
		x0 = (int16_t)(((uint16_t)(receiveArray[0]) << 8) + (uint16_t)(receiveArray[1]));
		y0 = (int16_t)(((uint16_t)(receiveArray[2]) << 8) + (uint16_t)(receiveArray[3]));
		phi0 = (int16_t)(((uint16_t)(receiveArray[4]) << 8) + (uint16_t)(receiveArray[5]));
		
		// *************************
		// Werte setzen
		// *************************
		ucSetGlobalPosition = 1;
		uiXOffset = x0;
		uiYOffset = y0;
		uiWinkelOffset = phi0;
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanPosition_Task                          ***
***   FUNCTION:            empfängt Daten von uC1 -> Position           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanSetSchleppfehler_Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[18];
	uint16_t x0, y0, phi0;

	if(Received_AppData_Available(&MCU1, SET_SCHLEPP_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, SET_SCHLEPP_PORTNBR, receiveArray);
		Schleppfehler = ((float)receiveArray[0])/100.0;
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanPosition_Task                          ***
***   FUNCTION:            empfängt Daten von uC1 -> Position           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanSetACC_Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[18];

	if(Received_AppData_Available(&MCU1, SET_ACC_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU1, SET_ACC_PORTNBR, receiveArray);
		paramComAntrieb.fAmaxUs = ((float)receiveArray[0])/100.0;
		paramComAntrieb.fAmaxDs = ((float)receiveArray[1])/100.0;
	}
	
	return(DISABLE);
}






