
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
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_serialconfig.h"
#include "rrt_applicationLayer.h"
#include "ports.h"
#include "define.h"
#include "global.h"
#include "usart.h"
#include "rrt_usart_driver.h"

uint16_t messageCountAntrieb = 1;
uint16_t messageCountAntriebClothoid = 1;

/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitTransmit(void)
{

}

/**************************************************************************
***   FUNCTIONNAME:        startLiftInit                                ***
***   FUNCTION:            Startet Lift Init                            ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO										    ***
**************************************************************************/
void startLiftInit_RRTLAN(USART_data_t *controller)
{
	//uint8_t sendArray[5];
	//
	//sendArray[0] = 1;
	//
	//if (controller == &MCU3)
	//{
	//Send_Application_Data(controller, START_LIFT_INIT_UC3_PORTNBR, sendArray, 1);
	//}
}

/**************************************************************************
***   FUNCTIONNAME:        setServo                                     ***
***   FUNCTION:            Setzt Servowinkel                            ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   nbrUsart ... Schnittstellenummer             ***
***                        nbrServo ... Servonummer                     ***
***                        winkelServo ... Servowinekl (0° ... 135°)    ***
**************************************************************************/
void setServo_RRTLAN(USART_data_t *controller, uint8_t nbrServo, int16_t winkelServo)
{
	uint8_t sendArray[5];
	
	convData_t d;
	d.int16[0] = winkelServo;
	
	sendArray[0] = nbrServo;
	sendArray[1] = d.uint8[0];
	sendArray[2] = d.uint8[1];

	if (controller == &MCU2)
	{
		Send_Application_Data(controller, SERVO_UC2_PORTNBR, sendArray, 2);
	}
	//else if (controller == &MCU3)
	//{
	//Send_Application_Data(controller, SERVO_UC3_PORTNBR, sendArray, 3);
	//}
}

/**************************************************************************
***   FUNCTIONNAME:        setMotor                                     ***
***   FUNCTION:            Setzt Motor                                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   nbrUsart ... Schnittstellenummer             ***
***                        nbrDiOut ... Nummer des digitalen Ausgangs   ***
***                        value ... 0/1 (EIN/AUS)                      ***
**************************************************************************/
void setDigitalOut_RRTLAN(USART_data_t *controller, uint8_t nbrDiOut, uint8_t value)
{
	uint8_t sendArray[4];
	
	sendArray[0] = nbrDiOut;
	sendArray[1] = value;

	if (controller == &MCU2)
	{
		Send_Application_Data(controller, DIGIOUT_UC2_PORTNBR, sendArray, 2);
	}
	//else if (controller == &MCU3)
	//{
	//Send_Application_Data(controller, DIGIOUT_UC3_PORTNBR, sendArray, 2);
	//}

}

/**************************************************************************
***   FUNCTIONNAME:        VelocityCommand                              ***
***   FUNCTION:            send veleocity command                       ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   controller ... dedicated controller          ***
***                        port ... message port								***
***                        vel ... velocity [m/s]                       ***
**************************************************************************/
void VelocityCommand_RRTLAN(USART_data_t *controller, uint8_t nbr, float vel)
{
	uint8_t sendArray[3];
	int16_t vel_i16;
	
	vel_i16 = (int16_t)(vel * 1000.0);
	
	sendArray[0] = nbr;
	sendArray[1] = (uint8_t)((uint16_t)(vel_i16) / 256);
	sendArray[2] = (uint8_t)((uint16_t)(vel_i16) % 256);
	
	if (controller == &MCU2)
	{
		Send_Application_Data(controller, MOT_VEL_UC2_PORTNBR, sendArray, 3);
	}
	//else if (controller == &MCU3)
	//{
	//Send_Application_Data(controller, MOT_VEL_UC3_PORTNBR, sendArray, 3);
	//}
}

/**************************************************************************
***   FUNCTIONNAME:        VelocityCommand                              ***
***   FUNCTION:            send veleocity command                       ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   controller ... dedicated controller          ***
***                        port ... message port								***
***                        vel ... velocity [m/s]                       ***
***                        s0 ... distance [m]                          ***
**************************************************************************/
void PositionCommand_RRTLAN(USART_data_t *controller, uint8_t nbr, float vel, float s0)
{
	uint8_t sendArray[5];
	int16_t vel_i16, s0_i16;

	vel_i16 = (int16_t)(vel * 1000.0);
	s0_i16 = (int16_t)(s0 * 1000.0);

	sendArray[0] = (uint8_t)((uint16_t)(vel_i16) >> 8);
	sendArray[1] = (uint8_t)((uint16_t)(vel_i16));
	sendArray[2] = (uint8_t)((uint16_t)(s0_i16) >> 8);
	sendArray[3] = (uint8_t)((uint16_t)(s0_i16));
	sendArray[4] = (uint8_t)nbr;
	
	if (controller == &MCU2)
	{
		Send_Application_Data(controller, MOT_POS_UC2_PORTNBR, sendArray, 5);
	}
	
}

/**************************************************************************
***   FUNCTIONNAME:        VoltageCommand                               ***
***   FUNCTION:            send voltage command                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   controller ... dedicated controller          ***
***                        port ... message port								***
***                        vol ... voltage [V]                          ***
**************************************************************************/
void VoltageCommand_RRTLAN(USART_data_t *controller, uint8_t port, float vol, uint8_t nbrMot)
{
	uint8_t sendArray[3];
	int16_t vol_i16;
	
	vol_i16 = (int16_t)(vol * 1000.0);
	
	sendArray[0] = (uint8_t)((uint16_t)(vol_i16) / 256);
	sendArray[1] = (uint8_t)((uint16_t)(vol_i16) % 256);
	sendArray[2] = nbrMot;
	
	Send_Application_Data(controller, port, sendArray, 3);
}

/**************************************************************************
***   FUNCTIONNAME:        setAntrieb                                   ***
***   FUNCTION:            Setzt Antrieb                                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vStart .. Anfangsgeschwindigkeit             ***
***                        vEnd ... Endgeschwindigkeit                  ***
***                        vMax ... Maximalgeschwindigkeit              ***
***                        phiSoll ... Winkel (bei Drehung und          ***
***                                    Kreisbogen)                      ***
***                        rSoll ... Radius (bei Kreisbogen)            ***
***                        xSoll ... X-Position                         ***
***                        ySoll ... Y-Position                         ***
***                        sSoll ... Weg                                ***
***                        type ... Bewegungsart                        ***
**************************************************************************/
void setAntrieb_RRTLAN(int16_t vStart,
int16_t vEnd,
int16_t vMax,
int16_t phiSoll,
uint16_t rSoll,
uint16_t xSoll,
uint16_t ySoll,
uint16_t sSoll,
uint8_t type,
uint8_t gegnerErkennung)
{
	uint8_t sendArray[20];
	
	statusAntrieb = 0;
	
	sendArray[0] = (uint8_t)(vStart >> 8);
	sendArray[1] = (uint8_t)(vStart);
	sendArray[2] = (uint8_t)(vEnd >> 8);
	sendArray[3] = (uint8_t)(vEnd);
	sendArray[4] = (uint8_t)(vMax >> 8);
	sendArray[5] = (uint8_t)(vMax);
	sendArray[6] = (uint8_t)(phiSoll >> 8);
	sendArray[7] = (uint8_t)(phiSoll);
	sendArray[8] = (uint8_t)(rSoll >> 8);
	sendArray[9] = (uint8_t)(rSoll);
	sendArray[10] = (uint8_t)(xSoll >> 8);
	sendArray[11] = (uint8_t)(xSoll);
	sendArray[12] = (uint8_t)(ySoll >> 8);
	sendArray[13] = (uint8_t)(ySoll);
	sendArray[14] = (uint8_t)(sSoll >> 8);
	sendArray[15] = (uint8_t)(sSoll);
	sendArray[16] = type;
	sendArray[17] = gegnerErkennung;
	sendArray[18] = (uint8_t)(messageCountAntrieb >> 8);
	sendArray[19] = (uint8_t)(messageCountAntrieb);
	
	messageCountAntrieb++;
	
	if(vMax >= 0)   geschwindigkeitsVorzeichen = 1.0;
	else            geschwindigkeitsVorzeichen = -1.0;
	
	Send_Application_Data(&MCU2, ANTRIEB_UC2_PORTNBR, sendArray, 20);
}

/**************************************************************************
***   FUNCTIONNAME:        setAntriebClothoid                           ***
***   FUNCTION:            Set drive with clothoid		                 ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vMax ... Max. velocity						***
***                        wP ... Pointer on way points (X, Y)			***
***						   nOP ... Number of points						***
**************************************************************************/
void setAntriebClothoid_RRTLAN(int16_t vMax, element_t *wP, uint8_t nOP)
{
	uint8_t i = 0;
	uint8_t sendArray[(nOP * 4) + 4];
	
	statusAntrieb = 0;
	
	sendArray[0] = (uint8_t)(vMax >> 8);
	sendArray[1] = (uint8_t)(vMax);
	sendArray[2] = (uint8_t)(messageCountAntriebClothoid >> 8);
	sendArray[3] = (uint8_t)(messageCountAntriebClothoid);
	
	messageCountAntriebClothoid++;
	
	for (i = 0; i < nOP; i++)
	{
		sendArray[(i*4)+4] = (uint8_t)(wP[i].Xpos >> 8);
		sendArray[(i*4)+5] = (uint8_t)(wP[i].Xpos);
		sendArray[(i*4)+6] = (uint8_t)(wP[i].Ypos >> 8);
		sendArray[(i*4)+7] = (uint8_t)(wP[i].Ypos);
	}
	
	if(vMax >= 0)   geschwindigkeitsVorzeichen = 1.0;
	else            geschwindigkeitsVorzeichen = -1.0;
	
	Send_Application_Data(&MCU2, ANTRIEB_CLOTHOID_UC2_PORTNBR, sendArray, ((nOP * 4) + 4));
}

/**************************************************************************
***   FUNCTIONNAME:        setACC Antrieb	                            ***
***   FUNCTION:            Setzt Beschleunigung Antrieb                                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   AmaxUs .. Anfangsbeschleunigung             ***
***                        AmaxDs ... Endbeschleunigung                  ***
**************************************************************************/
void setACCAntrieb_RRTLAN(uint8_t AmaxUs,uint8_t AmaxDs)
{
	uint8_t sendArray[2];
	
	sendArray[0] = AmaxUs;
	sendArray[1] = AmaxDs;
			
	Send_Application_Data(&MCU2, SET_ACC_UC2_PORTNBR, sendArray, 2);
}

/**************************************************************************
***   FUNCTIONNAME:        setPosition                                  ***
***   FUNCTION:            Setzt Position des Antriebs                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   x0 ... X-Position (in mm)                    ***
***                        y0 ... Y-Position (in mm)                    ***
***                        phi0 ... Winkel (in 0.1°)                    ***
**************************************************************************/
void setPosition_RRTLAN(uint16_t x0, uint16_t y0, uint16_t phi0)
{
	uint8_t sendArray[6];
	
	sendArray[0] = (uint8_t)(x0 >> 8);
	sendArray[1] = (uint8_t)(x0);
	sendArray[2] = (uint8_t)(y0 >> 8);
	sendArray[3] = (uint8_t)(y0);
	sendArray[4] = (uint8_t)(phi0 >> 8);
	sendArray[5] = (uint8_t)(phi0);
	
	Send_Application_Data(&MCU2, POS_UC2_PORTNBR, sendArray, 6);
}


/**************************************************************************
***   FUNCTIONNAME:        setDataLCD_RRTLAN                            ***
***   FUNCTION:            Setzt Punkte und Zeit am LCD                 ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   points ... points		                    ***
***                        time ... time to play                        ***
**************************************************************************/
void setDataLCD_RRTLAN(uint8_t points, uint8_t time)
{
	uint8_t sendArray[6];
	
	sendArray[0] = points;
	sendArray[1] = time;
	
	// 	Send_Application_Data(&LCD, VALUE_LCD_PORTNBR, sendArray, 2);
}

/**************************************************************************
***   FUNCTIONNAME:        setDataWIFI_RRTLAN                            ***
***   FUNCTION:            Kommunikation zwischen Roboter                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   points ... points		                    ***
***                        time ... time to play                        ***
**************************************************************************/
void setDataWIFI_RRTLAN(uint8_t eP)
{
	uint8_t sendArray[1];
	
	// 	sendArray[0] = eP;
	//
	// 	Send_Application_Data(&WIFI, WIFI_MOSI_PORTNBR, sendArray, 1);
}
/**************************************************************************
***   FUNCTIONNAME:        sendPosDataToEnemyDetect_RRLAN                                  ***
***   FUNCTION:            Sendet dem uC3 die Pos Daten                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   x0 ... X-Position (in mm)                    ***
***                        y0 ... Y-Position (in mm)                    ***
***                        phi0 ... Winkel (in °)                       ***
***                        enemy ... Daten der Gegner		            ***
**************************************************************************/
void sendPosDataEnemy_from_WIFI_RRLAN(uint16_t xPos, uint16_t yPos, uint16_t phiPos, EnemyDataRaw_t *enemy)
{
	uint8_t sendArray[50];
	
	// Eigene Position des Roboters
	sendArray[0] = (uint8_t)(xPos >> 8);
	sendArray[1] = (uint8_t)(xPos);
	sendArray[2] = (uint8_t)(yPos >> 8);
	sendArray[3] = (uint8_t)(yPos);
	sendArray[4] = (uint8_t)(phiPos >> 8);
	sendArray[5] = (uint8_t)(phiPos);
	
	sendArray[6] = enemy->count;
	
	for (uint8_t i = 0 ; i < 3; i++)
	{
		sendArray[7+(i*6)] = (uint8_t)(enemy->x[i] >> 8);
		sendArray[8+(i*6)] = (uint8_t)(enemy->x[i]);

		sendArray[9+(i*6)] = (uint8_t)(enemy->y[i] >> 8);
		sendArray[10+(i*6)] = (uint8_t)(enemy->y[i]);

		sendArray[11+(i*6)] = (uint8_t)(enemy->var[i]>> 8);
		sendArray[12+(i*6)] = (uint8_t)(enemy->var[i]);
	}
	
	
	//Send_Application_Data(&MCU3, ENEMYDATA_RAW_FROM_WIFI_PORTNBR, sendArray, 37);
}

/**************************************************************************
***   FUNCTIONNAME:                                      ***
***   FUNCTION:                             ***
***   TRANSMIT-PARAMETER:                                             ***
***   RECEIVE-PARAMETER:                    ***
**************************************************************************/
void setSchleppfehler_RRLAN(uint8_t schleppdistance)
{
	uint8_t sendArray[5];
	
	sendArray[0] = (schleppdistance);
	
	Send_Application_Data(&MCU2, SET_SCHLEPP_UC2_PORTNBR, sendArray, 1);
}


/**************************************************************************
***   FUNCTIONNAME:        sendPosDataToEnemyDetect_RRLAN                                  ***
***   FUNCTION:            Sendet dem uC3 die Pos Daten                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   x0 ... X-Position (in mm)                    ***
***                        y0 ... Y-Position (in mm)                    ***
***                        phi0 ... Winkel (in °)                       ***
***                        enemy ... Daten der Gegner		            ***
**************************************************************************/
void sendPos2EnemyDetection_RRLAN()
{
	uint8_t sendArray[50];
	int16_t phi = phiPos / 10;
	convData_t d;
	
	
	// Eigene Position des Roboters
	d.int16[0] = xPos;
	sendArray[0] = d.uint8[0];
	sendArray[1] = d.uint8[1];
	
	d.int16[0] = yPos;
	sendArray[2] = d.uint8[0];
	sendArray[3] = d.uint8[1];

	d.int16[0] = phi;
	sendArray[4] = d.uint8[0];
	sendArray[5] = d.uint8[1];
	
	
	Send_Application_Data(&GEGNER, ROBO_POS_MSG_GEGNER_PORTNBR, sendArray, 6);
}
