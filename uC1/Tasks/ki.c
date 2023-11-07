/***************************************************************

************************************
** FH OBEROESTERREICH CAMPUS WELS **
************************************

Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      KI.c
Version :  V 1.0
Date    :  23.03.2011
Author  :  ZAUNER MICHAEL

Comments:

Last edit:
Programmchange:

*)....
*).....

Chip type           : Xmega256A
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024

Copyright (c) 2011 by FH-Wels
All Rights Reserved.
****************************************************************/

#define _KI_EXTERN

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>
#include "multitask.h"
#include "ki.h"
#include "ports.h"
#include "define.h"
#include "rrt_usart_driver.h"
#include "global.h"
#include "adc_driver.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "servo.h"
#include "usart.h"
#include "rrt_serialconfig.h"
#include "Pfadplanung.h"
#include "poi.h"
#include "rrt_transmittingtask.h"
#include "command.h"
#include "PSE541.h"
#include "observation.h"
#include "nextion.h"


#define _DEBUG_MOTION_


char text[200];

/* to stop the robot only once at end of match */
uint8_t stopEngin = 0;





/* the last 5 sec no new task will be chosen */
#define KI_DISABLE_TIME			0


// Distanzen für Gegnererkennung (Ultraschallsensoren)
#define  WATCH_DIS_FRONT		600
#define TURN_AROUND_DIS			WATCH_DIS_FRONT + 200


element_t wp_KI[10];
uint8_t wpNbr;




/**************************************************************************
***   FUNKTIONNAME: InitKI                                              ***
***   FUNKTION: initialisiert die KI                                    ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitKI(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_TASK(KI_TASKNBR, CYCLE);
	SET_CYCLE(KI_TASKNBR, 100);
	SET_TASK_HANDLE(KI_TASKNBR, KiTask);
	
	// zyklischer Task - Zykluszeit: 100 ms
	SET_TASK(KI_WATCH_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_TASKNBR, 100);
	SET_TASK_HANDLE(KI_WATCH_TASKNBR, KiWatchTask);

	// KiWatchRobotPositionTask initialisieren
	SET_TASK(KI_WATCH_ROBOT_POS_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_ROBOT_POS_TASKNBR, 100);
	SET_TASK_HANDLE(KI_WATCH_ROBOT_POS_TASKNBR, KiWatchRobotPositionTask);
	
	
	KI_State = 0;
	HomePositionReached=0;
	
	
	/* ******************* */
	/* Hindernisse anlegen */
	/* ******************* */
	/* zuerst alle Hindernisse sperren */
	for (uint8_t i = 0; i < PATH_STATIC_OBSTICAL_LIST_LENGHT; i++)
	{
		PATH_DISABLE_OBSTACLE(i);
	}
	/* Start-Zone der Ladybugs */
	PATH_SetStaticObstacle(0, 900, 0, 2100, 300);
	PATH_ENABLE_OBSTACLE(0);
	///* geschützte Start-Zone blauer Roboter */
	//PATH_SetStaticObstacle(1, 2550, 0, 3000, 450);
	//PATH_ENABLE_OBSTACLE(1);
	///* geschützte Start-Zone gelber Roboter */
	//PATH_SetStaticObstacle(2, 0, 0, 450, 450);
	//PATH_ENABLE_OBSTACLE(2);
	
	Punkte = 5;
	
	// Enemy detection
	gegnerErkennung = OFF;
	
	/* ****************************************** */
	/* alle Tasks initialisieren und deaktivieren */
	/* ****************************************** */
	for (uint8_t i = 0; i<MAX_KI_TASKS;i++)
	{
		KI_Task[i].Status = DONE;
		KI_Task[i].Start = (uint16_t)i*1000;
		KI_Task[i].Priority = 0;
	}

	
	// ********************************************************************
	// ******   A U F G A B E N   I N I T I A L I S I E R E N
	// *******************************************



	// *********************************************************************************************************************************
	// *********************************************************************************************************************************
	// *********************************************************************************************************************************
	// Strategy BOTH
	// *********************************************************************************************************************************
	// *********************************************************************************************************************************
	// *********************************************************************************************************************************
	KI_Task[1].Priority = 100;
	KI_Task[1].Status = OPEN;


	if (Strategie != STRATEGY_HOMOLOGATION)
	{
		switch (SpielFarbe)
		{

			switch (Strategie)
			{
				// Strategy M1 ==> R1 ==> R2 bzw. M1 ==> L1 ==> L2
				case NEXTION_STRATEGY_L1_L2_P1: case NEXTION_STRATEGY_R1_R2_P1:
				{
					
					KI_Task[1].Priority = 100;
					KI_Task[6].Priority = 99;
					KI_Task[5].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[4].Priority = 97;

					break;
				}
				//Strategy M1 ==> M2 ==> R2  bzw. M1 ==> M2 ==> L2
				case NEXTION_STRATEGY_L1_L2_A1: case NEXTION_STRATEGY_R1_R2_A1:
				{
					KI_Task[1].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[5].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[6].Priority = 97;
					
					break;
				}
				//Strategy L1 ==> R1 ==> R2  bzw. R1 ==> L1 ==> L2
				case NEXTION_STRATEGY_L1_L2_R1: case NEXTION_STRATEGY_R1_R2_R1:
				{
					KI_Task[6].Priority = 100;
					KI_Task[2].Priority = 99;
					KI_Task[3].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy L1 ==> M1 ==> R1  bzw. R1 ==> M1 ==> L1
				case NEXTION_STRATEGY_L1_L2_R2: case NEXTION_STRATEGY_L1_R3_R2: case NEXTION_STRATEGY_R1_R2_R2: case NEXTION_STRATEGY_R1_L3_R2:
				{
					KI_Task[6].Priority = 100;
					KI_Task[1].Priority = 99;
					KI_Task[2].Priority = 98;
					
					KI_Task[3].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy L1 ==> M1 ==> R2  bzw. R1 ==> M1 ==> L2
				case NEXTION_STRATEGY_L1_L2_R3: case NEXTION_STRATEGY_L1_R1_R2: case NEXTION_STRATEGY_R1_R2_R3: case NEXTION_STRATEGY_R1_L1_R2:
				{
					KI_Task[6].Priority = 100;
					KI_Task[1].Priority = 99;
					KI_Task[3].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy L1 ==> M2 ==> R2  bzw. R1 ==> M2 ==> L2
				case NEXTION_STRATEGY_L1_L2_R4: case NEXTION_STRATEGY_L1_R1_R1: case NEXTION_STRATEGY_R1_R2_R4: case NEXTION_STRATEGY_R1_L1_R1:
				{
					KI_Task[6].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[3].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[2].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy L1 ==> L2  bzw. R1 ==> R2
				case NEXTION_STRATEGY_L1_R1_P1: case NEXTION_STRATEGY_L1_R3_P1: case NEXTION_STRATEGY_R1_L1_P1: case NEXTION_STRATEGY_R1_L3_P1:
				{
					KI_Task[6].Priority = 100;
					KI_Task[5].Priority = 99;
					
					KI_Task[1].Priority = 97;
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[4].Priority = 97;
					
					break;
				}
				//Strategy L1 ==> M2 ==> L2  bzw. R1 ==> M2 ==> R2
				case NEXTION_STRATEGY_L1_R1_P2: case NEXTION_STRATEGY_R1_L1_P2:
				{
					KI_Task[6].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[5].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					
					break;
				}
				//Strategy L1 ==> L2 ==> M2  bzw. R1 ==> R2 ==> M2
				case NEXTION_STRATEGY_L1_R1_P3: case NEXTION_STRATEGY_L1_R3_R1: case NEXTION_STRATEGY_R1_L1_P3: case NEXTION_STRATEGY_R1_L3_R1:
				{
					KI_Task[6].Priority = 100;
					KI_Task[5].Priority = 99;
					KI_Task[4].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					
					break;
				}
				//Strategy M1 ==> M2 ==> L2  bzw. M1 ==> M2 ==> R2
				case NEXTION_STRATEGY_L1_R1_A1: case NEXTION_STRATEGY_L1_R3_A2: case NEXTION_STRATEGY_R1_L1_A1: case NEXTION_STRATEGY_R1_L3_A2:
				{
					KI_Task[1].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[5].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[6].Priority = 97;
					
					break;
				}
				//Strategy M1 ==> M2 ==> L1  bzw. M1 ==> M2 ==> R1
				case NEXTION_STRATEGY_L1_R1_A2: case NEXTION_STRATEGY_L1_R3_A1: case NEXTION_STRATEGY_R1_L1_A2: case NEXTION_STRATEGY_R1_L3_A1:
				{
					KI_Task[1].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[6].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}

				//Strategy M1 ==> L2 ==> L1  bzw. M1 ==> R2 ==> R1
				case NEXTION_STRATEGY_L1_R3_P2: case NEXTION_STRATEGY_R1_L3_P2:
				{
					KI_Task[1].Priority = 100;
					KI_Task[5].Priority = 99;
					KI_Task[6].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[4].Priority = 97;
					
					break;
				}
				//Strategy R2 ==> M2 ==> L1  bzw. L2 ==> M2 ==> R1
				case NEXTION_STRATEGY_R2_R1_P1: case NEXTION_STRATEGY_R2_L2_R3: case NEXTION_STRATEGY_L2_L1_P1: case NEXTION_STRATEGY_L2_R2_R3:
				{
					KI_Task[3].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[6].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[2].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R2 ==> M2 ==> L2  bzw. L2 ==> M2 ==> R2
				case NEXTION_STRATEGY_R2_R1_P2: case NEXTION_STRATEGY_L2_L1_P2:
				{
					KI_Task[3].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[5].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[2].Priority = 97;
					KI_Task[6].Priority = 97;
					
					break;
				}
				//Strategy R2 ==> M1 ==> L1  bzw. L2 ==> M1 ==> R1
				case NEXTION_STRATEGY_R2_R1_A1: case NEXTION_STRATEGY_R2_L2_R2: case NEXTION_STRATEGY_L2_L1_A1: case NEXTION_STRATEGY_L2_R2_R2:
				{
					KI_Task[3].Priority = 100;
					KI_Task[1].Priority = 99;
					KI_Task[6].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R2 ==> M2 ==> M1  bzw. L2 ==> M2 ==> M1
				case NEXTION_STRATEGY_R2_R1_A2: case NEXTION_STRATEGY_R2_L2_A2: case NEXTION_STRATEGY_L2_L1_A2: case NEXTION_STRATEGY_L2_R2_A2:
				{
					KI_Task[3].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[1].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[5].Priority = 97;
					KI_Task[6].Priority = 97;
					
					break;
				}
				//Strategy R1 ==> M1 ==> L1  bzw. L1 ==> M1 ==> R1
				case NEXTION_STRATEGY_R2_R3_P1: case NEXTION_STRATEGY_R2_L2_R1: case NEXTION_STRATEGY_L2_L3_P1: case NEXTION_STRATEGY_L2_R2_R1:
				{
					KI_Task[3].Priority = 100;
					KI_Task[1].Priority = 99;
					KI_Task[6].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R1 ==> M1 ==> M2  bzw. L1 ==> M1 ==> M2
				case NEXTION_STRATEGY_R2_R3_A1:  case NEXTION_STRATEGY_L2_L3_A1:
				{
					KI_Task[3].Priority = 100;
					KI_Task[1].Priority = 99;
					KI_Task[6].Priority = 98;
					
					KI_Task[2].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R1 ==> M2 ==> L2  bzw. L1 ==> M2 ==> R2
				case NEXTION_STRATEGY_R2_R3_R1:  case NEXTION_STRATEGY_L2_L3_R1:
				{
					KI_Task[3].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[5].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[2].Priority = 97;
					KI_Task[6].Priority = 97;
					
					break;
				}
				//Strategy R1 ==> M2 ==> L1  bzw. L1 ==> M2 ==> R1
				case NEXTION_STRATEGY_R2_R3_R2:  case NEXTION_STRATEGY_L2_L3_R2:
				{
					KI_Task[2].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[6].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R1 ==> R2
				case NEXTION_STRATEGY_R2_L2_P1:  case NEXTION_STRATEGY_L2_R2_P1:
				{
					KI_Task[2].Priority = 100;
					KI_Task[3].Priority = 99;
					
					KI_Task[1].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R2 ==> R1
				case NEXTION_STRATEGY_R2_L2_P2:  case NEXTION_STRATEGY_L2_R2_P2:
				{
					KI_Task[3].Priority = 100;
					KI_Task[2].Priority = 99;
					
					KI_Task[1].Priority = 97;
					KI_Task[3].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R2 ==> R1 ==> M1
				case NEXTION_STRATEGY_R2_L2_A1:  case NEXTION_STRATEGY_L2_R2_A1:
				{
					KI_Task[3].Priority = 100;
					KI_Task[2].Priority = 99;
					KI_Task[1].Priority = 98;
					
					KI_Task[3].Priority = 97;
					KI_Task[4].Priority = 97;
					KI_Task[5].Priority = 97;
					
					break;
				}
				//Strategy R1 ==> M2 ==> R2
				case NEXTION_STRATEGY_R2_L2_A3:  case NEXTION_STRATEGY_L2_R2_A3:
				{
					KI_Task[2].Priority = 100;
					KI_Task[4].Priority = 99;
					KI_Task[3].Priority = 98;
					
					KI_Task[1].Priority = 97;
					KI_Task[5].Priority = 97;
					KI_Task[6].Priority = 97;
					
					break;
				}
			}
		}
	}
	else
	{
		// *******************************************
		// Strategy: STRATEGY_HOMOLOGATION
		// *******************************************
		if(Strategie == STRATEGY_HOMOLOGATION)
		{
			
		}
		
	}
	
	if(SpielFarbe == Yellow_L2 || SpielFarbe == Yellow_R1 || SpielFarbe == Yellow_R3)
	{
		ChangePrioToYellow();
	}
}


/**************************************************************************
***   FUNKTIONNAME: SetNextStepKI                                         ***
***   FUNKTION: Setzt die nächsten Schritte                             ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: Current ... aktueller State                   ***
***                       Next ... nächster State -> bei OK             ***
***                       Error ... nächster State -> bei Error         ***
**************************************************************************/
void SetNextStepKI(unsigned int Current, unsigned int Next, unsigned int Error)
{
	KI_State = 65000;
	KI_StateNext = Next;
	KI_StateOld = Current;
	KI_StateError = Error;
}

/**************************************************************************
***   FUNKTIONNAME: AddDifferencePerQuadrant                            ***
***   FUNKTION: ?ndern der Prios f?r Blau                               ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
point_t AddDifferencePerQuadrant(point_t start, point_t ziel )
{
	point_t middlePoint;
	
	/* Declare the relative distance you want to the Plants for the Middle Point manually below*/
	middlePoint.Xpos = 50;
	middlePoint.Ypos = 50;
	
	if(ziel.Xpos >start.Xpos && ziel.Ypos > start.Ypos)
	/*Quadrant 1*/
	{
		middlePoint.Xpos = ziel.Xpos - middlePoint.Xpos;
		middlePoint.Ypos = ziel.Ypos - middlePoint.Ypos;
	}
	else if(ziel.Xpos < start.Xpos && ziel.Ypos > start.Ypos)
	/*Quadrant 2*/
	{
		middlePoint.Xpos = ziel.Xpos + middlePoint.Xpos;
		middlePoint.Ypos = ziel.Ypos - middlePoint.Ypos;
	}
	else if(ziel.Xpos < start.Xpos && ziel.Ypos < start.Ypos)
	/*Quadrant 3*/
	{
		middlePoint.Xpos = ziel.Xpos + middlePoint.Xpos;
		middlePoint.Ypos = ziel.Ypos + middlePoint.Ypos;
	}
	else if(ziel.Xpos > start.Xpos && ziel.Ypos < start.Ypos)
	/*Quadrant 4*/
	{
		middlePoint.Xpos = ziel.Xpos - middlePoint.Xpos;
		middlePoint.Ypos = ziel.Ypos + middlePoint.Ypos;
	}
	else
	/* Error Handling: if start and ziel are equal, return ziel as middlePoint */
	{
		middlePoint.Xpos = ziel.Xpos;
		middlePoint.Ypos = ziel.Ypos;
		return middlePoint;
	}
	return middlePoint;
}

/**************************************************************************
***   FUNKTIONNAME: ChangePrioToYellow                                  ***
***   FUNKTION: Ändern der Prios für Blau                               ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void ChangePrioToYellow()
{
	int prio2 = KI_Task[2].Priority;
	int prio3 = KI_Task[3].Priority;
	int prio5 = KI_Task[5].Priority;
	int prio6 = KI_Task[6].Priority;
	
	KI_Task[2].Priority = prio6;
	KI_Task[3].Priority = prio5;
	KI_Task[5].Priority = prio3;
	KI_Task[6].Priority = prio2;
}





/**************************************************************************
***   FUNCTIONNAME:        KiTask                                       ***
***   FUNCTION:            KI                                           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiTask(void)
{
	char text1[200];
	// Cycle per default auf 50 ms setzen
	SET_CYCLE(KI_TASKNBR, 100);


	static uint16_t lastState = -1;
	
	// 	if (lastState != KI_State)
	// 	{
	// 		lastState = KI_State;
	// 		sprintf(text1, "#State: %ld Zeit: %d / Xpos: %d  YPos: %d  Phi: %d\n*", (uint32_t)KI_State, spielZeit, xPos, yPos, phiPos);
	// 		writeString_usart(&WIFI_IF, text1);
	//
	// 	}
	
	// ********************************************************************
	// ********************************************************************
	// ******
	// ******   S P I E L Z E I T U E B E R W A C H U N G
	// ******
	// ********************************************************************
	// ********************************************************************
	// ***********************



	// in der letzten Sekunde
	//		-> Roboter gegebenfalls stoppen
	// ***********************
	if(spielZeit < 1)
	{
		// ******************************************************
		// Hier alle notwendigen Systeme deaktivieren
		// ******************************************************
		cmd_SetDigitalOut(DO_LED1_NBR, 1);
		// ******************************************************
		// wenn eine Bewegung ausgeführt wird
		//		-> Roboter stoppen und auf Leerlaufstate springen
		// ******************************************************
		if((statusAntrieb == 0) && (stopEngin == 0))
		{
			setAntrieb_RRTLAN(0, 0, 0, 0, 0, 0, 0, 0, MOTION_INTERRUPT, GEGNER_OFF);
			SetNextStepKI(KI_State, 49000, 49000);
			
			stopEngin = 1;
		}
		// -> ansonsten nur auf Leerlaufstate springen
		else
		{
			
		}
	}
	
	// 	//Master bricht Task ab wenn Zeit knapp wird und fährt nach Hause
	// 	if (spielZeit < 15 && DriveHome == 1 && RobotType_RAM == MASTER_ROBOT &&
	// 	((xPos > 600 && SpielFarbe == PURPLE) || (xPos < 2400 && SpielFarbe == YELLOW)) &&
	// 	yPos > 250  && yPos < 1750 && KI_State != 65000)
	// 	{
	// 		KI_State = 48000;
	// 	}
	
	sprintf(text1, "#%ld;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;\r\n*", (uint32_t)KI_State, enemyRobot[0].Xpos, enemyRobot[0].Ypos,
	enemyRobot[1].Xpos, enemyRobot[1].Ypos,
	enemyRobot[2].Xpos, enemyRobot[2].Ypos,
	enemyRobot[3].Xpos, enemyRobot[3].Ypos,
	enemyRobot[4].Xpos, enemyRobot[4].Ypos);
	writeString_usart(&WIFI_IF, text1);
	
	
	// ********************************************************************
	// ********************************************************************
	// ******
	// ******   K I  -  S T A T E M A C H I N E
	// ******
	// ********************************************************************
	// ********************************************************************
	switch(KI_State)
	{
		
		//	vacTimeout++;
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   K I  -  V E R T E I L E R
		// ******
		// ********************************************************************
		// ********************************************************************
		// In den 1. State der KI springen


		case 0:
		{
			KI_State = 20;

			return(ENABLE);
			
			break;
		}
		
		case 10:
		{

		}
		
		// *****************************************
		// KI Verteiler
		// *****************************************
		case 20:
		{
			uint8_t i, Aufgabe_gefunden = 0;
			int8_t remainingTime = 100;
			
			// ************************************************************
			// Aufgabe mit der höchsten Priorität fürs Punkten suchen
			// ************************************************************
			if(spielZeit >= KI_DISABLE_TIME)
			{
				KI_maxPriority = 0;

				// *********************************************************
				// Die Aufgabe mit der höchsten Priorität suchen
				// *********************************************************
				for(i = 1; i < MAX_KI_TASKS; i++)
				{
					if((KI_Task[i].Priority >= KI_Task[KI_maxPriority].Priority) && (KI_Task[i].Priority > 0))
					{
						// ***************************************************
						// Überprüfen, ob die Aufgabe noch nicht erledigt ist
						// ***************************************************
						
						// Restzeit für Ausführung der Task berechnen
						// -10s => Zur Endposition fahren (keine timeToTask bei der Task zu Endpos fahren!!)
						//remainingTime = spielZeit - 5 - KI_Task[i].taskTime - KI_Task[i].timeToTask;
						
						if((KI_Task[i].Status == OPEN) && (remainingTime >= 0))
						{
							KI_maxPriority = i;
							Aufgabe_gefunden = 1;
						}
						// 						// Wenn weniger als 10s Spielzeit ist, dann auf Endpos fahren
						// 						else if(spielZeit < 10 && ((KI_Task[48].Status != DONE && SpielFarbe == PURPLE) || (KI_Task[50].Status != DONE && SpielFarbe == YELLOW)))
						// 						{
						// 							KI_maxPriority = (RobotType_RAM == MASTER_ROBOT) ? 48 : 50;
						// 							Aufgabe_gefunden = 1;
						// 						}
					}
				}
				// Wenn eine Aufgabe gefunden ist, wird auf diese Aufgabe gesprungen
				if(Aufgabe_gefunden == 1)
				{
					KI_State = KI_Task[KI_maxPriority].Start;

					sprintf(text1, "# 20: -> %d,   \r\n*", KI_maxPriority);
					writeString_usart(&WIFI_IF, text1);

					return(ENABLE);
				}
				else
				{
					//sprintf(text1, "# 20: UIUIUI leider nichts gefunden\r\n*");
					//writeString_usart(&WIFI_IF, text1);
					
				}
			}
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******	Task 1000 - Pflanzen an der Position 1500/500
		// ******
		// ********************************************************************
		// ********************************************************************
		//SET_PIN(LED_PORT1, LED3);
		//TOGGLE_PIN(LED_PORT2, LED4);
		
		case 1000:
		{
			point_t start, ziel, plants;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			// Position where the plants stand
			plants.Xpos = 1500;
			plants.Ypos = 500;
			
			// Middle Point to move correctly to the plant from the correct Quadrant
			ziel = AddDifferencePerQuadrant(start,plants);
			
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				// Set Plants-Position as 3rd Position to move to
				wp_KI[wpNbr].Xpos = plants.Xpos;
				wp_KI[wpNbr].Ypos = plants.Ypos;
				wpNbr++;
				cmd_Drive(0,0,500,0,0,0,0,0,0,ON,wp_KI,wpNbr);
				KI_State = 1010;
			}
			
			break;
		}

		case 1010:
		{
			
			
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					
					KI_State = 1020;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 1000;
					break;
				}
			}
			break;
		}
		
		case 1020:
		{
			//Wait specific time
			SET_CYCLE(KI_TASKNBR, 2000);
			KI_Task[1].Status = DONE;
			// Jump to KI-Verteiler-State
			KI_State = 20;
			
			break;
		}

		
		


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 2000 - Pflanzen an der Position 1000/700
		// ******
		// ********************************************************************
		// ********************************************************************
		case 2000:
		{
			point_t start, ziel, plants;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			// Position where the plants stand
			plants.Xpos = 1000;
			plants.Ypos = 700;
			
			// Middle Point to move correctly to the plant from the correct Quadrant
			ziel = AddDifferencePerQuadrant(start,plants);
			
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				// Set Plants-Position as 3rd Position to move to
				wp_KI[wpNbr].Xpos = plants.Xpos;
				wp_KI[wpNbr].Ypos = plants.Ypos;
				wpNbr++;
				cmd_Drive(0,0,500,0,0,0,0,0,0,ON,wp_KI,wpNbr);
				KI_State = 2010;
			}
			
			break;
		}
		
		case 2010:
		{
			
			
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 2020;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 2000;
					break;
				}
			}
			break;
		}
		
		case 2020:
		{
			//Wait specific time
			SET_CYCLE(KI_TASKNBR, 2000);
			KI_Task[2].Status = DONE;
			// Jump to KI-Verteiler-State
			KI_State = 20;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 3000 - Pflanzen an der Position 1000/1300
		// ******
		// ********************************************************************
		// ********************************************************************
		case 3000:
		{
			point_t start, ziel, plants;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			// Position where the plants stand
			plants.Xpos = 1000;
			plants.Ypos = 1300;
			
			// Middle Point to move correctly to the plant from the correct Quadrant
			ziel = AddDifferencePerQuadrant(start,plants);
			
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				// Set Plants-Position as 3rd Position to move to
				wp_KI[wpNbr].Xpos = plants.Xpos;
				wp_KI[wpNbr].Ypos = plants.Ypos;
				wpNbr++;
				cmd_Drive(0,0,500,0,0,0,0,0,0,ON,wp_KI,wpNbr);
				KI_State = 3010;
			}
			
			break;
		}
		
		case 3010:
		{
			
			
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 3020;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 3000;
					break;
				}
			}
			break;
		}
		
		case 3020:
		{
			//Wait specific time
			SET_CYCLE(KI_TASKNBR, 2000);
			KI_Task[3].Status = DONE;
			// Jump to KI-Verteiler-State
			KI_State = 20;
			
			break;
		}
		
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 4000 - Pflanzen an der Position 1500/1500
		// ******
		// ********************************************************************
		// ********************************************************************
		case 4000:
		{
			point_t start, ziel, plants;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			// Position where the plants stand
			plants.Xpos = 1500;
			plants.Ypos = 1500;
			
			// Middle Point to move correctly to the plant from the correct Quadrant
			ziel = AddDifferencePerQuadrant(start,plants);
			
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				// Set Plants-Position as 3rd Position to move to
				wp_KI[wpNbr].Xpos = plants.Xpos;
				wp_KI[wpNbr].Ypos = plants.Ypos;
				wpNbr++;
				cmd_Drive(0,0,500,0,0,0,0,0,0,ON,wp_KI,wpNbr);
				KI_State = 4010;
			}
			
			break;
		}
		
		case 4010:
		{
			
			
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 4020;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 4000;
					break;
				}
			}
			break;
		}
		
		case 4020:
		{
			//Wait specific time
			SET_CYCLE(KI_TASKNBR, 2000);
			KI_Task[4].Status = DONE;
			// Jump to KI-Verteiler-State
			KI_State = 20;
			
			break;
		}
		
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 5000 - Pflanzen an der Position 2000/1300
		// ******
		// ********************************************************************
		// ********************************************************************
		case 5000:
		{
			point_t start, ziel, plants;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			// Position where the plants stand
			plants.Xpos = 2000;
			plants.Ypos = 1300;
			
			// Middle Point to move correctly to the plant from the correct Quadrant
			ziel = AddDifferencePerQuadrant(start,plants);
			
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				// Set Plants-Position as 3rd Position to move to
				wp_KI[wpNbr].Xpos = plants.Xpos;
				wp_KI[wpNbr].Ypos = plants.Ypos;
				wpNbr++;
				cmd_Drive(0,0,500,0,0,0,0,0,0,ON,wp_KI,wpNbr);
				KI_State = 5010;
			}
			
			break;
		}
		
		case 5010:
		{
			
			
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 5020;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 5000;
					break;
				}
			}
			break;
		}
		
		case 5020:
		{
			//Wait specific time
			SET_CYCLE(KI_TASKNBR, 2000);
			KI_Task[5].Status = DONE;
			// Jump to KI-Verteiler-State
			KI_State = 20;
			
			break;
		}
		
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 6000 - Pflanzen an der Position 2000/700
		// ******
		// ********************************************************************
		// ********************************************************************
		case 6000:
		{
			point_t start, ziel, plants;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			// Position where the plants stand
			plants.Xpos = 2000;
			plants.Ypos = 700;
			
			// Middle Point to move correctly to the plant from the correct Quadrant
			ziel = AddDifferencePerQuadrant(start,plants);
			
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				// Set Plants-Position as 3rd Position to move to
				wp_KI[wpNbr].Xpos = plants.Xpos;
				wp_KI[wpNbr].Ypos = plants.Ypos;
				wpNbr++;
				cmd_Drive(0,0,500,0,0,0,0,0,0,ON,wp_KI,wpNbr);
				KI_State = 6010;
			}
			
			break;
		}
		
		case 6010:
		{
			
			
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 6020;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 6000;
					break;
				}
			}
			break;
		}
		
		case 6020:
		{
			//Wait specific time
			SET_CYCLE(KI_TASKNBR, 2000);
			KI_Task[6].Status = DONE;
			// Jump to KI-Verteiler-State
			KI_State = 20;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 7000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 7000:
		{
			KI_State = 7000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 8000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 8000:
		{
			KI_State = 8000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 9000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 9000:
		{
			KI_State = 9000;
			
			
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 10000 - Eintscheidung für Ablage auf der linken Seite
		// ******
		// ********************************************************************
		// ********************************************************************
		case 10000:
		{
			KI_State = 10000;

			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 11000 - Ablage im blauen Planter neben der Startzone
		// ******                der Ladybugs
		// ******
		// ********************************************************************
		// ********************************************************************
		case 11000:
		{
			KI_State = 11000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 12000 - Ablage im Feld L1 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 12000:
		{
			KI_State = 12000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 13000 - Ablage im Planter zwische Feld L1 und L2 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 13000:
		{
			KI_State = 13000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 14000 - Ablage im Feld L2 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 14000:
		{
			KI_State = 14000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 15000 - Ablage im Planter zwische Feld L2 und L3 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 15000:
		{
			KI_State = 15000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 16000 - Ablage im Feld L3 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 16000:
		{
			KI_State = 16000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 17000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 17000:
		{
			KI_State = 17000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 18000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 18000:
		{
			KI_State = 18000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 19000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 19000:
		{
			KI_State = 19000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 20000 - Eintscheidung für Ablage auf der rechten Seite
		// ******
		// ********************************************************************
		// ********************************************************************
		case 20000:
		{
			KI_State = 20000;

			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 21000 - Ablage im gelben Planter neben der Startzone
		// ******                der Ladybugs
		// ******
		// ********************************************************************
		// ********************************************************************
		case 21000:
		{
			KI_State = 21000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 22000 - Ablage im Feld R1 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 22000:
		{
			KI_State = 22000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 23000 - Ablage im Planter zwische Feld R1 und R2 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 23000:
		{
			KI_State = 23000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 24000 - Ablage im Feld R2 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 24000:
		{
			KI_State = 24000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 25000 - Ablage im Planter zwische Feld R2 und R3 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 25000:
		{
			KI_State = 25000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 26000 - Ablage im Feld R3 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 26000:
		{
			KI_State = 26000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 27000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 27000:
		{
			KI_State = 27000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 28000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 28000:
		{
			KI_State = 28000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 29000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 29000:
		{
			KI_State = 29000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 30000 - Solarpanele (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 30000:
		{
			KI_State = 30000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 31000 - Solarpanele (beide)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 31000:
		{
			KI_State = 31000;
			
			break;
		}
		
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 32000 - Solarpanele (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 32000:
		{
			KI_State = 32000;

			break;
			
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 33000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 33000:
		{
			KI_State = 33000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 34000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 34000:
		{
			KI_State = 34000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 35000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 35000:
		{
			KI_State = 35000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 36000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 36000:
		{
			KI_State = 36000;

			break;
		}
		
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 37000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 37000:
		{
			KI_State = 37000;

			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 38000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 38000:
		{
			KI_State = 38000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 39000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 39000:
		{
			KI_State = 39000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 40000 - Eintscheidung für Stehlen auf der linken Seite
		// ******
		// ********************************************************************
		// ********************************************************************
		case 40000:
		{
			KI_State = 40000;

			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 41000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 41000:
		{
			KI_State = 41000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 42000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 42000:
		{
			KI_State = 42000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 43000 - Stehlen aus Planter zwische Feld L1 und L2 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 43000:
		{
			KI_State = 43000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 44000 - Stehlen aus Feld L2 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 44000:
		{
			KI_State = 44000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 45000 - Stehlen aus Planter zwische Feld L2 und L3 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 45000:
		{
			KI_State = 45000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 46000 - Stehlen aus Feld L3 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 46000:
		{
			KI_State = 46000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 47000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 47000:
		{
			KI_State = 47000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 48000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 48000:
		{
			KI_State = 48000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 49000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 49000:
		{
			KI_State = 49000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 50000 - Eintscheidung für Stehlen auf der rechten Seite
		// ******
		// ********************************************************************
		// ********************************************************************
		case 50000:
		{
			KI_State = 50000;

			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 51000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 51000:
		{
			KI_State = 51000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 52000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 52000:
		{
			KI_State = 52000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 53000 - Stehlen aus Planter zwische Feld R1 und R2 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 53000:
		{
			KI_State = 53000;
			
			break;
		}
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 54000 - Stehlen aus Feld R2 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 54000:
		{
			KI_State = 54000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 55000 - Stehlen aus Planter zwische Feld R2 und R3 (blau)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 55000:
		{
			KI_State = 55000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 56000 - Stehlen aus Feld R3 (gelb)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 56000:
		{
			KI_State = 56000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 57000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 57000:
		{
			KI_State = 57000;
			
			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 58000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 58000:
		{
			KI_State = 58000;
			
			break;
		}
		

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 59000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 59000:
		{
			KI_State = 59000;
			
			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 60000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 60000:
		{
			KI_State = 60000;

			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 61000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 61000:
		{
			KI_State = 61000;

			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 62000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 62000:
		{
			KI_State = 62000;

			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 63000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 63000:
		{
			KI_State = 63000;

			break;
		}

		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 64000 - Keine Aufgabe
		// ******
		// ********************************************************************
		// ********************************************************************
		case 64000:
		{
			KI_State = 64000;

			break;
		}


		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 65000 - PERIMETER
		// ******
		// ********************************************************************
		// ********************************************************************
		case 65000:
		{
			KI_State = 65000;
			
			break;
		}
	}

	return(CYCLE);
}


/**************************************************************************
***   FUNCTIONNAME:        KiWatchTask                                  ***
***   FUNCTION:            überwacht ob Aufgaben bereits erledigt sind  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiWatchTask(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_CYCLE(KI_WATCH_TASKNBR, 100);


	
	return (CYCLE);
}

/**************************************************************************
***   FUNCTIONNAME:        KiWatchRobotPositionTask                     ***
***   FUNCTION:            Sendet die Daten an den Nebenroboter			***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiWatchRobotPositionTask(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_CYCLE(KI_WATCH_ROBOT_POS_TASKNBR, 100);
	
	// ********************************************************
	// Status2SmallRobot
	// ********************************************************
	// 0000 | 0001 ... Startschnur-Status
	// 0000 | 0010 ... Spielfeld-Farbe
	// 0000 | 0100 ... xxx
	// 0000 | 1000 ... xxx
	// 0001 | 0000 ... xxx
	// 0010 | 0000 ... xxx
	// 0100 | 0000 ... xxx
	// 1000 | 0000 ... xxx
	// ********************************************************
	
	// 	// Startschnur-Status
	// 	if(START_SCHNU)R
	// 	Status2SmallRobot |= 0x01;
	// 	else
	// 	Status2SmallRobot &= ~0x01;
	//
	// 	// Spielfeld-Farbe
	// 	if(SpielFarbe == GELB)
	// 	Status2SmallRobot |= 0x02;
	// 	else
	// 	Status2SmallRobot &= ~0x02;


	
	// Send message to small robot
	//	Send2SmallRobot();
	
	return(CYCLE);
}

// ****************************************************
// Dot2D, Norm2D, AngleToXAxis2D from path_math.c (µC2)
// ****************************************************
float Dot2D(float* vectorA, float* vectorB)
{
	return (vectorA[X_KOR] * vectorB[X_KOR] + vectorA[Y_KOR] * vectorB[Y_KOR]);
}

float Norm2D(float* vector)
{
	return ((float)(sqrt(pow(vector[X_KOR], 2.0) + pow(vector[Y_KOR], 2.0))));
}

float AngleToXAxis2D(float* vector)
{
	float phi;
	float e_x[2] = {1, 0};
	float delta_x, delta_y;
	
	// Calculating phi_max -> Angle between the X-axis and the vector: Initial point <-> Destination point
	phi = acos(Dot2D(e_x, vector) / (Norm2D(e_x) * Norm2D(vector)));
	
	delta_y = vector[Y_KOR] - e_x[Y_KOR];
	delta_x = vector[X_KOR] - e_x[X_KOR];
	
	// If necessary, correction of the angle
	if((delta_y < 0.0) && (delta_x < 0.0))
	phi = 2 * M_PI - phi;
	if((delta_y < 0.0) && (delta_x > 0.0))
	phi = 2 * M_PI - phi;

	return (phi);
}

// ****************************************************
// GetPx, GetPy for shooting the mammoth
// ****************************************************
int16_t GetPx(int16_t Mx, int16_t My)
{
	int16_t Px = 0;
	float R = 11025.0;		// Rx = 10,5 cm
	
	float Mx_x = (float)(Mx - xPos);
	float My_y = (float)(My - yPos);
	float Mx_x2 = pow(Mx_x, 2.0);
	float My_y2 = pow(My_y, 2.0);
	
	float result1 = R * Mx_x;
	float result2 = (float)xPos * (Mx_x2 + My_y2);
	float result3 = sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
	float result4 = Mx_x2 + My_y2;
	
	Px = (int16_t)((result1 + result2 + result3) / result4);
	
	return(Px);
}

int16_t GetPy(int16_t Mx, int16_t My)
{
	int16_t Py = 0;
	float R = 11025.0;		// Rx = 10,5 cm
	
	float Mx_x = (float)(Mx - xPos);
	float My_y = (float)(My - yPos);
	float Mx_x2 = pow(Mx_x, 2.0);
	float My_y2 = pow(My_y, 2.0);
	
	float result1 = R * My_y2;
	float result2 = Mx * sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
	float result3 = (float)xPos * sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
	float result4 = (Mx_x2 + My_y2) * My_y * (float)yPos;
	float result5 = (Mx_x2 + My_y2) * My_y;
	
	Py = (int16_t)((result1 - result2 + result3 + result4) / result5);
	
	return(Py);
}
