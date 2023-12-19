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
#include "logger.h"
#include "ki_helper.h"
#include "spielZeit.h"


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
	
	//Init Variables
	KI_State = 0;
	OldKI_State = 0;
	StateOfGame = GetPlants;
	HomePositionReached=0;
	PlantsInRobot = 0;
	ParkedPlants = 0;
	OpenPlants = 6;
	motionFailureCount = 0;
	nReachableCnt1000 = 0;
	nReachableCnt2000 = 0;
	nReachableCnt3000 = 0;
	nReachableCnt4000 = 0;
	nReachableCnt5000 = 0;
	nReachableCnt6000 = 0;
	planedPlants = 0;
	ConfigPlanter = ConfigPlanter_Nextion;
	ConfigStehlen = ConfigStehlen_Nextion;
	velocity = STANDARD_VELOCITY;
	
	//Set Plant Positions
	Plant1000.Xpos = 1500;
	Plant1000.Ypos = 500;
	Plant2000.Xpos = 1000;
	Plant2000.Ypos = 700;
	Plant3000.Xpos = 1000;
	Plant3000.Ypos = 1300;
	Plant4000.Xpos = 1500;
	Plant4000.Ypos = 1500;
	Plant5000.Xpos = 2000;
	Plant5000.Ypos = 1300;
	Plant6000.Xpos = 2000;
	Plant6000.Ypos = 700;
	
	//Set Park Positions
	PlanterL1.Xpos = 2700;
	PlanterL1.Ypos = 612;
	PlanterL2.Xpos = 2700;
	PlanterL2.Ypos = 1387;
	FieldL1.Xpos = 2700;
	FieldL1.Ypos = 300;
	FieldL3.Xpos = 2700;
	FieldL3.Ypos = 1700;
	PlanterMidleBlue.Xpos = 2237;
	PlanterMidleBlue.Ypos = 300;
	SolarPanelsBlue.Xpos = 2500;
	SolarPanelsBlue.Ypos = 1700;
	
	PlanterR1.Xpos = 300;
	PlanterR1.Ypos = 612;
	PlanterR2.Xpos = 300;
	PlanterR2.Ypos = 1387;
	FieldR1.Xpos = 300;
	FieldR1.Ypos = 300;
	FieldR3.Xpos = 300;
	FieldR3.Ypos = 1700;
	PlanterMidleYellow.Xpos = 762;
	PlanterMidleYellow.Ypos = 300;
	SolarPanelsYellow.Xpos = 500;
	SolarPanelsYellow.Ypos = 1700;
	
	SolarPanelsMiddle.Xpos = 1500;
	SolarPanelsMiddle.Ypos = 1700;

	//Set Gamecolors
	if(SpielFarbe_Nextion == BLUE_L1 || SpielFarbe_Nextion == BLUE_L3 || SpielFarbe_Nextion == BLUE_R2)
	{
		SpielFarbe = BLUE;
	}
	if(SpielFarbe_Nextion == Yellow_L2 || SpielFarbe_Nextion == Yellow_R1 || SpielFarbe_Nextion == Yellow_R3)
	{
		SpielFarbe = Yellow;
	}
	// ********************************************************************
	//Set Obstacles
	// ********************************************************************
	for (uint8_t i = 0; i < PATH_STATIC_OBSTICAL_LIST_LENGHT; i++)
	{
		PATH_DISABLE_OBSTACLE(i);
	}
	/* Start-Zone der Ladybugs */
	PATH_SetStaticObstacle(0, 950, 0, 2050, 250);
	PATH_ENABLE_OBSTACLE(0);
	
	///* geschützte Start-Zone blauer Roboter */
	if(SpielFarbe == Yellow)
	{
		PATH_SetStaticObstacle(1, 2500, 0, 3000, 500);
		PATH_ENABLE_OBSTACLE(1);
	}

	//Plant 1000
	PATH_SetStaticObstacle(3, 1250, 250, 1750, 750);
	//Plant 2000
	PATH_SetStaticObstacle(4, 750, 450, 1250, 950);
	//Plant 3000
	PATH_SetStaticObstacle(5, 750, 1050, 1250, 1550);
	//Plant 4000
	PATH_SetStaticObstacle(6, 1250, 1250, 1750, 1750);
	//Plant 5000
	PATH_SetStaticObstacle(7, 1750, 1050, 2250, 1550);
	//Plant 6000
	PATH_SetStaticObstacle(8, 1750, 450, 2250, 950);
	///* geschützte Start-Zone gelber Roboter */
	if(SpielFarbe == BLUE)
	{
		PATH_SetStaticObstacle(2, 0, 0, 500, 500);
		PATH_ENABLE_OBSTACLE(2);
	}
	
	// ********************************************************************
	/* alle Tasks initialisieren und deaktivieren */
	// ********************************************************************
	for (uint8_t i = 0; i<MAX_KI_TASKS;i++)
	{
		KI_Task[i].Status = DONE;
		KI_Task[i].Start = (uint16_t)i*1000;
		KI_Task[i].Priority = 0;
	}

	
	// ********************************************************************
	// ******   A U F G A B E N   I N I T I A L I S I E R E N
	// ********************************************************************

	////////////////////Initialize Plant Tasks////////////////////

	if (Strategie != NEXTION_STRATEGY_HOMOLOGATION)
	{
		switch (Strategie)
		{
			// Strategy M1 ==> R1 ==> R2 bzw. M1 ==> L1 ==> L2
			case NEXTION_STRATEGY_L1_L2_P1: case NEXTION_STRATEGY_R1_R2_P1:
			{
				
				KI_Task[1].Priority = 99;
				KI_Task[2].Priority = 98;
				KI_Task[3].Priority = 97;
				
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;

				ConfigPlanter = 1;
				break;
			}
			//Strategy M1 ==> M2 ==> R2  bzw. M1 ==> M2 ==> L2
			case NEXTION_STRATEGY_L1_L2_A1: case NEXTION_STRATEGY_R1_R2_A1:
			{
				KI_Task[1].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[3].Priority = 97;
				
				KI_Task[2].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
			//Strategy L1 ==> R1 ==> R2  bzw. R1 ==> L1 ==> L2
			case NEXTION_STRATEGY_L1_L2_R1: case NEXTION_STRATEGY_R1_R2_R1:
			{
				KI_Task[6].Priority = 99;
				KI_Task[2].Priority = 98;
				KI_Task[3].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
			//Strategy L1 ==> M1 ==> R1  bzw. R1 ==> M1 ==> L1
			case NEXTION_STRATEGY_L1_L2_R2: case NEXTION_STRATEGY_L1_R3_R2: case NEXTION_STRATEGY_R1_R2_R2: case NEXTION_STRATEGY_R1_L3_R2:
			{
				KI_Task[6].Priority = 99;
				KI_Task[1].Priority = 98;
				KI_Task[2].Priority = 97;
				
				KI_Task[3].Priority = 90;
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
			//Strategy L1 ==> M1 ==> R2  bzw. R1 ==> M1 ==> L2
			case NEXTION_STRATEGY_L1_L2_R3: case NEXTION_STRATEGY_L1_R1_R2: case NEXTION_STRATEGY_R1_R2_R3: case NEXTION_STRATEGY_R1_L1_R2:
			{
				KI_Task[6].Priority = 99;
				KI_Task[1].Priority = 98;
				KI_Task[3].Priority = 97;
				
				KI_Task[2].Priority = 90;
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
			//Strategy L1 ==> M2 ==> R2  bzw. R1 ==> M2 ==> L2
			case NEXTION_STRATEGY_L1_L2_R4: case NEXTION_STRATEGY_L1_R1_R1: case NEXTION_STRATEGY_R1_R2_R4: case NEXTION_STRATEGY_R1_L1_R1:
			{
				KI_Task[6].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[3].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[2].Priority = 90;
				KI_Task[5].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
			//Strategy L1 ==> L2  bzw. R1 ==> R2
			case NEXTION_STRATEGY_L1_R1_P1: case NEXTION_STRATEGY_L1_R3_P1: case NEXTION_STRATEGY_R1_L1_P1: case NEXTION_STRATEGY_R1_L3_P1:
			{
				KI_Task[6].Priority = 99;
				KI_Task[5].Priority = 98;
				
				KI_Task[1].Priority = 90;
				KI_Task[2].Priority = 90;
				KI_Task[3].Priority = 90;
				KI_Task[4].Priority = 90;
				
				break;
			}
			//Strategy L1 ==> M2 ==> L2  bzw. R1 ==> M2 ==> R2
			case NEXTION_STRATEGY_L1_R1_P2: case NEXTION_STRATEGY_R1_L1_P2:
			{
				KI_Task[6].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[5].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[2].Priority = 90;
				KI_Task[3].Priority = 90;
				break;
			}
			//Strategy L1 ==> L2 ==> M2  bzw. R1 ==> R2 ==> M2
			case NEXTION_STRATEGY_L1_R1_P3: case NEXTION_STRATEGY_L1_R3_R1: case NEXTION_STRATEGY_R1_L1_P3: case NEXTION_STRATEGY_R1_L3_R1:
			{
				KI_Task[6].Priority = 99;
				KI_Task[5].Priority = 98;
				KI_Task[4].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[2].Priority = 90;
				KI_Task[3].Priority = 90;
				
				break;
			}
			//Strategy M1 ==> M2 ==> L2  bzw. M1 ==> M2 ==> R2
			case NEXTION_STRATEGY_L1_R1_A1: case NEXTION_STRATEGY_L1_R3_A2: case NEXTION_STRATEGY_R1_L1_A1: case NEXTION_STRATEGY_R1_L3_A2:
			{
				KI_Task[1].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[5].Priority = 97;
				
				KI_Task[2].Priority = 90;
				KI_Task[3].Priority = 90;
				KI_Task[6].Priority = 90;
				
				break;
			}
			//Strategy M1 ==> M2 ==> L1  bzw. M1 ==> M2 ==> R1
			case NEXTION_STRATEGY_L1_R1_A2: case NEXTION_STRATEGY_L1_R3_A1: case NEXTION_STRATEGY_R1_L1_A2: case NEXTION_STRATEGY_R1_L3_A1:
			{
				KI_Task[1].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[6].Priority = 97;
				
				KI_Task[2].Priority = 90;
				KI_Task[3].Priority = 90;
				KI_Task[5].Priority = 90;
				
				break;
			}
			//Strategy M1 ==> L2 ==> L1  bzw. M1 ==> R2 ==> R1
			case NEXTION_STRATEGY_L1_R3_P2: case NEXTION_STRATEGY_R1_L3_P2:
			{
				KI_Task[1].Priority = 99;
				KI_Task[5].Priority = 98;
				KI_Task[6].Priority = 97;
				
				KI_Task[2].Priority = 90;
				KI_Task[3].Priority = 90;
				KI_Task[4].Priority = 90;
				
				break;
			}
			//Strategy R2 ==> M2 ==> L1  bzw. L2 ==> M2 ==> R1
			case NEXTION_STRATEGY_R2_R1_P1: case NEXTION_STRATEGY_R2_L2_R3: case NEXTION_STRATEGY_L2_L1_P1: case NEXTION_STRATEGY_L2_R2_R3:
			{
				KI_Task[3].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[6].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[2].Priority = 90;
				KI_Task[5].Priority = 90;
				
				break;
			}
			//Strategy R2 ==> M2 ==> L2  bzw. L2 ==> M2 ==> R2
			case NEXTION_STRATEGY_R2_R1_P2: case NEXTION_STRATEGY_L2_L1_P2:
			{
				KI_Task[3].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[5].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[2].Priority = 90;
				KI_Task[6].Priority = 90;
				
				break;
			}
			//Strategy R2 ==> M1 ==> L1  bzw. L2 ==> M1 ==> R1
			case NEXTION_STRATEGY_R2_R1_A1: case NEXTION_STRATEGY_R2_L2_R2: case NEXTION_STRATEGY_L2_L1_A1: case NEXTION_STRATEGY_L2_R2_R2:
			{
				KI_Task[3].Priority = 99;
				KI_Task[1].Priority = 98;
				KI_Task[6].Priority = 97;
				
				KI_Task[2].Priority = 90;
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				
				break;
			}
			//Strategy R2 ==> M2 ==> M1  bzw. L2 ==> M2 ==> M1
			case NEXTION_STRATEGY_R2_R1_A2: case NEXTION_STRATEGY_R2_L2_A2: case NEXTION_STRATEGY_L2_L1_A2: case NEXTION_STRATEGY_L2_R2_A2:
			{
				KI_Task[3].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[1].Priority = 97;
				
				KI_Task[2].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;
				
				break;
			}
			//Strategy R1 ==> M1 ==> L1  bzw. L1 ==> M1 ==> R1
			case NEXTION_STRATEGY_R2_R3_P1: case NEXTION_STRATEGY_R2_L2_R1: case NEXTION_STRATEGY_L2_L3_P1: case NEXTION_STRATEGY_L2_R2_R1:
			{
				KI_Task[2].Priority = 99;
				KI_Task[1].Priority = 98;
				KI_Task[6].Priority = 97;
				
				KI_Task[3].Priority = 90;
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				
				break;
			}
			//Strategy R1 ==> M1 ==> M2  bzw. L1 ==> M1 ==> M2
			case NEXTION_STRATEGY_R2_R3_A1:  case NEXTION_STRATEGY_L2_L3_A1:
			{
				KI_Task[2].Priority = 99;
				KI_Task[1].Priority = 98;
				KI_Task[4].Priority = 97;
				
				KI_Task[3].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;
				
				break;
			}
			//Strategy R1 ==> M2 ==> L2  bzw. L1 ==> M2 ==> R2
			case NEXTION_STRATEGY_R2_R3_R1:  case NEXTION_STRATEGY_L2_L3_R1:
			{
				KI_Task[2].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[5].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[3].Priority = 90;
				KI_Task[6].Priority = 90;
				
				break;
			}
			//Strategy R1 ==> M2 ==> L1  bzw. L1 ==> M2 ==> R1
			case NEXTION_STRATEGY_R2_R3_R2:  case NEXTION_STRATEGY_L2_L3_R2:
			{
				KI_Task[2].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[6].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[3].Priority = 90;
				KI_Task[5].Priority = 90;
				
				break;
			}
			//Strategy R1 ==> R2 bzw. L1 ==> L2
			case NEXTION_STRATEGY_R2_L2_P1:  case NEXTION_STRATEGY_L2_R2_P1:
			{
				KI_Task[2].Priority = 99;
				KI_Task[3].Priority = 98;
				
				KI_Task[1].Priority = 90;
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
			//Strategy R2 ==> R1 bzw. L2 ==> L1
			case NEXTION_STRATEGY_R2_L2_P2:  case NEXTION_STRATEGY_L2_R2_P2:
			{
				KI_Task[3].Priority = 99;
				KI_Task[2].Priority = 98;
				
				KI_Task[1].Priority = 90;
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
			//Strategy R2 ==> R1 ==> M1
			case NEXTION_STRATEGY_R2_L2_A1:  case NEXTION_STRATEGY_L2_R2_A1:
			{
				KI_Task[3].Priority = 99;
				KI_Task[2].Priority = 98;
				KI_Task[1].Priority = 97;
				
				KI_Task[4].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;
				
				break;
			}
			//Strategy R1 ==> M2 ==> R2
			case NEXTION_STRATEGY_R2_L2_A3:  case NEXTION_STRATEGY_L2_R2_A3:
			{
				KI_Task[2].Priority = 99;
				KI_Task[4].Priority = 98;
				KI_Task[3].Priority = 97;
				
				KI_Task[1].Priority = 90;
				KI_Task[5].Priority = 90;
				KI_Task[6].Priority = 90;
				
				ConfigPlanter = 1;
				break;
			}
		}
		// *******************************************
		// Set State of Parking Positions and Solar Panels and Change Plants Prios to Color Yellow
		// *******************************************
		if(SpielFarbe == Yellow)
		{
			ChangePrioToYellow();
			
			//Parking Positions
			KI_Task[11].Status = LOCKED;
			KI_Task[12].Status = LOCKED;
			KI_Task[13].Status = LOCKED;
			KI_Task[16].Status = LOCKED;
			KI_Task[24].Status = LOCKED;
			KI_Task[25].Status = LOCKED;
			
			KI_Task[14].Status = LOCKED;
			KI_Task[21].Status = OPEN;
			KI_Task[22].Status = OPEN;
			KI_Task[23].Status = OPEN;
			KI_Task[26].Status = PENDING;
			
			if(ConfigPlanter == 1)
			{
				KI_Task[15].Status = OPEN;
			}
			else
			{
				KI_Task[15].Status = PENDING;
			}
			
			//Solar Panels
			KI_Task[30].Status = LOCKED;
			KI_Task[31].Status = OPEN;
			KI_Task[32].Status = OPEN;
		}

		if(SpielFarbe == BLUE)
		{
			//Parking Positions
			KI_Task[14].Status = LOCKED;
			KI_Task[15].Status = LOCKED;
			KI_Task[21].Status = LOCKED;
			KI_Task[22].Status = LOCKED;
			KI_Task[23].Status = LOCKED;
			KI_Task[26].Status = LOCKED;
			
			KI_Task[11].Status = OPEN;
			KI_Task[12].Status = OPEN;
			KI_Task[13].Status = OPEN;
			KI_Task[16].Status = PENDING;
			KI_Task[24].Status = LOCKED;
			
			if(ConfigPlanter == 1)
			{
				KI_Task[25].Status = OPEN;
			}
			else
			{
				KI_Task[25].Status = PENDING;
			}
			
			//Solar Panels
			KI_Task[30].Status = OPEN;
			KI_Task[31].Status = OPEN;
			KI_Task[32].Status = LOCKED;
		}
		
		//Prios Solar Panels
		KI_Task[30].Priority = 78;
		KI_Task[32].Priority = 78;
		KI_Task[31].Priority = 79;
		
		//Drive Home Task
		KI_Task[60].Status = OPEN;
	}



	else
	{
		// *******************************************
		// Strategy: STRATEGY_HOMOLOGATION
		// *******************************************
	}

	// *******************************************
	// Set State of First Plant to Open if in pre-adjusted plan or to pending if not
	// *******************************************
	for (int i=1; i<7; i++)
	{
		if(KI_Task[i].Priority == 99)
		{
			KI_Task[i].Status = OPEN;
		}
		else
		{
			KI_Task[i].Status = PENDING;
		}
		if (KI_Task[i].Priority > 90)
		{
			planedPlants++;
		}
	}

	// *******************************************
	// Set Plant as Obstacle if not used
	// *******************************************
	ActivatePlantAsObstacle();

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
***   FUNCTIONNAME:        KiTask                                       ***
***   FUNCTION:            KI                                           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiTask(void)
{
	// Cycle per default to 100 ms
	SET_CYCLE(KI_TASKNBR, 100);
	
	char text1[300];
	
	//Position of enemy and my own robot
	uint8_t index [] = {1,2,3,4,5,6};
	robot_t enemySend [5];
	for (uint8_t i = 0; i < 5; i++)
	{
		enemySend[i].Xpos = ((enemyRobot[i].Xpos == 10000) ? 9999.0 : enemyRobot[i].Xpos);
		enemySend[i].Ypos = ((enemyRobot[i].Ypos == 10000) ? 9999.0 : enemyRobot[i].Ypos);
	}
	uint16_t XPosition[] = { xPos, enemySend[0].Xpos,enemySend[1].Xpos,enemySend[2].Xpos,enemySend[3].Xpos,enemySend[4].Xpos};
	uint16_t YPosition[] = {yPos, enemySend[0].Ypos,enemySend[1].Ypos,enemySend[2].Ypos,enemySend[3].Ypos,enemySend[4].Ypos};
	SendPlaygroundPositionMessage(index, XPosition,YPosition,6);
	
	//State and Priority of Tasks
	uint8_t index1 [] = {1,2,3,4,5,6,11,12,13,15,16,21,22,23,25,26,30,31,32};
	uint8_t state[] = {	KI_Task[1].Status,KI_Task[2].Status,KI_Task[3].Status,KI_Task[4].Status,KI_Task[5].Status,KI_Task[6].Status,
		KI_Task[11].Status,KI_Task[12].Status,KI_Task[13].Status,KI_Task[15].Status,KI_Task[16].Status,
	KI_Task[21].Status,KI_Task[22].Status,KI_Task[23].Status,KI_Task[25].Status,KI_Task[26].Status,KI_Task[30].Status,KI_Task[31].Status,KI_Task[32].Status};
	
	uint8_t priority[] = {	KI_Task[1].Priority,KI_Task[2].Priority,KI_Task[3].Priority,KI_Task[4].Priority,KI_Task[5].Priority,KI_Task[6].Priority,
		KI_Task[11].Priority,KI_Task[12].Priority,KI_Task[13].Priority,KI_Task[15].Priority,KI_Task[16].Priority,
	KI_Task[21].Priority,KI_Task[22].Priority,KI_Task[23].Priority,KI_Task[25].Priority,KI_Task[26].Priority,KI_Task[30].Priority,KI_Task[31].Priority,KI_Task[32].Priority};
	SendTaskInfo(index1,state,priority);


	
	// ********************************************************************
	// ******   S P I E L Z E I T U E B E R W A C H U N G
	// ********************************************************************

	if(spielZeit < 10)
	{
		// Hier alle notwendigen Systeme deaktivieren
		cmd_SetDigitalOut(DO_LED1_NBR, 1);

		//Roboter stoppen und auf Leerlaufstate springen
		if((statusAntrieb == 0) && (stopEngin == 0))
		{
			setAntrieb_RRTLAN(0, 0, 0, 0, 0, 0, 0, 0, MOTION_INTERRUPT, GEGNER_OFF);
			SetNextStepKI(KI_State, 49000, 49000);
			
			stopEngin = 1;
		}
		//ansonsten nur auf Leerlaufstate springen
		else
		{
			
		}
	}
	
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
				for(i = StateOfGame; i < MAX_KI_TASKS; i++)
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

					//sprintf(text1, "# 20: -> %d,   \r\n*", KI_maxPriority);
					//writeString_usart(&WIFI_IF, text1);

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
		// *****************************************
		// Check Plants
		// *****************************************
		case 500:
		{
			uint8_t done = 0;
			uint8_t panelsMiddleNotFree = Path_IsInArea(1000,1600,2000,1600);
			uint8_t time;
			RePrioritisePlantTasks();
			//Zeit Berechnen Abstellen
			CalcOpenPlanter();
			if (OpenPlants != 0)
			{
				time = CalcTimeRemainingPlants();
				
				sprintf(text1, "Time: %d", time);
				SendDebugMessage(text1,1);

			}
			CalcOpenParkPositions();
			
			if(OpenPlants > 0 && PlantsInRobot < 3 && spielZeit > (time + TimeAllSolarPanelsHome))
			{
				//Nächste Pflanze von PENDING to OPEN
				for (int prio = 98;prio>90; prio--)
				{
					for (int i = 1;i<7; i++)
					{
						if((KI_Task[i].Priority == prio) && (KI_Task[i].Status == PENDING))
						{
							KI_Task[i].Status = OPEN;
							done = 1;
							break;
						}
						
					}
					if(done==1)
					{
						break;
					}
				}
			}
			ActivatePlantAsObstacle();

			//Select Next Step
			if(spielZeit < TimeSolarPanelsHome)
			{
				StateOfGame = driveHome;
				KI_State = 20;
			}

			else if((OpenPlanter == 0 || (OpenPlants == 0 && PlantsInRobot == 0)) &&((spielZeit < time + TimeAllSolarPanelsHome) || OpenPlants == 0)
			&& spielZeit > TimeSolarPanelsHome)
			{
				KI_State = 20;
				StateOfGame = SolarPanels;
			}
			else if(OpenPlants > 0 && PlantsInRobot < 3 && spielZeit > (time + TimeAllSolarPanelsHome))
			{
				KI_State = 20;
				StateOfGame = GetPlants;
			}
			else if(PlantsInRobot > 0)
			{
				KI_State = 10000;
				StateOfGame = ParkPlants;
			}
			else
			{
				KI_State = 20;
				StateOfGame = driveHome;
			}
			break;
		}
		
		// *****************************************
		// Enable Next Plant
		// *****************************************
		case 550:
		{
			uint8_t done = 0;
			motionFailureCount = 0;
			
			for (int prio = 99;prio>90; prio--)
			{
				for (int i = 1;i<7; i++)
				{
					if((KI_Task[i].Priority == prio) && (KI_Task[i].Status == PENDING))
					{
						KI_Task[i].Status = OPEN;
						ActivatePlantAsObstacle();
						done = 1;
						break;
					}
					
				}
				if(done==1)
				{
					break;
				}
			}
			if(done==1 )
			{
				KI_State = 20;
				break;
			}
			else
			{
				KI_State = 500;
				break;
			}
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
			point_t start, ziel;
			
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;

			float distance;
			
			/* calculate the distance to drive */
			distance = CalcDistance(start,Plant1000);
			
			/* if the distance to drive is smaller as 300 mm -> drive direct to the goal */
			if (distance > 500.0)
			{
				// Middle Point to move correctly to the plant from the correct Quadrant
				ziel = AddMiddlePoint(start,Plant1000);
			}
			else
			{
				ziel = Plant1000;
			}
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				if(distance>500.0)
				{
					// Set Plants-Position as 3rd Position to move to
					wp_KI[wpNbr].Xpos = Plant1000.Xpos;
					wp_KI[wpNbr].Ypos = Plant1000.Ypos;
					wpNbr++;
				}
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,PLANT_ACC);
				KI_State = 1010;
			}
			else
			{
				KI_State = 1030;
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
					//Set Task to Done
					KI_Task[1].Status = DONE;
					velocity = STANDARD_VELOCITY;
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 2000);
					
					//Count up Plants in Robot
					PlantsInRobot++;
					
					CalcOpenPlants();
					if(spielZeit < (TimeToHome + 20))
					{
						KI_State = 500;
					}
					if(PlantsInRobot<planedPlants && ParkedPlants == 0 && OpenPlants > 0)
					{
						KI_State = 550;
					}
					else if((PlantsInRobot == planedPlants && ParkedPlants == 0))
					{
						KI_State = 10000;
						StateOfGame = ParkPlants;
					}
					else
					{
						KI_State = 500;
					}
					
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 1020;
					velocity = ENEMY_VELOCITY;
					break;
				}
			}
			break;
		}
		
		case 1020:
		{
			motionFailureCount++;

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 1025;
			}
			else
			{
				KI_State = 1000;
			}
			break;

			
		}
		
		case 1025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[1].Status != DONE && spielZeit > (TimeToHome + 20)) //Time to this Plant has to be added
					{
						KI_State = 1000;
					}
					else
					{
						KI_State = 500;
						nReachableCnt1000++;
						if(nReachableCnt1000 >= 3)
						{
							KI_Task[1].Status = DID;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 1020;
					break;
				}
			}
			break;
		}
		
		case 1030:
		{
			KI_State = 1020;
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
			point_t start, ziel;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			float distance;
			
			/* calculate the distance to drive */
			distance = CalcDistance(start,Plant2000);
			if (distance > 500.0)
			{

				// Middle Point to move correctly to the plant from the correct Quadrant
				ziel = AddMiddlePoint(start,Plant2000);
			}
			else
			{
				ziel = Plant2000;
			}
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				if(distance>500.0)
				{
					// Set Plants-Position as 3rd Position to move to
					wp_KI[wpNbr].Xpos = Plant2000.Xpos;
					wp_KI[wpNbr].Ypos = Plant2000.Ypos;
					wpNbr++;
				}
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,PLANT_ACC);
				KI_State = 2010;
			}
			else
			{
				KI_State = 2030;
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
					//Set Task to Done
					KI_Task[2].Status = DONE;
					velocity = STANDARD_VELOCITY;
					
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 2000);
					
					//Count up Plants in Robot
					PlantsInRobot++;
					
					CalcOpenPlants();
					if(spielZeit < (TimeToHome + 20))
					{
						KI_State = 500;
					}
					if(PlantsInRobot<planedPlants && ParkedPlants == 0 && OpenPlants > 0)
					{
						KI_State = 550;
					}
					else if((PlantsInRobot == planedPlants && ParkedPlants == 0))
					{
						KI_State = 10000;
						StateOfGame = ParkPlants;
					}
					else
					{
						KI_State = 500;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 2020;
					velocity = ENEMY_VELOCITY;
					break;
				}
			}
			break;
		}
		
		case 2020:
		{
			motionFailureCount++;

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 2025;
			}
			else
			{
				KI_State = 2000;
			}
			break;

		}

		
		case 2025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[2].Status != DONE && spielZeit > (TimeToHome + 20)) //Time to this Plant has to be added
					{
						KI_State = 2000;
					}
					else
					{
						KI_State = 500;
						
						nReachableCnt2000++;
						
						if(nReachableCnt2000 >= 3)
						{
							KI_Task[2].Status = DID;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 2020;
					break;
				}
			}
			break;
		}
		
		case 2030:
		{
			KI_State = 2020;
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
			point_t start, ziel;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			float distance;
			
			/* calculate the distance to drive */
			distance = CalcDistance(start,Plant3000);
			if (distance > 500.0)
			{

				// Middle Point to move correctly to the plant from the correct Quadrant
				ziel = AddMiddlePoint(start,Plant3000);
			}
			else
			{
				ziel = Plant3000;
			}
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				if(distance>500.0)
				{
					// Set Plants-Position as 3rd Position to move to
					wp_KI[wpNbr].Xpos = Plant3000.Xpos;
					wp_KI[wpNbr].Ypos = Plant3000.Ypos;
					wpNbr++;
				}
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,PLANT_ACC);
				KI_State = 3010;
			}
			else
			{
				KI_State = 3030;
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
					//Set Task to Done
					KI_Task[3].Status = DONE;
					velocity = STANDARD_VELOCITY;
					
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 2000);
					
					//Count up Plants in Robot
					PlantsInRobot++;
					
					CalcOpenPlants();
					if(spielZeit < (TimeToHome + 20))
					{
						KI_State = 500;
					}
					if(PlantsInRobot<planedPlants && ParkedPlants == 0 && OpenPlants > 0)
					{
						KI_State = 550;
					}
					else if((PlantsInRobot == planedPlants && ParkedPlants == 0))
					{
						KI_State = 10000;
						StateOfGame = ParkPlants;
					}
					else
					{
						KI_State = 500;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 3020;
					velocity = ENEMY_VELOCITY;
					break;
				}
			}
			break;
		}
		
		case 3020:
		{
			motionFailureCount++;


			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 3025;
			}
			else
			{
				KI_State = 3000;
			}
			break;
		}
		
		case 3025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[3].Status != DONE)
					{
						KI_State = 3000;
					}
					else
					{
						KI_State = 500;
						
						nReachableCnt3000++;
						
						if(nReachableCnt3000 >= 3)
						{
							KI_Task[3].Status = DID;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 3020;
					break;
				}
			}
			break;
		}
		case 3030:
		{
			KI_State = 3020;
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
			point_t start, ziel;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			// Position where the plants stand
			Plant4000.Xpos = 1500;
			Plant4000.Ypos = 1500;
			
			float distance;
			
			/* calculate the distance to drive */
			distance = CalcDistance(start,Plant4000);
			if (distance > 500.0)
			{

				// Middle Point to move correctly to the plant from the correct Quadrant
				ziel = AddMiddlePoint(start,Plant4000);
			}
			else
			{
				ziel = Plant4000;
			}
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				if(distance>500.0)
				{
					// Set Plants-Position as 3rd Position to move to
					wp_KI[wpNbr].Xpos = Plant4000.Xpos;
					wp_KI[wpNbr].Ypos = Plant4000.Ypos;
					wpNbr++;
				}
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,PLANT_ACC);
				KI_State = 4010;
			}
			else
			{
				KI_State = 4030;
			}
			break;
		}

		
		case 4010:
		{
			sprintf(text1, "Wait 4010");
			SendDebugMessage(text1,1);
			/* check observation-result */
			switch (GetObservationResult())
			{

				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[4].Status = DONE;
					
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 2000);
					
					//Count up Plants in Robot
					PlantsInRobot++;
					
					CalcOpenPlants();
					if(spielZeit < (TimeToHome + 20))
					{
						KI_State = 500;
					}
					if(PlantsInRobot<planedPlants && ParkedPlants == 0 && OpenPlants > 0)
					{
						KI_State = 550;
					}
					else if((PlantsInRobot == planedPlants && ParkedPlants == 0))
					{
						KI_State = 10000;
						StateOfGame = ParkPlants;
					}
					else
					{
						KI_State = 500;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 4020;
					velocity = ENEMY_VELOCITY;
					break;
				}
			}
			break;
		}
		
		case 4020:
		{
			motionFailureCount++;

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 4025;
			}
			else
			{
				KI_State = 4000;
			}
			break;
			
		}
		
		case 4025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[4].Status != DONE && spielZeit > (TimeToHome + 20)) //Time to this Plant has to be added
					{
						KI_State = 4000;
					}
					else
					{
						KI_State = 500;
						
						nReachableCnt4000++;
						
						if(nReachableCnt4000 >= 3)
						{
							KI_Task[4].Status = DID;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 4020;
					break;
				}
			}
			break;
		}
		case 4030:
		{
			KI_State = 4020;
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
			point_t start, ziel;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			float distance;
			
			/* calculate the distance to drive */
			distance = CalcDistance(start,Plant5000);
			if (distance > 500.0)
			{

				// Middle Point to move correctly to the plant from the correct Quadrant
				ziel = AddMiddlePoint(start,Plant5000);
			}
			else
			{
				ziel = Plant5000;
			}
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				if(distance>500.0)
				{
					// Set Plants-Position as 3rd Position to move to
					wp_KI[wpNbr].Xpos = Plant5000.Xpos;
					wp_KI[wpNbr].Ypos = Plant5000.Ypos;
					wpNbr++;
				}
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,PLANT_ACC);
				KI_State = 5010;
			}
			else
			{
				KI_State = 5030;
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
					//Set Task to Done
					KI_Task[5].Status = DONE;
					velocity = STANDARD_VELOCITY;
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 2000);
					
					//Count up Plants in Robot
					PlantsInRobot++;
					
					CalcOpenPlants();
					if(spielZeit < (TimeToHome + 20))
					{
						KI_State = 500;
					}
					if(PlantsInRobot<planedPlants && ParkedPlants == 0 && OpenPlants > 0)
					{
						KI_State = 550;
					}
					else if((PlantsInRobot == planedPlants && ParkedPlants == 0))
					{
						KI_State = 10000;
						StateOfGame = ParkPlants;
					}
					else
					{
						KI_State = 500;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 5020;
					velocity = ENEMY_VELOCITY;
					break;
				}
			}
			break;
		}
		
		case 5020:
		{
			motionFailureCount++;

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 5025;
			}
			else
			{
				KI_State = 5000;
			}
			break;

		}
		
		case 5025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[5].Status != DONE && spielZeit > (TimeToHome + 20)) //Time to this Plant has to be added
					{
						KI_State = 5000;
					}
					else
					{
						KI_State = 500;
						
						nReachableCnt5000++;
						
						if(nReachableCnt5000 >= 3)
						{
							KI_Task[5].Status = DID;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 5020;
					break;
				}
			}
			break;
		}
		case 5030:
		{
			KI_State = 5020;
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
			point_t start, ziel;
			
			// Start Position to begin movement from
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			float distance;
			
			/* calculate the distance to drive */
			distance = CalcDistance(start,Plant6000);
			if (distance > 500.0)
			{

				// Middle Point to move correctly to the plant from the correct Quadrant
				ziel = AddMiddlePoint(start,Plant6000);
			}
			else
			{
				ziel = Plant6000;
			}
			
			if (PATH_DriveToAbsPos(start, ziel, wp_KI, &wpNbr))
			{
				if(distance>500.0)
				{
					// Set Plants-Position as 3rd Position to move to
					wp_KI[wpNbr].Xpos = Plant6000.Xpos;
					wp_KI[wpNbr].Ypos = Plant6000.Ypos;
					wpNbr++;
				}
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,PLANT_ACC);
				KI_State = 6010;
			}
			else
			{
				KI_State = 6030;
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
					//Set Task to Done
					KI_Task[6].Status = DONE;
					velocity = STANDARD_VELOCITY;
					
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 2000);
					
					//Count up Plants in Robot
					PlantsInRobot++;
					
					CalcOpenPlants();
					if(spielZeit < (TimeToHome + 20))
					{
						KI_State = 500;
					}
					if(PlantsInRobot<planedPlants && ParkedPlants == 0 && OpenPlants > 0)
					{
						KI_State = 550;
					}
					else if((PlantsInRobot == planedPlants && ParkedPlants == 0))
					{
						KI_State = 10000;
						StateOfGame = ParkPlants;
					}
					else
					{
						KI_State = 500;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 6020;
					velocity = ENEMY_VELOCITY;
					break;
				}
			}
			break;
		}
		
		case 6020:
		{
			motionFailureCount++;

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 6025;
			}
			else
			{
				KI_State = 6000;
			}
			break;
			
		}
		
		case 6025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[6].Status != DONE && spielZeit > (TimeToHome + 20)) //Time to this Plant has to be added
					{
						KI_State = 6000;
					}
					else
					{
						KI_State = 500;
						
						nReachableCnt6000++;
						
						if(nReachableCnt6000 >= 3)
						{
							KI_Task[6].Status = DID;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 6020;
					break;
				}
			}
			break;
		}
		case 6030:
		{
			KI_State = 6020;
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
			//Prio 89 = Planter 2  Task 15/25  ==> Changeable
			//Prio 88 = Planter Midle Task 11/21
			//Prio 87 = Field 1 Side Task 12/22 ==> Changeable
			//Prio 86 = Planter 1 Task 13/23
			//Prio 85 = Field 1 Task 12/22 ==> Changeable
			//Prio 83 = Planter 2 Task 15/25  ==> Changeable
			//Prio 82 = Field 3 Task 16/26
			
			point_t PlanterMidlePos, Planter2Pos, field1Pos, field3Pos, aktPos, Planter1Pos;
			float distance_PlanterMidle;
			float distance_Planter2;
			float distance_Planter1;
			float distance_Field1;
			float distance_Field3;
			uint8_t enemyRobotInPlanter2;
			
			motionFailureCount = 0;
			
			// Start Position to begin movement from
			aktPos.Xpos = xPos;
			aktPos.Ypos = yPos;
			
			//Blue
			if(SpielFarbe == BLUE )
			{
				// Position of Planter Midle
				PlanterMidlePos = PlanterMidleBlue;
				//Position of Planter 1
				Planter1Pos = PlanterL1;
				//Position of Planter 2
				Planter2Pos = PlanterR2;
				//Position of Field 1
				field1Pos = FieldL1;
				//Position of Field 3
				field3Pos = FieldL3;
			}
			if(SpielFarbe == Yellow )
			{
				// Position of Planter Midle
				PlanterMidlePos = PlanterMidleYellow;
				//Position of Planter 1
				Planter1Pos = PlanterR1;
				//Position of Planter 2
				Planter2Pos = PlanterL2;
				//Position of Field 1
				field1Pos = FieldR1;
				//Position of Field 3
				field3Pos = FieldR3;
			}
			
			//Distance to Planter Midle
			distance_PlanterMidle = CalcDistance(aktPos,PlanterMidlePos);
			//Distance to Planter 1
			distance_Planter1 = CalcDistance(aktPos,Planter1Pos);
			//Distance to Planter2
			distance_Planter2 = CalcDistance(aktPos,Planter2Pos);
			//Distance to Field1
			distance_Field1 = CalcDistance(aktPos,field1Pos);;
			//Distance to Field3
			distance_Field3 = CalcDistance(aktPos,field3Pos);;
			
			//Default Prios
			KI_Task[12].Priority = 80;
			KI_Task[22].Priority = 80;
			
			//Priority for fixed Tasks
			KI_Task[11].Priority = 88;
			KI_Task[21].Priority = 88;
			KI_Task[13].Priority = 86;
			KI_Task[23].Priority = 86;
			KI_Task[16].Priority = 82;
			KI_Task[26].Priority = 82;
			
			//Detect if Enemy is in Area Planter 2
			if(SpielFarbe == BLUE)
			{
				enemyRobotInPlanter2 = Path_IsInArea(0,1000,600,2000);
			}
			if(SpielFarbe == Yellow)
			{
				enemyRobotInPlanter2 = Path_IsInArea(2400,1000,3000,2000);
			}
			//Planter 2
			if((KI_Task[15].Status == OPEN || KI_Task[25].Status == OPEN)
			&& ((distance_Planter2 < distance_PlanterMidle)||(KI_Task[11].Status != OPEN && KI_Task[21].Status != OPEN))
			&& ((distance_Planter2 < distance_Planter1)||(KI_Task[13].Status != OPEN && KI_Task[23].Status != OPEN))
			&& (enemyRobotInPlanter2 == 0))
			{
				KI_Task[15].Priority = 89;
				KI_Task[25].Priority = 89;
			}
			else
			{
				KI_Task[15].Priority = 83;
				KI_Task[25].Priority = 83;
			}

			//Task field 1 as Prio 87
			if((KI_Task[11].Status != OPEN && KI_Task[21].Status != OPEN && PlantsInRobot > 1) && (KI_Task[13].Status == OPEN || KI_Task[23].Status == OPEN) && PlantsInRobot >= 2)
			{
				KI_Task[12].Priority = 87;
				KI_Task[22].Priority = 87;
			}
			else
			{
				//Task field1  as Prio 85
				KI_Task[12].Priority = 85;
				KI_Task[22].Priority = 85;
			}

			if(spielZeit > (TimeToHome + 20))
			{
				KI_State = 20;
			}
			else
			{
				KI_State = 500;
			}

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
			point_t start;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			if (PATH_DriveToAbsPos(start, PlanterMidleBlue, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 11010;
			}
			else
			{
				KI_State = 11030;
			}
			break;
		}

		case 11010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[11].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 3000);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 500;

					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 11020;
					break;
				}
			}
			break;
		}
		
		case 11020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);
			
			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 11025;
			}
			else
			{
				KI_State = 11000;
			}
			break;
		}
		
		case 11025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[11].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 11000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 11020;
					break;
				}
			}
			break;
		}
		
		case 11030:
		{
			KI_State = 11020;
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
			point_t start;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			if (PATH_DriveToAbsPos(start, FieldL1, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 12010;
			}
			else
			{
				KI_State = 12030;
			}
			break;
		}

		case 12010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[12].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 500);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 12012;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 12020;
					break;
				}
			}
			break;
		}
		case 12012:
		{
			cmd_Drive(0,0,-200,0,0,2500,yPos,0,POS_ABS,ON,NULL,NULL,STANDARD_ACC,STANDARD_ACC);
			KI_State=12015;
			break;
		}
		case 12015:
		{
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 500;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 12012;
					SET_CYCLE(KI_TASKNBR, 1000);
					break;
				}
			}
			break;
		}
		case 12020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);
			
			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 12025;
			}
			else
			{
				KI_State = 12000;
			}
			break;
		}
		case 12025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[12].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 12000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 12020;
					break;
				}
			}
			break;
		}
		
		case 12030:
		{
			KI_State = 12020;
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
			point_t start , goal;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			//Check if Enemy Robot is in Area Next to Planter
			goal.Xpos = PlanterL1.Xpos;
			goal.Ypos = PlanterL1.Ypos;
			
			if(Path_IsInArea(2550,775,3000,1225))
			{
				goal.Ypos = 562.5;
				velocity = ENEMY_VELOCITY;
			}
			
			if (PATH_DriveToAbsPos(start,goal, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 13010;
			}
			else
			{
				KI_State = 13030;
			}
			velocity = STANDARD_VELOCITY;
			break;
		}

		case 13010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[13].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 3000);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 500;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 13020;
					break;
				}
			}
			break;
		}
		
		case 13020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);
			
			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 13025;
			}
			else
			{
				KI_State = 13000;
			}
			break;
		}
		
		case 13025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[13].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 13000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 13020;
					break;
				}
			}
			break;
		}
		
		case 13030:
		{
			KI_State = 13020;
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
			point_t start, goal;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			//Check if Enemy Robot is in Area Next to Planter
			goal.Xpos = PlanterL2.Xpos;
			goal.Ypos = PlanterL2.Ypos;
			
			if(Path_IsInArea(2550,1550,3000,2000))
			{
				goal.Ypos = 1337.5;
				velocity = ENEMY_VELOCITY;
			}
			
			if (PATH_DriveToAbsPos(start, PlanterL2, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 15010;
			}
			else
			{
				KI_State = 15030;
			}
			velocity = STANDARD_VELOCITY;
			break;
		}

		case 15010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[15].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 3000);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 500;
					//if(PlantsInRobot < 3 || OpenParkPos == 0)
					//{
					//KI_State = 500;
					//StateOfGame = GetPlants; // gehört raus
					//}
					//else
					//{
					//KI_State = 10000;
					//}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 15020;
					break;
				}
			}
			break;
		}
		
		case 15020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);
			
			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 15025;
			}
			else
			{
				KI_State = 15000;
			}
			break;
		}
		
		case 15025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[15].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 15000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 15020;
					break;
				}
			}
			break;
		}
		
		case 15030:
		{
			KI_State = 15020;
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
			point_t start;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			if (PATH_DriveToAbsPos(start, FieldL3, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 16010;
			}
			else
			{
				KI_State = 16030;
			}
			break;
		}

		case 16010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					if(PlantsInRobot == 0)
					{
						KI_Task[16].Status = DONE;
					}

					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 500);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 16012;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 16020;
					break;
				}
			}
			break;
		}
		case 16012:
		{
			cmd_Drive(0,0,-200,0,0,2500,yPos,0,POS_ABS,ON,NULL,NULL,STANDARD_ACC,STANDARD_ACC);
			KI_State=16015;
			break;
		}
		case 16015:
		{
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(PlantsInRobot == 0)
					{
						KI_State = 60010;
						KI_Task[16].Status = DONE;
					}
					else
					{
						KI_State = 16000;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 16012;
					SET_CYCLE(KI_TASKNBR, 1000);
					break;
				}
			}
			break;
		}
		
		case 16020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 16025;
			}
			else
			{
				KI_State = 16000;
			}
			break;
		}
		
		case 16025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[16].Status == OPEN)
					{
						KI_State = 16000;
					}
					else
					{
						KI_State = 16000;
						motionFailureCount = 0;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 16020;
					break;
				}
			}
			break;
		}
		
		case 16030:
		{
			KI_State = 16020;
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
		// ******   Task 20000 - Keine Aufgabe
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
			point_t start;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			if (PATH_DriveToAbsPos(start, PlanterMidleYellow, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 21010;
			}
			else
			{
				KI_State = 21030;
			}
			
			break;
		}

		case 21010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[21].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 3000);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 500;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 21020;
					break;
				}
			}
			break;
		}
		
		case 21020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 21025;
			}
			else
			{
				KI_State = 21000;
			}
			break;
		}
		
		case 21025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[21].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 21000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 21020;
					break;
				}
			}
			break;
		}
		
		case 21030:
		{
			KI_State = 21020;
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
			point_t start;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			if (PATH_DriveToAbsPos(start, FieldR1, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 22010;
			}
			else
			{
				KI_State = 22030;
			}
			
			break;
		}

		case 22010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[22].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 500);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 22012;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 22020;
					break;
				}
			}
			break;
		}
		case 22012:
		{
			cmd_Drive(0,0,-200,0,0,500,yPos,0,POS_ABS,ON,NULL,NULL,STANDARD_ACC,STANDARD_ACC);
			KI_State=22015;
			break;
		}
		case 22015:
		{
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 500;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 22012;
					SET_CYCLE(KI_TASKNBR, 1000);
					break;
				}
			}
			break;
		}
		case 22020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 22025;
			}
			else
			{
				KI_State = 22000;
			}
			break;
		}
		
		case 22025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[22].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 22000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 22020;
					break;
				}
			}
			break;
		}
		
		case 22030:
		{
			KI_State = 22020;
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
			point_t start, goal;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			//Check if Enemy Robot is in Area Next to Planter
			goal.Xpos = PlanterR1.Xpos;
			goal.Ypos = PlanterR1.Ypos;
			
			if(Path_IsInArea(0,775,450,1225))
			{
				goal.Ypos = 562.5;
				velocity = ENEMY_VELOCITY;
			}
			
			if (PATH_DriveToAbsPos(start, goal, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 23010;
			}
			else
			{
				KI_State = 23030;
			}
			
			velocity = STANDARD_VELOCITY;
			
			break;
		}

		case 23010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				velocity = STANDARD_VELOCITY;
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					//Set Task to Done
					KI_Task[23].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 3000);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 500;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 23020;
					break;
				}
			}
			break;
		}
		
		case 23020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 23025;
			}
			else
			{
				KI_State = 23000;
			}
			break;
		}
		
		case 23025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[23].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 23000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 23020;
					break;
				}
			}
			break;
		}
		
		case 23030:
		{
			KI_State = 23020;
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
			point_t start, goal;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			//Check if Enemy Robot is in Area Next to Planter
			goal.Xpos = PlanterR2.Xpos;
			goal.Ypos = PlanterR2.Ypos;
			
			if(Path_IsInArea(0,1550,450,2000))
			{
				goal.Ypos = 562.5;
				velocity = ENEMY_VELOCITY;
			}
			
			if (PATH_DriveToAbsPos(start, goal, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 25010;
			}
			else
			{
				KI_State = 25030;
			}
			
			break;
		}

		case 25010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[25].Status = DONE;
					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 3000);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					//if(PlantsInRobot < 3 || OpenParkPos == 0)
					//{
					//KI_State = 500;
					//StateOfGame = GetPlants;
					//}
					//else
					//{
					//KI_State = 10000;
					//}
					KI_State = 500;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 25020;
					break;
				}
			}
			break;
		}
		
		case 25020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 25025;
			}
			else
			{
				KI_State = 25000;
			}
			break;
		}
		
		case 25025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[25].Status == OPEN && spielZeit > (TimeToHome + 20)) //Time to this Parkpos has to be added
					{
						KI_State = 25000;
					}
					else
					{
						KI_State = 500;

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 25020;
					break;
				}
			}
			break;
		}
		
		case 25030:
		{
			KI_State = 25020;
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
			point_t start;
			
			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;
			
			if (PATH_DriveToAbsPos(start, FieldR3, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 26010;
			}
			else
			{
				KI_State = 26030;
			}
			break;
		}

		case 26010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					if(PlantsInRobot = 0)
					{
						KI_Task[26].Status = DONE;
					}

					CalcOpenParkPositions();
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 500);
					
					//Count up Plants in Robot
					PlantsInRobot--;
					ParkedPlants++;
					
					KI_State = 26012;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 26020;
					break;
				}
			}
			break;
		}
		case 26012:
		{
			cmd_Drive(0,0,-200,0,0,500,yPos,0,POS_ABS,ON,NULL,NULL,STANDARD_ACC,STANDARD_ACC);
			KI_State=22015;
			break;
		}
		case 26015:
		{
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(PlantsInRobot == 0 )
					{
						KI_State = 60010;
						KI_Task[26].Status = DONE;
					}
					else
					{
						KI_State = 26000;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 26012;
					SET_CYCLE(KI_TASKNBR, 1000);
					break;
				}
			}
			break;
		}
		
		case 26020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 26025;
			}
			else
			{
				KI_State = 26000;
			}
			break;
		}
		
		case 26025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[26].Status == OPEN)
					{
						KI_State = 26000;
					}
					else
					{
						KI_State = 26000;
						motionFailureCount = 0;
						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 26020;
					break;
				}
			}
			break;
		}
		
		case 26030:
		{
			KI_State = 26020;
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
			point_t start, goal;

			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;

			//Check if Enemy Robot is in Area Next to Planter
			goal.Xpos = SolarPanelsBlue.Xpos;
			goal.Ypos = SolarPanelsBlue.Ypos;


			if (PATH_DriveToAbsPos(start, goal, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 30010;
			}
			else
			{
				KI_State = 30030;
			}

			velocity = STANDARD_VELOCITY;

			break;
		}
		
		case 30010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[30].Status = DONE;
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 8000);
					
					KI_State = 60000;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 30020;
					break;
				}
			}
			break;
		}
		
		case 30020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 30025;
			}
			else
			{
				KI_State = 30000;
			}
			break;
		}
		
		case 30025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[30].Status == OPEN)
					{
						KI_State = 30000;
					}
					else
					{
						if(spielZeit < TimeToHome)
						{
							KI_State = 60000;
						}
						else
						{
							KI_State = 30000;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 30020;
					break;
				}
			}
			break;
		}
		
		case 30030:
		{
			KI_State = 30020;
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
			point_t start, goal;

			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;

			//Check if Enemy Robot is in Area Next to Planter
			goal.Xpos = SolarPanelsMiddle.Xpos;
			goal.Ypos = SolarPanelsMiddle.Ypos;


			if (PATH_DriveToAbsPos(start, goal, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 31010;
			}
			else
			{
				KI_State = 31030;
			}

			velocity = STANDARD_VELOCITY;

			break;
		}
		
		case 31010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[31].Status = DONE;
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 8000);
					
					KI_State = 31027;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 31020;
					break;
				}
			}
			break;
		}
		
		case 31020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 31025;
			}
			else
			{
				KI_State = 31000;
			}
			break;
		}
		
		case 31025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					KI_State = 31027;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 31020;
					break;
				}
			}
			break;
		}
		case 31027:
		{
			if(spielZeit > (TimeAllSolarPanelsHome + 20) && OpenPlants == 0)
			{
				KI_State = 31027; // evtl. nicht bis Sekunde 15 warten zum wegfahren sondern gleich nach 8 Sekunden fahren (blockieren evtl bloed wegen Fair play)
			}
			else
			{
				KI_State = 500;
			}
			
			velocity = STANDARD_VELOCITY;
			break;
		}
		
		
		case 31030:
		{
			KI_State = 31020;
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
			point_t start, goal;

			// Start Position to begin from movement
			start.Xpos = xPos;
			start.Ypos = yPos;

			//Check if Enemy Robot is in Area Next to Planter
			goal.Xpos = SolarPanelsYellow.Xpos;
			goal.Ypos = SolarPanelsYellow.Ypos;


			if (PATH_DriveToAbsPos(start, goal, wp_KI, &wpNbr))
			{
				cmd_Drive(0,0,velocity,0,0,0,0,0,0,ON,wp_KI,wpNbr,STANDARD_ACC,STANDARD_ACC);
				KI_State = 32010;
			}
			else
			{
				KI_State = 32030;
			}

			velocity = STANDARD_VELOCITY;

			break;
		}

		case 32010:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					velocity = STANDARD_VELOCITY;
					//Set Task to Done
					KI_Task[32].Status = DONE;
					//Wait specific time
					SET_CYCLE(KI_TASKNBR, 8000);
					
					KI_State = 60000;
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					velocity = ENEMY_VELOCITY;
					KI_State = 32020;
					break;
				}
			}
			break;
		}

		case 32020:
		{
			motionFailureCount++;
			SET_CYCLE(KI_TASKNBR, 500);

			//Drive Back
			if(DriveBack(100,200))
			{
				KI_State = 32025;
			}
			else
			{
				KI_State = 32000;
			}
			break;
		}

		case 32025:
		{
			/* check observation-result */
			switch (GetObservationResult())
			{
				/* motion was OK */
				case OBSERVATION_MOTION_OK:
				{
					if(motionFailureCount<3 && KI_Task[32].Status == OPEN)
					{
						KI_State = 32000;
					}
					else
					{
						if(spielZeit < TimeToHome)
						{
							KI_State = 60000;
						}
						else
						{
							KI_State = 32000;
						}

						velocity = STANDARD_VELOCITY;
					}
					break;
				}
				/* error happened during the motion */
				case OBSERVATION_MOTION_ERROR:
				{
					KI_State = 32020;
					break;
				}
			}
			break;
		}

		case 32030:
		{
			KI_State = 32020;
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
		// ******   Task 60000 - Heimfahren
		// ******
		// ********************************************************************
		// ********************************************************************
		case 60000:
		{
			//Plants are no Obstacles
			for (int i = 1; i<7; i++)
			{
				PATH_DISABLE_OBSTACLE(i+2);
			}
			
			//Drive Home
			if(SpielFarbe == BLUE)
			{
				KI_State = 16000;
				KI_Task[16].Status = OPEN;
				
			}
			else if (SpielFarbe == Yellow)
			{
				KI_State = 26000;
				KI_Task[26].Status = OPEN;
			}
			
			KI_Task[60].Status = DONE;
			
			break;
		}
		
		case 60010:
		{
			//Ende im Gelände
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


	//sprintf(text1, "Side: %d Front: %d", observationDisSide , observationDisFront);
	//SendDebugMessage(text1,1);
	//Write Message to Logger
	if(KI_State != OldKI_State)
	{
		sprintf(text1, "State: %6ld PIR: %d OPP: %d", (uint32_t)KI_State, PlantsInRobot , OpenParkPos);
		SendDebugMessage(text1,1);
	}
	OldKI_State = KI_State;

	return(CYCLE);
}


