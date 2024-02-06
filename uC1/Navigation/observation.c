/*
* observation.c
*
* Created: 24.05.2023 11:25:30
*  Author: P20087
*/

#define _OBSERVATION_EXTERN_

#include <avr/io.h>
#include "multitask.h"
#include "define.h"
#include "global.h"
#include "observation.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include <math.h>
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>


#define _DEBUG_MOTION_


/* ************************************************************** */
/*! \brief Initialize observation-task.
*
*  Function initialize the observation-task
*
*  \version 24.05.2023
*
*/
/* ************************************************************** */
void InitObservation(void)
{
	/* cyclic task - cycle time: 500 ms */
	SET_CYCLE(OBSERVATION_TASKNBR, 500);
	SET_TASK(OBSERVATION_TASKNBR, CYCLE);
	SET_TASK_HANDLE(OBSERVATION_TASKNBR, ObservationTask);
	
	observationResult = OBSERVATION_RUNNING;
	observationStarted = OBSERVATION_PENDING;
	
	observationDisFront = 500;
	observationDisSide = 350;
}



/* ************************************************************** */
/*! \brief Debug-task.
*
*  Here different functions can be tested
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
uint8_t ObservationTask(void)
{
	char text[100];
	char text1[200];
	
	/* loop-index  */
	int8_t k = 0;
	/* vector from robot to point on bounding-box */
	float a[2];
	/* vectors from robot to the enemy-robots */
	float b[6][2];
	/* distance from robot to the enemy-robots */
	float dis2Robots[6];
	/* distance from robot to the goal */
	float dis2Goal;
	/* scalar-product between vector A and B */
	float skalarAB = 0.0;
	/* orthogonal projection from vector B to A */
	float orthoBA[2];
	/*  */
	float aPow2 = 0.0;
	float distanzPunktPow2 = 0.0;
	float cPow2 = 0.0;
	uint8_t enemyEanable = 0;
	
	point_t pos;
	matrixpoint_t gridPoint;
	
	/* set observertion-task intervall to 10ms */
	SET_CYCLE(OBSERVATION_TASKNBR, 10);
	

	/* observation is started -> observe the motion of the robot */
	if (observationStarted == OBSERVATION_STARTED)
	{
		/*    ________front_______  _ _ _
		|                    |   s
		|                    |   i
		____|__                  |   d
		|       |                 |   e
		|   X   | - - - - - - - - | - - - > driving direction
		|_______|                 |
		robot                   |
		|                    |
		|____________________|
		observed area                                    */
		
		/**************************************************************************
		***   motion observation    			                                ***
		***   ---------------------------------------------------------------   ***
		***   Date    :  23.05.2023                                             ***
		***   Author  :  Michael Zauner							                ***
		**************************************************************************/
		/* set observation distance depending on the actual speed */
		uint16_t Amax_DS_MI = 200;
		
		// 		if (vIst > 0)
		{
			float t_brake = 0.0;
			float t_react = 0.2;
			float vIst_Enemy = abs(vIst);
			float t_react_Enemy = 0.2;
			float AmaxDs_Enemy = 100.0;
			float t_brake_Enemy = 0.0;
			uint16_t brakeingDistance = 0;
			uint16_t brakeingDistance_Enemy = 0;

			
			t_brake = (float)abs(vIst) / ((float)Amax_DS_MI*10.0);
			t_brake_Enemy = (float)vIst_Enemy / ((float)AmaxDs_Enemy*10.0);
			
			brakeingDistance =	(uint16_t)((float)abs(vIst) * t_react - ((float)Amax_DS_MI * 10.0 * t_brake * t_brake)/2.0 + (float)vIst * t_brake);  // reaction distance and Braking distance
			brakeingDistance_Enemy = (uint16_t)(vIst_Enemy * t_react_Enemy - (AmaxDs_Enemy * 10.0 * t_brake_Enemy * t_brake_Enemy)/2.0 + vIst_Enemy * t_brake_Enemy);  // reaction distance and Braking distance Enemy
			observationDisFront = 250 + brakeingDistance + brakeingDistance_Enemy;
			
			
			
			/* add the (PATH_GRID_RESOLUTION / 2) to the pos -> so the result is rounded correctly */
			/* example1: X = 1010, X += 50, X = 1060, X /= 100 -> result = 10 - OK! 1010 is closer to 1000 */
			/* example1: X = 1070, X += 50, X = 1120, X /= 100 -> result = 11 - OK! 1070 is closer to 1100 */
			pos.Xpos = xPos + (PATH_GRID_RESOLUTION / 2);
			pos.Ypos = yPos + (PATH_GRID_RESOLUTION / 2);
			
			/* calculate nearest grid-point */
			gridPoint.Xpos = (int8_t)((pos.Xpos - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION);
			gridPoint.Ypos = (int8_t)((pos.Ypos - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION);
			
			/* limit grid-point in x */
			gridPoint.Xpos = ((gridPoint.Xpos >= PATH_GRID_DIM_X) ? PATH_GRID_DIM_X-1 : gridPoint.Xpos);
			gridPoint.Xpos = ((gridPoint.Xpos < 0) ? 0 : gridPoint.Xpos);

			/* limit grid-point in y */
			gridPoint.Ypos = ((gridPoint.Ypos >= PATH_GRID_DIM_Y) ? PATH_GRID_DIM_Y-1 : gridPoint.Ypos);
			gridPoint.Ypos = ((gridPoint.Ypos < 0) ? 0 : gridPoint.Ypos);
			
			
			if ((path.isInObservationArea) && (path.occupancyGrid[gridPoint.Xpos][gridPoint.Ypos] == PATH_INFINITY))
			{
				observationDisSide = 150;
			}
			else
			{
				if(brakeingDistance > brakeingDistance_Enemy)
				{
					observationDisSide = (uint16_t)(((float)brakeingDistance + 150.0)/2.0) + 150; //Auf Mitte Roboter Bezogen
				}
				else
				{
					observationDisSide = (uint16_t)(((float)brakeingDistance_Enemy + 150.0)/2.0) + 250; //Auf Mitte Roboter Bezogen
				}
			}
			
			//observationDisFront = 370 + (int16_t)((float)(abs(vIst))*0.8);
		}
		// 		else if (vIst < 0)
		// 		{
		// 			observationDisFront = 150 + abs(vIst);
		// 		}
		

		// ***************************************
		// observation of all oponent robots
		// ***************************************
		if((gegnerErkennung == ON) && ((abs(observationStartPos.Xpos - xPos) > 5) || (abs(observationStartPos.Ypos - yPos ) > 5)))
		{
			/* caclate the distance to the goal-position */
			dis2Goal = sqrt(pow((xPos - observationGoalPos.Xpos), 2.0) + pow(yPos - observationGoalPos.Ypos, 2.0)) + 500;
			
			/* set enemy-robot positions -> if value = 0 then set it to 10000 (robot is not dedected) */
			for (uint8_t i = 0; i < 5; i++)
			{
				enemyRobot[i].Xpos = ((enemyRobot[i].Xpos == 0) ? 10000.0 : enemyRobot[i].Xpos);
				enemyRobot[i].Ypos = ((enemyRobot[i].Ypos == 0) ? 10000.0 : enemyRobot[i].Ypos);
			}
			
			/* check if an enemy-robot is nearer than the distance to goal */
			for (uint8_t i = 0; i < 5; i++)
			{
				/* enemy */
				/* first index     second index */
				/* [1] ... robot   [0] ... X */
				/*                 [1] ... Y */
				b[i][0] = (float)enemyRobot[i].Xpos - (float)xPos;
				b[i][1] = (float)enemyRobot[i].Ypos - (float)yPos;
				
				dis2Robots[i] = sqrt(pow(b[i][0], 2.0) + pow(b[i][1], 2.0));
				
				/* if at least one robot is nearer then enable the observation */
				if (dis2Robots[i] < dis2Goal)
				{
					enemyEanable = 1;
				}
			}
			
			
			// calculate the point in front of the robot
			a[0] = geschwindigkeitsVorzeichen * ((float)observationDisFront * cos((float)(phiPos) * M_PI / 1800));
			a[1] = geschwindigkeitsVorzeichen * ((float)observationDisFront * sin((float)(phiPos) * M_PI / 1800));
			
			/*             /- _
			/    -_
			/  O   /
			_____  - _ /      /  enemy
			|     |     b /- _/      _____
			|  O  |enemy  _/________|___  |
			|____\|      |/         |_/O| |enemy  .
			b\_____/|   b__,--'     |___|_|      /|\ c
			| \  /__|--'           |         |
			|  `O---|------------->O
			|_______|      a     Punkt vor Roboter
			Roboter|              |
			|______________|
			Bereich der überwacht wird */
			
			// up to 5 robots will be observed
			for (k = 0; k < 5; k++)
			{
				skalarAB = b[k][0] * a[0] + b[k][1] * a[1];
				/* if the scalar product is positive -> 0°...90° -> the opponent is in front of its own robot.
				in addition, the coordinates of the opponent's robot must not be 0. */
				if((skalarAB > 0.0) && (b[k][0] != 0.0) && (b[k][1] != 0.0))
				{
					/* calculate the square distance between robot and the virtual point in front of the robot */
					aPow2 = (float)observationDisFront * (float)observationDisFront;
					
					/* ********************* */
					/* orthogonal projection */
					/* ********************* */
					/* If the factor by which the vector A is multiplied is greater than 1, the opponent lies behind the second target point (B).
					scalarAB < aPow2 ... the orthogonal projection of the opponent is shorter than the vector a
					scalarAB = aPow2 ... the orthogonal projection of the opponent is equal to the vector a
					scalarAB > aPow2 ... the orthogonal projection of the opponent is longer than the vector a => need not be considered
					
					If the opponent's robot is in front of the robot to be monitored (scalarAB > aPow2) the own robot can continue.
					If the opponent's robot lies in the monitoring area (scalarAB < aPow2), the calculation must be continued. */						if((skalarAB < aPow2) && (enemyEanable == 1))
					if((skalarAB < aPow2) && (enemyEanable == 1))
					{
						orthoBA[0] = ((skalarAB / aPow2) * a[0]);
						orthoBA[1] = ((skalarAB / aPow2) * a[1]);
						
						/* distance between the opponent and the straight line squared */
						distanzPunktPow2 = pow((orthoBA[0] - b[k][0]),2.0) + pow((orthoBA[1] - b[k][1]),2.0);
						
						cPow2 = pow((float)observationDisSide,2.0);
						
						/* Is there an enemy robot in the defined area between your own robot and the virtual point? */
						/*  -> yes */
						if(distanzPunktPow2 < cPow2)
						{
							/* break -> interrupt the actual motion */
							setACCAntrieb_RRTLAN(100,(uint8_t)Amax_DS_MI);
							setAntrieb_RRTLAN(0, 0, 0, 0, 0, 0, 0, 0, MOTION_INTERRUPT, GEGNER_OFF);
							//cmd_Drive(0,0,0,0,0,0,0,0,MOTION_INTERRUPT,0,NULL,NULL,100,100);
							gegnerErkennung = OFF;
							motionIR = 1;
							
							break;
						}
					}
				}
			}
		}

		
		//if(gegnerErkennung == ON && ((abs(startPos.Xpos - xPos) > 10) || abs(startPos.Ypos - yPos) > 10))
		//{
		//// Roboter fährt vorwärts
		//if ( geschwindigkeitsVorzeichen > 0.0)
		//{
		//if(usFrontLeft < WATCH_DIS_FRONT || usFrontRight < WATCH_DIS_FRONT)
		//{
		//// *******************************************************************************
		//// ****** Ultraschallüberwachung vorne, mit Überwachung ob außerhalb vom Tisch ***
		//// *******************************************************************************
		//
		////////////// Ultraschall vorne ///////////////
		//float xFrontRight;
		//float xFrontLeft;
		//float yFrontRight;
		//float yFrontLeft;
		//
		//// X und Y Koordinaten vom Hindernis des rechten vorderen Sensors
		//xFrontRight = (float)xPos + (float)(OFFSET_FRONT_X + usFrontRight) * cos(DEG2RAD(phiPos/10)) + (float)OFFSET_FRONT_Y * sin(DEG2RAD(phiPos/10));
		//yFrontRight = (float)yPos - (float)OFFSET_FRONT_Y * cos(DEG2RAD(phiPos/10)) + (OFFSET_FRONT_X + usFrontRight) * sin(DEG2RAD(phiPos/10));
		//
		//// X und Y Koordinaten vom Hindernis des linken vorderen Sensors
		//xFrontLeft = (float)xPos + (float)(OFFSET_FRONT_X + usFrontLeft) * cos(DEG2RAD(phiPos/10)) - (float)OFFSET_FRONT_Y * sin(DEG2RAD(phiPos/10));
		//yFrontLeft = (float)yPos + (float)OFFSET_FRONT_Y * cos(DEG2RAD(phiPos/10)) + (float)(OFFSET_FRONT_X + usFrontLeft) * sin(DEG2RAD(phiPos/10));
		//
		//
		//// *******************************************************************************
		//// ****** Ultraschallüberwachung vorne, mit Überwachung ob außerhalb vom Tisch ***
		//// *******************************************************************************
		//
		//
		//// Abbruch ist nur wenn einer der beiden Sensoren im Spielfeld ein Hindernis detektiert
		//// Randbereich wurde mit Offset von 100mm deklariert
		//if ((xFrontRight > 100.0 && xFrontRight < 2900.0 && yFrontRight > 100.0 && yFrontRight < 1900.0) ||
		//(xFrontLeft > 100.0 && xFrontLeft < 2900.0 && yFrontLeft > 100.0 && yFrontLeft < 1900.0))
		//{
		//setAntrieb_RRTLAN(0, 0, 0, 0, 0, 0, 0, 0, MOTION_INTERRUPT, GEGNER_OFF);
		//gegnerErkennung = OFF;
		//
		//#ifdef _DEBUG_MOTION_
		//sprintf(text,"MOTION INTERRUPT IN FORWARD-DIRECTION !!!!\n");
		//writeString_usart(&WIFI_IF, text);
		//#endif
		//}
		//
		//}
		//}
		//// Roboter fährt rückwärts
		//else
		//{
		//if(usRearLeft < WATCH_DIS_FRONT || usRearRight < WATCH_DIS_FRONT)
		//{
		//// *******************************************************************************
		//// ****** Ultraschallüberwachung hinten, mit Überwachung ob außerhalb vom Tisch ***
		//// *******************************************************************************
		//float xRearRight;
		//float xRearLeft;
		//float yRearRight;
		//float yRearLeft;
		//
		//// X und Y Koordinaten vom Hindernis des linken hinteren Sensors
		//xRearLeft = (float)xPos - (float)(OFFSET_REAR_X + usRearLeft) * cos(DEG2RAD(phiPos/10)) - (float)OFFSET_REAR_Y * sin(DEG2RAD(phiPos/10));
		//yRearLeft = (float)yPos + (float)OFFSET_REAR_Y * cos(DEG2RAD(phiPos/10)) - (float)(OFFSET_REAR_X + usRearLeft) * sin(DEG2RAD(phiPos/10));
		//
		//// X und Y Koordinaten vom Hindernis des rechten hinteren Sensors
		//xRearRight = (float)xPos - (float)(OFFSET_REAR_X + usRearRight) * cos(DEG2RAD(phiPos/10)) + (float)OFFSET_REAR_Y * sin(DEG2RAD(phiPos/10));
		//yRearRight = (float)yPos - (float)OFFSET_REAR_Y * cos(DEG2RAD(phiPos/10)) - (float)(OFFSET_REAR_X + usRearRight) * sin(DEG2RAD(phiPos/10));
		//// *******************************************************************************
		//// ****** Ende Ultraschallüberwachung hinten, mit Überwachung ob außerhalb vom Tisch ***
		//// *******************************************************************************
		//
		//// Abbruch ist nur wenn einer der beiden Sensoren im Spielfeld ein Hindernis detektiert
		//// Randbereich wurde mit Offset von 100mm deklariert
		//if ((xRearRight > 100.0 && xRearRight < 2900.0 && yRearRight > 100.0 && yRearRight < 1900.0) ||
		//(xRearLeft > 100.0 && xRearLeft < 2900.0 && yRearLeft > 100.0 && yRearLeft < 1900.0))
		//{
		//setAntrieb_RRTLAN(0, 0, 0, 0, 0, 0, 0, 0, MOTION_INTERRUPT, GEGNER_OFF);
		//gegnerErkennung = OFF;
		//#ifdef _DEBUG_MOTION_
		//sprintf(text,"MOTION INTERRUPT IN BACKWARD-DIRECTION !!!!\n");
		//writeString_usart(&WIFI_IF, text);
		//#endif
		//}
		//
		//}
		//}
		//}
		
		/* ************* */
		/* motion was OK */
		/* ************* */
		if(statusAntrieb == MOTION_OK)
		{
			gegnerErkennung = OFF;
			/* end observation */
			observationStarted = OBSERVATION_PENDING;
			/* set result to MOTION OK */
			observationResult = OBSERVATION_MOTION_OK;
			
			#ifdef _DEBUG_MOTION_
			sprintf(text,"#MOTION OK - READY !!!! (%ld)\n*", (uint32_t)KI_StateNext);
			writeString_usart(&WIFI_IF, text);
			#endif
			
		}
		/* **************************** */
		/* motion interrupt has occured */
		/* **************************** */
		else if (statusAntrieb == MOTION_ENEMY_ERROR)
		{
			gegnerErkennung = OFF;
			/* end observation */
			observationStarted = OBSERVATION_PENDING;
			/* set result to MOTION ERROR */
			observationResult = OBSERVATION_MOTION_ERROR;
			
			#ifdef _DEBUG_MOTION_
			sprintf(text,"#MOTION ERROR - READY !!!! (%ld)\n*", (uint32_t)KI_StateError);
			writeString_usart(&WIFI_IF, text);
			#endif

		}
		/* *********************** */
		/* slippering was dedected */
		/* *********************** */
		else if (statusAntrieb == MOTION_SCHLEPP_ERROR)
		{
			/* end observation */
			observationStarted = OBSERVATION_PENDING;
			/* set result to MOTION OK */
			observationResult = OBSERVATION_MOTION_ERROR;
			

			#ifdef _DEBUG_MOTION_
			sprintf(text,"#SCHLEPPFEHLER!!!! (%ld)\n*", (uint32_t)KI_StateError);
			writeString_usart(&WIFI_IF, text);
			#endif
		}
		
	}
	
	return(CYCLE);
}


/* ************************************************************** */
/*! \brief observationAreaIsFree.
*
*  check if an area is free
*
*  \version 24.05.2023
*
*/
/* ************************************************************** */
uint8_t observationAreaIsFree(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	for (uint8_t i = 0; i < 5; i++)
	{
		if ((enemyRobot[i].Xpos > x0) && (enemyRobot[i].Xpos < x1) && (enemyRobot[i].Ypos > y0) && (enemyRobot[i].Ypos < y1))
		{
			return(FALSE);
		}
	}
	
	return(TRUE);
}


uint8_t GetObservationResult()
{
	return(observationResult);
}


