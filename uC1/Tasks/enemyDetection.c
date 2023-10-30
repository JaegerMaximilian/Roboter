/*
* enemyDetection.c
*
* Created: 12.05.2021 08:42:36
*  Author: anonym_lenovo
*/
#define _ENEMYDETECTION_EXTERN


#include <avr/io.h>
#include "multitask.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "usart.h"
#include <stdint.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "sensor.h"
#include "rrt_receivetask.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "rrt_transmittingtask.h"
#include "ki.h"
#include "command.h"
#include "enemyDetection.h"
#include "../../uC3/HAL/usart.h"
#include "wifi.h"
#include "usDataFusion.h"

// *******************************************************************************
// ****** Ultraschallüberwachung vorne, mit Überwachung ob außerhalb vom Tisch ***
// *******************************************************************************

//////////// Ultraschall vorne ///////////////
#define OFFSET_ENEMY_X		150.0
#define OFFSET_ENEMY_Y		150.0
uint8_t text1[150];

/* ************************************************************** */
/*! \brief Initialize Enemydetection-task.
*
*  Function initialize the debug-task
*
*  \version 11.09.2021
*
*/
/* ************************************************************** */
void InitEnemyDetection()
{
	/* cyclic task - cycle time: 100 ms */
	SET_CYCLE(ENEMEY_DETECTION_TASKNBR, 100);
	SET_TASK(ENEMEY_DETECTION_TASKNBR, CYCLE);
	SET_TASK_HANDLE(ENEMEY_DETECTION_TASKNBR, EnemyDetectionTask);
	
	// Front
	dynamicObstacles[0].usLeft = &usFrontLeft;
	dynamicObstacles[0].usRight = &usFrontRight;
	dynamicObstacles[0].range = 900;
	dynamicObstacles[0].offsetPhi = 0;
	dynamicObstacles[0].offsetX = OFFSET_FRONT_X;
	dynamicObstacles[0].offsetY = OFFSET_FRONT_Y;
	dynamicObstacles[0].variance = 50;
	dynamicObstacles[0].pos.Xpos = ED_NO_ENEMY_DETECTED;
	dynamicObstacles[0].pos.Ypos = ED_NO_ENEMY_DETECTED;
	
	//Rear
	dynamicObstacles[2].usLeft = &usRearRight;
	dynamicObstacles[2].usRight = &usRearLeft;
	dynamicObstacles[2].range = 900;
	dynamicObstacles[2].offsetPhi = 180;
	dynamicObstacles[2].offsetX = OFFSET_REAR_X;
	dynamicObstacles[2].offsetY = OFFSET_REAR_Y;
	dynamicObstacles[2].variance = 50;
	dynamicObstacles[2].pos.Xpos = ED_NO_ENEMY_DETECTED;
	dynamicObstacles[2].pos.Ypos = ED_NO_ENEMY_DETECTED;
	
	//Left
	dynamicObstacles[1].usLeft = &usLeftFront;
	dynamicObstacles[1].usRight = &usLeftFront;
	dynamicObstacles[1].range = 450;
	dynamicObstacles[1].offsetPhi = 90;
	dynamicObstacles[1].offsetX = OFFSET_FRONT_X;
	dynamicObstacles[1].offsetY = OFFSET_FRONT_Y;
	dynamicObstacles[1].variance = 50;
	dynamicObstacles[1].pos.Xpos = ED_NO_ENEMY_DETECTED;
	dynamicObstacles[1].pos.Ypos = ED_NO_ENEMY_DETECTED;
	
	//Right
	dynamicObstacles[3].usLeft = &usRightFront;
	dynamicObstacles[3].usRight = &usRightFront;
	dynamicObstacles[3].range = 450;
	dynamicObstacles[3].offsetPhi = 270;
	dynamicObstacles[3].offsetX = OFFSET_FRONT_X;
	dynamicObstacles[3].offsetY = OFFSET_FRONT_Y;
	dynamicObstacles[3].variance = 50;
	dynamicObstacles[3].pos.Xpos = ED_NO_ENEMY_DETECTED;
	dynamicObstacles[3].pos.Ypos = ED_NO_ENEMY_DETECTED;
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
uint8_t EnemyDetectionTask()
{
	for (uint8_t i=0; i<4;i++) // i+2 weil links und rechts keine US-Sensoren vorhanden sind!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	{
		int16_t phi = phiPos/10 + dynamicObstacles[i].offsetPhi;
		uint16_t usLeft = *(dynamicObstacles[i].usLeft);
		uint16_t usRight = *(dynamicObstacles[i].usRight);
		
		
		if(usRight < dynamicObstacles[i].range || usLeft < dynamicObstacles[i].range)
		{
			// TO DO => Function for variance
			dynamicObstacles[i].variance = 50;
		}
		
		if(usRight < dynamicObstacles[i].range && usLeft < dynamicObstacles[i].range)
		{
			// Wenn beide etwas detektieren muss der Mittelpunkt ausgerechnet werden
			float tempLeftX  =(float)xPos + (float)(dynamicObstacles[i].offsetX + OFFSET_ENEMY_X + usLeft)  * cos(DEG2RAD(phi)) - ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y)				 *	sin(DEG2RAD(phi));
			float tempLeftY  =(float)yPos + ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y)			* cos(DEG2RAD(phi)) + ((float)dynamicObstacles[i].offsetX + OFFSET_ENEMY_X + usLeft)	 *	sin(DEG2RAD(phi));
			float tempRightX =(float)xPos + (float)(dynamicObstacles[i].offsetX + OFFSET_ENEMY_X + usRight) * cos(DEG2RAD(phi)) + ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y)				 *	sin(DEG2RAD(phi));
			float tempRightY =(float)yPos - ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y)			* cos(DEG2RAD(phi)) + ((float)dynamicObstacles[i].offsetX + OFFSET_ENEMY_X + usRight)	 *	sin(DEG2RAD(phi));
			
			dynamicObstacles[i].pos.Xpos = (tempRightX + tempLeftX) / 2;
			dynamicObstacles[i].pos.Ypos = (tempRightY + tempLeftY) / 2;
			

			sprintf(text1, "Daten:(%d/%d/%d/%d/%d/%d)\n", usRight, usLeft, xPos, yPos, dynamicObstacles[i].offsetPhi, i);
			writeString_usart(&usartF0, text1);
			sprintf(text1, "Berechnung:(%d/%d/%d/%d)\n", tempLeftX, tempLeftY,tempRightX,tempRightY);
			writeString_usart(&usartF0, text1);
			sprintf(text1, "Ergebnis:(%d/%d)\n", dynamicObstacles[i].pos.Xpos, dynamicObstacles[i].pos.Ypos);
			writeString_usart(&usartF0, text1);
		}

		else if(usLeft < dynamicObstacles[i].range)
		{
			// X und Y Koordinaten vom Hindernis des linken vorderen Sensors
			dynamicObstacles[i].pos.Xpos = (float)xPos + (float)(dynamicObstacles[i].offsetX + usLeft + OFFSET_ENEMY_X) * cos(DEG2RAD(phi)) - ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y) * sin(DEG2RAD(phi));
			dynamicObstacles[i].pos.Ypos = (float)yPos + ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y) * cos(DEG2RAD(phi)) + (float)(dynamicObstacles[i].offsetX + usLeft + OFFSET_ENEMY_X) * sin(DEG2RAD(phi));

		}
		else if(usRight < dynamicObstacles[i].range)
		{
			// X und Y Koordinaten vom Hindernis des rechten vorderen Sensors
			dynamicObstacles[i].pos.Xpos = (float)xPos + (float)(dynamicObstacles[i].offsetX + OFFSET_ENEMY_X + usRight) * cos(DEG2RAD(phi)) + ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y) * sin(DEG2RAD(phi));
			dynamicObstacles[i].pos.Ypos = (float)yPos - ((float)dynamicObstacles[i].offsetY + OFFSET_ENEMY_Y) * cos(DEG2RAD(phi)) + (dynamicObstacles[i].offsetX + OFFSET_ENEMY_X +usRight) * sin(DEG2RAD(phi));

		}
		else
		{
			dynamicObstacles[i].pos.Xpos = ED_NO_ENEMY_DETECTED;
			dynamicObstacles[i].pos.Ypos = ED_NO_ENEMY_DETECTED;
		}
		
	}


	// 		// Sendet die Rohdaten des Lidar(Daten vom uC3) an den anderen Roboter
	char msg[200];
	sprintf(msg, "#e%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d%04d*",
	dynamicObstacles[0].pos.Xpos, dynamicObstacles[0].pos.Ypos, dynamicObstacles[0].variance,
	dynamicObstacles[1].pos.Xpos, dynamicObstacles[1].pos.Ypos, dynamicObstacles[1].variance,
	dynamicObstacles[2].pos.Xpos, dynamicObstacles[2].pos.Ypos, dynamicObstacles[2].variance,
	dynamicObstacles[3].pos.Xpos, dynamicObstacles[3].pos.Ypos, dynamicObstacles[3].variance);
	
	writeString_usart(&WIFI_IF, msg);
	
	
	SET_CYCLE(ENEMEY_DETECTION_TASKNBR,100);
	return(CYCLE);
	
	validEnemyDetection = 1;
}