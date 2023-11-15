/*
* kiWatchRobotPosition.c
*
* Created: 15.11.2023 20:30:17
*  Author: marku
*/

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>

#include "kiWatchRobotPosition.h"
#include "ki.h"
#include "define.h"
#include "global.h"
#include "command.h"
#include "Pfadplanung.h"
#include "multitask.h"

void InitKiWatchRobotPositionTask(void)
{
	// KiWatchRobotPositionTask initialisieren
	SET_TASK(KI_WATCH_ROBOT_POS_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_ROBOT_POS_TASKNBR, 100);
	SET_TASK_HANDLE(KI_WATCH_ROBOT_POS_TASKNBR, KiWatchRobotPositionTask);
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
