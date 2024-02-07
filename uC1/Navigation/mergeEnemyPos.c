/*
 * mergeEnemyPos.c
 *
 * Created: 06.02.2024 16:56:37
 *  Author: Markus Zehetner	
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>

#include "define.h"
#include "observation.h"
#include "../../uC2/RTOS/multitask.h"
#include "mergeEnemyPos.h"
#include <stdint.h>
#include "global.h"

void InitMergeEnemyPos(void)
{
	DISABLE_TASK(MERGEENEMYPOS_TASKNBR);
	SET_TASK_HANDLE(MERGEENEMYPOS_TASKNBR, MergeEnemyPosTask);
}

uint8_t MergeEnemyPosTask(void)
{

	/**************************************************************************
	***   merge Enemy Positions from Camera and Lidar                       ***
	***   ---------------------------------------------------------------   ***
	***   Date    :  18.01.2024                                             ***
	***   Author  :  Markus Zehetner						                ***
	**************************************************************************/
	uint16_t timedif = abs(enemyRobotLidar[0].time - enemyPosRobotKamera[0].time);
	
	for (uint8_t i = 0; i <= 5; i++)
	{		
		enemyRobot[i].Xpos = 10000;
		enemyRobot[i].Ypos = 10000;
	}
	
	if((timedif <= 5) && (enemyRobotLidar[0].point.Xpos != 10000))
	{
		enemyRobot[0].Xpos = (enemyRobotLidar[0].point.Xpos + enemyPosRobotKamera[0].point.Xpos)/2;
		enemyRobot[0].Ypos = (enemyRobotLidar[0].point.Ypos + enemyPosRobotKamera[0].point.Ypos)/2;
	}
	else
	{
		if((enemyRobotLidar[0].time >= enemyPosRobotKamera[0].time) && (enemyRobotLidar[0].point.Xpos == 10000))
		{
			enemyRobot[0].Xpos = enemyPosRobotKamera[0].point.Xpos;
			enemyRobot[0].Ypos = enemyPosRobotKamera[0].point.Ypos;
		}
		else
		{
			enemyRobot[0].Xpos = enemyRobotLidar[0].point.Xpos;
			enemyRobot[0].Ypos = enemyRobotLidar[0].point.Ypos;
			
		}
	}
	//sprintf(text1, "Lidar: %d Kam: %d En: %d  ",enemyRobotLidar[0].point.Xpos, enemyPosRobotKamera[0].point.Xpos, enemyRobot[0].Xpos);
	//SendDebugMessage(text1,2);
	return(DISABLE);	
}


