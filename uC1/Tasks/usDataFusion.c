/*
* us_data_fusion.c
*
* Created: 17.11.2021 10:44:49
*  Author: johan
*/

#define	 _US_DATA_FUSION_EXTERN

#include <avr/io.h>
#include "multitask.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "usDataFusion.h"
#include "enemyDetection.h"
#include "wifi.h"
#include "usart.h"

uint8_t counterWifi;
uint8_t counterEnemyDetection;
	uint8_t text1[150];
	uint8_t text2[150];
	
void usDataFusion_Init()
{
	/* cyclic task - cycle time: 100 ms */
	SET_CYCLE(US_DATA_SYNCHRON_TASKNBR, 100);
	SET_TASK(US_DATA_SYNCHRON_TASKNBR, CYCLE);
	SET_TASK_HANDLE(US_DATA_SYNCHRON_TASKNBR, dataFusionTask);
	
	counterWifi = 0;
	counterEnemyDetection = 0;

}

uint8_t dataFusionTask()
{
	SET_CYCLE(US_DATA_SYNCHRON_TASKNBR, 100);
	SET_TASK(US_DATA_SYNCHRON_TASKNBR, CYCLE); 

	sprintf(text1, "(%d:%d)/(%d:%d)/(%d:%d)/(%d:%d)\n", dynamicObstacles[0].pos.Xpos, dynamicObstacles[0].pos.Ypos,dynamicObstacles[1].pos.Xpos, dynamicObstacles[1].pos.Ypos
	,dynamicObstacles[2].pos.Xpos, dynamicObstacles[2].pos.Ypos,dynamicObstacles[3].pos.Xpos, dynamicObstacles[3].pos.Ypos);
	
	sprintf(text2, "*(%d:%d)/(%d:%d)/(%d:%d)/(%d:%d)\n", EnemyDataRaw.x[0], EnemyDataRaw.y[0], EnemyDataRaw.x[1], EnemyDataRaw.y[1]
	,EnemyDataRaw.x[2], EnemyDataRaw.y[2], EnemyDataRaw.x[3], EnemyDataRaw.y[3]);
	
	writeString_usart(&usartF0, text1);
	writeString_usart(&usartF0, text2);
	
	if ((validWIFI || counterWifi > 3) && (validEnemyDetection || counterEnemyDetection > 3 ))
	{
		// OUTPUT
		//enemyRobot[i]

	
		// INPUT
		//dynamicObstacles[i].pos + EnemyDataRaw + otherRobot
		
			
		for (uint8_t i = 0 ; i < 4; i++)
		{
			// Zweiter Roboter erkennt ersten Roboter
			if (abs(EnemyDataRaw.x[i] - xPos) < 250 && abs(EnemyDataRaw.y[i] - yPos) < 250
			&& EnemyDataRaw.x[i] > 100 && EnemyDataRaw.x[i] < 2900 && EnemyDataRaw.y[i] > 100 && EnemyDataRaw.y[i] < 1900)
			{
				EnemyDataRaw.x[i] = ED_NO_ENEMY_DETECTED;
				EnemyDataRaw.y[i] = ED_NO_ENEMY_DETECTED;
			}
				
			// Erster Roboter erkennt zweiten Roboter
			if (abs(dynamicObstacles[i].pos.Xpos - otherRobot.Xpos) < 250 && abs(dynamicObstacles[i].pos.Ypos - otherRobot.Ypos) < 250)
			{
				dynamicObstacles[i].pos.Xpos = ED_NO_ENEMY_DETECTED;
				dynamicObstacles[i].pos.Ypos = ED_NO_ENEMY_DETECTED;
			}
		}
		
		
		//Anzahl eingetragene Roboter
		uint8_t k = 0;
		
		//Übereinstimmungen prüfen
		for (uint8_t i = 0; i < 4;i++)
		{
			for (uint8_t j = 0; j < 4;j++)
			{
				if (abs(dynamicObstacles[i].pos.Xpos - EnemyDataRaw.x[i]) < 250 && abs(dynamicObstacles[i].pos.Ypos - EnemyDataRaw.y[i]) < 250
				&& dynamicObstacles[i].pos.Xpos != 0 && EnemyDataRaw.x[i] != 0)
				{
					//Werte mit Varianz mitteln
					
					//Derzeit noch Fehler bei Varianz - bekomme von Wlan 0 statt 50 ... (uint32_t)EnemyDataRaw.var) statt 50
					enemyRobot[k].Xpos =  (int16_t)(((uint32_t)dynamicObstacles[i].pos.Xpos * (uint32_t)dynamicObstacles[i].variance+(uint32_t)EnemyDataRaw.x[i]* (uint32_t)EnemyDataRaw.var[i])/((uint32_t)dynamicObstacles[i].variance+(uint32_t)EnemyDataRaw.var[i]));
					enemyRobot[k].Ypos =  (int16_t)(((uint32_t)dynamicObstacles[i].pos.Ypos * (uint32_t)dynamicObstacles[i].variance+(uint32_t)EnemyDataRaw.y[i]*(uint32_t)EnemyDataRaw.var[i])/((uint32_t)dynamicObstacles[i].variance+(uint32_t)EnemyDataRaw.var[i]));
					
					//Anzahl ingetragene Roboter +1
					k++;
					
					//Aus ursprünglicher Liste löschen
					dynamicObstacles[i].pos.Xpos=0;
					dynamicObstacles[i].pos.Ypos=0;
					EnemyDataRaw.x[i]= 0;
					EnemyDataRaw.y[i]= 0;
					break;
				}
				
			}
		}
		
		//Restliche Liste überprüfen (keine Übereinstimmung)
		for (uint8_t i = 0; i < 4;i++)
		{
			if (dynamicObstacles[i].pos.Xpos != 0 && k < 4)
			{
				enemyRobot[k].Xpos = dynamicObstacles[i].pos.Xpos;
				enemyRobot[k].Ypos = dynamicObstacles[i].pos.Ypos;
				k++;
			}
		}
		
		for (uint8_t i = 0; i < 4;i++)
		{
			if (EnemyDataRaw.x[i] != 0 && k < 4)
			{
				enemyRobot[k].Xpos = EnemyDataRaw.x[i];
				enemyRobot[k].Ypos = EnemyDataRaw.y[i];
				k++;
			}
		}
		
		//Falls weniger Roboter erkannt wurden als im vorherigen Durchlauf -> alten Eintrag löschen
		for (uint8_t i = k; i < 4; i++)
		{
				enemyRobot[k].Xpos = 0;
				enemyRobot[k].Ypos = 0;
				k++;
		}
		
		counterEnemyDetection = 0;
		counterWifi = 0;
		
	}
	
	else
	{
		if (validWIFI == 0 && counterWifi < 5)
		{
			counterWifi++;
		}
		if (validEnemyDetection == 0 && counterEnemyDetection < 5)
		{
			counterEnemyDetection++;
		}
	}
	
	sprintf(text1, "**(%d/%d)/(%d/%d)/(%d/%d)/(%d/%d)\n", enemyRobot[0].Xpos, enemyRobot[0].Ypos, enemyRobot[1].Xpos, enemyRobot[1].Ypos
	,enemyRobot[2].Xpos, enemyRobot[2].Ypos,enemyRobot[3].Xpos, enemyRobot[3].Ypos);
						
	writeString_usart(&usartF0, text1);
	
return(CYCLE);
}
