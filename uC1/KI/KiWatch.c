/*
* KiWatch.c
*
* Created: 15.11.2023 20:09:49
*  Author: marku
*/

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>

#include "kiWatch.h"
#include "ki.h"
#include "define.h"
#include "global.h"
#include "command.h"
#include "Pfadplanung.h"
#include "multitask.h"
#include "logger.h"
#include "nextion.h"
#include "observation.h"

void InitKiWatch(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_TASK(KI_WATCH_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_TASKNBR, 200);
	SET_TASK_HANDLE(KI_WATCH_TASKNBR, KiWatchTask);
}

/**************************************************************************
***   FUNCTIONNAME:        KiWatchTask                                  ***
***   FUNCTION:            überwacht ob Aufgaben bereits erledigt sind  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiWatchTask(void)
{
	// zyklischer Task - Zykluszeit: 200 ms
	SET_CYCLE(KI_WATCH_TASKNBR, 100);

	static uint8_t CounterArea1000 = 0;
	static uint8_t CounterArea2000 = 0;
	static uint8_t CounterArea3000 = 0;
	static uint8_t CounterArea4000 = 0;
	static uint8_t CounterArea5000 = 0;
	static uint8_t CounterArea6000 = 0;
	
	static uint8_t InArea1000 = 0;
	static uint8_t InArea2000 = 0;
	static uint8_t InArea3000 = 0;
	static uint8_t InArea4000 = 0;
	static uint8_t InArea5000 = 0;
	static uint8_t InArea6000 = 0;
	
	uint8_t enemyRobotInPlanter1;
	uint8_t enemyRobotInPlanter2;
	uint8_t enemyRobotInPlanterMiddle;
	uint8_t enemyRobotInField1;
	static int8_t counterPlanter1 = 0;
	static int8_t counterPlanter2 = 0;
	static int8_t counterPlanterMiddle = 0;
	static int8_t counterField1 = 0;
	static int8_t counterSolarPanelsMiddle = 0;
	
	uint8_t enemyRobotInSolarPanelsMiddle;

	char text1[200];
	
	/**************************************************************************
	***   Set Plants as DID when enemy was there	                        ***
	**************************************************************************/
	InArea1000 = Path_IsInArea(1200,200,1800,8000);
	InArea2000 = Path_IsInArea(700, 400, 1300, 1000);
	InArea3000 = Path_IsInArea(700, 1000, 1300, 1600);
	InArea4000 = Path_IsInArea(1200, 1200, 1800, 1800);
	InArea5000 = Path_IsInArea(1700, 1000, 2300, 1600);
	InArea6000 = Path_IsInArea(1700, 400, 2300, 1000);
	
	//Position 1000
	if(InArea1000)
	{
		if(CounterArea1000 < MaxCountInArea)
		{
			CounterArea1000++;
		}
		
	}
	else
	{
		if(CounterArea1000 > 0)
		{
			CounterArea1000--;
		}

	}
	
	if(CounterArea1000 >= MaxCountInArea)
	{
		KI_Task[1].Status = DID;
	}
	
	//Position 2000
	if(InArea2000)
	{
		if(CounterArea2000 < MaxCountInArea)
		{
			CounterArea2000++;
		}
		
	}
	else
	{
		if(CounterArea2000 > 0)
		{
			CounterArea2000--;
		}

	}
	
	if(CounterArea2000 >= MaxCountInArea)
	{
		KI_Task[2].Status = DID;
	}
	
	//Position 3000
	if(InArea3000)
	{
		if(CounterArea3000 < MaxCountInArea)
		{
			CounterArea3000++;
		}
		
	}
	else
	{
		if(CounterArea3000 > 0)
		{
			CounterArea3000--;
		}

	}
	
	if(CounterArea3000 >= MaxCountInArea)
	{
		KI_Task[3].Status = DID;
	}
	
	//Position 4000
	if(InArea4000)
	{
		if(CounterArea4000 < MaxCountInArea)
		{
			CounterArea4000++;
		}
		
	}
	else
	{
		if(CounterArea4000 > 0)
		{
			CounterArea4000--;
		}

	}
	
	if(CounterArea4000 >= MaxCountInArea)
	{
		KI_Task[4].Status = DID;
	}
	
	//Position 5000
	if(InArea5000)
	{
		if(CounterArea5000 < MaxCountInArea)
		{
			CounterArea5000++;
		}
		
	}
	else
	{
		if(CounterArea5000 > 0)
		{
			CounterArea5000--;
		}

	}
	if(CounterArea5000 >= MaxCountInArea)
	{
		KI_Task[5].Status = DID;
	}
	
	//Position 6000
	if(InArea6000)
	{
		if(CounterArea6000 < MaxCountInArea)
		{
			CounterArea6000++;
		}
		
	}
	else
	{
		if(CounterArea6000 > 0)
		{
			CounterArea6000--;
		}

	}
	
	if(CounterArea6000 >= MaxCountInArea)
	{
		KI_Task[6].Status = DID;
	}
	
	
	/**************************************************************************
	***   Set Park Positions to PENDING when enemy is there	                ***
	**************************************************************************/
	if(SpielFarbe == BLUE)
	{
		//Planter 1
		enemyRobotInPlanter1 = Path_IsInArea(2550,312,3000,912);
		if(!enemyRobotInPlanter1)
		{
			counterPlanter1 = ((++counterPlanter1 > 10) ? 10 : counterPlanter1);
		}
		else
		{
			counterPlanter1 = ((--counterPlanter1 < 0) ? 0 : counterPlanter1);
		}
		
		if(enemyRobotInPlanter1 && KI_Task[13].Status == OPEN)
		{
			KI_Task[13].Status = PENDING;
		}
		else if(!enemyRobotInPlanter1 && KI_Task[13].Status == PENDING && counterPlanter1 >= 10)
		{
			KI_Task[13].Status = OPEN;
			counterPlanter1 = 0;
		}
		
		//Planter 2
		
		enemyRobotInPlanter2 = Path_IsInArea(0,1000,600,2000);
		if(!enemyRobotInPlanter2)
		{
			counterPlanter2 = ((++counterPlanter2 > 10) ? 10 : counterPlanter2);
		}
		else
		{
			counterPlanter2 = ((--counterPlanter2 < 0) ? 0 : counterPlanter2);
		}
		
		if(enemyRobotInPlanter2 && KI_Task[25].Status == OPEN)
		{
			KI_Task[25].Status = PENDING;
			
		}

		else if(!enemyRobotInPlanter2 && KI_Task[25].Status == PENDING && ConfigPlanter && counterPlanter2 >= 10) // wenn Gegner stehen bleibt || (!ConfigPlanter && SpielZeit > 50)
		{
			KI_Task[25].Status = OPEN;
			counterPlanter2 = 0;
		}
		
		//Planter Middle
		enemyRobotInPlanterMiddle = Path_IsInArea(1937,0,2537,450);
		if(!enemyRobotInPlanterMiddle)
		{
			counterPlanterMiddle = ((++counterPlanterMiddle > 10) ? 10 : counterPlanterMiddle);
		}
		else
		{
			counterPlanterMiddle = ((--counterPlanterMiddle < 0) ? 0: counterPlanterMiddle);
		}
		
		if(enemyRobotInPlanterMiddle && KI_Task[11].Status == OPEN)
		{
			KI_Task[11].Status = PENDING;
		}
		else if(!enemyRobotInPlanterMiddle && KI_Task[11].Status == PENDING && counterPlanterMiddle >=10)
		{
			KI_Task[11].Status = OPEN;
			counterPlanterMiddle = 0;
		}
		
		//Field 1
		enemyRobotInField1 = Path_IsInArea(2550,0,3000,450);
		if(!enemyRobotInField1)
		{
			counterField1 = ((++counterField1 > 10) ? 10 : counterField1);
		}
		else
		{
			counterField1 = ((--counterField1 < 0) ? 0 : counterField1);
		}
		
		if(enemyRobotInField1 && KI_Task[12].Status == OPEN)
		{
			KI_Task[12].Status = PENDING;
		}
		else if(!enemyRobotInField1 && KI_Task[12].Status == PENDING && counterField1 >= 10)
		{
			KI_Task[12].Status = OPEN;
			counterField1 = 0;
		}
		
	}
	
	if(SpielFarbe == Yellow)
	{
		//Planter 1
		enemyRobotInPlanter1 = Path_IsInArea(0,312,450,912);
		if(!enemyRobotInPlanter1)
		{
			counterPlanter1 = ((++counterPlanter1 > 10) ? 10 : counterPlanter1);
		}
		else
		{
			counterPlanter1 = ((--counterPlanter1 < 0) ? 0 : counterPlanter1);
		}
		
		if(enemyRobotInPlanter1 && KI_Task[23].Status == OPEN)
		{
			KI_Task[23].Status = PENDING;
		}
		else if(!enemyRobotInPlanter1 && KI_Task[23].Status == PENDING && counterPlanter1 >= 10)
		{
			KI_Task[23].Status = OPEN;
			counterPlanter1 = 0;
		}
		
		//Planter 2
		enemyRobotInPlanter2 = Path_IsInArea(2400,1000,3000,2000);
		if(!enemyRobotInPlanter2)
		{
			counterPlanter2 = ((++counterPlanter2 > 10) ? 10 : counterPlanter2);
		}
		else
		{
			counterPlanter2 = ((--counterPlanter2 < 0) ? 0 : counterPlanter2);
		}
		
		if(enemyRobotInPlanter2 && KI_Task[15].Status == OPEN)
		{
			KI_Task[15].Status = PENDING;
		}
		else if(!enemyRobotInPlanter2 && KI_Task[15].Status == PENDING && ConfigPlanter && counterPlanter2 >= 10) // wenn Gegner stehen bleibt || (!ConfigPlanter && SpielZeit > 50)
		{
			KI_Task[15].Status = OPEN;
			counterPlanter2 = 0;
		}
		
		//Planter Middle
		enemyRobotInPlanterMiddle = Path_IsInArea(462,0,1062,450);
		if(!enemyRobotInPlanterMiddle)
		{
			counterPlanterMiddle = ((++counterPlanterMiddle > 10) ? 10 : counterPlanterMiddle);
		}
		else
		{
			counterPlanterMiddle = ((--counterPlanterMiddle < 0) ? 0 : counterPlanterMiddle);
		}
		
		if(enemyRobotInPlanterMiddle && KI_Task[21].Status == OPEN)
		{
			KI_Task[21].Status = PENDING;
		}
		else if(!enemyRobotInPlanterMiddle && KI_Task[21].Status == PENDING && counterPlanterMiddle >= 10)
		{
			KI_Task[21].Status = OPEN;
			counterPlanterMiddle = 0;
		}
		
		//Field 1
		enemyRobotInField1 = Path_IsInArea(0,0,450,450);
		if(!enemyRobotInField1)
		{
			counterField1 = ((++counterField1 > 10) ? 10 : counterField1);
		}
		else
		{
			counterField1 = ((--counterField1 < 0) ? 0 : counterField1);
		}
		
		if(enemyRobotInField1 && KI_Task[22].Status == OPEN)
		{
			KI_Task[22].Status = PENDING;
		}
		else if(!enemyRobotInField1 && KI_Task[22].Status == PENDING && counterField1 >= 10)
		{
			KI_Task[22].Status = OPEN;
			counterField1 = 0;
		}
	}
	
	/**************************************************************************
	***   Solar Panels to Is Doing when Enemy is there						***
	**************************************************************************/
	enemyRobotInSolarPanelsMiddle = Path_IsInArea(1000,1600,2000,1600);
	
	if(!enemyRobotInSolarPanelsMiddle)
	{
		counterSolarPanelsMiddle = ((++counterSolarPanelsMiddle > 10) ? 10 : counterSolarPanelsMiddle);
	}
	else
	{
		counterSolarPanelsMiddle = ((--counterSolarPanelsMiddle < 0) ? 0 : counterSolarPanelsMiddle);
	}
	
	if(enemyRobotInSolarPanelsMiddle)
	{
		KI_Task[31].Status = IS_DOING;
	}
	else if(!enemyRobotInSolarPanelsMiddle && KI_Task[31].Status == IS_DOING && counterSolarPanelsMiddle >= 10)
	{
		KI_Task[31].Status = OPEN;
		counterSolarPanelsMiddle = 0;
	}

	/**************************************************************************
	***   Set Solar Panels Middle to Done if not enought Time               ***
	**************************************************************************/
	
	if (spielZeit < (4 *TimeSolarpanels + 2*TimeSetPosAtSolarPanels + TimeSolarPanelsMiddleToHome + TimeParkPlantsAtHome + TimeToSolarPanelsMiddle)) //Each Solor Panels + Get Position At Solar Panels + Time to drive
	{
		KI_Task[31].Status = DONE;
	}
	
	/**************************************************************************
	***   Set Solar Panels at Home Position to Done if not enought Time     ***
	**************************************************************************/
	if(spielZeit < TimeSolarPanelsHome + TimeToHome + TimeParkPlantsAtHome)
	{
		KI_Task[30].Status = DONE;
		KI_Task[32].Status = DONE;
	}
	
	/**************************************************************************
	***   Calculate Velocity of Enemy									    ***
	**************************************************************************/
	
	//Überwachung wenn Positionsänderung größer als 0,5 m ist ==> fail
	//Position g
	point_t aktPoint, oldPoint;
	float distance;
	uint16_t timedif;
	static uint8_t started = 0;
	static uint16_t oldTime = 1000;
	static uint16_t enemyStandstillCounter = 0;
	
	
	//Init
	if(started == 0)
	{
		started = 1;
		oldPoint.Xpos = enemyRobot[0].Xpos;
		oldPoint.Ypos = enemyRobot[0].Ypos;
	}
	
	aktPoint.Xpos = enemyRobot[0].Xpos;
	aktPoint.Ypos = enemyRobot[0].Ypos;
	
	if((aktPoint.Xpos == oldPoint.Xpos) && (aktPoint.Ypos == oldPoint.Ypos))
	{
		enemyStandstillCounter = enemyStandstillCounter <= 5 ? enemyStandstillCounter++ : 5;
	}
	else
	{
		enemyStandstillCounter = 0;
	}
	
	//Calcualte Enemy Speed
	if(aktPoint.Xpos != 10000 && oldPoint.Xpos != 10000 && (oldTime != spielZeit)
	&& ((enemyStandstillCounter >= 5) || (aktPoint.Xpos != oldPoint.Xpos) || (aktPoint.Ypos != oldPoint.Ypos)))
	{
		distance = CalcDistance(aktPoint,oldPoint);
		timedif = oldTime - spielZeit;
		
		oldTime = spielZeit;
		
		VelocityEnemy = distance / (float)timedif;
		
		VelocityEnemy = ((VelocityEnemy>700.0) ? 700.0 : VelocityEnemy);
		
		sprintf(text1, "Velocity: %f", VelocityEnemy);
		SendDebugMessage(text1,2);
		
	}

	if(aktPoint.Xpos != oldPoint.Xpos)
	{
		oldPoint.Xpos = aktPoint.Xpos;
	}
	if(aktPoint.Ypos != oldPoint.Ypos)
	{
		oldPoint.Ypos = aktPoint.Ypos;
	}
	
	/**************************************************************************
	***   Show Change of State												***
	**************************************************************************/
	if(KI_State != OldKI_State)
	{
		sprintf(text1, "State: %6ld PIR: %d OPP: %d OP: %d ParP: %d", (uint32_t)KI_State, PlantsInRobot , OpenParkPos, OpenPlants, ParkedPlants);
		SendDebugMessage(text1,1);
		
		//sprintf(text1, "TPNP: %d", TimeParkNextPlant);
		//SendDebugMessage(text1,1);
	}
	
	

	OldKI_State = KI_State;
	
	return(CYCLE);
}