/*
* ki_helper.c
*
* Created: 15.11.2023 19:14:22
*  Author: marku
*/
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>

#include "ki.h"
#include "ki_helper.h"
#include "define.h"
#include "global.h"
#include "command.h"
#include "Pfadplanung.h"
#include "observation.h"
#include "logger.h"
#include "nextion.h"

/**************************************************************************
***   FUNKTIONNAME: AddMiddlePoint                                      ***
***   FUNKTION: Add a middle Point before Goal-Point                    ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
point_t AddMiddlePoint(point_t start, point_t ziel )
{
	point_t middlePoint;
	float StreckeX = (float)ziel.Xpos-(float)start.Xpos;
	float StreckeY = (float)ziel.Ypos-(float)start.Ypos;
	float HypotenuseC = CalcDistance(start,ziel);
	
	middlePoint.Xpos = (int16_t)((((HypotenuseC-250)/HypotenuseC)*StreckeX)+start.Xpos);
	middlePoint.Ypos = (int16_t)((((HypotenuseC-250)/HypotenuseC)*StreckeY)+start.Ypos);
	return middlePoint;
}

/**************************************************************************
***   FUNKTIONNAME: ChangePrioToYellow                                  ***
***   FUNKTION: �ndern der Prios von Blau auf Gelb                     ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void ChangePrioToYellow(void)
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
***   FUNKTIONNAME: AktivatePlantsAsObstacle                            ***
***   FUNKTION: Sets Plants which arent used in the actual plan as		***
***   Obstacles															***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void ActivatePlantAsObstacle(void)
{
	for (int i = 1; i<7; i++)
	{
		if(KI_Task[i].Status == PENDING)
		{
			PATH_ENABLE_OBSTACLE(i+2);
		}
		else
		{
			PATH_DISABLE_OBSTACLE(i+2);
		}
	}
}

/**************************************************************************
***   FUNKTIONNAME: Drive Back										    ***
***   FUNKTION: Drives a defined Way Back and checks if					***
***	  position is inside field											***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
uint8_t DriveBack(uint8_t distance, uint8_t speed)
{
	if((xPos>(200+distance))&&(xPos<(2800-distance))&&(yPos>(200+distance))&&(yPos<(1800-distance)))
	{
		cmd_Drive(0,0,-speed,0,0,0,0,distance,POS_REL,OFF,NULL,NULL,STANDARD_ACC,PLANT_ACC);
		return(1);
	}
	return(0);
}
/**************************************************************************
***   FUNKTIONNAME: CalcDistance										***
***   FUNKTION: Calculates distance from one point to another			***
***	  position is inside field											***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
float CalcDistance(point_t firstPoint, point_t secondPoint)
{
	float distance;
	
	distance = sqrtf(pow(((float)firstPoint.Xpos - (float)secondPoint.Xpos), 2.0) + pow(((float)firstPoint.Ypos - (float)secondPoint.Ypos), 2.0));
	
	return(distance);
}

/**************************************************************************
***   FUNKTIONNAME: RePrioritise Plants									***
***   FUNKTION: RePrioritise Plants										***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void RePrioritisePlantTasks(void)
{
	//Distance to Plants and Enemy
	point_t aktpos;
	aktpos.Xpos = xPos;
	aktpos.Ypos = yPos;
	
	float distance_1000 = CalcDistance(aktpos,PosPlant1000);
	float distance_2000 = CalcDistance(aktpos,PosPlant2000);
	float distance_3000 = CalcDistance(aktpos,PosPlant3000);
	float distance_4000 = CalcDistance(aktpos,PosPlant4000);
	float distance_5000 = CalcDistance(aktpos,PosPlant5000);
	float distance_6000 = CalcDistance(aktpos,PosPlant6000);
	float distanceEnemy_1000 = 3000.0;
	float distanceEnemy_2000 = 3000.0;
	float distanceEnemy_3000 = 3000.0;
	float distanceEnemy_4000 = 3000.0;
	float distanceEnemy_5000 = 3000.0;
	float distanceEnemy_6000 = 3000.0;
	
	//Distance to enemy
	for (int i = 0;i<5;i++)
	{
		if (enemyRobot[i].Xpos != 10000 && enemyRobot[i].Ypos != 10000)
		{
			if(CalcDistance(PosPlant1000,enemyRobot[i])<distanceEnemy_1000)
			{
				distanceEnemy_1000 = CalcDistance(PosPlant1000,enemyRobot[i]);
			}
			if(CalcDistance(PosPlant2000,enemyRobot[i])<distanceEnemy_2000)
			{
				distanceEnemy_2000 = CalcDistance(PosPlant2000,enemyRobot[i]);
			}
			if(CalcDistance(PosPlant3000,enemyRobot[i])<distanceEnemy_3000)
			{
				distanceEnemy_3000 = CalcDistance(PosPlant3000,enemyRobot[i]);
			}
			if(CalcDistance(PosPlant4000,enemyRobot[i])<distanceEnemy_4000)
			{
				distanceEnemy_4000 = CalcDistance(PosPlant4000,enemyRobot[i]);
			}
			if(CalcDistance(PosPlant5000,enemyRobot[i])<distanceEnemy_5000)
			{
				distanceEnemy_5000 = CalcDistance(PosPlant5000,enemyRobot[i]);
			}
			if(CalcDistance(PosPlant6000,enemyRobot[i])<distanceEnemy_6000)
			{
				distanceEnemy_6000 = CalcDistance(PosPlant6000,enemyRobot[i]);
			}
		}
	}
	
	//Whole Distance
	float quantifierEnemy = 1.0;
	
	float wholeDistance_1000 = distance_1000; //+ 0.8*(3000.0 - distanceEnemy_1000);
	float wholeDistance_2000 = distance_2000;// + 0.8*(3000.0 - distanceEnemy_2000);
	float wholeDistance_3000 = distance_3000;	// + 0.8*(3000.0 - distanceEnemy_3000);
	float wholeDistance_4000 = distance_4000; //+ 0.8*(3000.0 - distanceEnemy_4000);
	float wholeDistance_5000 = distance_5000; //+ 0.8*(3000.0 - distanceEnemy_5000);
	float wholeDistance_6000 = distance_6000; // + 0.8*(3000.0 - distanceEnemy_6000);
	
	char text1 [200];
	
	sprintf(text1, "%d; %d; %d; %d; %d; %d", (uint16_t)distanceEnemy_1000,(uint16_t)distanceEnemy_2000,(uint16_t)distanceEnemy_3000,(uint16_t)distanceEnemy_4000,(uint16_t)distanceEnemy_5000,(uint16_t)distanceEnemy_6000);
	SendDebugMessage(text1,2);
	
	//Sort values
	float a = 0;
	float distances[] =  {wholeDistance_1000,wholeDistance_2000,wholeDistance_3000,wholeDistance_4000,wholeDistance_5000,wholeDistance_6000 };

	for (int i = 0; i < 6; i++)
	{
		for (int j = i + 1; j < 6; j++)
		{
			if (distances[i] > distances[j])
			{
				a = distances[i];
				distances[i] = distances[j];
				distances[j] = a;
			}
		}
	}
	
	//Prioritis to Task
	for (int i = 0; i < 6; i++)
	{
		if(distances[i] == wholeDistance_1000)
		{
			KI_Task[1].Priority = 96-i;
		}
		if(distances[i] == wholeDistance_2000)
		{
			KI_Task[2].Priority = 96-i;
		}
		if(distances[i] == wholeDistance_3000)
		{
			KI_Task[3].Priority = 96-i;
		}
		if(distances[i] == wholeDistance_4000)
		{
			KI_Task[4].Priority = 96-i;
		}
		if(distances[i] == wholeDistance_5000)
		{
			KI_Task[5].Priority = 96-i;
		}
		if(distances[i] == wholeDistance_6000)
		{
			KI_Task[6].Priority = 96-i;
		}
	}
}


/**************************************************************************
***   FUNKTIONNAME: CalcTimeMoveSolarPanelsMiddle					    ***
***   FUNKTION: Calc time to Move Solar Panels Middle					***
***   TRANSMIT PARAMETER: float                                         ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
float CalcTimeMoveSolarPanelsMiddle(void)
{
	float fixedVelocity = 0.3; // in [m/s]
	
	point_t aktpos;
	aktpos.Xpos = xPos;
	aktpos.Ypos = yPos;
	
	point_t solarPanelsMiddle;
	solarPanelsMiddle.Xpos = 1500; // ENTER REAL VALUES OR MAKE THEM GLOBAL AT LEAST
	solarPanelsMiddle.Ypos = 2000; // for clarity, use the postion of the middle solar panel of the 3
	
	float totalDistance = CalcDistance(aktpos,solarPanelsMiddle);
	
	float timeToDrive = fixedVelocity / totalDistance;
	
	float timeToMoveSolarPanels = 7.00; // [s] ERFAHRUNGSWERT
	
	return timeToDrive + timeToMoveSolarPanels;
}

/**************************************************************************
***   FUNKTIONNAME: OpenPlants											***
***   FUNKTION: OpenPlants												***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void CalcOpenPlants(void)
{
	OpenPlants = 0;
	for (int i = 1; i < 7; i++)
	{
		if(KI_Task[i].Status == OPEN || KI_Task[i].Status == PENDING)
		{
			OpenPlants++;
		}
	}
}

/**************************************************************************
***   FUNKTIONNAME: CalcOpen Park-Positions 							***
***   FUNKTION: Calculate Open Park Positions							***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void CalcOpenParkPositions(void)
{
	OpenParkPos = 0;
	for (int i = 11; i < 27; i++)
	{
		if(KI_Task[i].Status == OPEN )
		{
			OpenParkPos++;
		}
	}
}

/**************************************************************************
***   FUNKTIONNAME: CalcOpen Planter		 							***
***   FUNKTION: Calculate Open Park Planter 							***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void CalcOpenPlanter(void)
{
	OpenPlanter = 0;
	if(KI_Task[11].Status == OPEN )
	{
		OpenPlanter++;
	}
	if(KI_Task[13].Status == OPEN )
	{
		OpenPlanter++;
	}
	if(KI_Task[15].Status == OPEN )
	{
		OpenPlanter++;
	}
	if(KI_Task[21].Status == OPEN )
	{
		OpenPlanter++;
	}
	if(KI_Task[23].Status == OPEN )
	{
		OpenPlanter++;
	}
	if(KI_Task[25].Status == OPEN )
	{
		OpenPlanter++;
	}
}

/**************************************************************************
***   FUNKTIONNAME: CalcTimeRemainingPlants								***
***   FUNKTION: Calculates Time it probably takes to Grab another Plant
and then park all Plants that are in the Robot                          ***
***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
uint16_t CalcTimeRemainingPlants(void)
{
	float totalDistance = 0.0;
	
	// SCHRITT 1:
	// FIND NEXT PLANT WITH MAX PRIO IF THERE IS ANY

	int IndexMaxPrioPflanze = 0;
	int MaxPrio = 0;
	uint16_t timeHandlenextPlant = 0;
	uint16_t waysToDrive = 0;
	uint8_t PrivatePlantsInRobot = PlantsInRobot;
	
	for (int i = 1; i <= 6; i++)
	{

		if((KI_Task[i].Status == OPEN || KI_Task[i].Status == PENDING) && KI_Task[i].Priority > MaxPrio )
		{
			MaxPrio = KI_Task[i].Priority;
			IndexMaxPrioPflanze = i;
			
			timeHandlenextPlant = TimeHandleNextPlant;
			PrivatePlantsInRobot = PrivatePlantsInRobot + 1;
			waysToDrive = waysToDrive + 1;
		}
		
	}
	// nach dem testen das Printen in den Logger wieder auskommentieren!!!!
	
	char text1[300];
	char text2[300];
	//sprintf(text1, "Next Plant: %d", IndexMaxPrioPflanze);
	//SendDebugMessage(text1,1);

	// SCHIRTT 2:
	// CALC DISTANCE TO MAXPRIO-PLANT AND MAKE IT TOTAL DISTANCE

	point_t aktpos;
	aktpos.Xpos = xPos;
	aktpos.Ypos = yPos;
	
	if(IndexMaxPrioPflanze == 1){
		totalDistance = CalcDistance(aktpos,PosPlant1000);
		aktpos = PosPlant1000;
		} else if (IndexMaxPrioPflanze == 2){
		totalDistance = CalcDistance(aktpos,PosPlant2000);
		aktpos = PosPlant2000;
		} else if (IndexMaxPrioPflanze == 3){
		totalDistance = CalcDistance(aktpos,PosPlant3000);
		aktpos = PosPlant3000;
		} else if (IndexMaxPrioPflanze == 4){
		totalDistance = CalcDistance(aktpos,PosPlant4000);
		aktpos = PosPlant4000;
		} else if (IndexMaxPrioPflanze == 5){
		totalDistance = CalcDistance(aktpos,PosPlant5000);
		aktpos = PosPlant5000;
		} else if (IndexMaxPrioPflanze == 6){
		totalDistance = CalcDistance(aktpos,PosPlant6000);
		aktpos = PosPlant6000;
	}
	

	// SCHRITT 3:
	// BEREITE PrivateKI_Task[] vor um Prios der Planter nicht global herumzuschieben
	
	task_t PrivateKI_Task[MAX_KI_TASKS];
	point_t PlanterOrFieldPos;
	
	for(int i= 0; i<65;i++)
	{
		PrivateKI_Task[i] = KI_Task[i];
	}

	// SCHRITT 4:
	// Berechne Zeit die es dauert, um alle Pflanzen der Reihe nach bei versch. Plantern abzustellen!!
	uint8_t IndexNextPlanter;
	uint16_t TimeToPark = 0;

	for(int i = 0; i< PrivatePlantsInRobot; i++)
	{
		IndexNextPlanter = PrivateSearchNextPlanter(aktpos, PrivateKI_Task);
		

		//sprintf(text1, "Planned Planter: %d PIR: %d", IndexNextPlanter,PrivatePlantsInRobot);
		//SendDebugMessage(text1,1);

		if (IndexNextPlanter==11){
			PlanterOrFieldPos = PosPlanterMidleBlue;
			} else if (IndexNextPlanter == 12){
			PlanterOrFieldPos = PosFieldL1;
			} else if (IndexNextPlanter == 13){
			PlanterOrFieldPos = PosPlanterL1;
			
			} else if (IndexNextPlanter == 15){
			PlanterOrFieldPos = PosPlanterL2;
			} else if (IndexNextPlanter == 16){
			PlanterOrFieldPos = PosFieldL3;
			} else if (IndexNextPlanter == 21){
			PlanterOrFieldPos = PosPlanterMidleYellow;
			} else if (IndexNextPlanter == 22){
			PlanterOrFieldPos = PosFieldR1;
			} else if (IndexNextPlanter == 23){
			PlanterOrFieldPos = PosPlanterR1;
			
			} else if (IndexNextPlanter == 25){
			PlanterOrFieldPos = PosPlanterR2;
			} else if (IndexNextPlanter == 26){
			PlanterOrFieldPos = PosFieldR3;
		}
		
		totalDistance = totalDistance + CalcDistance(aktpos, PlanterOrFieldPos); // add distance
		waysToDrive = waysToDrive + 1;
		

		if (IndexNextPlanter == 12 || IndexNextPlanter == 14 || IndexNextPlanter == 22 || IndexNextPlanter == 24 ) // if park at fields
		{
			TimeToPark =  TimeToPark + TimeParkNextPlantInField;
		}
		else if (IndexNextPlanter ==  11 || IndexNextPlanter ==  13 || IndexNextPlanter ==  15 || IndexNextPlanter ==  21 ||
		IndexNextPlanter ==  23 || IndexNextPlanter ==  25 ) // if park at planters
		{
			TimeToPark = TimeToPark + TimeParkNextPlantInPlanter;
		}
		
			// ANMERKUNGEN F�R N�CHSTES MAL: 
			
			// �berpr�fen ob er auch erkennt, wenn mehrere Pflanzen Daheim abgestellt werden, oder ob er immer denkt dass wir
			// nur die letzte Pflanze daheim abstellen!!!

		aktpos = PlanterOrFieldPos;
		
		if(IndexNextPlanter != 0)
		{
			PrivateKI_Task[IndexNextPlanter].Status = DONE;
		}
		else
		{
			break;
		}
	}
	
	// SCHRITT 4.5: ADDIERE WEG VON PLANTER ZU SOLARPANELS MITTE UND VON SOLARPANELS MITTE ZU HOME
	if (SpielFarbe == BLUE)
	{
		totalDistance = totalDistance + CalcDistance(PlanterOrFieldPos,PosPlanterMidleBlue) + CalcDistance(PosPlanterMidleBlue,PosFieldL3);
		waysToDrive = waysToDrive + 2;
	}
	else if (SpielFarbe == Yellow)
	{
		totalDistance = totalDistance + CalcDistance(PlanterOrFieldPos,PosPlanterMidleYellow) + CalcDistance(PosPlanterMidleYellow,PosFieldR3);
		waysToDrive = waysToDrive + 2;
	}
	
	
	// LETZTER SCHRITT:
	// devide a fixed Velocity by the total Distance
	
	uint16_t spazi = 0;

	uint16_t timeToDrive = ((uint16_t)((totalDistance/ STANDARD_VELOCITY)*10.0) + timeHandlenextPlant + TimeToPark +  spazi + waysToDrive*TimeForACCAndDCC);
	
	
	//sprintf(text1, "TTP: %d Dis: %f", TimeToPark,totalDistance);
	//SendDebugMessage(text1,1);
	
	return(timeToDrive);
}

/**************************************************************************
***   FUNKTIONNAME: PrivateSearchNextPlanter		 					   ***
***   FUNKTION: Change Prios of Private Planter Classes and return highest ***
***   TRANSMIT PARAMETER: int                                              ***
***   RECEIVE PARAMETER.:												   ***
**************************************************************************/
uint8_t PrivateSearchNextPlanter(point_t aktpos, task_t PrivateKI_Task[])
{
	
	
	point_t PlanterMidlePos, Planter2Pos, field1Pos, field3Pos, aktPos, Planter1Pos;
	float distance_PlanterMidle;
	float distance_Planter2;
	float distance_Planter1;
	float distance_Field1;
	float distance_Field3;
	int enemyRobotInPlanter2;
	
	// Start Position to begin movement from
	//Blue
	if(SpielFarbe == BLUE )
	{
		// Position of Planter Midle
		PlanterMidlePos = PosPlanterMidleBlue;
		//Position of Planter 1
		Planter1Pos = PosPlanterL1;
		//Position of Planter 2
		Planter2Pos = PosPlanterR2;
		//Position of Field 1
		field1Pos = PosFieldL1;
		//Position of Field 3
		field3Pos = PosFieldL3;
	}
	if(SpielFarbe == Yellow )
	{
		// Position of Planter Midle
		PlanterMidlePos = PosPlanterMidleYellow;
		//Position of Planter 1
		Planter1Pos = PosPlanterR1;
		//Position of Planter 2
		Planter2Pos = PosPlanterL2;
		//Position of Field 1
		field1Pos = PosFieldR1;
		//Position of Field 3
		field3Pos = PosFieldR3;
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
	PrivateKI_Task[12].Priority = 80;
	PrivateKI_Task[22].Priority = 80;
	
	//Priority for fixed Tasks
	PrivateKI_Task[11].Priority = 88;
	PrivateKI_Task[21].Priority = 88;
	PrivateKI_Task[13].Priority = 86;
	PrivateKI_Task[23].Priority = 86;
	
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
	if((PrivateKI_Task[15].Status == OPEN || PrivateKI_Task[25].Status == OPEN)
	&& ((distance_Planter2 < distance_PlanterMidle)||(PrivateKI_Task[11].Status != OPEN && PrivateKI_Task[21].Status != OPEN))
	&& ((distance_Planter2 < distance_Planter1)||(PrivateKI_Task[13].Status != OPEN && PrivateKI_Task[23].Status != OPEN))
	&& (enemyRobotInPlanter2 == 0))
	{
		PrivateKI_Task[15].Priority = 89;
		PrivateKI_Task[25].Priority = 89;
	}
	else
	{
		PrivateKI_Task[15].Priority = 83;
		PrivateKI_Task[25].Priority = 83;
	}

	//Task field 1 as Prio 87
	if((PrivateKI_Task[11].Status != OPEN && PrivateKI_Task[21].Status != OPEN && PlantsInRobot > 1) && (PrivateKI_Task[13].Status == OPEN || PrivateKI_Task[23].Status == OPEN))
	{
		PrivateKI_Task[12].Priority = 87;
		PrivateKI_Task[22].Priority = 87;
	}
	
	//Task field1 and field3 as Prio 84/85
	if((PrivateKI_Task[11].Status != OPEN && PrivateKI_Task[21].Status != OPEN && PrivateKI_Task[12].Status != OPEN && PrivateKI_Task[22].Status != OPEN))
	{
		if(distance_Field1<distance_Field3)
		{
			PrivateKI_Task[12].Priority = 85;
			PrivateKI_Task[22].Priority = 85;
			PrivateKI_Task[16].Priority = 84;
			PrivateKI_Task[26].Priority = 84;
		}
		else
		{
			
			PrivateKI_Task[12].Priority = 84;
			PrivateKI_Task[22].Priority = 84;
			PrivateKI_Task[16].Priority = 85;
			PrivateKI_Task[26].Priority = 85;
			
		}
	}
	
	int MaxPlanterPrio = 0;
	uint8_t IndexMaxPrioPlanter = 0;
	
	for(uint8_t i= 11; i<=26;i++)
	{
		if((i<=16 || i>=21) && i != 24 && i!= 14 && PrivateKI_Task[i].Status == OPEN &&
		PrivateKI_Task[i].Priority > MaxPlanterPrio)
		{
			MaxPlanterPrio = PrivateKI_Task[i].Priority;
			IndexMaxPrioPlanter = i;
		}
	}
	//printf("MaxIndexPlanter:  ");
	//printf("%d", IndexMaxPrioPlanter);
	//printf("\n");
	
	return IndexMaxPrioPlanter;
}


/**************************************************************************
***   FUNKTIONNAME: FindRobotPosition		 									***
***   FUNKTION: Find Position of Robot again with the 3 Beacons					***
***   TRANSMIT PARAMETER: point_t                                               ***
***   RECEIVE PARAMETER.:	float d1 : distance to first beacon in m			***
float d2 : distance to second beacon	in m		***
float d3 : distance to third beacon	in m		***
**************************************************************************/
point_t FindRobotPosition(float d1, float d2, float d3)
{
	// kann ich mal testen mit Testpunkten die ich weiss, zB von Position 0/0 alle Distanzen zu den Beacons, dann soll die Funktion wieder 0/0 ausgeben
	
	point_t returnPoint;
	returnPoint.Xpos = 0;
	returnPoint.Ypos = 0;
	
	float x1;
	float y1;
	float x2;
	float y2;
	float x3;
	float y3;
	
	
	// !!! WICHTIG: DER LIDAR MUSS DIE DISTANZEN AUCH ABH�NGIG VON DER SPIELFARBE UND IN DER REIHENFOLGE WIE IM "if-else" unterhalb SCHICKEN!!!!
	if (SpielFarbe == Yellow)
	{
		//Beacon 1
		x1 = 0;
		y1 = 0;
		//Beacon2
		x2 = 0;
		y2= 2;
		//Beacon 3
		x3=3;
		y3=1;
	}
	else if (SpielFarbe == BLUE)
	{
		//Beacon 1
		x1 = 3;
		y1 = 0;
		//Beacon2
		x2 = 3;
		y2= 2;
		//Beacon 3
		x3=0;
		y3=1;
	}
	
	if (d1!= 0 && d2 !=0 && d3 != 0)
	// wenn alle 3 Beacons von Lidar erkannt
	// Quelle fuer Berechnung: https://math.stackexchange.com/questions/884807/find-x-location-using-3-known-x-y-location-using-trilateration
	{
		float A =  (2*x2 - 2*x1);
		float B =  (2*y2 - 2*y1);
		float D =  (2*x3 - 2*x2);
		float E =  (2*y3 - 2*y2);
		
		float C =(float) ( pow((double)d1,2) - pow((double)d2,2) - pow((double)x1,2) + pow((double)x2,2) - pow((double)y1,2) + pow((double)y2,2) );
		float F =(float) ( pow((double)d2,2) - pow((double)d3,2) - pow((double)x2,2) + pow((double)x3,2) - pow((double)y2,2) + pow((double)y3,2) );
		
		returnPoint.Xpos = (int16_t) (1000* ((C*E-F*B) / (E*A-B*D)) );
		returnPoint.Ypos = (int16_t)(1000*  ((C*D-A*F) / (B*D-A*E)) );
	}
	else if ((d1==0 && d2!=0 && d3!=0) || (d2==0 && d1!=0 && d3!=0) || (d3==0 && d2!=0 && d1!=0))
	// wenn nur 2 Beacons vom Lidar erkannt: berechne die 2 M�glichen L�sungspunkte und schlie�e einen aus falls er au�erhalb des Tisches liegt
	// Quelle fuer Berechnung: https://math.stackexchange.com/questions/4629840/find-position-knowing-2-points-and-distances-to-those-points
	
	
	{
		if (d1 == 0)
		{
			x1 = x3;
			y1 = y3;

		}
		else if (d2==0)
		{
			x2 = x3;
			y2= y3;
		}
		
		
		float d = (float) sqrt((double) (pow((double)(x1-x2),2) + pow((double)(y1-y2),2)) );
		float l = (float) ( (pow((double)d1,2)-pow((double)d2,2)+pow((double)d,2)) / ((double)2*d) );
		
		float h = (float) sqrt(pow((double)d1,2)-pow((double)l,2));
		
		float Xpos1 = (float) ( ((l/d)*(x2-x1)) + ((h/d)*(y2-y1)) + x1 );
		float Ypos1 = (float) ( ((l/d)*(y2-y1)) - ((h/d)*(x2-x1)) + y1 );
		
		float Xpos2 = (float) ( ((l/d)*(x2-x1)) - ((h/d)*(y2-y1)) + x1 );
		float Ypos2 = (float) ( ((l/d)*(y2-y1)) + ((h/d)*(x2-x1)) + y1 );
		
		// wenn einer der beiden m�glichen Punkte ausserhalb des Spielfeldes liegt -> den anderen nehmen (Plausibilit�t)
		if(Xpos1 > 3.0 || Ypos1 > 2.0)
		{
			returnPoint.Xpos = (int16_t)(1000*Xpos2);
			returnPoint.Ypos = (int16_t)(1000*Ypos2);
		}
		// diese Grenzen mit 3.0 und 2.0 Meter sind keine harten Grenzen, kann man auch nach innen verschieben weil die Robotermitte ja nie bei 3.0 oder 2.0 metern ist!!
		// -> d.h. kann man noch anpassen empirisch!!! wenn neuer LIDAR da ist.
		else if (Xpos2 > 3.0 || Ypos2 > 2.0)
		{
			returnPoint.Xpos = (int16_t)(1000*Xpos1);
			returnPoint.Ypos = (int16_t)(1000*Ypos1);
		}
		
		
		
	}
	
	
	
	
	return returnPoint;

	
	
}

//// ****************************************************
//// Dot2D, Norm2D, AngleToXAxis2D from path_math.c (�C2)
//// ****************************************************
//float Dot2D(float* vectorA, float* vectorB)
//{
//return (vectorA[X_KOR] * vectorB[X_KOR] + vectorA[Y_KOR] * vectorB[Y_KOR]);
//}
//
//float Norm2D(float* vector)
//{
//return ((float)(sqrt(pow(vector[X_KOR], 2.0) + pow(vector[Y_KOR], 2.0))));
//}
//
//float AngleToXAxis2D(float* vector)
//{
//float phi;
//float e_x[2] = {1, 0};
//float delta_x, delta_y;
//
//// Calculating phi_max -> Angle between the X-axis and the vector: Initial point <-> Destination point
//phi = acos(Dot2D(e_x, vector) / (Norm2D(e_x) * Norm2D(vector)));
//
//delta_y = vector[Y_KOR] - e_x[Y_KOR];
//delta_x = vector[X_KOR] - e_x[X_KOR];
//
//// If necessary, correction of the angle
//if((delta_y < 0.0) && (delta_x < 0.0))
//phi = 2 * M_PI - phi;
//if((delta_y < 0.0) && (delta_x > 0.0))
//phi = 2 * M_PI - phi;
//
//return (phi);
//}
//
//// ****************************************************
//// GetPx, GetPy for shooting the mammoth
//// ****************************************************
//int16_t GetPx(int16_t Mx, int16_t My)
//{
//int16_t Px = 0;
//float R = 11025.0;		// Rx = 10,5 cm
//
//float Mx_x = (float)(Mx - xPos);
//float My_y = (float)(My - yPos);
//float Mx_x2 = pow(Mx_x, 2.0);
//float My_y2 = pow(My_y, 2.0);
//
//float result1 = R * Mx_x;
//float result2 = (float)xPos * (Mx_x2 + My_y2);
//float result3 = sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
//float result4 = Mx_x2 + My_y2;
//
//Px = (int16_t)((result1 + result2 + result3) / result4);
//
//return(Px);
//}
//
//int16_t GetPy(int16_t Mx, int16_t My)
//{
//int16_t Py = 0;
//float R = 11025.0;		// Rx = 10,5 cm
//
//float Mx_x = (float)(Mx - xPos);
//float My_y = (float)(My - yPos);
//float Mx_x2 = pow(Mx_x, 2.0);
//float My_y2 = pow(My_y, 2.0);
//
//float result1 = R * My_y2;
//float result2 = Mx * sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
//float result3 = (float)xPos * sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
//float result4 = (Mx_x2 + My_y2) * My_y * (float)yPos;
//float result5 = (Mx_x2 + My_y2) * My_y;
//
//Py = (int16_t)((result1 - result2 + result3 + result4) / result5);
//
//return(Py);
//}
//
