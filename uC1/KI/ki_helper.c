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
***   FUNKTION: Ändern der Prios von Blau auf Gelb                     ***
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
		cmd_Drive(0,0,-speed,0,0,0,0,distance,POS_REL,ON,NULL,NULL);
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
	
	float distance_1000 = CalcDistance(aktpos,Plant1000);
	float distance_2000 = CalcDistance(aktpos,Plant2000);
	float distance_3000 = CalcDistance(aktpos,Plant3000);
	float distance_4000 = CalcDistance(aktpos,Plant4000);
	float distance_5000 = CalcDistance(aktpos,Plant5000);
	float distance_6000 = CalcDistance(aktpos,Plant6000);
	float distanceEnemy_1000 = 3000;
	float distanceEnemy_2000 = 3000;
	float distanceEnemy_3000 = 3000;
	float distanceEnemy_4000 = 3000;
	float distanceEnemy_5000 = 3000;
	float distanceEnemy_6000 = 3000;
	
	//Distance to enemy
	for (int i = 0;i<5;i++)
	{
		if (enemyRobot[i].Xpos != 10000 && enemyRobot[i].Ypos != 10000)
		{
			if(CalcDistance(Plant1000,enemyRobot[i])<distanceEnemy_1000)
			{
				distanceEnemy_1000 = CalcDistance(Plant1000,enemyRobot[i]);
			}
			if(CalcDistance(Plant2000,enemyRobot[i])<distanceEnemy_2000)
			{
				distanceEnemy_2000 = CalcDistance(Plant2000,enemyRobot[i]);
			}
			if(CalcDistance(Plant3000,enemyRobot[i])<distanceEnemy_3000)
			{
				distanceEnemy_3000 = CalcDistance(Plant3000,enemyRobot[i]);
			}
			if(CalcDistance(Plant4000,enemyRobot[i])<distanceEnemy_4000)
			{
				distanceEnemy_4000 = CalcDistance(Plant4000,enemyRobot[i]);
			}
			if(CalcDistance(Plant5000,enemyRobot[i])<distanceEnemy_5000)
			{
				distanceEnemy_5000 = CalcDistance(Plant5000,enemyRobot[i]);
			}
			if(CalcDistance(Plant6000,enemyRobot[i])<distanceEnemy_6000)
			{
				distanceEnemy_6000 = CalcDistance(Plant6000,enemyRobot[i]);
			}
		}
	}
	
	//Whole Distance
	float quantifierEnemy = 1.0;
	
	float wholeDistance_1000 = distance_1000 + 0.5*(3000 - distanceEnemy_1000);
	float wholeDistance_2000 = distance_2000 + 0.5*(3000 - distanceEnemy_2000);
	float wholeDistance_3000 = distance_3000 + 0.5*(3000 - distanceEnemy_3000);
	float wholeDistance_4000 = distance_4000 + 0.5*(3000 - distanceEnemy_4000);
	float wholeDistance_5000 = distance_5000 + 0.5*(3000 - distanceEnemy_5000);
	float wholeDistance_6000 = distance_6000 + 0.5*(3000 - distanceEnemy_6000);
	
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
			KI_Task[1].Priority = 97-i;
		}
		if(distances[i] == wholeDistance_2000)
		{
			KI_Task[2].Priority = 97-i;
		}
		if(distances[i] == wholeDistance_3000)
		{
			KI_Task[3].Priority = 97-i;
		}
		if(distances[i] == wholeDistance_4000)
		{
			KI_Task[4].Priority = 97-i;
		}
		if(distances[i] == wholeDistance_5000)
		{
			KI_Task[5].Priority = 97-i;
		}
		if(distances[i] == wholeDistance_6000)
		{
			KI_Task[6].Priority = 97-i;
		}
	}
}
/**************************************************************************
***   FUNKTIONNAME: OpenPlants											***
***   FUNKTION: OpenPlants												***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
uint8_t CalcOpenPlants(void)
{
	OpenPlants = 0;
	for (int i = 1; i < 7; i++)
	{
		if(KI_Task[i].Status == OPEN || KI_Task[i].Status == PENDING)
		{
			OpenPlants++;
		}
	}
	return(OpenPlants);
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

