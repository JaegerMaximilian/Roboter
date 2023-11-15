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

#include "ki_helper.h"
#include "ki.h"
#include "define.h"
#include "global.h"
#include "command.h"
#include "Pfadplanung.h"

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
	float HypotenuseC = sqrt(StreckeX*StreckeX + StreckeY*StreckeY);
	
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
***   FUNKTIONNAME: AktivatePlantsAsObstacle                            ***
***   FUNKTION: Sets Plants which arent used in the actual plan as		***
***   Obstacles															***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.:												***
**************************************************************************/
void ActivatePlantAsObstacle()
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

