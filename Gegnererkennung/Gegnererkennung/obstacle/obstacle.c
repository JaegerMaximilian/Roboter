/*
* obstacle.c
*
* Created: 03.06.2021 09:44:53
*  Author: P20087
*/

#define _OBSTACLE_EXTERN


#include <avr/io.h>
#include "obstacle.h"
#include <stdlib.h>
#include <math.h>
#include "usart.h"
#include <stdio.h>
#include "multitask.h"
#include "define.h"
#include "rplidar.h"
#include "global.h"
#include "rrt_transmittingtask.h"

/* scan-array -> all valid points are stored here */
OBSTACLE_scanPoint_t scanArray[OBSTACLE_SCAN_ARRAY_LENGTH];
/* index to write to scan-array and read from scan-array */
uint16_t scanArray_In = 0;
uint16_t scanArray_Out = 0;
/* position in scan-array of first element */
uint16_t firstScanElement;
/* amount of scan-points in actual scan */
uint16_t nbrScanElement;





uint16_t timeStamp = 0;


/**********************************************************
NAME: OBSTACLE_InitDetection
FUNC.: initialize the obstacle detection
IN: -
OUT: -
VERSION: 1.0 - Michael Zauner (03.06.2021)
**********************************************************/
void OBSTACLE_InitDetection()
{
	/* cyclic task - cycle time: 1 ms */
	SET_TASK(OBSTACLE_DETECTION_TASKNBR, DISABLE);
	SET_TASK_HANDLE(OBSTACLE_DETECTION_TASKNBR, OBSTACLE_DetectionTask);
}



/**********************************************************
NAME: OBSTACLE_DetectionTask
FUNC.: process the obstacle detection
-> task is enabled when lidar finished one rotation
IN: -
OUT: task is disabled after execution
VERSION: 1.0 - Michael Zauner (03.06.2021)
**********************************************************/
uint8_t OBSTACLE_DetectionTask()
{
	uint8_t obstacleIndex = 0;
	uint8_t text[1000];
	
	/* *************************************** */
	/* reset all elements in the obstacle list */
	/* *************************************** */
	for (uint8_t i = 0; i < OBSTACLE_LIST_LENGTH; i++)
	{
		/* set the point-counter of each obstacle to zero */
		obstacleList[i].nbr = 0;
		obstacleList[i].nbrObstacle = 0;
	}
	
	/* ********************************************************************* */
	/* check all points from one scan and separate it to different obstacles */
	/* ********************************************************************* */
	for (uint16_t j = 0; j < nbrScanElement; j++)
	{
		/* if actual point is close to last point from obstacle or the first scan-element from actual scan */
		/*  -> add point to point-list from the actual obstacle */
		if (((abs(scanArray[scanArray_Out].phi - obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr - 1].phi) < OBSTACLE_MAX_DIFF_ANGLE) &&
		(abs(scanArray[scanArray_Out].r - obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr - 1].r) < OBSTACLE_MAX_DIFF_DISTANCE)
		&& (obstacleList[obstacleIndex].nbr >= 1)) ||
		(j == 0))
		{
			/* copy point from scan-list to the obstacle */
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].phi = scanArray[scanArray_Out].phi;
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].r = scanArray[scanArray_Out].r;
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].x = scanArray[scanArray_Out].x;
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].y = scanArray[scanArray_Out].y;
			/* increment index to scan-list */
			scanArray_Out = ((++scanArray_Out >= OBSTACLE_SCAN_ARRAY_LENGTH) ? 0 : scanArray_Out);
			/* increment counter for point-list for the first obstacle */
			obstacleList[obstacleIndex].nbr = ((++(obstacleList[obstacleIndex].nbr) >= OBSTACLE_POINT_LENGTH) ? (OBSTACLE_POINT_LENGTH - 1) : obstacleList[obstacleIndex].nbr);
		}
		else
		{
			if (obstacleList[obstacleIndex].nbr >= OBSTACLE_MIN_NBR_POINTS)
			{
				/* switch to the next obstacle */
				obstacleIndex = ((++obstacleIndex >= OBSTACLE_LIST_LENGTH) ? (OBSTACLE_LIST_LENGTH - 1) : obstacleIndex);
			}
			else
			{
				obstacleList[obstacleIndex].nbr = 0;
			}
			
			
			/* copy point from scan-list to the new obstacle */
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].phi = scanArray[scanArray_Out].phi;
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].r = scanArray[scanArray_Out].r;
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].x = scanArray[scanArray_Out].x;
			obstacleList[obstacleIndex].scan[obstacleList[obstacleIndex].nbr].y = scanArray[scanArray_Out].y;
			/* increment index to scan-list */
			scanArray_Out = ((++scanArray_Out >= OBSTACLE_SCAN_ARRAY_LENGTH) ? 0 : scanArray_Out);
			/* increment counter for point-list for the new obstacle */
			(obstacleList[obstacleIndex].nbr)++;
		}
	}
	
	/* ******************* */
	/* check all obstacles */
	/* ******************* */
	for (uint8_t k = 0; k < OBSTACLE_LIST_LENGTH; k++)
	{
		if (obstacleList[k].nbr >= OBSTACLE_MIN_NBR_POINTS)
		{
			/* calcualte the expantion of the obstacle */
			obstacleList[k].deltaX = abs((int16_t)obstacleList[k].scan[obstacleList[k].nbr - 1].x - (int16_t)obstacleList[k].scan[0].x);
			obstacleList[k].deltaY = abs((int16_t)obstacleList[k].scan[obstacleList[k].nbr - 1].y - (int16_t)obstacleList[k].scan[0].y);
			/* calculate the size of the obstacle */
			obstacleList[k].dim = (uint16_t)sqrtf((float)obstacleList[k].deltaX * (float)obstacleList[k].deltaX + (float)obstacleList[k].deltaY * (float)obstacleList[k].deltaY);
			/* calculate the vector heading to the center of the obstacle */
			obstacleList[k].vec[0] = ((float)obstacleList[k].deltaY / (float)obstacleList[k].dim) * (-1.0);
			obstacleList[k].vec[1] = ((float)obstacleList[k].deltaX / (float)obstacleList[k].dim);
			
			/* if size of an obstacle is to big -> split the object into to obstacles */
			// 			if (obstacleList[k].dim > OBSTACLE_MAX_SIZE)
			// 			{
			// 				/* calculate the centers of the objects */
			// 				/* first object */
			// 				obstacleList[k].centerX[0] = obstacleList[k].scan[obstacleList[k].nbr / 4].x + (uint16_t)(obstacleList[k].vec[0] * (float)obstacleList[k].dim / 2.0);
			// 				obstacleList[k].centerY[0] = obstacleList[k].scan[obstacleList[k].nbr / 4].y + (uint16_t)(obstacleList[k].vec[1] * (float)obstacleList[k].dim / 2.0);
			// 				/* second object */
			// 				obstacleList[k].centerX[1] = obstacleList[k].scan[(obstacleList[k].nbr * 3) / 4].x + (uint16_t)(obstacleList[k].vec[0] * (float)obstacleList[k].dim / 2.0);
			// 				obstacleList[k].centerY[1] = obstacleList[k].scan[(obstacleList[k].nbr * 3) / 4].y + (uint16_t)(obstacleList[k].vec[1] * (float)obstacleList[k].dim / 2.0);
			// 				/* set number of 2 -> two obstacles */
			// 				obstacleList[k].nbrObstacle = 2;
			// 			}
			// 			else
			{
				/* calculate the center of the object */
				obstacleList[k].centerX[0] = obstacleList[k].scan[obstacleList[k].nbr / 2].x; //+ (uint16_t)(obstacleList[k].vec[0] * (float)obstacleList[k].dim / 2.0);
				obstacleList[k].centerY[0] = obstacleList[k].scan[obstacleList[k].nbr / 2].y; //+ (uint16_t)(obstacleList[k].vec[1] * (float)obstacleList[k].dim / 2.0);
				/* set number of 1 -> one obstacles */
				obstacleList[k].nbrObstacle = 1;
				
				#ifdef _DEBUG_LASER
				sprintf(text,"#;O%d;%d;%d\r\n*",k,obstacleList[k].centerX[0],obstacleList[k].centerY[0]);
				debugMsg(text);
				#endif
				
			}
		}
	}
	
	enemyPosMsg();
	
	return(DISABLE);
}



/**********************************************************
NAME: OBSTACLE_Scan2Pos
FUNC.: calculate the x/y-coordinate of scanpoint and store
it in the scan-list when the point is on playground
IN: scan ... scan from rplidar
OUT: -
VERSION: 1.0 - Michael Zauner (03.06.2021)
**********************************************************/
void OBSTACLE_Scan2Pos(rpLidar_Scan_t* scan)
{
	static float lastAngle = 0.0;
	static uint16_t nbr = 0;
	static uint16_t start = 0;
	int16_t x, y;
	float r;
	
	uint8_t text[1000];
	
	// 	xPos = 330;
	// 	yPos = 1140;
	// 	phiPos = 00;
	
	/* check if an overrun in scan happend -> angle turns from 360° to 0° */
	/* scan is complete -> wakeup OBSTACLE_DetectionTask                  */
	if ((scan->angle < lastAngle))// && (nbr > 0))
	//	if (nbr > 50)
	{
		//for (uint8_t i = 0; i < nbr; i++)
		//{
		//sprintf(text,"%d;%d;%d;%d\r\n",scanArray[i].timeStamp,scanArray[i].phi,scanArray[i].x,scanArray[i].y);
		//writeString_usart(&usartD0, text);
		//}
		#ifdef _DEBUG_LASER
		sprintf(text,"XXX:%d;%d\r\n",start, nbr);
		debugMsg(text);
		#endif
		
		
		SET_TASK(OBSTACLE_DETECTION_TASKNBR, ENABLE);
		/* store address of first element in scan-array */
		firstScanElement = start;
		/* store number of elements */
		nbrScanElement = nbr;
		
		/* set first element of next scan */
		start = scanArray_In;
		/* reset number of elements */
		nbr = 0;
	}
	/* store actual angle */
	lastAngle = scan->angle;
	
	/* ************************************************************** */
	/* transformation from polar coordinates to cartesian coordinates */
	/* ************************************************************** */
	/* 1. add offset between lidar and odometrie to the actual x/y-coordinates */
	x = xPos + (int16_t)(OBSTACLE_LIDAR_OFFSET_X * cos(DEG2RAD((float)(phiPos)))) - (int16_t)(OBSTACLE_LIDAR_OFFSET_Y * sin(DEG2RAD((float)(phiPos))));
	y = yPos + (int16_t)(OBSTACLE_LIDAR_OFFSET_Y * cos(DEG2RAD((float)(phiPos)))) + (int16_t)(OBSTACLE_LIDAR_OFFSET_X * sin(DEG2RAD((float)(phiPos))));
	/* 2. add measured distance from lidar to the actual x/y-coordinates */
	x += (int16_t)(cos(DEG2RAD(RPLIDAR_CONV_ANGLE(scan->angle) + (float)phiPos)) * scan->distance);
	y += (int16_t)(sin(DEG2RAD(RPLIDAR_CONV_ANGLE(scan->angle) + (float)phiPos)) * scan->distance);

	r = sqrtf(((float)x-(float)xPos)*((float)x-(float)xPos) + ((float)y-(float)yPos)*((float)y-(float)yPos));

	/* check if the point is on playground */
	if ((x > 100) && (x < OBSTACLE_DIM_PLAYGROUND_X) && (y > 100) && (y < OBSTACLE_DIM_PLAYGROUND_Y) && (r > 150.0) && (nbr < OBSTACLE_SCAN_ARRAY_LENGTH))
	{
		#ifdef _DEBUG_LASER
		sprintf(text,"%d;%d;%d;%.0f;%.0f;\r\n",timeStamp,x,y,scan->angle,scan->distance);
		debugMsg(text);
		#endif
		
		/* store point to scan-array */
		scanArray[scanArray_In].x = (uint16_t)x;
		scanArray[scanArray_In].y = (uint16_t)y;
		scanArray[scanArray_In].phi = (uint16_t)scan->angle;
		scanArray[scanArray_In].r = (uint16_t)scan->distance;
		
		/* increment in-index */
		scanArray_In = ((++scanArray_In >= OBSTACLE_SCAN_ARRAY_LENGTH) ? 0 : scanArray_In);
		/* increment the number of valid scan-points */
		nbr++;
	}
}