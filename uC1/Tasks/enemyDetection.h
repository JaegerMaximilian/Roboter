/*
* enemyDetection.h
*
* Created: 12.05.2021 08:42:54
*  Author: anonym_lenovo
*/


#ifndef ENEMYDETECTION_H_
#define ENEMYDETECTION_H_


#include "ki.h"

#ifndef _ENEMYDETECTION_EXTERN
#define _ENEMYDETECTION_EXTERN extern
#endif


#define ED_NO_ENEMY_DETECTED	0

typedef struct
{
	uint16_t *usLeft;	// pointer to left US
	uint16_t *usRight;	// pointer to right US
	uint16_t range;		// measure range (in mm)
	uint16_t offsetX;	// offset to center of robot in x-direction (in mm)
	uint16_t offsetY;	// offset to center of robot in y-direction (in mm)
	uint16_t offsetPhi;	// offset to inertial-system (orientation) of robot (in °)
	point_t pos;		// position of the detected obstacle (in mm)
	uint8_t variance;	// quality of measurement (0 .. 100)
} usObstacleDetection_t;


_ENEMYDETECTION_EXTERN usObstacleDetection_t dynamicObstacles[4];

_ENEMYDETECTION_EXTERN float xPosEnemy;
_ENEMYDETECTION_EXTERN float yPosEnemy;

uint8_t EnemyDetectionTask();
void InitEnemyDetection();

#endif /* ENEMYDETECTION_H_ */