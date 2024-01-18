/*
 * observation.h
 *
 * Created: 24.05.2023 11:25:52
 *  Author: P20087
 */ 


#ifndef OBSERVATION_H_
#define OBSERVATION_H_

#include <avr/io.h>
#include "Pfadplanung.h"
#include "ki.h"

#ifndef _OBSERVATION_EXTERN_
	#define _OBSERVATION_EXTERN_
#endif

/* ***************************** */
/*			  DEFINES			 */
/* ***************************** */
/* result for no problems during motion dedected */
#define OBSERVATION_MOTION_OK		1
/* result for problems during motion dedected */
#define OBSERVATION_MOTION_ERROR	2
/* observation is running */
#define OBSERVATION_RUNNING			0

/* observation is started */
#define OBSERVATION_STARTED			1
/* observation is finished and is waiting */
#define OBSERVATION_PENDING			0

/* result of observation */
_OBSERVATION_EXTERN_ uint8_t observationResult;
/* observation started (yes/not) */
_OBSERVATION_EXTERN_ uint8_t observationStarted;

/* observation distance (front) */
_OBSERVATION_EXTERN_ uint16_t observationDisFront;	
/* observation distance (side) */	
_OBSERVATION_EXTERN_ uint16_t observationDisSide;

_OBSERVATION_EXTERN_ point_t observationStartPos, observationGoalPos;
_OBSERVATION_EXTERN_ uint8_t motionIR;	

_OBSERVATION_EXTERN_ robot_t enemyRobot[5];	
_OBSERVATION_EXTERN_ robot_time_t enemyRobotLidar[5];	
_OBSERVATION_EXTERN_ robot_time_t enemyPosRobotKamera[5],ownPosKamera[5];


/* ***************************** */
/*			PROTOTYPES			 */
/* ***************************** */
void InitObservation(void);
uint8_t ObservationTask(void);
uint8_t observationAreaIsFree(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
uint8_t GetObservationResult();

#endif /* OBSERVATION_H_ */