/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Includes general definitions, e.g. task-numbers,... .
 *
 *      
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. 
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *
 *
 * \par Documentation
 *      The file includes general definitions.
 *
 * \author
 *      Michael Zauner
 *      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
 *      Support email: roboracing@fh-wels.at
 *
 * $Revision: 1 $
 * $Date: 2012-09-11  $  \n
 *
 * Copyright (c) 2012, RRT (University of Applied Sciences Upper Austria) All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of RRT may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RRT (University of Applied Sciences Upper Austria) 
 * "AS IS" AND ANY EXPRESS OR IMPLIED  * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/ 

#ifndef _DEFINE_H
#define _DEFINE_H

#include "global.h"
#include "servo.h"
#include "rrt_receivedata.h"

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
#define _MASTER_

#define TRUE      1
#define FALSE     0

#define INPUT     0
#define OUTPUT    1

#define ON        1
#define OFF       0

#define X_KOR			0
#define Y_KOR			1

/* ************ */
/* task numbers */
/* ************ */
#define DEBUG_TASKNBR							0
#define START_TASKNBR							1
#define SERIAL_RECEIVE_TASKNBR					2
#define TIMEOUT_TASKNBR							3
#define TIMEOUTMANAGER_TASKNBR					4
#define SERVO_TASKNBR							5
#define LIFT_MOTOR_TASKNBR						6

#define PARSER_TASKNBR							7
#define NEXTION_TASKNBR							8


#define SPIELFARBE_TASKNBR						11
#define SPIELZEIT_TASKNBR						12
#define STRATEGIE_TASKNBR						13
#define TESTBETRIEB_TASKNBR						15
#define KI_TASKNBR								16
#define KI_WATCH_TASKNBR						17
#define KI_Calculate_Times_TASKNBR				18
#define DEBUG_MSG_UC2_TASKNBR					19
#define SENSOR_UC2_TASKNBR						20
#define SENSOR_TASKNBR							21
#define DEBUG_MSG_GEGNER_TASKNBR				22
#define ENEMY_POS_TASKNBR						23
#define ANTRIEB_UC2_TASKNBR						24
#define POS_UC2_TASKNBR							25
#define EINSTELLUNG_LCD_TASKNBR					26
#define DEBUG_MSG_LCD_TASKNBR					27
#define ENEMEY_DETECTION_TASKNBR				28
#define WIFI_MOSI_TASKNBR						29
#define WIFI_TASKNBR							30
#define WIFI_CYCLIC_TASKNBR						31
#define ENEMY_LIDAR_TASKNBR						32
#define ENEMY_TO_WIFI_TASKNBR					33
#define US_DATA_SYNCHRON_TASKNBR				34
#define OBSERVATION_TASKNBR						35
#define MERGEENEMYPOS_TASKNBR					36


/* ********************************** */
/* don't change these tasknumbers	  */
/* ********************************** */ 
// #define XBEE_COORDS_TASKNBR					30
// #define XBEE_SMALL_ROBOT_TASKNBR				31

/* ********************************** */
/* port definitions  (must be unique) */
/* ********************************** */
// µC2
#define DEBUG_MSG_UC2_PORTNBR			1
#define SERVO_UC2_PORTNBR				2
#define SENSOR_UC2_PORTNBR				3
#define ANTRIEB_UC2_PORTNBR				4
#define POS_UC2_PORTNBR					5
#define ANTRIEB_CLOTHOID_UC2_PORTNBR	6
#define DIGIOUT_UC2_PORTNBR				7
#define MOT_POS_UC2_PORTNBR				8
#define MOT_VEL_UC2_PORTNBR				9
#define SET_SCHLEPP_UC2_PORTNBR			10
#define SET_ACC_UC2_PORTNBR				11


// GEGNER
#define DEBUG_MSG_GEGNER_PORTNBR		101
#define POS_MSG_GEGNER_PORTNBR			102
#define ROBO_POS_MSG_GEGNER_PORTNBR		103

// LCD
#define DEBUG_MSG_LCD_PORTNBR		201
#define EINSTELLUNG_LCD_PORTNBR		202
#define VALUE_LCD_PORTNBR			203

// WIFI
#define WIFI_MOSI_PORTNBR			205
#define WIFI_MISO_PORTNBR			206

// X-Bee
#define _XBEE_COORDS_PORT_NBR       50
#define _XBEE_SMALL_ROBOT_PORT_NBR  51

#define POS_REL						0
#define POS_ABS						1
#define TURN_REL					2
#define TURN_ABS					3
#define ARC							4
#define MOTION_INTERRUPT			5

#define MOTION_OK					1
#define MOTION_ENEMY_ERROR			2
#define MOTION_SCHLEPP_ERROR		3

#define GEGNER_ON					1
#define GEGNER_OFF					2


/**************************************************************************
***		                       SPIELFARBE
**************************************************************************/



/**************************************************************************
***		                       ANALOGE EINGÄNGE
**************************************************************************/


/**************************************************************************
***		                       DIGITALE EINGÄNGE
**************************************************************************/


/**************************************************************************
***		                       DIGITALE AUSGÄNGE
**************************************************************************/

	

/**************************************************************************
***		                       SERVOS                      			
**************************************************************************/


/**************************************************************************
***		                       MOTOREN
**************************************************************************/

#define GEGNER_ERKENNUNG_BEIDE		8
#define GEGNER_ERKENNUNG_SMALL		9
#define GEGNER_ERKENNUNG_BIG		10
#define TECH_ABNAHME				11

/**************************************************************************
***		                       STARTPOSITIONEN                      			
**************************************************************************/

/* ROBOTER BLUE L1 MASTER */
#define X0_POS_BLUE_L1_ZONE				2818
#define Y0_POS_BLUE_L1_ZONE				219
#define PHI0_BLUE_L1_ZONE				1800

/* ROBOTER Yellow L2 MASTER */
#define X0_POS_Yellow_L2_ZONE			2818
#define Y0_POS_Yellow_L2_ZONE			994
#define PHI0_Yellow_L2_ZONE				1800

/* ROBOTER BLUE L3 MASTER */
#define X0_POS_BLUE_L3_ZONE				2818
#define Y0_POS_BLUE_L3_ZONE				1775
#define PHI0_BLUE_L3_ZONE				1800

/* ROBOTER Yellow R1 MASTER */
#define X0_POS_Yellow_R1_ZONE			203
#define Y0_POS_Yellow_R1_ZONE			219
#define PHI0_Yellow_R1_ZONE				0

/* ROBOTER BLUE R2 MASTER */
#define X0_POS_BLUE_R2_ZONE				203
#define Y0_POS_BLUE_R2_ZONE				994
#define PHI0_BLUE_R2_ZONE				0

/* ROBOTER Yellow R3 MASTER */
#define X0_POS_Yellow_R3_ZONE			203
#define Y0_POS_Yellow_R3_ZONE			1775
#define PHI0_Yellow_R3_ZONE				0

#endif
