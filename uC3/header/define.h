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

#define STATUS_LED

/* ************ */
/* task numbers */
/* ************ */
#define DEBUG_TASKNBR						0
#define MOTOR_TASKNBR						1
#define START_TASKNBR						2
#define SERIAL_RECEIVE_TASKNBR				3
#define TIMEOUT_TASKNBR						4
#define TIMEOUTMANAGER_TASKNBR				5
#define SENSOR_TASKNBR						6
#define ANTRIEB_TASKNBR						7
#define RRTLAN_ANTRIEB_TASKNBR				8
#define SERVO_TASKNBR						9
#define RRTLAN_SERVO_TASKNBR				10
#define RRTLAN_DIGIOUT_TASKNBR				11
#define RPLIDAR_MSG_HANDLER_TASKNBR			12
#define RPLIDAR_TASKNBR						13
#define Evaluation							14
#define RRTLAN_POSENEMY_TASKNBR				15
#define LIFT_MOTOR_TASKNBR					16
#define RRTLAN_LIFT_VEL_TASKNBR				17
#define ARM_LEFT_TASKNBR					18
#define ARM_RIGHT_TASKNBR					19
#define RRTLAN_LIFT_INIT_TASKNBR			20
#define RRTLAN_MOT_POS_TASKNBR				21


/* ********************************** */
/* port definitions  (must be unique) */
/* ********************************** */
// µC3
#define DEBUG_MSG_PORTNBR		101
#define SERVO_PORTNBR			102
#define SENSOR_PORTNBR			103
#define MOTOR_PORTNBR			104
#define DIGIOUT_PORTNBR			105
#define ENEMYDATA_RAW_FROM_WIFI_PORTNBR			106
#define ENEMYDATA_TO_PATHPLANER_PORTNBR			107
#define ENEMYDATA_RAW_TO_WIFI_PORTNBR			108
#define LIFT_POS_PORTNBR						109
#define LIFT_VEL_PORTNBR						110
#define START_LIFT_INIT_PORTNBR					111
#define MOT_POS_PORTNBR							112

/* ****************** */
/* digital out (LEDs) */
/* ****************** */
#define LED1_DIGIOUT			1
#define LED2_DIGIOUT			2
#define LED3_DIGIOUT			3
#define VACUUM_LEFT				4
#define VACUUM_RIGHT			5
#define MV_LEFT					6
#define MV_RIGHT				7


#endif
