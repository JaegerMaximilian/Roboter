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


//#define _DEBUG_RRTLAN_
//#define _DEBUG_ODOMETRY_
// #define _DEBUG_RRTLAN_CYCLIC_READ_
//#define _DEBUG_CAN_
//#define _DEBUG_SERIAL_


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
#define _MASTER_

#define HW_MAIN_VERSION		3
#define HW_SUB_VERSION		0
#define HW_SUBSUB_VERSION	0


#define SW_MAIN_VERSION		3
#define SW_SUB_VERSION		0
#define SW_SUBSUB_VERSION	150429

#define TRUE      1
#define FALSE     0

#define INPUT     0
#define OUTPUT    1

#define ON        1
#define OFF       0

#define STATUS_LED	

/* error-numbers */
#define ERROR_U_MIN			0x01
#define ERROR_U_MAX			0x02
#define ERROR_TEMP_MAX		0x04
#define ERROR_BAT				0x08
#define ERROR_SHOWN			0x80

/* modes */
#define AUTOMATIK_BETRIEB				0
#define HAND_BETRIEB						1


/* task numbers */
#define DEBUG_TASKNBR									0
#define RPLIDAR_TASKNBR							1
#define RPLIDAR_MSG_HANDLER_TASKNBR				2
#define OBSTACLE_DETECTION_TASKNBR				3

#define SERIAL_RECEIVE_TASKNBR					4
#define TIMEOUT_TASKNBR							5
#define TIMEOUTMANAGER_TASKNBR					6
#define TRANSMITTING_TASKNBR_1					7
#define RRTLAN_ROBO_POS_TASKNBR					8



// GEGNER
#define DEBUG_MSG_GEGNER_PORTNBR		101
#define POS_MSG_GEGNER_PORTNBR			102
#define ROBO_POS_MSG_GEGNER_PORTNBR		103






/* port numbers */
#define GET_DATA_FROM_LCD_PORT		1
#define DEBUG_MSG_PORT					2
#define SEND_DATA_TO_LCD_PORT			3





#endif
