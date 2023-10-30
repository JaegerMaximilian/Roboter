/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Includes basic definitions, e.g. clock frequency,... .
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
 *      The file includes basic definitions.
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

#ifndef _GLOBAL_H
#define _GLOBAL_H

#include <math.h>
#include <stdint.h>

/* intern/extern switch */
#ifndef _GLOBAL_EXTERN
   #define _GLOBAL_EXTERN extern
#endif 


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
#define _LOCKED_ANTI_PHASE_PWM_CONTROL

#define MAX_PWM_VALUE      1600
#define MAX_INCS_PER_ROT   5000L
#define MAX_INCS_PER_ROT_MR   5000L
#define MAX_INCS_PER_ROT_MOT_L  1000L
#define MAX_INCS_PER_ROT_MOT_R  5000L

#define SYSTEM_CLOCK 32000000L
#define F_CPU 32000000UL



#define false		0
#define true		1

/* PWM Channel */
#define CH_A    0
#define CH_B    1
#define CH_C    2
#define CH_D    3

#define DEG2RAD(a)(a*M_PI/180.0)
#define RAD2DEG(a)(a*180/M_PI)




/* set dedicated pin */
#define SET_PIN(port, pin) port.OUT |= (0x01 << pin)
/* clear dedicated pin */
#define CLR_PIN(port, pin) port.OUT &= ~(0x01 << pin)
/* toggle dedicated pin */
#define TOGGLE_PIN(port, pin) port.OUT ^= (0x01 << pin)
/* test a pin if it is set */
#define TEST_PIN(port, pin) port.IN & (0x01 << pin)

/* union to convert different data types in each other */
typedef union
{
	float f;
	uint32_t uint32;
	int32_t int32;
	uint16_t uint16[2];
	int16_t int16[2];
	uint8_t uint8[4];
	int8_t int8[4];
} convData_t;


/*! \brief Output structure of the object detection */
typedef struct
{
	
	uint16_t center[3][2], variance[3];
	uint8_t count;
} objectdetectionoutput_t;

typedef struct
{
	
	uint8_t count;
	uint16_t center[3][2];
} centerpoints_t;

//Ausgabe der Detektion für den zweiten Roboter und Input der Detektion des zweiten Roboters
_GLOBAL_EXTERN objectdetectionoutput_t outputdetection;
_GLOBAL_EXTERN objectdetectionoutput_t detectionotherrobot;



//Output of the Kalman-filter
_GLOBAL_EXTERN centerpoints_t objectcenter;

_GLOBAL_EXTERN uint16_t Robx, Roby, Roba;




/* **************************** */
/* ***      prototypes      *** */
/* **************************** */

#endif
