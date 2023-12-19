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
#include "Pfadplanung.h"

/* intern/extern switch */
#ifndef _GLOBAL_EXTERN
   #define _GLOBAL_EXTERN extern
#endif 


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
#define _LOCKED_ANTI_PHASE_PWM_CONTROL

#define INT16_MAXVALUE    32767
#define ADJ_DIMENSION        30

#define SYSTEM_CLOCK 32000000L
#define F_CPU 32000000UL



#define false		0
#define true		1

/* PWM Channel */
#define CH_A    0
#define CH_B    1
#define CH_C    2
#define CH_D    3


// Umwandlerfunktion DEG -> RAD und RAD -> DEG
#define DEG2RAD(a) ((float)(a) * M_PI / 180.0)
#define RAD2DEG(a) ((float)(a) * 180.0 / M_PI)


/* set dedicated pin */ 
#define SET_PIN(port, pin) port.OUT |= (0x01 << pin)
/* clear dedicated pin */
#define CLR_PIN(port, pin) port.OUT &= ~(0x01 << pin)
/* toggle dedicated pin */
#define TOGGLE_PIN(port, pin) port.OUT ^= (0x01 << pin)
/* test a pin if it is set */
#define TEST_PIN(port, pin) (port.IN & (0x01 << pin))

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




_GLOBAL_EXTERN float Winkel_motor1, Winkel_motor2, Winkel_motor3;
_GLOBAL_EXTERN float geschwindigkeitsVorzeichen;

_GLOBAL_EXTERN uint8_t spielZeit, spielfarbeRAM, RobotType_RAM, Punkte_Kirschen, Punkte, Punkte_SlaveRobot;

_GLOBAL_EXTERN uint8_t PlantsInRobot;
_GLOBAL_EXTERN uint8_t OpenParkPos;
_GLOBAL_EXTERN uint8_t OpenPlanter;
_GLOBAL_EXTERN uint8_t ParkedPlants;
_GLOBAL_EXTERN uint8_t OpenPlants;
_GLOBAL_EXTERN uint8_t motionFailureCount;
_GLOBAL_EXTERN uint8_t planedPlants;
_GLOBAL_EXTERN uint8_t nReachableCnt1000, nReachableCnt2000,nReachableCnt3000,nReachableCnt4000,nReachableCnt5000,nReachableCnt6000;

_GLOBAL_EXTERN uint16_t velocity;

_GLOBAL_EXTERN uint8_t StateOfGame;

_GLOBAL_EXTERN uint16_t sysTime;
_GLOBAL_EXTERN uint8_t gegnerErkennung;

_GLOBAL_EXTERN uint8_t SpielFarbe_Nextion, Strategie, Page, ConfigPlanter_Nextion, ConfigStehlen_Nextion;
_GLOBAL_EXTERN uint8_t SpielFarbe, Strategie, Page, ConfigPlanter, ConfigStehlen;
_GLOBAL_EXTERN point_t Plant1000,Plant2000,Plant3000,Plant4000,Plant5000,Plant6000;
_GLOBAL_EXTERN point_t PlanterL1,PlanterR1,PlanterL2,PlanterR2,FieldL1,FieldR1,FieldL3,FieldR3,PlanterMidleBlue,PlanterMidleYellow,SolarPanelsBlue, SolarPanelsYellow, SolarPanelsMiddle;

//_GLOBAL_EXTERN point_t startPos;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */

#endif
