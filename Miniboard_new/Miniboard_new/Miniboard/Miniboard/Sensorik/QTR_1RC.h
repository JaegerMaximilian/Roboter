/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      QTR-1RC sensor source file.
 *
 *      This file contains the driver for the QTR-1RC sensor with all components.
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
 * \par Documentation
 *      The file provide functions for the multitasking system. 
 *
 * \author
 *      Michael Zauner
 *      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
 *      Support email: roboracing@fh-wels.at
 *
 * $Revision: 1 $
 * $Date: 2019-02-26  $  \n
 *
 * Copyright (c) 2019, RRT (University of Applied Sciences Upper Austria) All rights reserved.
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

#ifndef QTR_1RC_H_
#define QTR_1RC_H_


#include <avr/io.h>

/* intern/extern switch */
#ifndef _QTR_1RC_EXTERN
	#define _QTR_1RC_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* sensor states (0 ... black, 1 .. white) */
#define SENSOR_BLACK 0
#define SENSOR_WHITE 1

/* number of sensors */
#define GROUNDS_SENSOR_NUMBER 4

/* sensor name */
#define GROUNDS_BACK_LEFT 0
#define GROUNDS_FRONT_LEFT 1
#define GROUNDS_BACK_RIGHT 2
#define GROUNDS_FRONT_RIGHT 3


/* struct of QTR-1RC sensor */
typedef struct 
{
	   PORT_t * qPort;	/*!< Sensor Port*/
	   uint8_t qPin;    /*!< Sensor Pin */
	   uint8_t status;	/*!< SensorStatus 0 = black, 1 = white */
	
} QTR_1RC_t;

/* sensor array */
_QTR_1RC_EXTERN QTR_1RC_t groundSensors[GROUNDS_SENSOR_NUMBER];

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void init_QTR_1RC();
uint8_t QTR_1RC_Task();

#endif /* QTR_1RC_H_ */