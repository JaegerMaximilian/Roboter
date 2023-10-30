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

#define _QTR_1RC_EXTERN

#include "QTR_1RC.h"
#include <avr/io.h>
#include "define.h"
#include <util/delay.h>
#include "multitask.h"


/* ************************************************************** */
/*! \brief QTR-1RC initialization.
 *
 *  Initialize the QTR-1RC driver  
 *
 *  \version 26.02.2019
 *
 */
/* ************************************************************** */
void init_QTR_1RC()
{
	/* set sensor 1 (...) */
	groundSensors[GROUNDS_BACK_LEFT].qPort = &PORTA;
	groundSensors[GROUNDS_BACK_LEFT].qPin = 4;
	groundSensors[GROUNDS_BACK_LEFT].status = SENSOR_BLACK;
	
	/* cyclic task - cycle time: 1 ms */
	SET_CYCLE(QTR_1RC_TASKNBR, 1);
	SET_TASK(QTR_1RC_TASKNBR, CYCLE);
	SET_TASK_HANDLE(QTR_1RC_TASKNBR, QTR_1RC_Task);	
}

/* ************************************************************** */
/*! \brief QTR-1RC task.
 *
 *  Handle QTR-1RC task  
 *
 *  \version 26.02.2019
 *
 */
/* ************************************************************** */
uint8_t QTR_1RC_Task ()
{
	/* state: 0 ... set pin to output and load capacitor, 1 ... check pin state */
	static uint8_t state =0;
	
	/* execute state-machine */
	switch (state)
	{
		/* ************************************ */
		/* set pin to output and load capacitor */
		/* ************************************ */
		case 0: 
		{
			/* set all pins as output and set it to high */
			for (uint8_t i = 0; i < GROUNDS_SENSOR_NUMBER; i++)
			{
				groundSensors[i].qPort->DIRSET = (0x01 << groundSensors[i].qPin);
				groundSensors[i].qPort->OUTSET = (0x01 << groundSensors[i].qPin);
			}
		
			/* wait 100 µs -> so the capacitor can be charged */
			_delay_us(100);
		
			/* set all pins to input */
			for (uint8_t i = 0; i < GROUNDS_SENSOR_NUMBER; i++)
			{
				groundSensors[i].qPort->DIRCLR = (0x01 << groundSensors[i].qPin);
			}
			
			/* change to state 1 */
			state = 1;
			/* wait 1 ms */
			SET_CYCLE(QTR_1RC_TASKNBR,1);
			return(CYCLE);
		}
		/* *************** */
		/* check pin state */
		/* *************** */
		case 1:
		{
			/* check the state of each pin */
			for (uint8_t i = 0; i < GROUNDS_SENSOR_NUMBER; i++)
			{
				/* if the capacitor is discharged -> underground is white */
				/* if the capacitor is already charged -> underground is black */
				groundSensors[i].status = ((groundSensors[i].qPort->IN & (0x01 << groundSensors[i].qPin))? SENSOR_BLACK : SENSOR_WHITE);
			}
			
			/* go back to state 0 -> start procedure again */
			state = 0;
			/* set Task to ENABLE */
			return(ENABLE);
		}
	}
	/* default return -> CYCLE */
	return(CYCLE);
}