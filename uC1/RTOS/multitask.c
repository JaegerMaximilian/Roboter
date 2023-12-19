/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
*
* \brief
*      Multitasking system source file.
*
*      This file contains the cooperative multitasking system with all components.
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
* $Date: 2012-08-23  $  \n
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

#define _MULTITASK_EXTERN

#include <avr/io.h>
#include "multitask.h"
#include "timer.h"
#include "global.h"
#include "define.h"
#include <avr/interrupt.h>
#include "ports.h"


extern uint16_t timeStamp;


/* ************************************************************** */
/*! \brief TCF0 ISR.
*
*  multitasking system cycle with 1 ms <-> 1 multitasking tick
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
ISR(TCF0_OVF_vect)
{
	uint8_t ucIndex;
	
	sysTime++;
	
	char text1[200];
	
	//if(sysTime > 50)
	//{
			//sprintf(text1, "KI: %d (%d) Debug: %d (%d)", sMultitasking[KI_TASKNBR].ucStatus, sMultitasking[KI_TASKNBR].uiIntervall, sMultitasking[DEBUG_TASKNBR].ucStatus , sMultitasking[DEBUG_TASKNBR].uiIntervall);
			//SendDebugMessage(text1,2);
			//sysTime = 0;
	//}


	/* execute the cyclic tasks */
	for(ucIndex = 0; ucIndex < MAX_TASK; ucIndex++)
	{
		/* task is a cyclic task */
		if(IS_CYCLE_TASK(ucIndex))
		{
			/* cycle time isn't run out ->
			decrement interval */
			if(sMultitasking[ucIndex].uiIntervall != 0)
			{
				sMultitasking[ucIndex].uiIntervall--;
			}
			/* cycle time is run out ->
			enable task, if it isn't happen before */
			else if(TASK_IS_ENABLED(ucIndex) == 0)
			{
				sMultitasking[ucIndex].ucStatus = 0;
				SET_TASK(ucIndex, ENABLE);
				SET_CALLER(ucIndex, CYCLE_CALL);
			}
		}
	}
}


/* ************************************************************** */
/*! \brief Multitasking system initialization.
*
*  Initialize and start multitasking system
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
void InitMultitasking(void)
{
	uint8_t ucIndex;
	
	
	
	/* disable all tasks */
	for(ucIndex = 0; ucIndex < MAX_TASK; ucIndex++)
	{
		sMultitasking[ucIndex].ucStatus = DISABLE;
		SET_CALLER(ucIndex, SYSTEM_CALL);
	}
	
	errorTask = 255;

	/* initialize TCF0 (multitasking system timer) */
	tcf0_init();

}


/* ************************************************************** */
/*! \brief Multitasking system.
*
*  Operate multitasking system
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
void MultitaskingSystem(void)
{
	static uint8_t ucIndex;
	
	/* if the dedicated task is enabled ->
	execute task */
	if(TASK_IS_ENABLED(ucIndex))
	{
		sMultitasking[ucIndex].ucStatus = 0;
		sMultitasking[ucIndex].ucStatus = sMultitasking[ucIndex].fpTaskHandle();
	}
	/* switch to next task */
	ucIndex = ((++ucIndex >= MAX_TASK)? 0 : ucIndex);
}
