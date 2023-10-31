/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
*
* \brief
*      Includes debug-task, here the software parts can be tested.
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
*      The file includes the parser-task. This task read out the debug-interface and controls
*      the the hardware of the robot
*
* \author
*      Michael Zauner
*      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
*      Support email: roboracing@fh-wels.at
*
* $Revision: 1 $
* $Date: 2020-10-27  $  \n
*
* Copyright (c) 2020, RRT (University of Applied Sciences Upper Austria) All rights reserved.
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

#define _NEXTION_EXTERN


#include <avr/io.h>
#include "nextion.h"
#include "multitask.h"
#include "ports.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "usart.h"
#include <stdint.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "rrt_receivetask.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "command.h"
#include "usart.h"

/* index to receive-array */
uint8_t NextionIndex = 0;
/* receive-array */
uint8_t NextionArray[400];


/* ************************************************************** */
/*! \brief Initialize parser-task.
*
*  Function initialize the parser-task
*
*  \version 27.10.2020
*
*/
/* ************************************************************** */
void InitNextion(void)
{
	/* initalize serial interface -> USARTC1 */
	usartc1_init(USART_C1_BUF_SIZE, USART_C1_BUF_SIZE);
	
	/* cyclic task - cycle time: 500 ms */
	SET_CYCLE(NEXTION_TASKNBR, 10);
	SET_TASK(NEXTION_TASKNBR, CYCLE);
	SET_TASK_HANDLE(NEXTION_TASKNBR, NextionTask);
}


/* ************************************************************** */
/*! \brief Set points in LCD.
*
*  Function set the shown points in the LCD
*
*  \param points points to set.
*
*  \version 30.10.2023
*
*/
/* ************************************************************** */
void NextionSetPoints(uint16_t points)
{
	uint8_t CmdEnd[4] = {0xff, 0xff, 0xff, 0x00};
	char text[50];

	sprintf(text, "txPoints.txt=\"%d\"", points);
	writeString_usart(&LCD_IF, CmdEnd);
	writeString_usart(&LCD_IF, text);
	writeString_usart(&LCD_IF, CmdEnd);
}


/* ************************************************************** */
/*! \brief Set playing-time in LCD.
*
*  Function set the shown playing-time in the LCD
*
*  \param time playing-time to set.
*
*  \version 30.10.2023
*
*/
/* ************************************************************** */
void NextionSetTime(uint16_t time)
{
	uint8_t CmdEnd[4] = {0xff, 0xff, 0xff};
	char text[50];
	
	sprintf(text, "pbProgress.val=%d", time);
	writeString_usart(&LCD_IF, CmdEnd);
	writeString_usart(&LCD_IF, text);
	writeString_usart(&LCD_IF, CmdEnd);
}

/* ************************************************************** */
/*! \brief Parser-task.
*
*  This task parses the debug-interface and read out the control data
*
*  \version 27.10.2020
*
*/
/* ************************************************************** */
uint8_t NextionTask(void)
{
	volatile getChar_t p;
	static uint8_t receiveEnable = FALSE;
	
	SET_CYCLE(NEXTION_TASKNBR, 10);
	
	while (1)
	{
		/* readout debug-interface */
		p = getChar_uart(&LCD_IF);
		
		/* if valid data received */
		if (p.Status == USART_REC_OK)
		{
			
			/* start receiption -> # */
			if (p.Data == NEXTION_BS)
			{
				NextionIndex = 0;
				receiveEnable = TRUE;
			}
			/* receiption complete -> * */
			else if (p.Data == NEXTION_BE)
			{
				if(NextionArray > 10)
				{
					SET_PIN(LED_PORT1, LED2);
				}
				
				receiveEnable = FALSE;
				/* message-structure: */
				/* 1. byte: Page*/
				/* 2. byte: Strategie */
				/* 3. byte: Startfeld */
				/* 4. byte: Config Planter */
				/* 5. byte: Config Stehlen */
				
				/* readout playing colour and strategy */
				Page = NextionArray[0];
				Strategie = NextionArray[1];
				SpielFarbe = NextionArray[2];
				ConfigPlanter = NextionArray[3];
				ConfigStehlen = NextionArray[4];
			}
			/* store received data */
			else if ((receiveEnable == TRUE) && (NextionIndex < sizeof(NextionArray)))
			{
				NextionArray[NextionIndex++] = p.Data;
			}
		}
		/* no data available -> break */
		else
		{
			break;
		}
		
	}
	
	return(CYCLE);
}



