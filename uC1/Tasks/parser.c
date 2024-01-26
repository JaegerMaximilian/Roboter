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

#define _PARSER_EXTERN


#include <avr/io.h>
#include "parser.h"
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
#include "adc_driver.h"
#include "accumulator.h"
#include "sensor.h"
#include "anaPos.h"
#include "servo.h"
#include "motor.h"
#include "rrt_receivetask.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "Pfadplanung.h"
#include "command.h"
#include "ki.h"
#include "nextion.h"
#include "logger.h"
#include "observation.h"

/* index to receive-array */
uint8_t pIndex = 0;
/* receive-array */
uint8_t pArray[400];


/* ************************************************************** */
/*! \brief Initialize parser-task.
*
*  Function initialize the parser-task
*
*  \version 27.10.2020
*
*/
/* ************************************************************** */
void InitParser(void)
{
	/* cyclic task - cycle time: 500 ms */
	SET_CYCLE(PARSER_TASKNBR, 10);
	SET_TASK(PARSER_TASKNBR, CYCLE);
	SET_TASK_HANDLE(PARSER_TASKNBR, ParserTask);
}

/* ************************************************************** */
/*! \brief ASCII to number.
*
*  Converts a ASCII-character to a number
*
*  \version 27.10.2020
*
*/
/* ************************************************************** */
uint8_t ASCII2Num(uint8_t ascii)
{
	switch (ascii)
	{
		case '0':
		{
			return(0);
		}
		case '1':
		{
			return(1);
		}
		case '2':
		{
			return(2);
		}
		case '3':
		{
			return(3);
		}
		case '4':
		{
			return(4);
		}
		case '5':
		{
			return(5);
		}
		case '6':
		{
			return(6);
		}
		case '7':
		{
			return(7);
		}
		case '8':
		{
			return(8);
		}
		case '9':
		{
			return(9);
		}
	}
	
	return(0);
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
uint8_t ParserTask(void)
{
	volatile getChar_t p;
	volatile uint8_t nbr;
	volatile int16_t sgn;
	volatile int16_t angle;
	static uint8_t receiveEnable = FALSE;
	

	
	SET_CYCLE(PARSER_TASKNBR, 10);
	
	while (1)
	{
		/* readout debug-interface */
		p = getChar_uart(USART_EUROBOTLOGGER);
		
		/* if valid data received */
		if (p.Status == USART_REC_OK)
		{
			/* #S01+120*   */
			
			/* start receiption -> # */

			
			if (p.Data == BS)
			{
				pIndex = 0;
				receiveEnable = TRUE;
			}
			/* receiption complete -> * */
			else if (p.Data == BE)
			{
				
				receiveEnable = FALSE;
				
				if((pArray[1] == CHECK_ROBOT_TYPE(ADR_MASTER_ID,ADR_SLAVE_ID)) || (pArray[1] == ADR_BROADCAST_ID))
				{
					/* servo-command -> Snnsvvv    */
					/* nn ... servo number        */
					/* vvv ... value (0° .. 135°) */
					if ((pArray[2] == CMD_SERVO))
					{
						nbr = ASCII2Num(pArray[3]) * 10 + ASCII2Num(pArray[4]);
						sgn = (pArray[5] == '+') ? 1 : -1;
						angle = (int16_t)ASCII2Num(pArray[6]) * 100 + (int16_t)ASCII2Num(pArray[7]) * 10 + (int16_t)ASCII2Num(pArray[8]);
						angle *= sgn;
						cmd_SetServo(nbr, angle);
					}
					/* motor-command -> Mnnvvvvppp */
					/* nn ... motor-number        */
					/* vvvv ... velocity [mm/s]    */
					/* ppp ... position [mm]      */
					else if(pArray[2] == CMD_MOTOR)
					{
						uint8_t nbr = ASCII2Num(pArray[1]) * 10 + ASCII2Num(pArray[2]);
						uint16_t vel = (uint16_t)(ASCII2Num(pArray[3])) * 1000 + (uint16_t)(ASCII2Num(pArray[4])) * 100 + (uint16_t)(ASCII2Num(pArray[5])) * 10 + (uint16_t)(ASCII2Num(pArray[6]));
						uint16_t pos = (uint16_t)(ASCII2Num(pArray[7])) * 100 + (uint16_t)(ASCII2Num(pArray[8])) * 10 + (uint16_t)(ASCII2Num(pArray[9]));
						cmd_SetMotorPos(nbr, (float)(vel)/1000.0, (float)(pos)/1000.0);
					}
					/* digital-output-command -> Onnv */
					/* nn ... output-number			  */
					/* v ... value (0/1)			  */
					else if(pArray[2] == CMD_DIGITAL_OUT)
					{
					}
					/* vacuum-command -> Vnnv */
					/* nn ... output-number	  */
					/* v ... value (0/1)	  */
					else if(pArray[2] == CMD_VACUUM)
					{
						uint8_t nbr = ASCII2Num(pArray[1]) * 10 + ASCII2Num(pArray[2]);
						uint8_t val = ASCII2Num(pArray[3]);
						cmd_CtrlVacuum(nbr, val);
					}
					else if(pArray[2] == CMD_TEILE && pArray[0] == ADR_VISION_ID_V)
					{
						for (uint8_t i = 0; i <= 5; i++)
						{
							ownPosKamera[i].point.Xpos = 10000;
							ownPosKamera[i].point.Ypos = 10000;
							enemyPosRobotKamera[i].point.Xpos = 10000;
							enemyPosRobotKamera[i].point.Ypos = 10000;
						}
						
						uint8_t ownPosCount = 0;
						uint8_t enemyPosCount = 0;
						for (int i = 3; i < 400; i++ )
						{
							if(pArray[i] == NULL)
							{
								break;
							}
							
							if (pArray[i] != DEL)
							{

								
								uint8_t ID = ASCII2Num(pArray[i]) * 10 + ASCII2Num(pArray[i+1]);
								uint16_t xpos = ASCII2Num(pArray[i+2]) * 1000 + ASCII2Num(pArray[i+3]) * 100 + ASCII2Num(pArray[i+4]) * 10 + ASCII2Num(pArray[i+5]);
								uint16_t ypos = ASCII2Num(pArray[i+6]) * 1000 + ASCII2Num(pArray[i+7]) * 100 + ASCII2Num(pArray[i+8]) * 10 + ASCII2Num(pArray[i+9]);
								i = i + 9;
								
								//Own Position
								if(((ID >= 1 && ID <= 5) && SpielFarbe == BLUE) || ((ID >= 6 && ID <= 10) && SpielFarbe == Yellow))
								{
									ownPosKamera[ownPosCount].point.Xpos = (int16_t)xpos;
									ownPosKamera[ownPosCount].point.Ypos = ypos;
									ownPosKamera[ownPosCount].time = spielzeit_100telSek;
									ownPosCount++;
								}
								//Enemy Position
								else if(((ID >= 1 && ID <= 5) && SpielFarbe == Yellow) || ((ID >= 6 && ID <= 10) && SpielFarbe == BLUE))
								{
									enemyPosRobotKamera[enemyPosCount].point.Xpos = (int16_t)xpos;
									enemyPosRobotKamera[enemyPosCount].point.Ypos = (int16_t)ypos;
									enemyPosRobotKamera[enemyPosCount].time = spielzeit_100telSek;
									enemyPosCount++;
								}
							}
						}
					}
					
				}
			}
			/* store received data */
			else if ((receiveEnable == TRUE) && (pIndex < sizeof(pArray)))
			{
				pArray[pIndex++] = p.Data;
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


