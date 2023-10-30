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

#define _WIFI_EXTERN


#include <avr/io.h>
#include "wifi.h"
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
#include "usDataFusion.h"

/* index to receive-array */
uint8_t wifiIndex = 0;
/* receive-array */
uint8_t wifiArray[200];
/* enable receive data from WIFI */
uint8_t wifiReceiveStart = 0;


/* ************************************************************** */
/*! \brief Initialize parser-task.
*
*  Function initialize the parser-task
*
*  \version 27.10.2020
*
*/
/* ************************************************************** */
void InitWifi()
{
	EndPositionSlave = 0;
	DriveHome = 0;
	
	/* cyclic task - cycle time: 10 ms */
	SET_CYCLE(WIFI_TASKNBR, 50);
	SET_TASK(WIFI_TASKNBR, CYCLE);
	SET_TASK_HANDLE(WIFI_TASKNBR, WifiTask);
	
	/* cyclic task - cycle time: 100 ms */
	SET_CYCLE(WIFI_CYCLIC_TASKNBR, 250);
	SET_TASK(WIFI_CYCLIC_TASKNBR, CYCLE);
	SET_TASK_HANDLE(WIFI_CYCLIC_TASKNBR, wifi_CyclicTask);
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
uint8_t ASCII2Num_Wifi(uint8_t ascii)
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
uint8_t WifiTask()
{
	volatile getChar_t p;
	uint8_t text1[150];
	uint8_t HexID;
	uint8_t ID;
	int16_t Xpos;
	int16_t Ypos;
	int16_t phi;
	
	SET_CYCLE(WIFI_TASKNBR, 50);
	
	while (1)
	{
		/* readout debug-interface */
		p = getChar_uart(&WIFI_IF);
		
		
		
		
		/* if valid data received */
		if (p.Status == USART_REC_OK)
		{
			/* start reception -> # */
			if (p.Data == BS_WIFI)
			{
				wifiIndex = 0;
				wifiReceiveStart = 1;
			}
			/* reception complete -> * */
			else if (p.Data == BE_WIFI)
			{
				wifiReceiveStart = 0;
				
					

				
				/* port-command -> north/south  */
				/* (#Pp* :: p -> 0 .. not defined, 1 .. south, 2 .. north) */
				if (wifiArray[0] == CMD_PORT)
				{
					if (RobotType_RAM == MASTER_ROBOT)
					{
						DriveHome = ASCII2Num_Wifi(wifiArray[1]);
					}
					else
					{
						EndPositionSlave = ASCII2Num_Wifi(wifiArray[1]);
					}
				}
				/* master position-command */
				/* (#mxxxxyyyy* :: xxxx -> x-position [mm], yyyy -> y-position [mm])         */
				else if(wifiArray[0] == CMD_MASTER_POS)
				{
					otherRobot.Xpos = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[1]) * 1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[2]) * 100 + (uint16_t)ASCII2Num_Wifi(wifiArray[3]) * 10 + (uint16_t)ASCII2Num_Wifi(wifiArray[4]));
					otherRobot.Ypos= (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[5]) * 1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[6]) * 100 + (uint16_t)ASCII2Num_Wifi(wifiArray[7]) * 10 + (uint16_t)ASCII2Num_Wifi(wifiArray[8]));
				}
				/* slave position-command */
				/* (#sxxxxyyyy* :: xxxx -> x-position [mm], yyyy -> y-position [mm])         */
				else if(wifiArray[0] == CMD_SLAVE_POS)
				{
					otherRobot.Xpos = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[1]) * 1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[2]) * 100 + (uint16_t)ASCII2Num_Wifi(wifiArray[3]) * 10 + (uint16_t)ASCII2Num_Wifi(wifiArray[4]));
					otherRobot.Ypos = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[5]) * 1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[6]) * 100 + (uint16_t)ASCII2Num_Wifi(wifiArray[7]) * 10 + (uint16_t)ASCII2Num_Wifi(wifiArray[8]));
					
				}
				/* Enemy position cmd*/
				/* (#exxxxyyyyvvvv...* :: xxxx -> x-position [mm], yyyy -> y-position [mm], vvvv -> variance)         */
				else if(wifiArray[0] == CMD_ENEMY_POS)
				{
					EnemyDataRaw.count = 4;
					for (uint8_t i = 0 ; i<4 ; i++)
					{
						EnemyDataRaw.x[i] =	(int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[1+i*12]) * 1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[2+i*12]) * 100 + (uint16_t)ASCII2Num_Wifi(wifiArray[3+i*12]) * 10 + (uint16_t)ASCII2Num_Wifi(wifiArray[4+i*12]));
						EnemyDataRaw.y[i] = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[5+i*12]) * 1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[6+i*12]) * 100 + (uint16_t)ASCII2Num_Wifi(wifiArray[7+i*12]) * 10 + (uint16_t)ASCII2Num_Wifi(wifiArray[8+i*12]));
						EnemyDataRaw.var[i] = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[9+i*12]) * 1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[10+i*12]) * 100 + (uint16_t)ASCII2Num_Wifi(wifiArray[11+i*12]) * 10 + (uint16_t)ASCII2Num_Wifi(wifiArray[12+i*12]));
					}
					validWIFI = 1;
				}
				else if(wifiArray[0] == CMD_HEX_POS)
				{
					for (uint8_t i = 0; i < 20; i++)
					{
						arucoCodes[i].iDetected = ((arucoCodes[i].iDetected > 0) ? arucoCodes[i].iDetected-- : 0);
					}
					for (uint8_t i = 0; i < (wifiIndex/10); i++)
					{
						if (spielZeit < 101)
						{
							ID = (int16_t)(ASCII2Num_Wifi(wifiArray[1+i*10])*10 + ASCII2Num_Wifi(wifiArray[2+i*10]));
							Xpos = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[3+i*10])*1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[4+i*10])*100 + (uint16_t)ASCII2Num_Wifi(wifiArray[5+i*10])*10);
							Ypos = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[6+i*10])*1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[7+i*10])*100 + (uint16_t)ASCII2Num_Wifi(wifiArray[8+i*10])*10);
							phi = (int16_t)(ASCII2Num_Wifi(wifiArray[9+i*10])*10 + ASCII2Num_Wifi(wifiArray[10+i*10]));
							
							if (SpielFarbe == YELLOW && Xpos > 1850 && Xpos < 2200 && Ypos > 1200 && Ypos < 1500)
							{
								if (ID == ARUCO_BLUE)
								{
									arucoCodes[0].ID = ID;
									arucoCodes[0].Xpos = Xpos;
									arucoCodes[0].Ypos = Ypos;
									arucoCodes[0].phi = phi;
										
									arucoCodes[0].iDetected = ((arucoCodes[0].iDetected < 10)? arucoCodes[0].iDetected + 2 : 10);	
 									sprintf(text1, "Blue at ID:%d X:%d Y:%d Phi:%d\r\n", ID,Xpos,Ypos,phi);
 									writeString_usart(&usartF0, text1);	
								}
								if (ID == ARUCO_RED)
								{
									arucoCodes[1].ID = ID;
									arucoCodes[1].Xpos = Xpos;
									arucoCodes[1].Ypos = Ypos;
									arucoCodes[1].phi = phi;
											   
									arucoCodes[1].iDetected = ((arucoCodes[1].iDetected < 10)? arucoCodes[1].iDetected + 2 : 10);
 									sprintf(text1, "Red at ID:%d X:%d Y:%d Phi:%d\r\n", ID,Xpos,Ypos,phi);
 									writeString_usart(&usartF0, text1);
								}
								if (ID == ARUCO_GREEN)
								{
									arucoCodes[2].ID = ID;
									arucoCodes[2].Xpos = Xpos;
									arucoCodes[2].Ypos = Ypos;
									arucoCodes[2].phi = phi;
									
									arucoCodes[2].iDetected = ((arucoCodes[2].iDetected < 10)? arucoCodes[2].iDetected + 2 : 10);
 									sprintf(text1, "Green at ID:%d X:%d Y:%d Phi:%d\r\n", ID,Xpos,Ypos,phi);
 									writeString_usart(&usartF0, text1);
								}
							}
							
							if (SpielFarbe == PURPLE && Xpos > 800 && Xpos < 1160 && Ypos > 1200 && Ypos < 1560)
							{
								if (ID == ARUCO_BLUE)
								{
									arucoCodes[0].ID = ID;
									arucoCodes[0].Xpos = Xpos;
									arucoCodes[0].Ypos = Ypos;
									arucoCodes[0].phi = phi;
																
									arucoCodes[0].iDetected = ((arucoCodes[0].iDetected < 10)? arucoCodes[0].iDetected + 2 : 10);
									sprintf(text1, "Blue at ID:%d X:%d Y:%d Phi:%d\r\n", ID,Xpos,Ypos,phi);
									writeString_usart(&usartF0, text1);
								}
								if (ID == ARUCO_RED)
								{
									arucoCodes[1].ID = ID;
									arucoCodes[1].Xpos = Xpos;
									arucoCodes[1].Ypos = Ypos;
									arucoCodes[1].phi = phi;
																
									arucoCodes[1].iDetected = ((arucoCodes[1].iDetected < 10)? arucoCodes[1].iDetected + 2 : 10);
									sprintf(text1, "Red at ID:%d X:%d Y:%d Phi:%d\r\n", ID,Xpos,Ypos,phi);
									writeString_usart(&usartF0, text1);
								}
								if (ID == ARUCO_GREEN)
								{
									arucoCodes[2].ID = ID;
									arucoCodes[2].Xpos = Xpos;
									arucoCodes[2].Ypos = Ypos;
									arucoCodes[2].phi = phi;
																
									arucoCodes[2].iDetected = ((arucoCodes[2].iDetected < 10)? arucoCodes[2].iDetected + 2 : 10);
									sprintf(text1, "Green at ID:%d X:%d Y:%d Phi:%d\r\n", ID,Xpos,Ypos,phi);
									writeString_usart(&usartF0, text1);
								}
							}
						}

						ID = (int16_t)(ASCII2Num_Wifi(wifiArray[1+i*10])*10 + ASCII2Num_Wifi(wifiArray[2+i*10]));
						Xpos = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[3+i*10])*1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[4+i*10])*100 + (uint16_t)ASCII2Num_Wifi(wifiArray[5+i*10])*10);
						Ypos = (int16_t)((uint16_t)ASCII2Num_Wifi(wifiArray[6+i*10])*1000 + (uint16_t)ASCII2Num_Wifi(wifiArray[7+i*10])*100 + (uint16_t)ASCII2Num_Wifi(wifiArray[8+i*10])*10);
						phi = (int16_t)(ASCII2Num_Wifi(wifiArray[9+i*10])*10 + ASCII2Num_Wifi(wifiArray[10+i*10]));
 						sprintf(text1, "ID:%d X:%d Y:%d Phi:%d\r\n", ID,Xpos,Ypos,phi);
 						writeString_usart(&usartF0, text1);
					}
		
				}
				
			}
			
			/* store received data */
			else if (wifiReceiveStart == 1)
			{
				wifiArray[wifiIndex++] = p.Data;
			}
		}
		/* no data available -> break */
		else
		{
			break;
		}
		
	}
	
	sendPosDataEnemy_from_WIFI_RRLAN(xPos, yPos, phiPos/10, &EnemyDataRaw);

	return(CYCLE);
}

/* ************************************************************** */
/*! \brief Port-Command (WIFI).
*
*  Sends cyclic Data
*
*  \version 13.05.2021
*/
/* ************************************************************** */
uint8_t wifi_CyclicTask()
{
	SET_CYCLE(WIFI_CYCLIC_TASKNBR, 250);
	
 	if(RobotType_RAM == MASTER_ROBOT)
 	{
	 	wifi_PortCommand(endPositionMaster);
	 	wifi_MasterPosCommand(xPos,yPos);
 	}
 	else
 	{
	 	wifi_PortCommand(DriveHome);
	 	wifi_SlavePosCommand(xPos,yPos);	 	
 	}
	return(CYCLE);
}


/* ************************************************************** */
/*! \brief Port-Command (WIFI).
*
*  Sends the Port-Command via WIFI
*
*  \param port ... port-number (0 .. not defined, 1 .. south, 2 .. north).
*
*  \version 13.05.2021
*/
/* ************************************************************** */
void wifi_PortCommand(uint8_t port)
{
	char msg[10];
	
	sprintf(msg, "#p%d*", port);
	writeString_usart(&WIFI_IF, msg);
}


/* ************************************************************** */
/*! \brief Master-Position-Command (WIFI).
*
*  Sends the Master-Position via WIFI
*
*  \param x ... x-position [mm] - always 4 digit.
*  \param y ... y-position [mm] - always 4 digit.
*
*  \version 13.05.2021
*/
/* ************************************************************** */
void wifi_MasterPosCommand(int16_t x, int16_t y)
{
	char msg[20];
	
	sprintf(msg, "#m%04d%04d* \n",x,y);
	writeString_usart(&WIFI_IF, msg);
}


/* ************************************************************** */
/*! \brief Slave-Position-Command (WIFI).
*
*  Sends the Slave-Position via WIFI
*
*  \param x ... x-position [mm] - always 4 digit.
*  \param y ... y-position [mm] - always 4 digit.
*
*  \version 13.05.2021
*/
/* ************************************************************** */
void wifi_SlavePosCommand(int16_t x, int16_t y)
{
	char msg[20];
	
	sprintf(msg, "#s%04d%04d* \n",x,y);
	writeString_usart(&WIFI_IF, msg);
}
