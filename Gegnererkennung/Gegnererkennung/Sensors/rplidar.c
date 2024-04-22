/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
*
* \brief
*      LIDAR code file.
*
*      This file contains the initialization of the LIDAR driver.
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
*      This file contains the initialization of the graphic-LCD.
*
* \author
*      Michael Zauner
*      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
*      Support email: roboracing@fh-wels.at
*
* $Revision: 1 $
* $Date: 2019-02-05  $  \n
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

#define _RPLIDAR_EXTERN

#include <avr/io.h>
#include "rplidar.h"
#include "define.h"
#include "global.h"
#include "multitask.h"
#include "usart.h"
#include <string.h>
#include "timer.h"
#include <stdio.h>
#include <math.h>
#include "obstacle.h"


getChar_t lidarData;



/* ************************************************************** */
/*! \brief Initialize the LIDAR.
*
*  Initialize the LIDAR.
*
*  \version 05.02.2019
*
*/
/* ************************************************************** */
void rpLidar_Init()
{
	/* cyclic task - cycle time: 1 ms */
	SET_CYCLE(RPLIDAR_MSG_HANDLER_TASKNBR, 1);
	SET_TASK(RPLIDAR_MSG_HANDLER_TASKNBR, CYCLE);
	SET_TASK_HANDLE(RPLIDAR_MSG_HANDLER_TASKNBR, rpLidar_MsgHandlerTask);

	/* cyclic task - cycle time: 500 ms */
	SET_CYCLE(RPLIDAR_TASKNBR, 500);
	SET_TASK(RPLIDAR_TASKNBR, CYCLE);
	SET_TASK_HANDLE(RPLIDAR_TASKNBR, rpLidar_Task);
	
	/* initialize display */
	rpLidar.usart = &usartC0;
	rpLidar.mode = RPLIDAR_IDLE;
	rpLidar.state = RPLIDAR_RESET_STATE;
	rpLidar.scan_active = 0;
	rpLidar.qTimer0 = &TCD0;
	rpLidar.timer = 0;
	rpLidar.channel = CH_A;
	rpLidar.msg_index = 0;
	
	/* inti timer D0 to 25 kHz PWM on channel A */
	tcd0_init();
	/* set PWM to 60 % */
	rpLidar_setPWM(&rpLidar, 35);
}


/* ************************************************************** */
/*! \brief Calculate the checksum.
*
*  Calculate the checksum (xor all bytes).
*
*  \param pointer to the message.
*  \param number of bytes.
*
*  \retval 8-bit checksum (xor all bytes).
*
*  \version 05.02.2019
*
*/
/* ************************************************************** */
uint8_t rpLidar_calcChecksum(uint8_t *msg, uint8_t n)
{
	uint8_t checksum = 0;
	
	/* sum up all bytes of the message */
	for (uint8_t i = 0; i < n; i++)
	{
		checksum ^= msg[i];
	}
	
	/* return checksum */
	return(checksum);
}


/* ************************************************************** */
/*! \brief Set PWM (0 ... 100 %).
*
*  Set PWM.
*
*  \param pointer to the LIDAR.
*  \param pwm - 0 ... 100.
*
*  \version 05.02.2019
*
*/
/* ************************************************************** */
void rpLidar_setPWM(rplidar_t *lidar, uint8_t pwm)
{
	uint16_t pwm_16;
	
	/* TIMER0 */
	if (lidar->timer == 0)
	{
		/* calculate pwm-value between 0 ... PER */
		pwm_16 = (uint16_t)(((uint32_t)pwm * (uint32_t)(lidar->qTimer0->PER)) / 100UL);
		/* set pwm-value */
		SetPWM0(lidar->qTimer0, lidar->channel, pwm_16);
	}
	/* TIMER1 */
	else
	{
		/* calculate pwm-value between 0 ... PER */
		pwm_16 = (uint16_t)(((uint32_t)pwm * (uint32_t)(lidar->qTimer1->PER)) / 100UL);
		/* set pwm-value */
		SetPWM1(lidar->qTimer1, lidar->channel, pwm_16);
	}
}


/* ************************************************************** */
/*! \brief Send a request to the LIDAR.
*
*  Send a request to the LIDAR.
*
*  \param pointer to the LIDAR.
*  \param request request number.
*
*  \version 05.02.2019
*
*/
/* ************************************************************** */
void rpLidar_sendRequest(rplidar_t *lidar, uint8_t request)
{
	uint8_t data[9];
	
	/* STX of request message */
	data[0] = RPLIDAR_STX1;
	/* request type */
	data[1] = request;
	
	/* if the request is an EXPRESS SCAN -> add message */
	if (request == RPLIDAR_EXPRESS_SCAN)
	{
		/* length of message */
		data[2] = 0x05;
		/* working mode -> standard scan mode */
		data[3] = 0x00;
		/* all other bytes in message are by default 0 */
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;
		data[7] = 0x00;
		/* add checksum to message */
		data[8] = 0x22;
		/* send data to usart */
		writePacket_usart(rpLidar.usart, data, 9);
	}
	/* no EXPRESS SCAN request -> calculate checksum */
	else
	{
		/* send data to usart */
		writePacket_usart(rpLidar.usart, data, 2);
	}
	
	/* set mode to the request number -> waiting for response */
	if ((request != RPLIDAR_STOP) && (request != RPLIDAR_RESET))
	{
		rpLidar.mode = request;
	}
	/* the STOP and RESET message don't have a response -> next state is IDLE */
	else
	{
		rpLidar.mode = RPLIDAR_IDLE;
	}
	
}


/* ************************************************************** */
/*! \brief RPLIDAR task.
*
*  The RPLIDAR task handles the communication to the LIDAR.
*
*  \version 05.02.2019
*
*/
/* ************************************************************** */
uint8_t rpLidar_MsgHandlerTask()
{
	uint8_t text[100];
	uint16_t x, y;
	
	
	/* set cycle-time to 1 ms */
	SET_CYCLE(RPLIDAR_MSG_HANDLER_TASKNBR, 1);
	
	/* *************************** */
	/* communication state-machine */
	/* *************************** */
	switch (rpLidar.mode)
	{
		/* ***************************************** */
		/* idle-state -> ready to send a new message */
		/* ***************************************** */
		case RPLIDAR_IDLE:
		{
			/* reset message index */
			rpLidar.msg_index = 0;
			rpLidar.scan_active = 0;
			
			break;
		}
		/* ******************************* */
		/* wait for an response (GET INFO) */
		/*  ****************************** */
		case RPLIDAR_GET_INFO:
		{
			/* readout serial data */
			lidarData = getChar_uart(rpLidar.usart);
			
			/* check if new date has been received */
			while (lidarData.Status == USART_REC_OK)
			{
				rpLidar.recBuf[rpLidar.msg_index] = lidarData.Data;
				(rpLidar.msg_index)++;
				
				/* after the reception of a complete message */
				if (rpLidar.msg_index == 27)
				{
					/* check response packet -> 0xA5::0x5A::0x14::0x00::0x00::0x00::0x04 */
					if ((rpLidar.recBuf[0] == RPLIDAR_STX1) &&
					(rpLidar.recBuf[1] == RPLIDAR_STX2) &&
					(rpLidar.recBuf[2] == 0x14) &&
					(rpLidar.recBuf[3] == 0x00) &&
					(rpLidar.recBuf[4] == 0x00) &&
					(rpLidar.recBuf[5] == 0x00) &&
					(rpLidar.recBuf[6] == 0x04))
					{
						/* readout RPLIDAR model ID */
						rpLidar.info.model = rpLidar.recBuf[7];
						/* readout error code */
						rpLidar.info.firmware = (uint16_t)rpLidar.recBuf[8] + (((uint16_t)rpLidar.recBuf[9]) << 8);
						/* readout Hardware version number */
						rpLidar.info.hardware = rpLidar.recBuf[10];
						/* readout 128bit unique serial number */
						for (uint8_t i = 0; i < 16; i++)
						{
							rpLidar.info.serialnumber[i] = rpLidar.recBuf[i + 11];
						}
					}

					/* switch back to IDLE mode */
					rpLidar.mode = RPLIDAR_IDLE;
					rpLidar.msg_index = 0;
					
					/* exit data receive */
					break;
				}
				/* readout new data */
				lidarData = getChar_uart(rpLidar.usart);
			}
			break;
		}
		/* ********************************* */
		/* wait for an response (GET HEALTH) */
		/*  ******************************** */
		case RPLIDAR_GET_HEALTH:
		{
			/* readout serial data */
			lidarData = getChar_uart(rpLidar.usart);
			
			/* check if new date has been received */
			while (lidarData.Status == USART_REC_OK)
			{
				rpLidar.recBuf[rpLidar.msg_index] = lidarData.Data;
				(rpLidar.msg_index)++;

				
				/* after the reception of a complete message */
				if (rpLidar.msg_index == 10)
				{
					/* check response packet -> 0xA5::0x5A::0x03::0x00::0x00::0x00::0x06 */
					if ((rpLidar.recBuf[0] == RPLIDAR_STX1) &&
					(rpLidar.recBuf[1] == RPLIDAR_STX2) &&
					(rpLidar.recBuf[2] == 0x03) &&
					(rpLidar.recBuf[3] == 0x00) &&
					(rpLidar.recBuf[4] == 0x00) &&
					(rpLidar.recBuf[5] == 0x00) &&
					(rpLidar.recBuf[6] == 0x06))
					{
						/* readout status (0 ... Good, 1 ... Warning, 2 ... Error) */
						rpLidar.health.status = rpLidar.recBuf[7];
						/* readout error code */
						rpLidar.health.error_code = (uint16_t)rpLidar.recBuf[8] + (((uint16_t)rpLidar.recBuf[9]) << 8);
						
						/* after a GET HEALTH message -> start scanning */
						rpLidar.state = RPLIDAR_START_SCAN_STATE;
						SET_CYCLE(RPLIDAR_TASKNBR, 500);
						SET_TASK(RPLIDAR_TASKNBR, CYCLE);
					}
					
					/* switch back to IDLE mode */
					rpLidar.mode = RPLIDAR_IDLE;
					rpLidar.msg_index = 0;
					
					/* exit data receive */
					break;
				}
				/* readout new data */
				lidarData = getChar_uart(rpLidar.usart);
			}
			
			break;
		}
		/* ************************************* */
		/* wait for an response (GET SAMPLERATE) */
		/*  ************************************ */
		case RPLIDAR_GET_SAMPLERATE:
		{
			/* readout serial data */
			lidarData = getChar_uart(rpLidar.usart);
			
			/* check if new date has been received */
			while (lidarData.Status == USART_REC_OK)
			{
				rpLidar.recBuf[rpLidar.msg_index] = lidarData.Data;
				(rpLidar.msg_index)++;

				/* after the reception of a complete message */
				if (rpLidar.msg_index == 11)
				{
					/* check response packet -> 0xA5::0x5A::0x04::0x00::0x00::0x00::0x15 */
					if ((rpLidar.recBuf[0] == RPLIDAR_STX1) &&
					(rpLidar.recBuf[1] == RPLIDAR_STX2) &&
					(rpLidar.recBuf[2] == 0x04) &&
					(rpLidar.recBuf[3] == 0x00) &&
					(rpLidar.recBuf[4] == 0x00) &&
					(rpLidar.recBuf[5] == 0x00) &&
					(rpLidar.recBuf[6] == 0x15))
					{
						/* readout sample rate in SCAN mode */
						rpLidar.sampleRate.Tstandard = (uint16_t)rpLidar.recBuf[7] + (((uint16_t)rpLidar.recBuf[8]) << 8);
						/* readout sample rate in EXPRESS SCAN mode */
						rpLidar.sampleRate.Texpress = (uint16_t)rpLidar.recBuf[9] + (((uint16_t)rpLidar.recBuf[10]) << 8);
					}
					
					/* switch back to IDLE mode */
					rpLidar.mode = RPLIDAR_IDLE;
					rpLidar.msg_index = 0;
					
					/* exit data receive */
					break;
				}
				/* readout new data */
				lidarData = getChar_uart(rpLidar.usart);
			}
			
			break;
		}
		/* *************************** */
		/* wait for an response (SCAN) */
		/*  ************************** */
		case RPLIDAR_SCAN:
		{
			/* readout serial data */
			lidarData = getChar_uart(rpLidar.usart);
			
			/* check if new date has been received */
			while (lidarData.Status == USART_REC_OK)
			{
				rpLidar.recBuf[rpLidar.msg_index] = lidarData.Data;
				
				/* limit message index to 99 -> buffer has only a size of 100! */
				rpLidar.msg_index = ((++(rpLidar.msg_index) <= 99) ? rpLidar.msg_index : 99);
				
				/* after the reception of a complete message */
				/* *************************** */
				/* scan is active -> read data */
				/* *************************** */
				if ((rpLidar.scan_active == 1) && (rpLidar.msg_index == 5))
				{
					/* reset timeout */
					SET_CYCLE(RPLIDAR_TASKNBR, 500);
					
					/* readout scan quality */
					rpLidar.scan.quality = rpLidar.recBuf[0] >> 2;
					/* readout scan quality */
					rpLidar.scan.angle = (float)(((uint16_t)rpLidar.recBuf[1] >> 1) + ((uint16_t)rpLidar.recBuf[2] << 7)) / 64.0;
					/* readout scan quality */
					rpLidar.scan.distance = (float)((uint16_t)rpLidar.recBuf[3] + ((uint16_t)rpLidar.recBuf[4] << 8)) / 4.0;


	
					/* reset message index */
					rpLidar.msg_index = 0;

					/* operate scan */
					if(rpLidar.scan.distance > 100.0 && rpLidar.scan.distance < 4000.0)
					{
						//uint16_t angle = (uint16_t)rpLidar.scan.angle;
						//uint16_t dis = (uint16_t)rpLidar.scan.distance;
					//uint8_t text1[150];
					//sprintf(text1, "angle:%u dis:%u \r\n",angle,dis);
					//writeString_usart(&usartD0,text1);
						OBSTACLE_Scan2Pos(&(rpLidar.scan));
					}
					
					// 					x = xPos + (uint16_t)(cos(DEG2RAD(RPLIDAR_CONV_ANGLE(rpLidar.scan.angle) + phiPos)) * rpLidar.scan.distance);
					// 					y = yPos + (uint16_t)(sin(DEG2RAD(RPLIDAR_CONV_ANGLE(rpLidar.scan.angle) + phiPos)) * rpLidar.scan.distance);
					// //					if ((rpLidar.scan.distance > 0.0) && (rpLidar.scan.distance < 3000.0))//((rpLidar.recBuf[0] & 0x01))
					//
					// 					if ((x > 0) && (x < 3000) && (y > 0) && (y < 2000))
					// 					{
					//
					// 						//sprintf(text,"%.0f;%.0f\r\n", rpLidar.scan.angle, rpLidar.scan.distance);
					// 					//	sprintf(text,"%.1f;%.0f\r\n", rpLidar.scan.angle, rpLidar.scan.distance);
					// 						sprintf(text,"R - %.d;%.d\r\n",x,y);
					// 						writeString_usart(&usartD0, text);
					// 					}
					

					

					
				}
				/* ******************************* */
				/* scan isn't active -> start scan */
				/* ******************************* */
				else if ((rpLidar.scan_active == 0) && (rpLidar.msg_index == 7))
				{
					/* check response packet -> 0xA5::0x5A::0x05::0x00::0x00::0x40::0x81 */
					if ((rpLidar.recBuf[0] == RPLIDAR_STX1) &&
					(rpLidar.recBuf[1] == RPLIDAR_STX2) &&
					(rpLidar.recBuf[2] == 0x05) &&
					(rpLidar.recBuf[3] == 0x00) &&
					(rpLidar.recBuf[4] == 0x00) &&
					(rpLidar.recBuf[5] == 0x40) &&
					(rpLidar.recBuf[6] == 0x81))
					{
						/* activate scan */
						rpLidar.scan_active = 1;
						/* reset message index */
						rpLidar.msg_index = 0;
						#ifdef _DEBUG_LIDAR_
						sprintf(text, "Start SCAN!\r\n");
						writeString_usart(&usartD0, text);
						#endif
					}
				}
				/* readout new data */
				lidarData = getChar_uart(rpLidar.usart);
			}
			
			break;
		}
		/* *********************************** */
		/* wait for an response (EXPRESS SCAN) */
		/*  ********************************** */
 		case RPLIDAR_EXPRESS_SCAN:
 		{
 			/* readout serial data */
 			lidarData = getChar_uart(rpLidar.usart);
 			
 			/* check if new date has been received */
 			while (lidarData.Status == USART_REC_OK)
 			{
	 			rpLidar.recBuf[rpLidar.msg_index] = lidarData.Data;
	 			
	 			/* limit message index to 99 -> buffer has only a size of 100! */
	 			rpLidar.msg_index = ((++(rpLidar.msg_index) <= 99) ? rpLidar.msg_index : 99);
	 			
	 			/* after the reception of a complete message */
	 			/* *************************** */
	 			/* scan is active -> read data */
	 			/* *************************** */
	 			if ((rpLidar.scan_active == 1) && (rpLidar.msg_index == 5))
	 			{
		 			/* reset timeout */
		 			SET_CYCLE(RPLIDAR_TASKNBR, 500);
		 			
		 			/* readout scan quality */
		 			rpLidar.scan.quality = rpLidar.recBuf[0] >> 2;
		 			/* readout scan quality */
		 			rpLidar.scan.angle = (float)(((uint16_t)rpLidar.recBuf[1] >> 1) + ((uint16_t)rpLidar.recBuf[2] << 7)) / 64.0;
		 			/* readout scan quality */
		 			rpLidar.scan.distance = (float)((uint16_t)rpLidar.recBuf[3] + ((uint16_t)rpLidar.recBuf[4] << 8)) / 4.0;

		 			/* reset message index */
		 			rpLidar.msg_index = 0;

		 			/* operate scan */
		 			if(rpLidar.scan.distance > 100.0)
		 			{
			 			OBSTACLE_Scan2Pos(&(rpLidar.scan));
		 			}
		 			
		 			// 					x = xPos + (uint16_t)(cos(DEG2RAD(RPLIDAR_CONV_ANGLE(rpLidar.scan.angle) + phiPos)) * rpLidar.scan.distance);
		 			// 					y = yPos + (uint16_t)(sin(DEG2RAD(RPLIDAR_CONV_ANGLE(rpLidar.scan.angle) + phiPos)) * rpLidar.scan.distance);
		 			// //					if ((rpLidar.scan.distance > 0.0) && (rpLidar.scan.distance < 3000.0))//((rpLidar.recBuf[0] & 0x01))
		 			//
		 			// 					if ((x > 0) && (x < 3000) && (y > 0) && (y < 2000))
		 			// 					{
		 			//
		 			// 						//sprintf(text,"%.0f;%.0f\r\n", rpLidar.scan.angle, rpLidar.scan.distance);
		 			// 					//	sprintf(text,"%.1f;%.0f\r\n", rpLidar.scan.angle, rpLidar.scan.distance);
		 			// 						sprintf(text,"R - %.d;%.d\r\n",x,y);
		 			// 						writeString_usart(&usartD0, text);
		 			// 					}
		 			

		 			

		 			
	 			}
	 			/* ******************************* */
	 			/* scan isn't active -> start scan */
	 			/* ******************************* */
	 			else if ((rpLidar.scan_active == 0) && (rpLidar.msg_index == 7))
	 			{
		 			/* check response packet -> 0xA5::0x5A::0x05::0x00::0x00::0x40::0x81 */
		 			if ((rpLidar.recBuf[0] == RPLIDAR_STX1) &&
		 			(rpLidar.recBuf[1] == RPLIDAR_STX2) &&
		 			(rpLidar.recBuf[2] == 0x54) &&
		 			(rpLidar.recBuf[3] == 0x00) &&
		 			(rpLidar.recBuf[4] == 0x00) &&
		 			(rpLidar.recBuf[5] == 0x40) &&
		 			(rpLidar.recBuf[6] == 0x82))
		 			{
			 			/* activate scan */
			 			rpLidar.scan_active = 1;
			 			/* reset message index */
			 			rpLidar.msg_index = 0;
			 			#ifdef _DEBUG_LIDAR_
			 			sprintf(text, "Start SCAN!\r\n");
			 			writeString_usart(&usartD0, text);
			 			#endif
		 			}
	 			}
	 			/* readout new data */
	 			lidarData = getChar_uart(rpLidar.usart);
 			}

 			break;
 		}
		/* ********************************* */
		/* wait for an response (FORCE SCAN) */
		/*  ******************************** */
		case RPLIDAR_FORCE_SCAN:
		{

			
			break;
		}
	}
	
	/* return cyclic */
	return(CYCLE);
}


/* ************************************************************** */
/*! \brief RPLIDAR task.
*
*  The RPLIDAR task handles the communication to the LIDAR.
*
*  \version 05.02.2019
*
*/
/* ************************************************************** */
uint8_t rpLidar_Task()
{
	uint8_t text[100];
	
	/* set cycle-time to 500 ms */
	SET_CYCLE(RPLIDAR_TASKNBR, 500);
	
	switch (rpLidar.state)
	{
		/* *********************** */
		/* check health from LIDAR */
		/* *********************** */
		case RPLIDAR_CHECK_HEALTH_STATE:
		{
			#ifdef _DEBUG_LIDAR_
			sprintf(text, "Get health information from LIDAR!\r\n");
			writeString_usart(&usartD0, text);
			#endif

			/* send GET HEALTH message */
			rpLidar_sendRequest(&rpLidar, RPLIDAR_GET_HEALTH);
			/* reset receive buffer */
			rpLidar.usart->RecIndexIn = 0;
			rpLidar.usart->RecIndexOut = 0;
			/* switch to START SCAN state */
			rpLidar.state = RPLIDAR_START_SCAN_STATE;
			return (DISABLE);
			
			break;
		}
		/* ************** */
		/* start scanning */
		/* ************** */
		case RPLIDAR_START_SCAN_STATE:
		{
			/* check health result */
			/* when the health is GOOD -> start scanning */
			if (rpLidar.health.status == RPLIDAR_HEALTH_GOOD)
			{
				#ifdef _DEBUG_LIDAR_
				sprintf(text, "Prepare to start LIDAR!\r\n");
				writeString_usart(&usartD0, text);
				#endif
				/* start scanning */
				rpLidar_sendRequest(&rpLidar, RPLIDAR_SCAN);
				/* set next state (RESET STATE) - occurs only if timeout happens */
				rpLidar.state = RPLIDAR_RESET_STATE;
				/* set first timeout to 5000 ms, so the LIDAR can startup and
				stabilize the motor rotation */
				SET_CYCLE(RPLIDAR_TASKNBR, 5000);
				return (CYCLE);
			}
			/* else -> reset laser */
			else
			{
				rpLidar.state = RPLIDAR_RESET_STATE;
				SET_CYCLE(RPLIDAR_TASKNBR, 5);
			}
			
			break;
		}
		/* ****************************************************************** */
		/* reset LIDAR - this state is active during scanning and is executed */
		/*   after a timeout                                                  */
		/* ****************************************************************** */
		case RPLIDAR_RESET_STATE:
		{
			#ifdef _DEBUG_LIDAR_
			sprintf(text, "Reset LIDAR!\r\n");
			writeString_usart(&usartD0, text);
			#endif

			/* reset LIDAR */
			rpLidar_sendRequest(&rpLidar, RPLIDAR_RESET);
			/* reset receive buffer */
			rpLidar.usart->RecIndexIn = 0;
			rpLidar.usart->RecIndexOut = 0;
			/* start again with a health request - after 10 ms
			(pause should be longer than 2 ms) */
			rpLidar.state = RPLIDAR_CHECK_HEALTH_STATE;
			SET_CYCLE(RPLIDAR_TASKNBR, 1000);
			break;
		}
	}

	/* return cyclic */
	return(CYCLE);
}