/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      EA_eDIPTFT code file.
 *
 *      This file contains the initialization of the graphic-LCD driver.
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
 * $Date: 2018-11-29  $  \n
 *
 * Copyright (c) 2018, RRT (University of Applied Sciences Upper Austria) All rights reserved.
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

#define _EA_EDIPTFT_EXTERN

#include <avr/io.h>
#include "EA_eDIPTFT.h"
#include "define.h"
#include "global.h"
#include "multitask.h"
#include "usart.h"
#include <string.h>
#include "LEDcontrol.h"
#include <avr/interrupt.h>


getChar_t lcdData;


/* ************************************************************** */
/*! \brief Extern interrupt handler for CAN controller.
 *
 *  \version 09.08.2017
 */
/* ************************************************************** */
ISR(PORTD_INT0_vect)
{
	TOGGLE_PIN(LED_PORT, LED_2);
	SET_TASK(EA_EDIPTFT_SBUF_TASKNBR, ENABLE);
}


/* ************************************************************** */
/*! \brief Initialize the EA eDIPTFT display.
 *
 *  Initialize the EA eDIPTFT display. 
 *
 *  \version 29.11.2018
 *
 */
/* ************************************************************** */
void EA_eDIPTFT_Init()
{
	/* cyclic task - cycle time: 5 ms */
	SET_CYCLE(EA_EDIPTFT_TASKNBR, 5);
	SET_TASK(EA_EDIPTFT_TASKNBR, CYCLE);
	SET_TASK_HANDLE(EA_EDIPTFT_TASKNBR, EA_eDIPTFT_Task);
	
	DISABLE_TASK(EA_EDIPTFT_SBUF_TASKNBR);
	SET_TASK_HANDLE(EA_EDIPTFT_SBUF_TASKNBR, EA_eDIPTFT_SBuf_Task);

	/* cyclic Check SBUF - cycle time: 10 ms */
	SET_CYCLE(EA_EDIPTFT_SBUF_CHECK_TASKNBR, 10);
	SET_TASK(EA_EDIPTFT_SBUF_CHECK_TASKNBR, CYCLE);
	SET_TASK_HANDLE(EA_EDIPTFT_SBUF_CHECK_TASKNBR, EA_eDIPTFT_CheckSBUF);
	
	/* initialize display */
	EA_eDIPTFT_Display.size_x = 320;
	EA_eDIPTFT_Display.size_y = 240;
	EA_eDIPTFT_Display.usart = &usartD1;
	EA_eDIPTFT_Display.state = EA_EDIPTFT_IDLE;
	EA_eDIPTFT_Display.status = EA_EDIPTFT_IDLE;
	
}	

uint8_t EA_eDIPTFT_CheckSBUF()
{
	SET_CYCLE(EA_EDIPTFT_SBUF_CHECK_TASKNBR, 10);

	if ((PORTD.IN & 0x01)==0)
	{
		//#ifdef DEBUG_SBufCheck
		//char text[50];
		//sprintf(text, "SBuf=0 (CheckTask)\r\n");
		//writeString_usart(&usartD0, text);
		//#endif
		TOGGLE_PIN(LED_PORT, LED_2);
		SET_TASK(EA_EDIPTFT_SBUF_TASKNBR, ENABLE);
	}
	
	/* return checksum */
	return(CYCLE);
}

/* ************************************************************** */
/*! \brief Calculate the small protocol checksum.
 *
 *  Calculate the checksum (sum over all bytes). 
 *
 *  \param pointer to the message.
 *  \param number of bytes.
 *
 *  \retval 8-bit checksum (sum over all bytes).
 *
 *  \version 06.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_calcChecksum(uint8_t *msg, uint8_t n)
{
	uint8_t checksum = 0;
	
	/* sum up all bytes of the message */
	for (uint8_t i = 0; i < n; i++)
	{
		checksum += msg[i];
	}
	
	/* return checksum */
	return(checksum);
}


/* ************************************************************** */
/*! \brief EA eDIPTFT read SBUF task.
 *
 *  The EA eDIPTFT task reads the SBUF from to the eDIPTFT-display. 
 *
 *  \version 21.02.2019
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_SBuf_Task()
{
	static uint8_t data[50];
	static uint8_t state = 0;

	if (state == 0)
	{
		for (uint8_t i = 0; i < 50; i++)
		{
			data[i] = 0;
		}
		EA_eDIPTFT_Display.readBuf = data;
		/* send "read SBUF" message */
		EA_eDIPTFT_GetSendBuffer(&EA_eDIPTFT_Display, EA_EDIPTFT_SBUF_TASKNBR);
		
		state = 1;
	} 
	else
	{
		/* */
		lastTouchEvent = data[3];
		
		state = 0;
	}
	
	return (DISABLE);
}

/* ************************************************************** */
/*! \brief EA eDIPTFT task.
 *
 *  The EA eDIPTFT task handles the communication to the eDIPTFT-display. 
 *
 *  \version 29.11.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_Task()
{
	/* set cycle-time to 1 ms */
	SET_CYCLE(EA_EDIPTFT_TASKNBR, 5);
	
	/* *************************** */
	/* communication state-machine */
	/* *************************** */
	switch (EA_eDIPTFT_Display.state)
	{
		/* ***************************************** */
		/* idle-state -> ready to send a new message */
		/* ***************************************** */
		case EA_EDIPTFT_IDLE:
		{
			EA_eDIPTFT_Display.status = EA_EDIPTFT_IDLE;
			EA_eDIPTFT_Display.resendCounter = EA_EDIPTFT_MAX_RESEND;
			EA_eDIPTFT_Display.timeout = EA_EDIPTFT_TIMEOUT;
			
			break;
		}
		/* **************************************************************** */
		/* wait-state -> wait for an answer from the display (with timeout) */
		/* **************************************************************** */
		case EA_EDIPTFT_WAIT:
		{
			/* readout serial data */
			lcdData = getChar_uart(EA_eDIPTFT_Display.usart);
			
			/* check if new date has been received */
			while (lcdData.Status == USART_REC_OK) 
			{
				/* display has sent an ACK */
				if (lcdData.Data == EA_EDIPTFT_ACK)
				{
					/* switch back to idle mode */
					EA_eDIPTFT_Display.state = EA_EDIPTFT_IDLE;
					/* activate callback task */
					SET_TASK(EA_eDIPTFT_Display.callbackTask, ENABLE);
					/* clear error */
					EA_eDIPTFT_Display.error = 0;

				}
				/* display has sent an NAK */
				else if (lcdData.Data == EA_EDIPTFT_NAK)
				{
					/* switch to resend procedure */
					EA_eDIPTFT_Display.state = EA_EDIPTFT_RESEND;
				}
				
				/* read new data */
				lcdData = getChar_uart(EA_eDIPTFT_Display.usart);
			} 
			
			/* handle timeout */
			/* if timeout (10ms) is over -> switch to resend procedure */
			if (--(EA_eDIPTFT_Display.timeout) == 0)
			{
				EA_eDIPTFT_Display.state = EA_EDIPTFT_RESEND;
			}

			break;
		}
		/* ************************************************* */
		/* resend-state -> if a NAK or no answer is received */
		/*   -> send message again (maximum 3 resends)		 */
		/* ************************************************* */
		case EA_EDIPTFT_RESEND:
		{
			if ((EA_eDIPTFT_Display.resendCounter)-- > 0)
			{
				/* send data to serial interface */
				writePacket_usart(EA_eDIPTFT_Display.usart, EA_eDIPTFT_Display.sendBuf, (EA_eDIPTFT_Display.sendBuf[1] + 3));
				/* change state to wait-state (wait for answer from the display) */
				EA_eDIPTFT_Display.state = EA_EDIPTFT_WAIT;
				/* set timeout again */
				EA_eDIPTFT_Display.timeout = EA_EDIPTFT_TIMEOUT;
			} 
			else
			{
				/* switch back to idle mode */
				EA_eDIPTFT_Display.state = EA_EDIPTFT_IDLE;
				/* set error */
				EA_eDIPTFT_Display.error = 1;
			}
			
			break;
		}
		/* ************************************************** */
		/* read-state -> read message from display (ACK/NAK)) */
		/* ************************************************** */
		case EA_EDIPTFT_READ:
		{
			/* **************** */
			/* read out ACK/NAK */ 
			/* **************** */
			/* readout serial data */
 			lcdData = getChar_uart(EA_eDIPTFT_Display.usart);
 			
 			/* check if new date has been received */
 			if (lcdData.Status == USART_REC_OK) 
 			{
 				/* display has sent an ACK */
 				if (lcdData.Data == EA_EDIPTFT_ACK)
 				{
 					/* switch back to idle mode */
 					EA_eDIPTFT_Display.state = EA_EDIPTFT_IDLE;
 					/* clear error */
 					EA_eDIPTFT_Display.error = 0;
					/* reset message index and checksum */
					EA_eDIPTFT_Display.msgIndex = 0;
					EA_eDIPTFT_Display.checksum = 0;  
					/* change state to read message-state (wait for answer from the display) */
					EA_eDIPTFT_Display.state = EA_EDIPTFT_READ_MSG;
 				}
 				/* display has sent an NAK */
 				else if (lcdData.Data == EA_EDIPTFT_NAK)
 				{
 					/* switch to resend procedure */
 					EA_eDIPTFT_Display.state = EA_EDIPTFT_RESEND;
					/* set error */
					EA_eDIPTFT_Display.error = 1;
					break;
 				}
 			} 
			break;
		}
		/* ************************************************* */
		/* read-state -> read message from display (message) */
		/* ************************************************* */
		case EA_EDIPTFT_READ_MSG:
		{
 			/* **************** */
			/* read out message */ 
			/* **************** */
			/* readout serial data */
			lcdData = getChar_uart(EA_eDIPTFT_Display.usart);
			
			/* check if new date has been received */
			while (lcdData.Status == USART_REC_OK) 
			{
				/* STX */
				if (EA_eDIPTFT_Display.msgIndex == 0)
				{
					/* check the STX of the message (DC1 or DC2) */
					if ((lcdData.Data != EA_EDIPTFT_DC1) && (lcdData.Data != EA_EDIPTFT_DC2))
					{
 						/* switch to resend procedure */
 						EA_eDIPTFT_Display.state = EA_EDIPTFT_RESEND;
						/* set error */
						EA_eDIPTFT_Display.error = 1;
					}
					(EA_eDIPTFT_Display.msgIndex)++;
				}
				/* length */
				else if (EA_eDIPTFT_Display.msgIndex == 1)
				{
					EA_eDIPTFT_Display.length = lcdData.Data;
					EA_eDIPTFT_Display.items = lcdData.Data / 4;
					(EA_eDIPTFT_Display.msgIndex)++;
				}
				/* checksum */
				else if ((EA_eDIPTFT_Display.msgIndex - 2) >= EA_eDIPTFT_Display.length)
				{
					/* checksum was not equal -> resend */
					if (lcdData.Data != EA_eDIPTFT_Display.checksum)
					{
 						/* switch to resend procedure */
 						EA_eDIPTFT_Display.state = EA_EDIPTFT_RESEND;
						/* set error */
						EA_eDIPTFT_Display.error = 1;
					}
					/* data were valid -> wakeup callbacktask */
					else
					{
						/* switch back to idle mode */
						EA_eDIPTFT_Display.state = EA_EDIPTFT_IDLE;
						/* activate callback task */
						SET_TASK(EA_eDIPTFT_Display.callbackTask, ENABLE);
						/* clear error */
						EA_eDIPTFT_Display.error = 0;
					}
				}
				/* STX from data package */
				else if (((EA_eDIPTFT_Display.msgIndex - 2) % 4) == 0)
				{
					if (lcdData.Data != EA_EDIPTFT_ESC_BINARY)
					{
 						/* switch to resend procedure */
 						EA_eDIPTFT_Display.state = EA_EDIPTFT_RESEND;
					}
					else
					{
 						/* store data */
 						EA_eDIPTFT_Display.readBuf[EA_eDIPTFT_Display.msgIndex - 2] = lcdData.Data;
					}
					(EA_eDIPTFT_Display.msgIndex)++;
				}
				/* message identifier / data */
				else 
				{
					/* store data */
					EA_eDIPTFT_Display.readBuf[EA_eDIPTFT_Display.msgIndex - 2] = lcdData.Data;
					(EA_eDIPTFT_Display.msgIndex)++;
				}
				/* calculate checksum with the received data */
				EA_eDIPTFT_Display.checksum += lcdData.Data;
				
				/* read new data */
				lcdData = getChar_uart(EA_eDIPTFT_Display.usart);
			} 
			break;
		}
	}
	
	/*  */
	return(CYCLE);
}


/* ************************************************************** */
/*! \brief send message in ASCII format.
 *
 *  Send message. 
 *
 *  \param pointer to display.
 *  \param callback task number.
 *  \param pointer to message.
 *
 *  \retval sending the message was OK/NOK.
 *
 *  \version 07.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_SendMessageDC1(EA_eDIPTFT_t *disp, uint8_t callback, uint8_t* msg, uint8_t len)
{
	uint8_t cs = EA_EDIPTFT_DC1;
	
	/* check if the display is available */
	if (disp->state == EA_EDIPTFT_IDLE)
	{
		/* send DC1 */
		putChar_uart(disp->usart, EA_EDIPTFT_DC1);
		/* send message length */
		putChar_uart(disp->usart, len);
		cs += len;
		
		/* send message */
		for (uint8_t i = 0; i < len; i++)
		{
			putChar_uart(disp->usart, msg[i]);
			cs += msg[i];
		}
		
		/* send checksum */
		putChar_uart(disp->usart, cs);
		
		/* change state to wait-state (wait for answer from the display) */
		disp->state = EA_EDIPTFT_WAIT;
		
		/* set recall task */
		disp->callbackTask = callback;
		
		/* return OK */
		return (EA_EDIPTFT_MSG_SEND_OK);
	}
	
	/* display is not available -> return NOK */
	return (EA_EDIPTFT_MSG_SEND_NOK);
}



/* ************************************************************** */
/*! \brief send message in binary format.
 *
 *  Send message. 
 *
 *  \param pointer to display.
 *  \param callback task number.
 *  \param pointer to message.
 *
 *  \retval sending the message was OK/NOK.
 *
 *  \version 07.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_SendMessageDC1_binary(EA_eDIPTFT_t *disp, uint8_t callback, uint8_t* msg, uint8_t len)
{
	uint8_t cs = EA_EDIPTFT_DC1;
	
	/* check if the display is available */
	if (disp->state == EA_EDIPTFT_IDLE)
	{
		/* send DC1 */
		putChar_uart(disp->usart, EA_EDIPTFT_DC1);
		/* send message length */
		putChar_uart(disp->usart, len);
		cs += len;
		
		/* send message */
		for (uint8_t i = 0; i < len; i++)
		{
			putChar_uart(disp->usart, msg[i]);
			cs += msg[i];
		}
		
		/* send checksum */
		putChar_uart(disp->usart, cs);
		
		/* change state to wait-state (wait for answer from the display) */
		disp->state = EA_EDIPTFT_WAIT;
		
		/* set recall task */
		disp->callbackTask = callback;
		
		/* return OK */
		return (EA_EDIPTFT_MSG_SEND_OK);
	}
	
	/* display is not available -> return NOK */
	return (EA_EDIPTFT_MSG_SEND_NOK);
}


/* ************************************************************** */
/*! \brief get send buffer from display.
 *
 *  Get send buffer from display. 
 *
 *  \param pointer to display.
 *  \param callback task number.
 *
 *  \retval sending the message was OK/NOK.
 *
 *  \version 07.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_GetSendBuffer(EA_eDIPTFT_t *disp, uint8_t callback)
{
	/* check if the display is available */
	if (disp->state == EA_EDIPTFT_IDLE)
	{
		/* store message to data-array */
		/* STX */
		disp->sendBuf[0] = EA_EDIPTFT_DC2;
		/* data */
		disp->sendBuf[1] = 1;
		disp->sendBuf[2] = 'S';

		/* checksum */
		disp->sendBuf[3] = EA_eDIPTFT_calcChecksum(disp->sendBuf, 3);
	
		/* send data to serial interface */
		writePacket_usart(disp->usart,disp->sendBuf,4);
		
		/* change state to read-state (wait for answer from the display) */
		disp->state = EA_EDIPTFT_READ;
		
		/* set recall task */
		disp->callbackTask = callback;
		
		/* return OK */
		return (EA_EDIPTFT_MSG_SEND_OK);
	}
	
	/* display is not available -> return NOK */
	return (EA_EDIPTFT_MSG_SEND_NOK);
}


/* ************************************************************** */
/*! \brief get send buffer information.
 *
 *  Get information about the send buffer. 
 *
 *  \param pointer to display.
 *  \param callback task number.
 *
 *  \retval sending the message was OK/NOK.
 *
 *  \version 10.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_GetSendBufferInfo(EA_eDIPTFT_t *disp, uint8_t callback)
{
	/* check if the display is available */
	if (disp->state == EA_EDIPTFT_IDLE)
	{
		/* store message to data-array */
		/* STX */
		disp->sendBuf[0] = EA_EDIPTFT_DC2;
		/* data */
		disp->sendBuf[1] = 1;
		disp->sendBuf[2] = 'I';

		/* checksum */
		disp->sendBuf[3] = EA_eDIPTFT_calcChecksum(disp->sendBuf, 3);
	
		/* send data to serial interface */
		writePacket_usart(disp->usart,disp->sendBuf,4);
		
		/* change state to read-state (wait for answer from the display) */
		disp->state = EA_EDIPTFT_READ;
		
		/* set recall task */
		disp->callbackTask = callback;
		
		/* return OK */
		return (EA_EDIPTFT_MSG_SEND_OK);
	}
	
	/* display is not available -> return NOK */
	return (EA_EDIPTFT_MSG_SEND_NOK);
}


/* ************************************************************** */
/*! \brief Set protocol parameters.
 *
 *  set protocol parameters. 
 *
 *  \param pointer to display.
 *  \param callback task number.
 *  \param ps packet size for send buffer.
 *  \param to timeout for send buffer.
 *
 *  \retval sending the message was OK/NOK.
 *
 *  \version 07.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_SetProtocolParameter(EA_eDIPTFT_t *disp, uint8_t callback, uint8_t ps, uint8_t to)
{
	/* check if the display is available */
	if (disp->state == EA_EDIPTFT_IDLE)
	{
		/* store message to data-array */
		/* STX */
		disp->sendBuf[0] = EA_EDIPTFT_DC2;
		/* data */
		disp->sendBuf[1] = 3;
		disp->sendBuf[2] = 'D';
		disp->sendBuf[3] = ps;
		disp->sendBuf[4] = to;

		/* checksum */
		disp->sendBuf[5] = EA_eDIPTFT_calcChecksum(disp->sendBuf, 5);
	
		/* send data to serial interface */
		writePacket_usart(disp->usart,disp->sendBuf,6);
		
		/* change state to wait-state (wait for answer from the display) */
		disp->state = EA_EDIPTFT_WAIT;

		/* set recall task */
		disp->callbackTask = callback;
		
		/* return OK */
		return (EA_EDIPTFT_MSG_SEND_OK);
	}
	
	/* display is not available -> return NOK */
	return (EA_EDIPTFT_MSG_SEND_NOK);
}


/* ************************************************************** */
/*! \brief Get protocol parameters.
 *
 *  get protocol parameters. 
 *
 *  \param pointer to display.
 *  \param callback task number.
 *
 *  \retval sending the message was OK/NOK.
 *
 *  \version 07.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_GetProtocolParameter(EA_eDIPTFT_t *disp, uint8_t callback)
{
	/* check if the display is available */
	if (disp->state == EA_EDIPTFT_IDLE)
	{
		/* store message to data-array */
		/* STX */
		disp->sendBuf[0] = EA_EDIPTFT_DC2;
		/* data */
		disp->sendBuf[1] = 1;
		disp->sendBuf[2] = 'P';

		/* checksum */
		disp->sendBuf[3] = EA_eDIPTFT_calcChecksum(disp->sendBuf, 3);
	
		/* send data to serial interface */
		writePacket_usart(disp->usart,disp->sendBuf,4);
		
		/* change state to read-state (wait for answer from the display) */
		disp->state = EA_EDIPTFT_READ;

		/* set recall task */
		disp->callbackTask = callback;
		
		/* return OK */
		return (EA_EDIPTFT_MSG_SEND_OK);
	}
	
	/* display is not available -> return NOK */
	return (EA_EDIPTFT_MSG_SEND_NOK);
}


/* ************************************************************** */
/*! \brief Repeat last message.
 *
 *  repeat last message. 
 *
 *  \param pointer to display.
 *  \param callback task number.
 *
 *  \retval sending the message was OK/NOK.
 *
 *  \version 10.12.2018
 *
 */
/* ************************************************************** */
uint8_t EA_eDIPTFT_RepeatLastMsg(EA_eDIPTFT_t *disp, uint8_t callback)
{
	/* check if the display is available */
	if (disp->state == EA_EDIPTFT_IDLE)
	{
		/* store message to data-array */
		/* STX */
		disp->sendBuf[0] = EA_EDIPTFT_DC2;
		/* data */
		disp->sendBuf[1] = 1;
		disp->sendBuf[2] = 'R';

		/* checksum */
		disp->sendBuf[3] = EA_eDIPTFT_calcChecksum(disp->sendBuf, 3);
	
		/* send data to serial interface */
		writePacket_usart(disp->usart,disp->sendBuf,4);
		
		/* change state to read-state (wait for answer from the display) */
		disp->state = EA_EDIPTFT_READ;

		/* set recall task */
		disp->callbackTask = callback;
		
		/* return OK */
		return (EA_EDIPTFT_MSG_SEND_OK);
	}
	
	/* display is not available -> return NOK */
	return (EA_EDIPTFT_MSG_SEND_NOK);
}


