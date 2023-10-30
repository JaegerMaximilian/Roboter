/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      EA_eDIPTFT header file.
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

#ifndef EA_EDIPTFT_H_
#define EA_EDIPTFT_H_



/* intern/extern switch */
#ifndef _EA_EDIPTFT_EXTERN
	#define _EA_EDIPTFT_EXTERN extern
#endif

#include "usart.h"

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* states */
/* idle state -> ready to send data */
#define EA_EDIPTFT_IDLE			0
/* wait state -> waiting for answer from display */
#define EA_EDIPTFT_WAIT			1
/* resend state -> send message again */
#define EA_EDIPTFT_RESEND		2
/* read state -> read message from display (ACK/NAK) */
#define EA_EDIPTFT_READ			3
/* read state -> read message from display (message) */
#define EA_EDIPTFT_READ_MSG		4

/* display busy */
#define EA_EDIPTFT_BUSY			1

/* maximum amount of resents */
#define EA_EDIPTFT_MAX_RESEND	3

/* resend timeout */
#define EA_EDIPTFT_TIMEOUT		10

/* acknowledge */
/* ACK */
#define EA_EDIPTFT_ACK			0x06
/* NAK */
#define EA_EDIPTFT_NAK			0x15

/* STX */
/* STX for LCD commands */
#define EA_EDIPTFT_DC1			0x11
/* STX for LCD request or system commands */
#define EA_EDIPTFT_DC2			0x12

/* ESC */
/* ESC in binary modus */
#define EA_EDIPTFT_ESC_BINARY	0x1B
/* ESC in ASCII modus */
#define EA_EDIPTFT_ESC_ASCII	0x23

/* it is possible to send a message */
#define EA_EDIPTFT_MSG_SEND_OK		1
/* it is not possible to send a message */
#define EA_EDIPTFT_MSG_SEND_NOK		0

/* control characters */
/* form feed FF -> clear display and set cursor to 1/1 */
#define EA_EDIPTFT_FF			0x0C
/* carriage return CR -> set cursor to n/1 */
#define EA_EDIPTFT_CR			0x0D
/* line feed LF -> set cursor to n+1/m */
#define EA_EDIPTFT_LF			0x0A

/* colour table */
#define EA_EDIPTFT_TRANSPARENT	0
#define EA_EDIPTFT_BLACK		1
#define EA_EDIPTFT_DARKBLUE		2
#define EA_EDIPTFT_RED			3
#define EA_EDIPTFT_LIGTHGREEN	4
#define EA_EDIPTFT_PINK			5
#define EA_EDIPTFT_LIGHTBLUE	6
#define EA_EDIPTFT_YELLOW		7
#define EA_EDIPTFT_WHITE		8
#define EA_EDIPTFT_DARKGREY		9
#define EA_EDIPTFT_ORANGE		10
#define EA_EDIPTFT_VIOLET		11
#define EA_EDIPTFT_MAGENTA		12
#define EA_EDIPTFT_TURQUOSIE	13
#define EA_EDIPTFT_LIGHTGREEN	14
#define EA_EDIPTFT_BLUE			15
#define EA_EDIPTFT_LIGHTGREY	16

/* font size */
#define EA_EDIPTFT_FONT_8X8		1
#define EA_EDIPTFT_FONT_8X16	2

/*!< task structure */
typedef struct 
{
	/*!< size of the display (in pixel) */
	uint16_t size_x;
	uint16_t size_y;
	/*!< state of the communication */
	uint8_t state;
	/*!< status of the display -> idle/busy */
	uint8_t status;
	/*!< counter to generate a timeout */
	uint16_t timeout;
	/*!< counter for resend */
	uint8_t resendCounter;
	/*!< send-buffer */
	uint8_t sendBuf[258];
	/*!< pointer to the USART */
	tsUsart *usart;
	/*!< pointer to read buffer -> here is the data from the display stored */
	uint8_t *readBuf;
	/*!< number of received items */
	uint8_t items;
	/*!< message index */
	uint8_t msgIndex;
	/*!< checksum of received message */
	uint8_t checksum;
	/*!< length of received message */
	uint8_t length;
	/*!< recall task number -> this task will be called when the data was received */
	uint8_t callbackTask;
	/*!< transmission status -> ok, error */
	uint8_t error;
} EA_eDIPTFT_t;

_EA_EDIPTFT_EXTERN EA_eDIPTFT_t EA_eDIPTFT_Display; 
_EA_EDIPTFT_EXTERN uint8_t lastTouchEvent;



/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void EA_eDIPTFT_Init();
uint8_t EA_eDIPTFT_SBuf_Task();
uint8_t EA_eDIPTFT_Task();
uint8_t EA_eDIPTFT_SendMessageDC1(EA_eDIPTFT_t *disp, uint8_t callback, uint8_t* msg, uint8_t len);
uint8_t EA_eDIPTFT_SendMessageDC1_binary(EA_eDIPTFT_t *disp, uint8_t callback, uint8_t* msg, uint8_t len);
uint8_t EA_eDIPTFT_GetSendBuffer(EA_eDIPTFT_t *disp, uint8_t callback);
uint8_t EA_eDIPTFT_GetSendBufferInfo(EA_eDIPTFT_t *disp, uint8_t callback);
uint8_t EA_eDIPTFT_SetProtocolParameter(EA_eDIPTFT_t *disp, uint8_t callback, uint8_t ps, uint8_t to);
uint8_t EA_eDIPTFT_GetProtocolParameter(EA_eDIPTFT_t *disp, uint8_t callback);
uint8_t EA_eDIPTFT_RepeatLastMsg(EA_eDIPTFT_t *disp, uint8_t callback);
uint8_t EA_eDIPTFT_CheckSBUF();


#endif /* EA_EDIPTFT_H_ */