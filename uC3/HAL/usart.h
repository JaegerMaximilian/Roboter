/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      usart header file.
 *
 *      This file contains the functions to setup and deal with the usart.
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
#ifndef _USART_H_
#define _USART_H_


#include <avr/io.h>

/* intern/extern switch */
#ifndef _USART_EXTERN
   #define _USART_EXTERN extern
#endif


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
//#define USART_BUF_SIZE        500
#define USART_REC_OK          0
#define USART_REC_NO_DATA     1
#define USART_TRANS_OK        0
#define USART_BUFF_FULL       1

#define USART_INIT_OK			0
#define USART_INIT_ERROR		1

/* struct of the USART */
struct Usart
{
   uint8_t *RecBuffer;
   uint16_t RecIndexIn;
   uint16_t RecIndexOut;
	uint16_t RecBufSize;

   uint8_t *TransBuffer;
   uint16_t TransIndexIn;
   uint16_t TransIndexOut;
	uint16_t TransBufSize;
   
   uint8_t transIrActiv;  
   USART_t *usart;
	
	uint8_t CTRLA_mask;
};

typedef struct Usart tsUsart; 

/* struct for getChar_uart */
struct getData
{
   unsigned char Data;
   unsigned char Status;
};

typedef struct getData getChar_t; 

/* buffer size */
#define USART_D0_BUF_SIZE        500
#define USART_F0_BUF_SIZE        400

/* usart buffer D0, E1, F0 */
_USART_EXTERN uint8_t recBufD0[USART_F0_BUF_SIZE], transBufD0[USART_F0_BUF_SIZE];
_USART_EXTERN uint8_t recBufF0[USART_F0_BUF_SIZE], transBufF0[USART_F0_BUF_SIZE];

/* usart D0, E0 */
_USART_EXTERN tsUsart usartF0, usartD0;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
uint8_t usartd0_init(uint16_t sizeRecBuf, uint16_t sizeTransBuf);
uint8_t usarte0_init(uint16_t sizeRecBuf, uint16_t sizeTransBuf);
uint8_t usartc0_init(uint16_t sizeRecBuf, uint16_t sizeTransBuf);
getChar_t getChar_uart(tsUsart *usart);
uint8_t putChar_uart(tsUsart *usart, uint8_t data);
void readData_usart(tsUsart *usart);
void writeData_usart(tsUsart *usart);
uint16_t getFreeSpace_TransBuffer_uart(tsUsart *usart);
void startTransmision_uart(tsUsart *usart);
void writeString_usart(tsUsart *usart, char *text);
void writePacket_usart(tsUsart *usart, uint8_t *packet, uint8_t n);

#endif
