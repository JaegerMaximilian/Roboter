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

/* intern/extern switch */
#ifndef _USART_EXTERN
   #define _USART_EXTERN extern
#endif


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
#define USART_BUF_SIZE        100
#define USART_REC_OK          0
#define USART_REC_NO_DATA     1
#define USART_TRANS_OK        0
#define USART_BUFF_FULL       1

/* struct of the USART */
struct Usart
{
   unsigned char RecBuffer[USART_BUF_SIZE];
   unsigned char RecIndexIn;
   unsigned char RecIndexOut;

   unsigned char TransBuffer[USART_BUF_SIZE];
   unsigned char TransIndexIn;
   unsigned char TransIndexOut;
   
   unsigned char transIrActiv;  
   USART_t *usart;
};

typedef struct Usart tsUsart; 

/* struct for getChar_uart */
struct getData
{
   unsigned char Data;
   unsigned char Status;
};

typedef struct getData getChar_t; 


_USART_EXTERN tsUsart usartC0;
_USART_EXTERN tsUsart usartC1;




/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void usartc1_init(void);
void usartc0_init(void);
getChar_t getChar_uart(tsUsart *usart);
uint8_t putChar_uart(tsUsart *usart, uint8_t data);
void readData_usart(tsUsart *usart);
void writeData_usart(tsUsart *usart);
void writeString_usart(tsUsart *usart, char *text);
void writePacket_usart(tsUsart *usart, char *packet, uint8_t n);

#endif
