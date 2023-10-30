/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      usart source file.
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

#define _USART_EXTERN

#include <avr/io.h>
#include "usart.h"
#include "define.h"
#include "global.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include "ports.h"




/* ########################################################################
   ########################################################################
   ######                        U S A R T F 0                       ######
   ########################################################################
   ######################################################################## */
/* ************************************************************** */
/*! \brief Initialize the USARTF0.
 *
 *  Function initialize the USARTF0 -> USB
 *
 *  \param size of receive buffer.
 *  \param size of transmit buffer.
 *
 *  \version 27.05.2013
 */
/* ************************************************************** */
uint8_t usartf0_init(uint16_t sizeRecBuf, uint16_t sizeTransBuf)
{
	/* allocate memory for receive buffer */
	usartF0.RecBuffer = recBufF0;
	/* allocate memory for transmit buffer */
	usartF0.TransBuffer = transBufF0;

	/* set buffer size */
	usartF0.RecBufSize = sizeRecBuf;
	usartF0.TransBufSize = sizeTransBuf;

   /* Note: the correct PORTC direction for the RxD, TxD and XCK signals
      is configured in the ports_init function */
   /* Transmitter is enabled
      Set TxD=1 */
   PORTF.OUTSET|=0x08;

   /* Communication mode: Asynchronous USART
      Data bits: 8
      Stop bits: 1
      Parity: Disabled */
   USARTF0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;

   /* Receive complete interrupt: Low Level
      Transmit complete interrupt: Low Level
      Data register empty interrupt: Disabled */
   USARTF0.CTRLA=(USARTF0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
      USART_RXCINTLVL_LO_gc | USART_TXCINTLVL_LO_gc | USART_DREINTLVL_OFF_gc;

   /* Required Baud rate: 115200
      BSEL = (F_osc / 16 / F_baud / 2^BSCALE) - 1 @ BSCALE >= 0
      BSEL = ((F_osc / 16 / F_baud) - 1) / 2^BSCALE @ BSCALE < 0
	   CTRLA: BSEL[7:0] */
	/* 115200 Baud */
   USARTF0.BAUDCTRLA=0x2E;
   /* CTRLB: BSCALE[3:0]:BSEL[11:8] */
   USARTF0.BAUDCTRLB=0x98;
	
	///* 1 MBaud */
	//USARTF0.BAUDCTRLA=0x80;
	//// CTRLB: BSCALE[3:0]:BSEL[11:8]
	//USARTF0.BAUDCTRLB=0x90;

	///* 9600 Baud */
	//USARTF0.BAUDCTRLA=0x0C;
	//// CTRLB: BSCALE[3:0]:BSEL[11:8]
	//USARTF0.BAUDCTRLB=0x40;


   /* Receiver: On
      Transmitter: On
      Double transmission speed mode: Off
      Multi-processor communication mode: Off */
   USARTF0.CTRLB=(USARTF0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
      USART_RXEN_bm | USART_TXEN_bm;   
      
   /* set pointer to USARTC0 register */
   usartF0.usart = &USARTF0;

	/* set interrupt level */
	usartF0.CTRLA_mask = USART_DREINTLVL_LO_gc;

	/* initialization was OK */
	return(USART_INIT_OK);
}




/* ************************************************************** */
/*! \brief Receive-ISR of the USARTF0.
 *
 *  Receive interrupt service routine of the USARTF0
 *
 *  \version 27.05.2013
 */
/* ************************************************************** */
ISR(USARTF0_RXC_vect)
{
	readData_usart(&usartF0);
}


/* ************************************************************** */
/*! \brief Receive-ISR of the USARTF0.
 *
 *  Receive interrupt service routine of the USARTF0
 *
 *  \version 27.05.2013
 */
/* ************************************************************** */
ISR(USARTF0_DRE_vect)
{
	writeData_usart(&usartF0);
}

/* ************************************************************** */
/*! \brief Transmit-ISR of the USARTF0.
 *
 *  Transmit interrupt service routine of the USARTF0
 *
 *  \version 27.05.2013
 */
/* ************************************************************** */
ISR(USARTF0_TXC_vect)
{
 // writeData_usart(&usartD0);
}


/* ************************************************************** */
/*! \brief Read a byte from the USART receive buffer.
 *
 *  Function used to read a byte from the USART
 *
 *  \param usart pointer to the dedicated USART.
 *
 *  \retval USART_REC_OK if byte was received.
 *  \retval USART_REC_NO_DATA if no byte was received.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
getChar_t getChar_uart(tsUsart *usart)
{
   getChar_t data;

   if(usart->RecIndexOut != usart->RecIndexIn)
   {
      data.Data = usart->RecBuffer[usart->RecIndexOut];
      (usart->RecIndexOut)++;
      if(usart->RecIndexOut >= usart->RecBufSize)
         usart->RecIndexOut = 0; 
      data.Status = USART_REC_OK;
   }
   else
   {
      data.Status = USART_REC_NO_DATA;
   }
   
   return(data); 
}



/* ************************************************************** */
/*! \brief Write a byte to the USART/Buffer.
 *
 *  Function used to write a byte to the USART/Buffer
 *
 *  \param usart pointer to the dedicated USART.
 *  \param data byte that should be sent.
 *
 *  \retval USART_BUFF_FULL if transmit buffer is full.
 *  \retval USART_TRANS_OK if byte was sent.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
uint8_t putChar_uart(tsUsart *usart, uint8_t data)
{
   int16_t diff;
   
   diff = (int16_t)(usart->TransIndexIn) - (int16_t)(usart->TransIndexOut);
   if(diff < 0)
      diff += (int16_t)(usart->TransBufSize);  
   
   if(diff >= (usart->TransBufSize - 1))
      return(USART_BUFF_FULL);
   
	usart->TransBuffer[usart->TransIndexIn] = data;
   (usart->TransIndexIn)++;
   if(usart->TransIndexIn >= usart->TransBufSize)
      usart->TransIndexIn = 0;   
    
   return(USART_TRANS_OK); 
}


/* ************************************************************** */
/*! \brief Read a byte from the USART.
 *
 *  Function used to read a byte from the USART
 *
 *  \param usart pointer to the dedicated USART.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void readData_usart(tsUsart *usart)
{
   uint8_t status;
   uint8_t data;
	uint8_t text[50];

   status = usart->usart->STATUS;
   data = usart->usart->DATA;

   if((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0)
   {
      usart->RecBuffer[usart->RecIndexIn] = data;
      (usart->RecIndexIn)++;
      if(usart->RecIndexIn >= usart->RecBufSize)
         usart->RecIndexIn = 0; 
   }
	else
	{
#ifdef _DEBUG_SERIAL_
sprintf(text,"SER-ERR: %d\r\n", status);
writeString_usart(&usartD0, text);
#endif
	}
}



/* ************************************************************** */
/*! \brief Write a byte to the USART.
 *
 *  Function used to write a byte to the USART
 *
 *  \param usart pointer to the dedicated USART.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void writeData_usart(tsUsart *usart)
{
   if(usart->TransIndexOut != usart->TransIndexIn)
   {
      usart->usart->DATA = usart->TransBuffer[usart->TransIndexOut];
      (usart->TransIndexOut)++;
      if(usart->TransIndexOut >= usart->TransBufSize)
         usart->TransIndexOut = 0;    
   }
   else
   {
      usart->usart->CTRLA &= ~0x03;
      usart->transIrActiv = 0;
   }

}


/* ************************************************************** */
/*! \brief Calculate the Free Space in USART/Buffer.
 *
 *  Function calculates the free space in transmit buffer
 *
 *  \param usart pointer to the dedicated USART.
 *
 *  \retval free space in transmit buffer.
 *
 *  \version 16.12.2016
 *
 */
/* ************************************************************** */
uint16_t getFreeSpace_TransBuffer_uart(tsUsart *usart)
{
   int16_t diff;
   
	/* calculate free space: Out - In */
   diff = (int16_t)(usart->TransIndexOut) - (int16_t)(usart->TransIndexIn);
   /* if diff < 0 -> add buffer size */
	if(diff <= 0)
      diff += (int16_t)(usart->TransBufSize);  
   
   return((uint16_t)diff); 
}

/* ************************************************************** */
/*! \brief Start data transmission.
 *
 *  Function starts the transmission of data
 *
 *  \param usart pointer to the dedicated USART.
 *
 *  \version 11.01.2017
 *
 */
/* ************************************************************** */
void startTransmision_uart(tsUsart *usart)
{
   if((usart->transIrActiv == 0) && (usart->TransIndexOut != usart->TransIndexIn))
	{
		writeData_usart(usart);
      usart->transIrActiv = 1;
      usart->usart->CTRLA |= usart->CTRLA_mask;	
	}

}

/* ************************************************************** */
/*! \brief Write a text to the USART/Buffer.
 *
 *  Function used to write a text to the USART/Buffer
 *
 *  \param usart pointer to the dedicated USART.
 *  \param text pointer to a null terminated text.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void writeString_usart(tsUsart *usart, char *text)
{
   uint16_t i = 0;

	/* write data to transmit buffer */
   while(text[i] != 0)
   {
      putChar_uart(usart, text[i++]);
   }
	/* start transmission */
	startTransmision_uart(usart);
}


/* ************************************************************** */
/*! \brief Write a packet to the USART/Buffer.
 *
 *  Function used to write a packet to the USART/Buffer
 *
 *  \param usart pointer to the dedicated USART.
 *  \param packet pointer to the data.
 *  \param data length.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void writePacket_usart(tsUsart *usart, uint8_t *packet, uint8_t n)
{
   uint16_t i = 0;
	
	/* write data to transmit buffer */
	while(n-- != 0)
	{
		putChar_uart(usart, packet[i++]);
	}
	/* start transmission */
	startTransmision_uart(usart);
}