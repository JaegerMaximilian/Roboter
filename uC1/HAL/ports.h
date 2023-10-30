/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      ports header file.
 *
 *      This file contains the initialization of the IO ports.
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
 *      The file provide functions for the IO ports. 
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
#ifndef _PORTS_H_
#define _PORTS_H_

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* *** */
/* LED */
/* *** */
#define LED_PORT1		PORTF
#define LED1			4
#define LED2			5
#define LED3			6

#define LED_PORT2		PORTC
#define LED4			0

/* *** */
/* IR-Sensoren */
/* *** */
#define IR_VR1		((PORTA.IN & 0x01) ? 1 : 0)
#define IR_VR2		((PORTA.IN & 0x80) ? 1 : 0)
#define IR_VR3		((PORTA.IN & 0x40) ? 1 : 0)
#define IR_VL1		((PORTA.IN & 0x10) ? 1 : 0)
#define IR_VL2		((PORTA.IN & 0x02) ? 1 : 0)
#define IR_VL3		((PORTA.IN & 0x08) ? 1 : 0)
#define IR_K1		((PORTA.IN & 0x20) ? 1 : 0)
#define IR_K2		((PORTA.IN & 0x04) ? 1 : 0)

/* ************* */
/*  ENDSCHALTER  */
/* ************* */
#define END_VL		((PORTF.IN & 0x01) ? 0 : 1)
#define END_VR		((PORTF.IN & 0x02) ? 0 : 1)

/* ************* */	
/* STARTING CORD */
/* ************* */	
#define START_SCHNUR		((PORTB.IN & 0x10) ? 1 : 0)




	
/* ******************** */
/* digital outputs µC 2 */
/* ******************** */
/* LED */
#define DO_LED1_UC2_NBR		1
#define DO_LED2_UC2_NBR		2
#define DO_LED3_UC2_NBR		3


/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void ports_init();


#endif
