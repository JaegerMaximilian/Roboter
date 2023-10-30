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
#define LED_PORT		PORTA
#define LED1			0
#define LED2			1
#define LED3			2

/* ******** */
/* PUMP, MV */
/* ******** */
/* vacuum pump left */
#define SET_VACUUM_PUMP_LEFT	{SET_PIN(PORTF, 6);}
#define CLR_VACUUM_PUMP_LEFT	{CLR_PIN(PORTF, 6);}
/* vacuum pump right */
#define SET_VACUUM_PUMP_RIGHT	{SET_PIN(PORTF, 7);}
#define CLR_VACUUM_PUMP_RIGHT	{CLR_PIN(PORTF, 7);}
/* magnet ventil left */
#define SET_MV_LEFT				{SET_PIN(PORTF, 4);}
#define CLR_MV_LEFT				{CLR_PIN(PORTF, 4);}
/* magnet ventil right */
#define SET_MV_RIGHT			{SET_PIN(PORTF, 5);}
#define CLR_MV_RIGHT			{CLR_PIN(PORTF, 5);}
	
	
/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void ports_init();


#endif
