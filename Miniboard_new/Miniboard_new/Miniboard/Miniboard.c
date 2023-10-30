/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Main source file.
 *
 *      This file contains the main function and the global initialization
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
 *      The file provide functions to initialize the µC and the main function 
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

#define _GLOBAL_EXTERN

#include "global.h"
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "ports.h"
#include "systemclock.h"
#include "timer.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "multitask.h"
#include "adc_driver.h"
#include <avr/eeprom.h>
#include "debug.h"
#include "usart.h"
#include "DIPswitch.h"
#include "accumulator.h"
#include "LEDcontrol.h"
#include "motor.h"
#include "start.h"
#include "QTR_1RC.h"




/* ************************************************************** */
/*! \brief Initialize µC.
 *
 *  Function initialize the µC 
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void initDevice ()
{
	uint8_t n;
   
	/* disable all interrupts */
	cli();
   
	/* Low level interrupt: On
		Round-robin scheduling for low level interrupt: Off
		Medium level interrupt: On
		High level interrupt: On
		The interrupt vectors will be placed at the start of the Application FLASH section */
	n=(PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
		PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	CCP=CCP_IOREG_gc;
	PMIC.CTRL=n;
	/* Set the default priority for round-robin scheduling */
	PMIC.INTPRI=0x00;

	/* ******************************** */
	/* HAL - Hardware Abstraction Layer */
	/* ******************************** */
	/* initialize the system clock */
	system_clocks_init();
	/* initialize the ports */
	ports_init();
   /* initialize the usart_c1 -> debug interface via USB */
	usartc1_init();
	/* initialize the DIP-switch */
	DIPswitch_Init();
	/* initialize the LED */ 
	LEDcontrol_Init();
	/* initialize the accumulator measurement */
	Accumulator_Init();

	/* ******************* */
	/* MULTITASKING SYSTEM */
	/* ******************* */
	/* initialize the multitasking system */
	InitMultitasking();
	/* ********* */
	/* set tasks */
	/* ********* */
	/* initialize debug task */
	InitDebug();
	/* ********** */
	/* USER-Tasks */
	/* ********** */
	/* initialize waiting for start */
	InitStart();
	

	

   /* enable all interrupts */
	sei();

}


/* ************************************************************** */
/*! \brief main routine.
 *
 *  call the µC initialize function and run the multitasking system
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
int main(void)
{
   
   /* initialize the µC */
	initDevice();
	
   
   /* MAIN-LOOP */
   while(1)
   {
		/* run the multitasking system */
		MultitaskingSystem();  
   }
}