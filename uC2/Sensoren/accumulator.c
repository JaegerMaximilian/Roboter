/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Measurement of the accumulator driver source file.
 *
 *      This file contains the function to deal with the measurement of the accumulator
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
 * \par Schematic Miniboard V3.0:
 *      Hardware details are shown in the schematic Miniboard V3.0, page 1, sector B5
 *
 * \par Documentation
 *      The file provide functions to measure the accumulator
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


#define _ACCUMULATOR_EXTERN


#include <avr/io.h> 
#include "adc_driver.h"




/* ************************************************************** */
/*! \brief Initialize the accumulator measurement.
 *
 *  Accumulator measurement initialization function.
 *  Sets PORTB.0 as input and initialize the ADC 
 *
 *  \version 28.08.2012
 *
 */
/* ************************************************************** */
void Accumulator_Init()
{
	/* PORTC initialization */
	/* Set PORTC.7 pin as input */
	PORTA.DIRCLR = 0x04;
	/* enable pull up resistors */
	PORTA.PIN2CTRL = 0x00;
	/* initialize the ADC */
	adca_init();
} 




/* ************************************************************** */
/*! \brief returns the accumulator voltage.
 *
 *  The function returns the accumulator voltage in volts (float)
 *
 *  \retval voltage in volts (float).
 *
 */
/* ************************************************************** */
float getAccumulatorVoltage()
{
	return((float)(adca_read(ADC_CH_0,3)) * 0.0080808-1.090909 );
} 


