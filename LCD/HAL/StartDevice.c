/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Start device driver source file.
 *
 *      This file contains the function to deal with the start device
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
 *      Hardware details are shown in the schematic Miniboard V3.0, page 1, sector C4
 *
 * \par Documentation
 *      The file provide functions to read out the start device 
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


#define _STARTDEVICE_EXTERN


#include <avr/io.h> 




/* ************************************************************** */
/*! \brief Initialize the start device.
 *
 *  start device initialization function.
 *  Sets PORTC.5 as input and enables the pull up resistors for each pin,
 *  if it isn't done before
 *
 */
/* ************************************************************** */
void StartDevice_Init()
{
	/* PORTC initialization */
	/* Set PORTC.5 pin as input */
	PORTC.OUTCLR = 0x20;
	PORTC.DIRCLR = 0x20;
	/* enable pull up resistors */
	PORTC.PIN5CTRL = 0x18;

} 


/* ************************************************************** */
/*! \brief Returns the value of the start device over all.
 *
 *  Start device read out function.
 *  check PORTC.5 and give back the value
 *  range -> 0/1 
 *
 *  \retval true  If pin is logical 1.
 *  \retval false If pin is logical 0.
 *
*/
/* ************************************************************** */
uint8_t StartDevice_ReadOut()
{
	return((PORTC.IN & 0x20) ? 1 : 0);
} 

