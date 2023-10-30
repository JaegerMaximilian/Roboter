/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      watchdog source file.
 *
 *      This file contains the functions to setup and deal with the wacthdog.
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include "watchdog.h"

/* ************************************************************** */
/*! \brief Initialize the watchdog.
 *
 *  Function initialize the watchdog
 *
 *  \version 11.09.2012
 */
/* ************************************************************** */
void watchdog_init(void)     
{
	uint8_t s,n;

	/* Optimize for speed
	   Save interrupts enabled/disabled state */
	s=SREG;
	/* Disable interrupts */
	cli();

	/* Watchdog Timer: Off */
	n=(WDT.CTRL & (~WDT_ENABLE_bm)) | WDT_CEN_bm;

	CCP=CCP_IOREG_gc;

	/*

	WDT.CTRL Watchdog Timer Control Register (Manual Page 119)
			  7        6        5        4        3        2       1        0
		 -------------------------------------------------------------------------
		 |   R    |   R    |              PER[3:0]            | ENABLE |  CEN    |
		 ------------------------------------------------------------------------- 
   
		 PER[3:0] Watchdog Timeout Period
					PER[3:0]        Group Config.   Typical imeout periods
			  0   0   0   0           8CLK                8 ms
			  0   0   0   1           16CLK               16 ms
			  0   0   1   0           32CLK               32 ms
			  0   0   1   1           64CLK               64 ms
			  0   1   0   0           125CLK              125 ms
			  0   1   0   1           250CLK              250 ms
			  0   1   1   0           500CLK              500 ms
			  0   1   1   1           1KCLK               1 s
			  1   0   0   0           2KCLK               2 s
			  1   0   0   1           4KCLK               4 s
			  1   0   1   0           8KCLK               8 s
			  1   0   1   1           Reserved
			  1   1   0   0           Reserved
			  1   1   0   1           Reserved
			  1   1   1   0           Reserved
			  1   1   1   1           Reserved
        
		 ENABLE Watchdog Enable (Manual Page 120)
			  This bit enables the WDT. Change CEN at the same time!
		 CEN Watchdog Change Enable
			  This bit enables the possibility to change the configuration      
	*/
	WDT.CTRL=n;

	/* Watchdog window mode: Off */
	n=(WDT.WINCTRL & (~WDT_WEN_bm)) | WDT_WCEN_bm;

	CCP=CCP_IOREG_gc;

	/* WDT.WINCTRL Manual Page 120 
			  7        6        5        4        3        2       1        0
		 -------------------------------------------------------------------------
		 |   R    |   R    |             WPER[3:0]            |  WEN   |  WCEN   |
		 ------------------------------------------------------------------------- 
			  WPER[3:0] Watchdog Window Mode Timeout Period
					WPER[3:0]        Group Config.   Typical timeout periods
			  0   0   0   0           8CLK                8 ms
			  0   0   0   1           16CLK               16 ms
			  0   0   1   0           32CLK               32 ms
			  0   0   1   1           64CLK               64 ms
			  0   1   0   0           125CLK              125 ms
			  0   1   0   1           250CLK              250 ms
			  0   1   1   0           500CLK              500 ms
			  0   1   1   1           1KCLK               1 s
			  1   0   0   0           2KCLK               2 s
			  1   0   0   1           4KCLK               4 s
			  1   0   1   0           8KCLK               8 s
			  1   0   1   1           Reserved
			  1   1   0   0           Reserved
			  1   1   0   1           Reserved
			  1   1   1   0           Reserved
			  1   1   1   1           Reserved
        
		 WEN Watchdog Window Mode Enable (Manual Page 121)
			  This bit enables the Watchdog Window Mode. Change WCEN at the same time!
		 WCEN Watchdog Window Mode Change Enable
			  This bit enables the possibility to change the WINCTRL Control Register configuration

	*/
	WDT.WINCTRL=n;

	/* Restore interrupts enabled/disabled state */
	SREG=s;

}