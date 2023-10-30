/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      system clock source file.
 *
 *      This file contains the functions to setup the system clock.
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


/* ************************************************************** */
/*! \brief Setup the system clock.
 *
 *  Function make a total setup of the system clock
 *
 *  \version 11.09.2012
 */
/* ************************************************************** */
/*
    XOSCCTRL - XOSC Control Register (Manual page 88/89)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |  FRQRANGE[1:0]  |X32KLPM |   R    |            XOSCSEL[3:0]           |
    -------------------------------------------------------------------------
    RFRQRANGE[1:0]  Crystal Oscillator Frequency Range Select
    RFRQRANGE   Group config.   Frequency Range     Recommended Range for Capacitors C1 and C2
    0   0       04TO2           0,4-2 MHz           100pF
    0   1       2TO9            2-9 MHz             15pF
    1   0       9TO12           9-12 MHz            15pF
    1   1       12TO16          12-16MHz            10pF
    
    X32KLPM     Crystal Oszillator 32kHz Low Power Mode
    etting this bit enables low power mode for the 32.768 kHz Crystal Oscillator. 
    This will reduce the swing on the TOSC2 pin.
    
    XOSCSEL[3:0]: Crystal Oscillator Selection
    These bits select the type and start-up time for the crystal or resonator 
    that is connected to the XTAL or TOSC pins. See Table 7-6 for crystal selections. 
    If external clock or external oscillator is selected as source for System Clock, 
    see ”CTRL - Oscillator Control Register” on page 87, this configuration can not be changed.
    XOSCSEL     Group config.       Selected Clock Source   Start-Up Time
    0 0 0 0     EXTCLK              External Clock          6CLK
    0 0 1 0     32kHz               32,768kHz TOSC          16k CLK
    0 0 1 1     XTAL_256CLK         0,4-16MHz XTAL          256 CLK
    0 1 1 1     XTAL_1KCLK          0,4-16MHz XTAL          1k CLK
    1 0 1 1     XTAL_16KCLK         0,4-16MHz XTAL          16k CLK
  
    
    CTRL - Oscilator Control Register (Manual page 87)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |   R    |   R    |   R    | PLLEN  | XOSCEN | RC32KEN|RC32MEN| RC2MEN  |
    -------------------------------------------------------------------------
    PLLEN   PLL Enable
    Setting this bit enables the PLL.
    
    XOSCEN  External Oscillator Enable
    Setting this bit enables the selected external clock source, refer to 
    ”XOSCCTRL - XOSC Control Register” on page 88 for details on how to 
    select and enable an external clock source.
    
    RC32KEN 32kHz Internal RC Oscillator Enable
    
    RC32MEN 32MHz Internal RC Oscillator Enable
    
    RC2MEN  2MHz Internal RC Oscillator Enable    


	 PLLCTRL PLL Control Register
	
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |  PLLSRC[1:0]    |   R    |                   PLLFAC[4:0]              |
    -------------------------------------------------------------------------
    PLLSRC[1:0] Clock Source
    PLLSRC      Group config    PLL input source
    0   0       RC2M            2MHz Internal RC Oscillator
    0   1       -               Reserved
    1   0       RC32M           32MHz Internal RC Oscillator
    1   1       XOSC            External Clock Source
    
    PLLFAC[4:0] Multiplication Factor
    The PLLFAC bits set the multiplication factor for the PLL. The multiplication 
    factor can be in the range from 1x to 31x. The output frequency from the PLL 
    should not exceed 200 MHz. The PLL  must have a minimum output frequency of 10 MHz.
*/
void system_clocks_init(void)
{


	unsigned char n,s;

	/* Optimize for speed
		Save interrupts enabled/disabled state */
	s=SREG;
	/* Disable interrupts */
	cli();

	/* Internal 32 kHz RC oscillator initialization
		Enable the internal 32 kHz RC oscillator */
	OSC.CTRL|=OSC_RC32KEN_bm;
	/* Wait for the internal 32 kHz RC oscillator to stabilize */
	while ((OSC.STATUS & OSC_RC32KRDY_bm)==0);

	/* Internal 32 MHz RC oscillator initialization
		Enable the internal 32 MHz RC oscillator */
	OSC.CTRL|=OSC_RC32MEN_bm;

	/* System Clock prescaler A division factor: 1
		System Clock prescalers B & C division factors: B:1, C:1
		ClkPer4: 32000,000 kHz
		ClkPer2: 32000,000 kHz
		ClkPer:  32000,000 kHz
		ClkCPU:  32000,000 kHz */
	n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
		CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=n;

	/* Internal 32 MHz RC osc. calibration reference clock source: 32.768 kHz Internal Osc. */
	OSC.DFLLCTRL&= ~(OSC_RC32MCREF_gm | OSC_RC2MCREF_bm);
	/* Enable the autocalibration of the internal 32 MHz RC oscillator */
	DFLLRC32M.CTRL|=DFLL_ENABLE_bm;

	/* Wait for the internal 32 MHz RC oscillator to stabilize */
	while ((OSC.STATUS & OSC_RC32MRDY_bm)==0);

	/* Select the system clock source: 32 MHz Internal RC Osc. */
	n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC32M_gc;
	CCP=CCP_IOREG_gc;
	CLK.CTRL=n;

	/* Disable the unused oscillators: 2 MHz, external clock/crystal oscillator, PLL */
	OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);

	/* Peripheral Clock output: Disabled */
	PORTCFG.CLKEVOUT=(PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;

	/* Restore interrupts enabled/disabled state */
	SREG=s;
	/* Restore optimization for size if needed */


}
