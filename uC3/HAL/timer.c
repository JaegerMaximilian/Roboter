/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      timer source file.
 *
 *      This file contains the functions to setup the timers.
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
#include "timer.h"
#include "global.h"
#include <avr/interrupt.h>


/* register overview */
/*
TC__.CTRLA  Control Register A (manual page 165)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |   R    |   R    |   R    |   R    |           CLKSEL[3:0]             |
    ------------------------------------------------------------------------- 
    CLKSEL[3:0] Clock Select
        These bit select the clock source for Timer/Counter
        CLKSEL[3:0]     Group config    Description
        0   0   0   0       OFF         None
        0   0   0   1       DIV1        Prescaler: clk/1
        0   0   1   0       DIV2        Prescaler: clk/2
        0   0   1   1       DIV4        Prescaler: clk/4
        0   1   0   0       DIV8        Prescaler: clk/8
        0   1   0   1       DIV64       Prescaler: clk/64
        0   1   1   0       DIV256      Prescaler: clk/256
        0   1   1   1       DIV1024     Prescaler: clk/1024
        1   x   x   x       EVCHn       Event channel n, n=[0,...,7]

TC__.CTRLB  Control Register B (Manual Seite 165)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    | CCDEN  | CCCEN  | CCBEN  | CCAEN  |   R    |       WGMODE[2:0]        |
    -------------------------------------------------------------------------
    CCxEN: Compare or Capture Enable
        Setting these bits in FRQ or PWM wafeform generation mode of operation will
        override of the port output register for the corresponding OCn output pin.
        When input capture operation is selected the CCxEN bits enabled the capture
        operation for the corresponding CC Channel.
    
    WGMODE[2:0] Wafeform Generation Mode
        These bits select the Wafeform Generation Mode.
        WGMODE[2:0]     Group config.   Mode of Operation   Top     Update      OVFIF/Event
        0   0   0       Normal          Normal              PER     TOP         TOP
        0   0   1       FRQ             FRQ                 CCA     TOP         TOP
        0   1   0                       Reserved
        0   1   1       SS              Single Slope PWM    PER     BOTTOM      BOTTOM
        1   0   0                       Reserved
        1   0   1       DS_T            Dual Slope PWM      PER     BOTTOM      TOP
        1   1   0       DS_TB           Dual Slope PWM      PER     BOTTOM      TOP and BOTTOM
        1   1   1       DS_B            Dual Slope PWM      PER     BOTTOM      BOTTOM
        
    CNTRLC  Control Register C
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |   R    |   R    |   R    |   R    | CMPD   | CMPC  |  CMPB |  CMPA    |
    -------------------------------------------------------------------------
    
    CMPx Compare Output Value n
    These Bits allow direct access to the Wafeform Generator's output compare value when 
    the Timer/Counter is set in "OFF" state.
    
TC__.CNTRLD  Control Register D (Manual page 167)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |        EVACT[2:0]        | EVDLY  |            EVSEL[3:0]             |
    ------------------------------------------------------------------------- 
    EVACT[2:0] Event Action
    These Bits define the Event Action the Timer will perform on an event.
    EVACT[2:0]      Group config.   Event Action
    0   0   0       OFF             None
    0   0   1       CAPT            Input Capture
    0   1   0       UPDOWN          Externally Controlled UP/Down Count
    0   1   1       QDEC            Quadrature Decoder
    1   0   0       RESTART         Restart Wafeform period
    1   0   1       FRQ             Frequenzy Capture
    1   1   0       PW              Pulse With capture
    1   1   1                       Reserved
    
    EVDLY   Time Delay Event
    When this bit is set, the selected event source is delayed by one peripheral 
    clock cycle.
    
    EVSEL[3:0] Timer Event Source Select (Manual page 167)
    These Bits select the event Channel source for the Timer/Counter
    EVSEL[3:0]      Group config        Event Source
    0   0   0   0   OFF                 None
    0   0   0   1                       Reserved 
    0   0   1   0                       Reserved
    0   0   1   1                       Reserved
    0   1   0   0                       Reserved
    0   1   0   1                       Reserved
    0   1   1   0                       Reserved
    0   1   1   1                       Reserved
    1   x   x   x   CHn                 Event channel n, n={0,...,7}
    
TC__.CTRLE Control Register E (Manual page 168)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |   R    |   R    |   R    |   R    |   R   |    R   |   R    |  BYTEM  |
    -------------------------------------------------------------------------
    BYTEM Byte Mode
    Enabling the byte mode, sets the Timer in 8-bit mode.
    
TC__.INTCTRLA Interrupt Enable Register A (Manual page 168/169)
          7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |     R    |        |   R    |   R    | ERRINTLVL[1:0] |OVFINTLVL[1:0]|
    ------------------------------------------------------------------------- 
    ERRINTLVL[1:0] Timer Error Interrupt Level 
    These bits enable the Timer Error Interrupt and select the interrupt level
    as described on manual page 123.
        
    OVFINTLVL[1:0] Timer Overflow/Underflow Interrupt level
    Describtion in "Interrupts and Programmable Multi-Level interrupt Conroller"
    on manual page 123.

TC__.INTCTRLB Interrupt Enable Register B (Manual page 169) 
          7        6        5        4        3        2       1        0
    --------------------------------------------------------------------------
    |   CCDINTLVL[1:0]  |  CCCINTLVL[1:0] | CCBINTLVL[1:0]  | CCAINTLVL[1:0] |
    --------------------------------------------------------------------------
    CC_INTLVL[1:0] Compare or Capture x Interrupt Level
    These bits enable the Timer Compare or Capture Interrupt and select the interrupt 
    level as described in ”Interrupts and Programmable Multi-level Interrupt Controller” 
    on page 123.
    
TC__.CTRLFCLR/CTRLFSET Control Register F Clear/Set (Manual page 169/170)

    CTRLFCRL
          7        6        5        4        3        2       1        0
    --------------------------------------------------------------------------
    |     R    |   R    |   R    |   R    |     CMD[1:0]   |  LUPD  |  DIR   |
    --------------------------------------------------------------------------
    
    CTRLFSET
          7        6        5        4        3        2       1        0
    --------------------------------------------------------------------------
    |     R    |   R    |   R    |   R    |     CMD[1:0]   |  LUPD  |  DIR   |
    --------------------------------------------------------------------------    
    This register is seperated into a set and a clear register. Bits can only be set
    in the Set-Register and cleared in the Clear-Register.
    
    CMD[1:0] Timer/Counter Command
        CMD     Group config.           Command Action
        0   0   NONE                    None
        0   1   UPDATE                  Force Update
        1   0   RESTART                 Force Restart
        1   1   RESET                   Force Reset
    
    LUPD  Lock Update
    When this bit is set no update of the buffered registers is performed, even though
    an UPDATE condition is occoured.
    
    DIR  Counter Direction
    If this bit is zero the counter is counting up (incrementing). When the bit is set
    to one, the counter is decrementing.
    
TC__.CTRLGCLR/CTRLGSET Control Register G Clear/Set (Manual page 170)
          7        6        5        4        3        2       1        0
    --------------------------------------------------------------------------
    |     R    |   R    |   R    | CCDBV  | CCCBV  | CCBBV | CCABV | PERBV   |
    --------------------------------------------------------------------------
    This register is seperated into a set and a clear register. Bits can only be set
    in the Set-Register and cleared in the Clear-Register.
    
    CCxBV   Compare or Capture x Buffer Valid
    These Bits are set if a new value is written to the corresponding CCxBUF register and
    automatically cleared on an UPDATE condition.
    
    PERBV Period Buffer Valid
    This bit is set when a new value is written to the PERBUF register. This bit is 
    automatically cleared on an UPDATE condition.
    
TC__.INTFLAFGS Interrupt Flag Register(Manual page 171)
          7        6        5        4        3        2       1        0
    --------------------------------------------------------------------------
    |  CCDIF   | DCCCIF | CCBIF  | CCAIF  |   R    |   R   | ERRIF | OVFIF   |
    --------------------------------------------------------------------------
    CCxIF Compare or Capture Channel x Interrupt Flag
    The Compare or Capture Interrupt Flag (CCxIF) is set on a compare match or 
    on an input capture event on the corresponding CC channel.
    
    ERRIF: Error Interrupt Flag
    The ERRIF is set on multiple occasions depending on mode of operation. 
    
    OVFIF: Overflow/Underflow Interrupt Flag
    The OVFIF is set either on a TOP (overflow) or BOTTOM (underflow) condition depending on
    the WGMODE setting. The OVFIF is automatically cleared when the corresponding interrupt
    vector is executed. The flag can also be cleared by writing a one to its bit location.
    
TC__.CNT - Counter Register (Manual page 172)
    The Counter Register is split into CNTH and CNTL.The CNTH and CNTL register pair represents 
    the 16-bit value CNT. CNT contains the 16-bit counter value in the Timer/Counter. The CPU 
    and DMA write access has priority over count, clear, or reload of the counter.
    
TC__.PER - Period Register (Manual page 172/173) 
    The PERH and PERL register pair represents the 16-bit value PER. PER contains the 16-bit
    TOP value in the Timer/Counter.
    
TC__.CCx - Compare or Capture Register n 
    The CCxH and CCxL register pair represents the 16-bit value CCx.    
    

*/          

/* ************************************************************** */
/*! \brief Disable the timer.
 *
 *  Function disables a timer of type TC0
 *
 *  \version 11.09.2012
 */
/* ************************************************************** */
void tc0_disable(TC0_t *ptc)
{
	/* Timer/Counter off */
	ptc->CTRLA=(ptc->CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_OFF_gc; 
	/* Issue a reset command */ 
	ptc->CTRLFSET=TC_CMD_RESET_gc;                                  
}


/* ************************************************************** */
/*! \brief Disable the timer.
 *
 *  Function disables a timer of type TC1
 *
 *  \version 11.09.2012
 */
/* ************************************************************** */
void tc1_disable(TC1_t *ptc)
{                                        
	/* Timer/Counter off */
	ptc->CTRLA=(ptc->CTRLA & (~TC1_CLKSEL_gm)) | TC_CLKSEL_OFF_gc; 
	/* Issue a reset command */ 
	ptc->CTRLFSET=TC_CMD_RESET_gc;                                 
}


/* ########################################################################
   ########################################################################
   ######                          T C F 0                           ######
   ########################################################################
   ######################################################################## */
/* ************************************************************** */
/*! \brief Setup TCF0.
 *
 *  Function makes a total setup of the TCF0 (1ms-tick for multitasking system)
 *
 *  \version 11.09.2012
 */
/* ************************************************************** */
void tcf0_init()
{
   unsigned char s;

   /* Save interrupts enabled/disabled state */
   s=SREG;
   /* Disable interrupts */
   cli(); 

   /* Disable and reset the timer/counter just to be sure */
   tc0_disable(&TCF0);
   /* Clock source: Peripheral Clock/1 */
   TCF0.CTRLA = 0x01;
   /* Mode: Normal Operation, Overflow Int./Event on TOP 
      Compare/Capture on channel A: Off
      Compare/Capture on channel B: Off */
   TCF0.CTRLB = 0x00;
   /* Capture event source: None
      Capture event action: None */
   TCF0.CTRLC = 0x00;
   TCF0.CTRLD = 0x00;
   TCF0.CTRLE = 0x00;

   /* Overflow interrupt: Low Level
      Error interrupt: Disabled */
   TCF0.INTCTRLA = 0x01;

   /* Compare/Capture channel A interrupt: Disabled
      Compare/Capture channel B interrupt: Disabled */
   TCF0.INTCTRLB = 0x00;

   /* Clear the interrupt flags */
   TCF0.INTFLAGS = TCF0.INTFLAGS;
   /* Set counter register */
   TCF0.CNT =0x0000;
   /* Set period register (1 ms) */
   TCF0.PER = 31999;
   /* Set channel A Compare/Capture register */
   TCF0.CCA = 0;
   /* Set channel B Compare/Capture register */
   TCF0.CCB = 0;
   /* Set channel C Compare/Capture register */
   TCF0.CCC = 0;
   /* Set channel D Compare/Capture register */
   TCF0.CCD = 0;

   /* Restore interrupts enabled/disabled state */
   SREG=s;
}

// ########################################################################
// ########################################################################
// ######                          T C D 0                           ######
// ########################################################################
// ########################################################################
/**************************************************************************
***   Funktionsname:    tcd0_init                                       ***
***   Erstellt mit AVR-Code Vision Code Wizard                     		***
***   Beschreibung:       LIDAR						                    ***
***   Parameter:          keine                                         ***
***   Rückgabeparameter:  keine                                         ***
**************************************************************************/
void tcd0_init()
{
	unsigned char s;
	
	// Save interrupts enabled/disabled state
	s = SREG;
	// Disable interrupts
	cli();
	
	// Disable and reset the timer/counter just to be sure
	tc0_disable(&TCD0);
	// ATxmega256A3 -> Peripheral Clock: 32MHz
	// Clock source: Peripheral Clock/1 (0x01) -> 32MHz
	TCD0.CTRLA = 0x01;
	// Mode: PWM
	// Compare/Capture on channel A: On
	// Compare/Capture on channel B: Off	
	// Compare/Capture on channel C: Off
	// Compare/Capture on channel D: Off
	TCD0.CTRLB = 0x13; //0xF3;
	// Capture event source: None
	// Capture event action: None
	TCD0.CTRLC = 0x00;
	TCD0.CTRLD = 0x00;
	TCD0.CTRLE = 0x00;

	// Overflow interrupt: Low Level
	// Error interrupt: Disabled
	TCD0.INTCTRLA = 0x00;

	// Compare/Capture channel A interrupt: Disabled
	// Compare/Capture channel B interrupt: Disabled
	TCD0.INTCTRLB = 0x00;

	// Clear the interrupt flags
	TCD0.INTFLAGS = TCD0.INTFLAGS;
	// Set counter register
	TCD0.CNT = 0x0000;
	/* Set period register (25 kHz) */
	TCD0.PER = 1280;
	// Set channel A Compare/Capture register
	TCD0.CCA = 0;
	// Set channel B Compare/Capture register
	TCD0.CCB = 0;
	// Set channel C Compare/Capture register
	TCD0.CCC = 0;
	// Set channel D Compare/Capture register
	TCD0.CCD = 0;

	// Restore interrupts enabled/disabled state
	SREG=s;
}


// ########################################################################
// ########################################################################
// ######                          T C E 0                           ######
// ########################################################################
// ########################################################################
/**************************************************************************
***   Funktionsname:    tce0_init                                       ***
***   Erstellt mit AVR-Code Vision Code Wizard                     		***
***   Beschreibung:     PWM-Kanäle für Servos und LIDAR                 ***
***   Parameter:          keine                                         ***
***   Rückgabeparameter:  keine                                         ***
**************************************************************************/
void tce0_init()
{
	unsigned char s;
	
	// Save interrupts enabled/disabled state
	s = SREG;
	// Disable interrupts
	cli();
	
	// Disable and reset the timer/counter just to be sure
	tc0_disable(&TCE0);
	// ATxmega256A3 -> Peripheral Clock: 32MHz
	// Clock source: Peripheral Clock/64 (0x05) -> 32MHz / 64 -> 500kHz
	TCE0.CTRLA = 0x05;
	// Mode: PWM
	// Compare/Capture on channel A: On
	// Compare/Capture on channel B: Off	
	// Compare/Capture on channel C: Off
	// Compare/Capture on channel D: Off
	TCE0.CTRLB = 0x13; //0xF3;
	// Capture event source: None
	// Capture event action: None
	TCE0.CTRLC = 0x00;
	TCE0.CTRLD = 0x00;
	TCE0.CTRLE = 0x00;

	// Overflow interrupt: Low Level
	// Error interrupt: Disabled
	TCE0.INTCTRLA = 0x00;

	// Compare/Capture channel A interrupt: Disabled
	// Compare/Capture channel B interrupt: Disabled
	TCE0.INTCTRLB = 0x00;

	// Clear the interrupt flags
	TCE0.INTFLAGS = TCE0.INTFLAGS;
	// Set counter register
	TCE0.CNT = 0x0000;
	// -> 500kHz / (1 + 9999) -> 50Hz -> 20ms
	// Further in the file servo.c the period time and ticks must be set to the following values:
	//		-> .T = 0.02;
	//		-> .Ticks = 10000;
	TCE0.PER = 9999;
	// Set channel A Compare/Capture register
	TCE0.CCA = 0;
	// Set channel B Compare/Capture register
	TCE0.CCB = 0;
	// Set channel C Compare/Capture register
	TCE0.CCC = 0;
	// Set channel D Compare/Capture register
	TCE0.CCD = 0;

	// Restore interrupts enabled/disabled state
	SREG=s;
}









/* ************************************************************** */
/*! \brief Set dedicated PWM channel of TC0.
 *
 *  Function used to set PWM channel
 *
 *  \param qTimer pointer to timer.
 *  \param ucChannel PWM channel (CH_A ... CH_D).
 *  \param uiPWMValue PWM value.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void SetPWM0(TC0_t * qTimer, uint8_t ucChannel, uint16_t uiPWMValue)
{
    if(ucChannel == CH_A)
        qTimer->CCA = uiPWMValue;
    else if(ucChannel == CH_B)
        qTimer->CCB = uiPWMValue;    
    else if(ucChannel == CH_C)
        qTimer->CCC = uiPWMValue;    
    else if(ucChannel == CH_D)
        qTimer->CCD = uiPWMValue;
}



/* ************************************************************** */
/*! \brief Set dedicated PWM channel of TC1.
 *
 *  Function used to set PWM channel
 *
 *  \param qTimer pointer to timer.
 *  \param ucChannel PWM channel (CH_A or CH_B).
 *  \param uiPWMValue PWM value.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void SetPWM1(TC1_t * qTimer, uint8_t ucChannel, uint16_t uiPWMValue)
{
    if(ucChannel == CH_A)
        qTimer->CCA = uiPWMValue;
    else if(ucChannel == CH_B)
        qTimer->CCB = uiPWMValue;    
}

