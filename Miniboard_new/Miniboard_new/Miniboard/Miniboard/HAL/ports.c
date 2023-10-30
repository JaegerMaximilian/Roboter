/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      ports source file.
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
#include <avr/io.h>


/* ************************************************************** */
/*! \brief Initialize ports.
 *
 *  Function initialize the IO ports 
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
/*
PORT_.OUT  Data Output Value (Manual page 139)
            This register sets the data output value for the individual pins in the port. If
            OUTn is written to one, pin n is driven high. if OUTn is written to zero, pin n
            is driven low. For this setting to have any effect the pin direction must be
            set as output.

PORT_.DIR  Data Direction Register (Manual page 138)
            This register sets the data direction for the individual pins in the port. If
            DIRn is written to one, pin n is configured as an output pin. If it is written
            to zero, it is defined as an input pin.

PINnCTRL Pin n Configuration Register(Manual page 142)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    | SRLEN  | INVEN  |         OPC[2:0]         |         ISC[2:0]          |
    ------------------------------------------------------------------------- 
    SRLEN   Slew Rate Limit Enable
        Setting this bit will enable slew-rate limiting on this pin 
        
    INVEN   Inverted I/O Enable
        Setting this bit will enable inverting output and input data on pin n.
        
    OPC[2:0] Output and Pull Configuration(Manual page 142) 
                    Group config.   Output config.   Pull config.
        0   0   0       TOTEM           Totempole       not applicable (n/a)   
        0   0   1       BUSKEEPER       Totempole       Bus keeper   
        0   1   0       PULLDOWN        Totempole       Pull-down (on input)
        0   1   1       PULLUP          Totempole       Pull-up (on input)
        1   0   0       WIREDOR         Wired OR        (n/a)
        1   0   1       WIREDAND        Wired AND       (n/a)
        1   1   0       WIREDORPULL     Wired OR        Pull-down
        1   1   1       WIREDANDPULL    Wired AND       Pull-up 
        
    ISC[2:0] Input/Sense Configuration (Manual page 143)
                    Group config.      Description
        0   0   0       BOTHEDGES       Sense both edges  
        0   0   1       RISING          Sense rising edge
        0   1   0       FALLING         Sense falling edge
        0   1   1       LEVEL           Sense low level (1)
        1   0   0                       Reserved
        1   0   1                       Reserved
        1   1   0                       Reserved
        1   1   1       INPUT_DISANBLE  Input buffer disabled (2)
        Note:   1:A low pin value will not generate events, and a high pin value will
                    continously generate events        
                2:Only Port A-F supports the input buffer disable option.
                
INTCTRL Interrupt Control Register (Manual page 141)
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |   R    |   R    |        |       |   INT1LVL[1:0]  |  INT0LVL[1:0]    |
    ------------------------------------------------------------------------- 
    INTnLVL Interrupt n Level
        These bits enable interrupt request for port interrupt and select the 
        interrupt level. (Interrupts and programmable multi-level interrupt 
        controller on page 123)
       INTnLVL
        0   0       Interrupt disabled
        0   1       Low level interrupt
        1   0       Medium level interrupt
        1   1       High level interrupt
        
INT0MASK Interrupt 0 Mask Register (Manual page 141)
    These bits are used to mask which pins can be used as source for port interrupt 0.
    If INT0MASKn is written to one, pin is used as source for port interrupt0.    

INT1MASK Interrupt 1 Mask Register (Manual page 141)
    These bits are used to mask which pins can be used as source for port interrupt 1.
    If INT0MASKn is written to one, pin is used as source for port interrupt1.
         
*/
void ports_init(void)
{

/* PORTA initialization */
PORTA.OUT=0x00;
PORTA.DIR=0xFF;
PORTA.PIN0CTRL = 0x00;  
PORTA.PIN1CTRL = 0x00;
PORTA.PIN2CTRL = 0x00;
PORTA.PIN3CTRL = 0x00;
PORTA.PIN4CTRL = 0x00;
PORTA.PIN5CTRL = 0x00;
PORTA.PIN6CTRL = 0x00;
PORTA.PIN7CTRL = 0x00;
PORTA.INTCTRL = 0x00;     
PORTA.INT0MASK = 0x00;
PORTA.INT1MASK = 0x00;

/* PORTB initialization */
PORTB.OUT = 0x00;
PORTB.DIR = 0x0E;
PORTB.PIN0CTRL = 0x00;
PORTB.PIN1CTRL = 0x00;
PORTB.PIN2CTRL = 0x00;
PORTB.PIN3CTRL = 0x00;
PORTB.INTCTRL = 0x00;
PORTB.INT0MASK=0x00;
PORTB.INT1MASK=0x00;

/* PORTC initialization */
PORTC.OUT = 0x80;
PORTC.DIR = 0x8F;
PORTC.PIN0CTRL = 0x00;
PORTC.PIN1CTRL = 0x00;
PORTC.PIN2CTRL = 0x00;
PORTC.PIN3CTRL = 0x00;
PORTC.PIN4CTRL = 0x00;
PORTC.PIN5CTRL = 0x00;
PORTC.PIN6CTRL = 0x00;
PORTC.PIN7CTRL = 0x00;
PORTC.INTCTRL = 0x00;
PORTC.INT0MASK = 0x00;
PORTC.INT1MASK = 0x00;


/* PORTD initialization */
PORTD.OUT = 0x00;
PORTD.DIR = 0xF0;
PORTD.PIN0CTRL = 0x00;
PORTD.PIN1CTRL = 0x00;
PORTD.PIN2CTRL = 0x00;
PORTD.PIN3CTRL = 0x00;
PORTD.PIN4CTRL = 0x00;
PORTD.PIN5CTRL = 0x00;
PORTD.PIN6CTRL = 0x00;
PORTD.PIN7CTRL = 0x00;
PORTD.INTCTRL = 0x00;
PORTD.INT0MASK = 0x00;
PORTD.INT1MASK = 0x00;

/* PORTE initialization */
PORTE.OUT = 0x00;
PORTE.DIR = 0x00;
PORTE.PIN0CTRL = 0x18;
PORTE.PIN1CTRL = 0x18;
PORTE.PIN2CTRL = 0x18;
PORTE.PIN3CTRL = 0x18;
PORTE.INTCTRL = 0x00;   
PORTE.INT0MASK = 0x00;
PORTE.INT1MASK = 0x00;


/* PORTR initialization */
PORTR.OUT = 0x00;
PORTR.DIR = 0x00;
PORTR.PIN0CTRL = 0x00;
PORTR.PIN1CTRL = 0x00;
PORTR.INTCTRL = 0x00; 
PORTR.INT0MASK = 0x00;
PORTR.INT1MASK = 0x00;
}

/* ************************************************************** */
/*! \brief Initialize virtual ports.
 *
 *  Function initialize the virtual ports 
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void vports_init(void)
{
/*
  Virtual port registers allow for port registers in the extended I/O memory space to be mapped virtually
in the I/O memory space. When mapping a port, writing to the virtual port register will be
the same as writing to the real port register. This enables use of I/O memory specific instructions
for bit-manipulation, and the I/O memory specific instructions IN and OUT on port register that
normally resides in the extended I/O memory space. There are four virtual ports, so up to four
ports can be mapped virtually at the same time. The mapped registers are IN, OUT, DIR and
INTFLAGS.
*/
/* PORTA mapped to VPORT0
   PORTB mapped to VPORT1 */
PORTCFG.VPCTRLA=PORTCFG_VP1MAP_PORTB_gc | PORTCFG_VP0MAP_PORTA_gc;
/* PORTC mapped to VPORT2
   PORTD mapped to VPORT3 */
PORTCFG.VPCTRLB=PORTCFG_VP3MAP_PORTD_gc | PORTCFG_VP2MAP_PORTC_gc;
}
