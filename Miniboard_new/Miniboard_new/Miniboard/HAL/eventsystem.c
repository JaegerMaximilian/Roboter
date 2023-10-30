/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Event system source file.
 *
 *      This file contains the function to deal with the event system.
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
 * \par Documentation
 *      The file provide functions to to deal with the event system.
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
/*! \brief Initialize event system.
 *
 *  Event system initialization function.
 *  Sets PORTE es input and enables the pull up resistors for each pin,
 *  if it isn't done before
 *
 */
/* ************************************************************** */
/*
  CHnMUX    Channel Multiplexer
  Bit Settings on manual pages 71 and 72, tables 6-3 and 6-4.
  
  CHnCTRL Event Channel n Control Register
        7        6        5        4        3        2       1        0
    -------------------------------------------------------------------------
    |   R    |   QDIRM[1:0]   |  QDIEN  |  QDEN  |      DIGFILT[2:0]        |
    ------------------------------------------------------------------------- 
    QDIRM Quadrature Decode Index Recognition Mode
        These bits determine the quadrature state for the QDPH0 and QDPH90 signals
        where a valid index signal is recognized and the counter index data event
        is given. These bits are only needed if a index signal is used.
        QDIRM       Index Recognition State
        0   0       {QDPH0, QDPH90} = 0b00
        0   1       {QDPH0, QDPH90} = 0b01
        1   0       {QDPH0, QDPH90} = 0b10
        1   1       {QDPH0, QDPH90} = 0b11
        
    QDIEN   Quadrature Decoder Index Enable
        When this bit is set the event channel will be used as QDEC index source, and 
        the index data event will be enabled.
        These bit is only available for CH0CTRL, CH2CTRL and CH4CTRL.
    QDEN    Quadrature Decoder Enable
        Setting this bit enables QDEC operation.
        These bit is only available for CH0CTRL, CH2CTRL and CH4CTRL.     
        
    DIGFILT[2:0] Digital Filter Coefficient(Manual page 73) 
        0   0   0       1Sample
        0   0   1       2Samples
        0   1   0       3Samples
        0   1   1       4Samples
        1   0   0       5Samples
        1   0   1       6Samples
        1   1   0       7Samples
        1   1   1       8Samples
*/

void event_system_init(void)
{
/* Event System Channel 0 source: None */
EVSYS.CH0MUX = 0x00;
/* Event System Channel 1 source: None */
EVSYS.CH1MUX = 0x00;
/* Event System Channel 2 source: None */
EVSYS.CH2MUX = 0x00;
/* Event System Channel 3 source: None */
EVSYS.CH3MUX = 0x00;
/* Event System Channel 4 source: None */
EVSYS.CH4MUX = 0x00;
/* Event System Channel 5 source: None */
EVSYS.CH5MUX = 0x00; 
/* Event System Channel 6 source: None */
EVSYS.CH6MUX = 0x00;
/* Event System Channel 7 source: None */
EVSYS.CH7MUX = 0x00;

/* Event System Channel 0 Digital Filter Coefficient: 1 Sample */
EVSYS.CH0CTRL=0x00;
/* Event System Channel 1 Digital Filter Coefficient: 1 Sample */
EVSYS.CH1CTRL=0x00;
/* Event System Channel 2 Digital Filter Coefficient: 1 Sample */
EVSYS.CH2CTRL=0x00;
/* Event System Channel 3 Digital Filter Coefficient: 1 Sample */
EVSYS.CH3CTRL=0x00;
/* Event System Channel 4 Digital Filter Coefficient: 1 Sample */
EVSYS.CH4CTRL=0x00;
/* Event System Channel 5 Digital Filter Coefficient: 1 Sample */
EVSYS.CH5CTRL=0x00;
/* Event System Channel 6 Digital Filter Coefficient: 1 Sample */
EVSYS.CH6CTRL=0x00;
/* Event System Channel 7 Digital Filter Coefficient: 1 Sample */
EVSYS.CH7CTRL=0x00;

/* Event System Channel 0 output: Disabled
   Note: the correct direction for the Event System Channel 0 output
   is configured in the ports_init function */
PORTCFG.CLKEVOUT=(PORTCFG.CLKEVOUT & (~PORTCFG_EVOUT_gm)) | PORTCFG_EVOUT_OFF_gc;
}

