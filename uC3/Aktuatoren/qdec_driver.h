/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      qdec header file.
 *
 *      This file contains the functions to deal with the XMEGA quadrature decoder.
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
 *      TAVR1600: Using the XMEGA quadrature decoder. 
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

#ifndef __QDEC_DRIVER_H__
#define __QDEC_DRIVER_H__

#include <avr/io.h>

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* Clockwise direction. */
#define CW_DIR   0 
/* Counter Clockwise direction. */
#define CCW_DIR  1 


/*! \brief This macro return the value of the capture register.
 *
 * \param  _tc   The Timer/Counter to get the capture value from.
 */
#define GetCaptureValue(_tc)  ( _tc.CCA )


/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
uint8_t QDEC_Total_Setup0(PORT_t * qPort,
                      uint8_t qPin,
                      uint8_t invIO,
                      uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      uint8_t useIndex,
                      EVSYS_QDIRM_t qIndexState,
                      TC0_t * qTimer,
                      TC_EVSEL_t qEventChannel,
                      uint16_t lineCount);
uint8_t QDEC_Total_Setup1(PORT_t * qPort,
                      uint8_t qPin,
                      uint8_t invIO,
                      uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      uint8_t useIndex,
                      EVSYS_QDIRM_t qIndexState,
                      TC1_t * qTimer,
                      TC_EVSEL_t qEventChannel,
                      uint16_t lineCount);

uint8_t QDEC_Port_Setup(PORT_t * qPort, uint8_t qPin, uint8_t useIndex, uint8_t invIO);

uint8_t QDEC_EVSYS_Setup(uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      uint8_t useIndex,
                      EVSYS_QDIRM_t qIndexState );

void QDEC_TC0_Dec_Setup(TC0_t * qTimer,
                       TC_EVSEL_t qEventChannel,
                       uint16_t lineCount);
void QDEC_TC1_Dec_Setup(TC1_t * qTimer,
                       TC_EVSEL_t qEventChannel,
                       uint16_t lineCount);




#endif /* __QDEC_DRIVER_H__ */