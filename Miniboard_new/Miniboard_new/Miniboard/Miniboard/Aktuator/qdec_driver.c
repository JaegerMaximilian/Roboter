/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      qdec source file.
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

#include <avr/io.h>
#include "qdec_driver.h"
#include "global.h"



/* ************************************************************** */
/*! \brief Total setup of QDEC with TC0x.
 *
 *  Function make a total setup of QDEC channel
 *
 *  \param qPort used port.
 *  \param qPin pin of A signal.
 *  \param invIO A and B signal invert (true) or not (false).
 *  \param qEvMux event channel (possible channels -> 0, 2 or 4).
 *  \param qPinInput input pin from QDPH0 to EVSYS.CHMUX.
 *  \param useIndex use index pin (true) or not (false).
 *  \param qIndexState status to trigger the index.
 *  \param qTimer used timer.
 *  \param qEventChannel used event channel.
 *  \param lineCount counts per rotation.
 *
 *  \retval true setup was OK.
 *  \retval false setup was not OK.
 *
 *  \version 11.09.2012
 */
/* ************************************************************** */
uint8_t QDEC_Total_Setup0(PORT_t * qPort,
                      uint8_t qPin,
                      uint8_t invIO,
                      uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      uint8_t useIndex,
                      EVSYS_QDIRM_t qIndexState,
                      TC0_t * qTimer,
                      TC_EVSEL_t qEventChannel,
                      uint16_t lineCount)
{
	if( !QDEC_Port_Setup(qPort, qPin, useIndex, invIO) )
		return false;
	if( !QDEC_EVSYS_Setup(qEvMux, qPinInput, useIndex, qIndexState ) )
		return false;
	QDEC_TC0_Dec_Setup(qTimer, qEventChannel, lineCount);

	return true;
}

/* ************************************************************** */
/*! \brief Total setup of QDEC with TC1x.
 *
 *  Function make a total setup of QDEC channel
 *
 *  \param qPort used port.
 *  \param qPin pin of A signal.
 *  \param invIO A and B signal invert (true) or not (false).
 *  \param qEvMux event channel (possible channels -> 0, 2 or 4).
 *  \param qPinInput input pin from QDPH0 to EVSYS.CHMUX.
 *  \param useIndex use index pin (true) or not (false).
 *  \param qIndexState status to trigger the index.
 *  \param qTimer used timer.
 *  \param qEventChannel used event channel.
 *  \param lineCount counts per rotation.
 *
 *  \retval true setup was OK.
 *  \retval false setup was not OK.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
uint8_t QDEC_Total_Setup1(PORT_t * qPort,
                      uint8_t qPin,
                      uint8_t invIO,
                      uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      uint8_t useIndex,
                      EVSYS_QDIRM_t qIndexState,
                      TC1_t * qTimer,
                      TC_EVSEL_t qEventChannel,
                      uint16_t lineCount)
{
	if( !QDEC_Port_Setup(qPort, qPin, useIndex, invIO) )
		return false;
	if( !QDEC_EVSYS_Setup(qEvMux, qPinInput, useIndex, qIndexState ) )
		return false;
	QDEC_TC1_Dec_Setup(qTimer, qEventChannel, lineCount);

	return true;
}

/* ************************************************************** */
/*! \brief Port setup of QDEC.
 *
 *  Function configure QDEC port
 *
 *  \param qPort used port.
 *  \param qPin pin of A signal.
 *  \param invIO A and B signal invert (true) or not (false).
 *  \param useIndex use index pin (true) or not (false).
 *
 *  \retval true setup was OK.
 *  \retval false setup was not OK.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
uint8_t QDEC_Port_Setup(PORT_t * qPort, uint8_t qPin, uint8_t useIndex, uint8_t invIO)
{
	/* Make setup depending on if Index signal is used. */
	if(useIndex){
		if(qPin > 5){
			return false;
		}
		qPort->DIRCLR = (0x07<<qPin);

		/* Configure Index signal sensing. */
		PORTCFG.MPCMASK = (0x04<<qPin);
		qPort->PIN0CTRL = (qPort->PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_BOTHEDGES_gc
		                  | (invIO ? PORT_INVEN_bm : 0);


	}else{
		if(qPin > 6){
			return false;
		}
		qPort->DIRCLR = (0x03<<qPin);
	}

	/* Set QDPH0 and QDPH1 sensing level. */
	PORTCFG.MPCMASK = (0x03<<qPin);
	qPort->PIN0CTRL = (qPort->PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc
	                  | (invIO ? PORT_INVEN_bm : 0);

	return true;
}




/* ************************************************************** */
/*! \brief Event system setup of QDEC.
 *
 *  Function make a total setup of QDEC channel
 *
 *  \param qEvMux event channel (possible channels -> 0, 2 or 4).
 *  \param qPinInput input pin from QDPH0 to EVSYS.CHMUX.
 *  \param useIndex use index pin (true) or not (false).
 *  \param qIndexState status to trigger the index.
 *
 *  \retval true setup was OK.
 *  \retval false setup was not OK.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
uint8_t QDEC_EVSYS_Setup(uint8_t qEvMux,
                      EVSYS_CHMUX_t qPinInput,
                      uint8_t useIndex,
                      EVSYS_QDIRM_t qIndexState )
{
	switch (qEvMux){
		case 0:
		    
		/* Configure event channel 0 for quadrature decoding of pins. */
		EVSYS.CH0MUX = qPinInput;
		EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
		if(useIndex){
			/*  Configure event channel 1 as index channel. Note
			 *  that when enabling Index in channel n, the channel
			 *  n+1 must be configured for the index signal.*/
			EVSYS.CH1MUX = qPinInput + 2;
			EVSYS.CH1CTRL = EVSYS_DIGFILT_2SAMPLES_gc;
			EVSYS.CH0CTRL |= (uint8_t) qIndexState | EVSYS_QDIEN_bm;

		}
		break;
		case 2:
		EVSYS.CH2MUX = qPinInput;
		EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
		if(useIndex){
			EVSYS.CH3MUX = qPinInput + 2;
			EVSYS.CH3CTRL = EVSYS_DIGFILT_2SAMPLES_gc;
			EVSYS.CH2CTRL |= (uint8_t) qIndexState | EVSYS_QDIEN_bm;
		}
		break;
		case 4:
		EVSYS.CH4MUX = qPinInput;
		EVSYS.CH4CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
		if(useIndex){
			EVSYS.CH5MUX = qPinInput + 2;
			EVSYS.CH5CTRL = EVSYS_DIGFILT_2SAMPLES_gc;
			EVSYS.CH4CTRL |= (uint8_t) qIndexState | EVSYS_QDIEN_bm;
		}
		break;
		default:
		return false;
	}
	return true;
}


/* ************************************************************** */
/*! \brief Timer setup of QDEC with TC0x.
 *
 *  Function make a total setup of QDEC channel
 *
 *  \param qTimer used timer.
 *  \param qEventChannel used event channel.
 *  \param lineCount counts per rotation.
 *
 *  \retval true setup was OK.
 *  \retval false setup was not OK.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void QDEC_TC0_Dec_Setup(TC0_t * qTimer, TC_EVSEL_t qEventChannel, uint16_t lineCount)
{
	/* Configure TC as a quadrature counter. */
	qTimer->CTRLD = (uint8_t) TC_EVACT_QDEC_gc | qEventChannel;
	qTimer->PER = (lineCount * 4) - 1;
	qTimer->CTRLA = TC_CLKSEL_DIV1_gc;
}

/* ************************************************************** */
/*! \brief Timer setup of QDEC with TC1x.
 *
 *  Function make a total setup of QDEC channel
 *
 *  \param qTimer used timer.
 *  \param qEventChannel used event channel.
 *  \param lineCount counts per rotation.
 *
 *  \retval true setup was OK.
 *  \retval false setup was not OK.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void QDEC_TC1_Dec_Setup(TC1_t * qTimer, TC_EVSEL_t qEventChannel, uint16_t lineCount)
{
	/* Configure TC as a quadrature counter. */
	qTimer->CTRLD = (uint8_t) TC_EVACT_QDEC_gc | qEventChannel;
	qTimer->PER = (lineCount * 4) - 1;
	qTimer->CTRLA = TC_CLKSEL_DIV1_gc;
}




