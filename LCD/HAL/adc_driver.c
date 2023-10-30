/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Provides routines to deal with the ADC.
 *
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
 *      The file provide functions to deal with the ADC.
 *
 * \author
 *      Michael Zauner
 *      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
 *      Support email: roboracing@fh-wels.at
 *
 * $Revision: 1 $
 * $Date: 2012-09-11  $  \n
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
#include "global.h"
#include <avr/io.h>
#include "adc_driver.h"
#include <util/delay.h>


#ifndef ADCACAL0_offset
	#define ADCACAL0_offset 0x20
#endif

#ifndef ADCACAL1_offset
	#define ADCACAL1_offset 0x21
#endif



uint8_t ADCinit_done = 0;


/* ************************************************************** */
/*! \brief Read calibration byte.
 *
 *  Function used to read the calibration byte from the
 *  signature row 
 *
 *  \param index index to signature row.
 *
 *  \retval calibration byte.
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
uint8_t read_calibration_byte(uint8_t index)
{
unsigned char r;
NVM.CMD=NVM_CMD_READ_CALIB_ROW_gc;
r=*((unsigned char*) index);
/* Clean up NVM command register */
NVM.CMD=NVM_CMD_NO_OPERATION_gc;
return r;
}

/* ************************************************************** */
/*! \brief Initialize ADC.
 *
 *  initialize the ADC
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void adca_init(void)
{
	if(ADCinit_done == 0)
	{
		/* ADCA is enabled
		   Resolution: 12 Bits
		   Load the calibration value for 12 Bit resolution
		   from the signature row */
		ADCA.CALL=read_calibration_byte(PROD_SIGNATURES_START+ADCACAL0_offset);
		ADCA.CALH=read_calibration_byte(PROD_SIGNATURES_START+ADCACAL1_offset);

		/* Free Running mode: Off
		   Conversion mode: Unsigned */
		ADCA.CTRLB=(ADCA.CTRLB & (~(ADC_CONMODE_bm | ADC_FREERUN_bm | ADC_RESOLUTION_gm))) |
			ADC_RESOLUTION_12BIT_gc;

		/* Clock frequency: 1000 kHz */
		ADCA.PRESCALER=(ADCA.PRESCALER & (~ADC_PRESCALER_gm)) | ADC_PRESCALER_DIV32_gc;

		/* Reference: Internal 1.00 V
		   Temperature reference: On */
		ADCA.REFCTRL=(ADCA.REFCTRL & ((~(ADC_REFSEL_gm | ADC_TEMPREF_bm)) | ADC_BANDGAP_bm)) |
			ADC_REFSEL_INT1V_gc | ADC_TEMPREF_bm | ADC_BANDGAP_bm;


		/* Initialize the ADC Compare register */
		ADCA.CMPL=0x00;
		ADCA.CMPH=0x00;

		/* ADC channel 0 gain: 1
		   ADC channel 0 input mode: Single-ended positive input signal */
		ADCA.CH0.CTRL=(ADCA.CH0.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) |
			ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

		/* ADC channel 0 positive input: ADC7 pin
		   ADC channel 0 negative input: GND */
		ADCA.CH0.MUXCTRL=(ADCA.CH0.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
			ADC_CH_MUXPOS_PIN7_gc;

		/* ADC channel 1 gain: 1
		   ADC channel 1 input mode: Internal positive input signal */
		ADCA.CH1.CTRL=(ADCA.CH1.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) |
			ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_INTERNAL_gc;

		/* ADC channel 1 positive input: Temp. Reference
		   ADC channel 1 negative input: GND */
		ADCA.CH1.MUXCTRL=(ADCA.CH1.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
			ADC_CH_MUXINT_TEMP_gc;

		/* ADC channel 2 gain: 1
		   ADC channel 2 input mode: Internal positive input signal */
		ADCA.CH2.CTRL=(ADCA.CH2.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) |
			ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_INTERNAL_gc;

		/* ADC channel 2 positive input: Temp. Reference
		   ADC channel 2 negative input: GND */
		ADCA.CH2.MUXCTRL=(ADCA.CH2.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
			ADC_CH_MUXINT_TEMP_gc;

		/* ADC channel 3 gain: 1
		   ADC channel 3 input mode: Internal positive input signal */
		ADCA.CH3.CTRL=(ADCA.CH3.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) |
			ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_INTERNAL_gc;

		/* ADC channel 3 positive input: Temp. Reference
		   ADC channel 3 negative input: GND */
		ADCA.CH3.MUXCTRL=(ADCA.CH3.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
			ADC_CH_MUXINT_TEMP_gc;

		/* AD conversion is started by software */
		ADCA.EVCTRL=ADC_EVACT_NONE_gc;

		/* Channel 0 interrupt: Disabled */
		ADCA.CH0.INTCTRL=(ADCA.CH0.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
			ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
		/* Channel 1 interrupt: Disabled */
		ADCA.CH1.INTCTRL=(ADCA.CH1.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
			ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
		/* Channel 2 interrupt: Disabled */
		ADCA.CH2.INTCTRL=(ADCA.CH2.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
			ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
		/* Channel 3 interrupt: Disabled */
		ADCA.CH3.INTCTRL=(ADCA.CH3.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
			ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

		/* Enable the ADC */
		ADCA.CTRLA|=ADC_ENABLE_bm;
		/* Insert a delay to allow the ADC common mode voltage to stabilize */
		_delay_us(2);
   
   
		ADCinit_done = 1;
	}	
}


/* ************************************************************** */
/*! \brief ADCA channel data read function using polled mode.
 *
 *  Function used to read ADC 
 *
 *  \param channel ADC-channel.
 *  \param Pin     ADC-pin.
 *
 *  \retval ADC-value (12 Bit).
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
uint16_t adca_read(uint8_t channel, uint8_t Pin)
{
   ADC_CH_t *pch=&ADCA.CH0+channel;
   uint16_t data;

   if(Pin > 11)
      return(-1); 

   Pin <<= 3;
   /* set pin */
   pch->MUXCTRL &= ~0x38;
   pch->MUXCTRL |= Pin;  

   /* start the AD conversion */
   pch->CTRL|=ADC_CH_START_bm;
   /* wait for the AD conversion to complete */
   while ((pch->INTFLAGS & ADC_CH_CHIF_bm)==0);
   /* clear the interrupt flag */
   pch->INTFLAGS=ADC_CH_CHIF_bm;
   /* read the AD conversion result */
   ((uint8_t *) &data)[0]=pch->RESL;
   ((uint8_t *) &data)[1]=pch->RESH;
   return data;
}
