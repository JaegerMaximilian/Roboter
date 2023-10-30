/**************************************************************************
***     Dateiname:       adc_driver.c                                   ***
***     Funktion:        behandelt ADC                                  ***
***     Erstellt:        Kreiseder Erich (22-06-2010)                   ***
***     Doku:            ATX-Mega A Manual Seite 289-316                ***
***    Änderungen:                                                      ***
***                                                                     ***
***                                                                     ***
***                    Copyright (c) 2010 by FH-Wels                    ***
***                        All Rights Reserved.                         ***
***                       Written by Kreiseder Erich                    ***
**************************************************************************/

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

#ifndef ADCBCAL0_offset
	#define ADCBCAL0_offset 0x24
#endif

#ifndef ADCBCAL1_offset
	#define ADCBCAL1_offset 0x25
#endif



// Function used to read the calibration byte from the
// signature row, specified by 'index'
unsigned char read_calibration_byte(unsigned char index)
{
unsigned char r;
NVM.CMD=NVM_CMD_READ_CALIB_ROW_gc;
r=*((unsigned char*) index);
// Clean up NVM command register
NVM.CMD=NVM_CMD_NO_OPERATION_gc;
return r;
}

// ADCA initialization
void adca_init(void)
{
   // ADCA is enabled
   // Resolution: 12 Bits
   // Load the calibration value for 12 Bit resolution
   // from the signature row
   ADCA.CALL=read_calibration_byte(PROD_SIGNATURES_START+ADCACAL0_offset);
   ADCA.CALH=read_calibration_byte(PROD_SIGNATURES_START+ADCACAL1_offset);

   // Free Running mode: Off
   // Conversion mode: Unsigned
   ADCA.CTRLB=(ADCA.CTRLB & (~(ADC_CONMODE_bm | ADC_FREERUN_bm | ADC_RESOLUTION_gm))) |
      ADC_RESOLUTION_12BIT_gc;

   // Clock frequency: 1000 kHz
   ADCA.PRESCALER=(ADCA.PRESCALER & (~ADC_PRESCALER_gm)) | ADC_PRESCALER_DIV32_gc;

   // Reference: Internal 1.00 V
   // Temperature reference: On
   ADCA.REFCTRL=(ADCA.REFCTRL & ((~(ADC_REFSEL_gm | ADC_TEMPREF_bm)) | ADC_BANDGAP_bm)) |
      ADC_REFSEL_INT1V_gc | ADC_TEMPREF_bm | ADC_BANDGAP_bm;


   // Initialize the ADC Compare register
   ADCA.CMPL=0x00;
   ADCA.CMPH=0x00;

   // ADC channel 0 gain: 1
   // ADC channel 0 input mode: Single-ended positive input signal
   ADCA.CH0.CTRL=(ADCA.CH0.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) |
      ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

   // ADC channel 0 positive input: ADC7 pin
   // ADC channel 0 negative input: GND
   ADCA.CH0.MUXCTRL=(ADCA.CH0.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
      ADC_CH_MUXPOS_PIN7_gc;

   // AD conversion is started by software
   ADCA.EVCTRL=ADC_EVACT_NONE_gc;

   // Channel 0 interrupt: Disabled
   ADCA.CH0.INTCTRL=(ADCA.CH0.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
      ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

   // Enable the ADC
   ADCA.CTRLA|=ADC_ENABLE_bm;
   // Insert a delay to allow the ADC common mode voltage to stabilize
   _delay_us(2);
}

// ADCA channel data read function using polled mode
unsigned int adca_read(unsigned char channel, unsigned char Pin)
{
   ADC_CH_t *pch=&ADCA.CH0+channel;
   unsigned int data;

   if(Pin > 15)
      return(-1); 

   Pin <<= 3;
   // Pin setzen
   pch->MUXCTRL &= ~0x38;
   pch->MUXCTRL |= Pin;  

   // Start the AD conversion
   pch->CTRL|=ADC_CH_START_bm;
   // Wait for the AD conversion to complete
   while ((pch->INTFLAGS & ADC_CH_CHIF_bm)==0);
   // Clear the interrupt flag
   pch->INTFLAGS=ADC_CH_CHIF_bm;
   // Read the AD conversion result
   ((unsigned char *) &data)[0]=pch->RESL;
   ((unsigned char *) &data)[1]=pch->RESH;
   return data;
}

void adcb_init(void)
{
	// ADCA is enabled
	// Resolution: 12 Bits
	// Load the calibration value for 12 Bit resolution
	// from the signature row
	ADCB.CALL=read_calibration_byte(PROD_SIGNATURES_START+ADCBCAL0_offset);
	ADCB.CALH=read_calibration_byte(PROD_SIGNATURES_START+ADCBCAL1_offset);

	// Free Running mode: Off
	// Conversion mode: Unsigned
	ADCB.CTRLB=(ADCB.CTRLB & (~(ADC_CONMODE_bm | ADC_FREERUN_bm | ADC_RESOLUTION_gm))) |
	ADC_RESOLUTION_12BIT_gc;

	// Clock frequency: 1000 kHz
	ADCB.PRESCALER=(ADCB.PRESCALER & (~ADC_PRESCALER_gm)) | ADC_PRESCALER_DIV32_gc;

	// Reference: Internal 1.00 V
	// Temperature reference: On
	ADCB.REFCTRL=(ADCB.REFCTRL & ((~(ADC_REFSEL_gm | ADC_TEMPREF_bm)) | ADC_BANDGAP_bm)) |
	ADC_REFSEL_INT1V_gc | ADC_TEMPREF_bm | ADC_BANDGAP_bm;


	// Initialize the ADC Compare register
	ADCB.CMPL=0x00;
	ADCB.CMPH=0x00;

	// ADC channel 0 gain: 1
	// ADC channel 0 input mode: Single-ended positive input signal
	ADCB.CH0.CTRL=(ADCB.CH0.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAIN_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 0 positive input: ADC7 pin
	// ADC channel 0 negative input: GND
	ADCB.CH0.MUXCTRL=(ADCB.CH0.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
	ADC_CH_MUXPOS_PIN7_gc;

	// AD conversion is started by software
	ADCB.EVCTRL=ADC_EVACT_NONE_gc;

	// Channel 0 interrupt: Disabled
	ADCB.CH0.INTCTRL=(ADCB.CH0.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
	ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

	// Enable the ADC
	ADCB.CTRLA|=ADC_ENABLE_bm;
	// Insert a delay to allow the ADC common mode voltage to stabilize
	_delay_us(2);
}

// ADCA channel data read function using polled mode
unsigned int adcb_read(unsigned char channel, unsigned char Pin)
{
	ADC_CH_t *pch=&ADCB.CH0+channel;
	unsigned int data;

	if(Pin > 15)
	return(-1);

	Pin <<= 3;
	// Pin setzen
	pch->MUXCTRL &= ~0x38;
	pch->MUXCTRL |= Pin;

	// Start the AD conversion
	pch->CTRL|=ADC_CH_START_bm;
	// Wait for the AD conversion to complete
	while ((pch->INTFLAGS & ADC_CH_CHIF_bm)==0);
	// Clear the interrupt flag
	pch->INTFLAGS=ADC_CH_CHIF_bm;
	// Read the AD conversion result
	((unsigned char *) &data)[0]=pch->RESL;
	((unsigned char *) &data)[1]=pch->RESH;
	return data;
}


