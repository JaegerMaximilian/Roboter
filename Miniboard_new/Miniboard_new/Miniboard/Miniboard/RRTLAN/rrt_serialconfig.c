/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      SERIALCONFIG.c
Version :  V 1.0
Date    :  28.02.2011
Author  :  MUCKENHUMER BERNHARD

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : ATXmega256a3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024                

               Copyright (c) 2008 by FH-Wels                           
                   All Rights Reserved.
****************************************************************/

#define RRTLAN_SERIALCONFIG_EXTERN

#include <avr/interrupt.h>
#include "rrt_serialconfig.h"
#include "rrt_transportLayer.h"
#include "rrt_dataLinkLayer.h"
#include "rrt_usart_driver.h"               
#include "define.h"
#include "LEDcontrol.h"
#include "usart.h"

uint8_t crc_init_done = FALSE;

/**************************************************************************
***   FUNKTIONNAME: InitUSARTS                                          ***
***   FUNKTION: initialisiert alle freigegebenen USARTS                 ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitUSARTS()
{
//////////////////////////////////////////////////////////////////////////////
//PC, Kamera Interface (serielles Protokoll wird nicht angewendet)
//////////////////////////////////////////////////////////////////////////////
  #ifdef _SERIAL_INTERFACE_PC_C0
    PORTC.DIRSET   = PIN3_bm;
    PORTC.DIRCLR   = PIN2_bm;
    InitUSART(&USART_data_PC_C0, &USARTC0, BAUDRATE_PC_C0, PC_INTERFACE);
  #endif  
  
  #ifdef _SERIAL_INTERFACE_PC_D0
    PORTD.DIRSET   = PIN3_bm;
    PORTD.DIRCLR   = PIN2_bm;
    InitUSART(&USART_data_PC_D0, &USARTD0, BAUDRATE_PC_D0, PC_INTERFACE);
  #endif 
  
  #ifdef _SERIAL_INTERFACE_PC_E0
    PORTE.DIRSET   = PIN3_bm;
    PORTE.DIRCLR   = PIN2_bm;
    InitUSART(&USART_data_PC_E0, &USARTE0, BAUDRATE_PC_E0, PC_INTERFACE);
  #endif     
  
//////////////////////////////////////////////////////////////////////////////
//Protokoll Interfaces
//////////////////////////////////////////////////////////////////////////////
  #ifdef _SERIAL_INTERFACE_C0
    PORTC.DIRSET   = PIN3_bm;
    PORTC.DIRCLR   = PIN2_bm;
    InitUSART(&USART_data_C0, &USARTC0, BAUDRATEC0, PROTOKOLL_INTERFACE);
    InitTransmissionStatus(&USART_data_C0); 
  #endif  
  
  #ifdef _SERIAL_INTERFACE_C1
    PORTC.DIRSET   = PIN7_bm;
    PORTC.DIRCLR   = PIN6_bm;
    InitUSART(&USART_data_C1, &USARTC1, BAUDRATEC1, PROTOKOLL_INTERFACE);
    InitTransmissionStatus(&USART_data_C1); 
  #endif
  
  #ifdef _SERIAL_INTERFACE_D0
    PORTD.DIRSET   = PIN3_bm;
    PORTD.DIRCLR   = PIN2_bm;
    InitUSART(&USART_data_D0, &USARTD0, BAUDRATED0, PROTOKOLL_INTERFACE);
    InitTransmissionStatus(&USART_data_D0); 
  #endif 
  
  #ifdef _SERIAL_INTERFACE_D1
    PORTD.DIRSET   = PIN7_bm;
    PORTD.DIRCLR   = PIN6_bm;
    InitUSART(&USART_data_D1, &USARTD1, BAUDRATED1, PROTOKOLL_INTERFACE);
    InitTransmissionStatus(&USART_data_D1); 
  #endif   
  
  #ifdef _SERIAL_INTERFACE_E0
    PORTE.DIRSET   = PIN3_bm;
    PORTE.DIRCLR   = PIN2_bm;
    InitUSART(&USART_data_E0, &USARTE0, BAUDRATEE0, PROTOKOLL_INTERFACE);
    InitTransmissionStatus(&USART_data_E0); 
  #endif 
  
  #ifdef _SERIAL_INTERFACE_E1
    PORTE.DIRSET   = PIN7_bm;
    PORTE.DIRCLR   = PIN6_bm;
    InitUSART(&USART_data_E1, &USARTE1, BAUDRATEE1, PROTOKOLL_INTERFACE);
    InitTransmissionStatus(&USART_data_E1); 
  #endif    
  
  #ifdef _SERIAL_INTERFACE_F0
    PORTF.DIRSET   = PIN3_bm;
    PORTF.DIRCLR   = PIN2_bm;
    InitUSART(&USART_data_F0, &USARTF0, BAUDRATEF0, PROTOKOLL_INTERFACE);
    InitTransmissionStatus(&USART_data_F0); 
  #endif    
  
///////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void InitUSART(USART_data_t* usart_data, USART_t* usart, unsigned long baudrate, uint8_t pc_interface)
{
    uint16_t bsel = 0;
    
    //Init data structure
    usart_data->unique_ID = 0;
    usart_data->receivestate = _FIND_STX_I;
    usart_data->first_block_counter = 0;
    usart_data->user_data_counter = 0;
    usart_data->stx_received = FALSE; 
    
    //Set USART parameter  
	USART_InterruptDriver_Initialize(usart_data, usart, USART_DREINTLVL_MED_gc);
	USART_Format_Set(usart_data->usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(usart_data->usart, USART_RXCINTLVL_MED_gc);
    
    //calculate bsel value
    bsel =  8 * (F_MCU / (16*baudrate)) - 1; 
    
    if(pc_interface)
    {
     USART_Baudrate_Set(usart, bsel , -3);
    }
    else
    {
     USART_Baudrate_Set(usart, bsel , -3); 
     
     if(!crc_init_done)
      {
        crcInit();
        crc_init_done = TRUE;
      }
    }

	/* Enable both RX and TX. */
	USART_Rx_Enable(usart_data->usart);
	USART_Tx_Enable(usart_data->usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_MEDLVLEX_bm | PMIC_LOLVLEX_bm;
    
    //Ports initialisieren
    InitPortApp();
    
	/* Enable global interrupts. */
	sei();
}

#ifdef _SERIAL_INTERFACE_C0
/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTC0_RXC_vect)
{
	USART_RXComplete(&USART_data_C0);
	TOGGLE_LED1;
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	
	USART_DataRegEmpty(&USART_data_C0);
	TOGGLE_LED2;
	
	
}

#endif

#ifdef _SERIAL_INTERFACE_C1
/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTC1_RXC_vect)
{
	USART_RXComplete(&USART_data_C1);
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC1_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_C1);
}

#endif

 
#ifdef _SERIAL_INTERFACE_D0
/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTD0_RXC_vect)
{
	USART_RXComplete(&USART_data_D0);
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTD0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_D0);
}

#endif

#ifdef _SERIAL_INTERFACE_D1
/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTD1_RXC_vect)
{
	USART_RXComplete(&USART_data_D1);
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTD1_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_D1);
}

#endif

#ifdef _SERIAL_INTERFACE_E0
/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTE0_RXC_vect)
{
	USART_RXComplete(&USART_data_E0);
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_E0);
}

#endif

#ifdef _SERIAL_INTERFACE_E1
/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTE1_RXC_vect)
{
	USART_RXComplete(&USART_data_E1);
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTE1_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_E1);
}

#endif

#ifdef _SERIAL_INTERFACE_F0
/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTF0_RXC_vect)
{
	USART_RXComplete(&USART_data_F0);
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTF0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_F0);
}

#endif



 



 



 



 