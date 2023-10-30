/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      TIMEOUTMANAGER.c
Version :  V 1.0
Date    :  28.02.2011
Author  :  MUCKENHUMER BERNHARD

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : ATmega644
Program type        : Application
Clock frequency     : 20,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024                

               Copyright (c) 2008 by FH-Wels                           
                   All Rights Reserved.
****************************************************************/


// ***************************************************************************************************************
// ***************************************************************************************************************
// *****  ÄNDERUNG 20.12.2012 - Michael Zauner - Checksume für Resend-Message neu berechnen -> da            *****
// *****                                         trial_counter verändert wurde!!                             *****
// ***************************************************************************************************************
// ***************************************************************************************************************


#define RRTLAN_TIMEOUTMANAGER_EXTERN

#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "multitask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "rrt_applicationLayer.h"
#include "ports.h"
#include "define.h" 
#include "global.h"
#include "../../uC1/RRTLAN/rrt_dataLinkLayer.h"


/**************************************************************************
***   FUNKTIONNAME: InitTimeoutManager                                  ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitTimeoutManager(void)
{
   SET_CYCLE(TIMEOUTMANAGER_TASKNBR, 5);
   SET_TASK(TIMEOUTMANAGER_TASKNBR, CYCLE);
   SET_TASK_HANDLE(TIMEOUTMANAGER_TASKNBR, TimeoutManager); 
}

/**************************************************************************
***   FUNCTIONNAME:        TimeoutManager                               ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char TimeoutManager(void)
{
 
 #ifdef _SERIAL_INTERFACE_C0
   Check_Timeout(&USART_data_C0); 
 #endif 
 
 #ifdef _SERIAL_INTERFACE_C1
   Check_Timeout(&USART_data_C1); 
 #endif
 
 #ifdef _SERIAL_INTERFACE_D0
   Check_Timeout(&USART_data_D0); 
 #endif
 
 #ifdef _SERIAL_INTERFACE_D1
   Check_Timeout(&USART_data_D1); 
 #endif
 
 #ifdef _SERIAL_INTERFACE_E0
   Check_Timeout(&USART_data_E0); 
 #endif
 
 #ifdef _SERIAL_INTERFACE_E1
   Check_Timeout(&USART_data_E1); 
 #endif 
 
 #ifdef _SERIAL_INTERFACE_F0
   Check_Timeout(&USART_data_F0); 
 #endif 

 SET_CYCLE(TIMEOUTMANAGER_TASKNBR, 5); 
 return(CYCLE);
}

/**************************************************************************
***   FUNCTIONNAME:        Check_Timeout                                ***
***   FUNCTION:            Timoutüberwachung der Schnittstellen         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Check_Timeout(USART_data_t* usart)
{
   uint8_t i,j,temp_nbr_of_bytes, *memptr;
   uint16_t checksum = 0;
   
   for(i = 0; i < NUM_MESSAGE_STATUS; i++)
   {
	   if(usart->message_status[i].allocated)
	   {
		   if(usart->message_status[i].nbr_timeouts++ >= 4)
		   {
			   if(usart->message_status[i].trial_counter < NBR_TRIALS)
			   {
				   memptr = usart->message_status[i].memptr + 2;
				   
// ***************************************************************************************************************
// ***************************************************************************************************************
// *****  ÄNDERUNG 20.12.2012 - Michael Zauner - Checksume für Resend-Message neu berechnen -> da            *****
// *****                                         trial_counter verändert wurde!!                             *****
// ***************************************************************************************************************
// ***************************************************************************************************************
				   usart->message_status[i].nbr_timeouts = 0;
				   usart->message_status[i].trial_counter++;
				   usart->message_status[i].memptr += 2;
				   *usart->message_status[i].memptr = *usart->message_status[i].memptr + 1;
				   usart->message_status[i].memptr += 3;
				   temp_nbr_of_bytes = *usart->message_status[i].memptr;
				   usart->message_status[i].memptr -= 5;
				   
				   //Checksum bilden und eintragen
				   checksum = crc_calc(memptr,(unsigned int)temp_nbr_of_bytes + FIRST_BLOCK);
				   memptr += (unsigned int)temp_nbr_of_bytes + FIRST_BLOCK;
				   *memptr = (uint8_t)(checksum >> 8);
				   memptr++;
				   *memptr = (uint8_t)(checksum);

				   
				   
				   //Daten in Sendefifo schreiben
				   j = 0;
				   while (j < (temp_nbr_of_bytes + NBR_HEADER_BYTES))
				   {
					   uint8_t byteToBuffer;
					   byteToBuffer = USART_TXBuffer_PutByte(usart, usart->message_status[i].memptr);
					   
					   if(byteToBuffer)
					   {
						   usart->message_status[i].memptr++;
						   j++;
					   }
				   }
				   
				   usart->message_status[i].memptr -=  (uint8_t)(((unsigned int)temp_nbr_of_bytes + NBR_HEADER_BYTES));
			   }
			   else
			   {
				   printf("\rTO %d", i);
				   
				   usart->message_status[i].trial_counter = 0;
				   usart->message_status[i].nbr_timeouts = 0;
				   usart->message_status[i].allocated = FALSE;
				   
				   //allokierten Speicher freigeben
				   cli();
				   free(usart->message_status[i].freeptr);
				   sei();
				   
				   usart->message_status[i].freeptr = NULL;
			   }
		   }
	   }
	   
	   //Timeout des Empfangspeichers kontrollieren
	   if(usart->receive_status[i].freeptr != NULL)
	   {
		   if(usart->receive_status[i].timeout_counter++ >= 3)
		   {
			   printf("\rD_N_F");
			   
			   cli();
			   free(usart->receive_status[i].freeptr);
			   sei();
			   
			   usart->receive_status[i].timeout_counter = 0;
			   usart->receive_status[i].freeptr = NULL;
		   }
	   }
   }
   
}
