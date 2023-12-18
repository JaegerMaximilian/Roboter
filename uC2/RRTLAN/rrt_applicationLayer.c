/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      APPLICATIONLAYER.c
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

#define RRTLAN_APPLICATIONLAYER_EXTERN
                                                               
#include "rrt_usart_driver.h"
#include "rrt_applicationLayer.h"
#include "rrt_serialconfig.h" 
#include "rrt_transportLayer.h" 
#include "define.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>


extern port_app_alloc_t port_alloc[NUM_MESSAGE_STATUS];

/**************************************************************************
***   FUNCTIONNAME:        Send_Application_Data                        ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t Send_Application_Data(USART_data_t* usart, uint8_t dest_portnbr, uint8_t* first_data_byte, uint8_t nbr_of_bytes)
{
   uint8_t* memptr;
   uint8_t i;
   uint8_t ok = FALSE;
     
   //ID zuweisen, wenn noch eine frei ist
   for(i = 0; i < NUM_MESSAGE_STATUS; i++)
   { 
     if(!(usart->message_status[i].allocated))
      {
        usart->unique_ID = i;
        ok = TRUE;
        break;
      }
   }
   
   //Wenn eine unique ID vergeben werden kann,...
   if(ok)
    { 
      //Sendespeicher allokieren 
      cli();
      memptr = (uint8_t*) malloc((unsigned int)((unsigned int)nbr_of_bytes + NBR_HEADER_BYTES)); 
      sei(); 

      if(memptr != NULL)
        { 
          //Anzahl der Nutzdaten in die Message eintragen 
          memptr += FIRST_BLOCK + 1;
         *memptr = nbr_of_bytes;
          memptr++;
          
          //Nutzdaten in die Message eintragen
          for(i = 0; i < nbr_of_bytes; i++)
           {
             *memptr = *first_data_byte;
              memptr++;
              first_data_byte++;
           }

          memptr -= (uint8_t)((unsigned int)nbr_of_bytes+1); 
          
          //Message an den TransportLayer übergeben
          Send_Transport_Data(usart, dest_portnbr, memptr);
        }
        else
        { 
          printf("MEMFULL");
          return(FALSE);
        } 
    } 
    else
    { 
      return(FALSE);
    }
	
	return (ok);
}

/**************************************************************************
***   FUNCTIONNAME:        Receive_Application_Data                     ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Receive_Application_Data(USART_data_t* usart, uint8_t local_portnbr, uint8_t* receiveArray)
{
  uint8_t i,j,nbr_bytes;  
  
  for(i = 0; i < NUM_MESSAGE_STATUS; i++)
    {
       if(local_portnbr == usart->receive_status[i].portnbr)
         { 
             nbr_bytes = *usart->receive_status[i].memptr;  
             
             //Nutzdaten auslesen
             for(j=0; j < nbr_bytes; j++)
              {  
               usart->receive_status[i].memptr++;
               *receiveArray = *usart->receive_status[i].memptr; 
               receiveArray++;
              }
              
             //allokierten Empfangsspeicher freigeben  
             cli();
             if(usart->receive_status[i].freeptr != NULL)
			 {
				 free(usart->receive_status[i].freeptr); 
			 } 
             sei();
             
             //Speicher initialisieren, damit dieser wieder belegt werden darf
             usart->receive_status[i].memptr = NULL;
             usart->receive_status[i].freeptr = NULL;
             usart->receive_status[i].portnbr = 0; 
             usart->receive_status[i].timeout_counter = 0; 
             
             break; 
         }
    }   
} 

/**************************************************************************
***   FUNCTIONNAME:        Received_AppData_Available                   ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t Received_AppData_Available(USART_data_t* usart, uint8_t local_portnbr, uint8_t* nbr_of_bytes)
{
  uint8_t i;
  
  for(i = 0; i < NUM_MESSAGE_STATUS; i++)
   { 
     //Wenn die Portnummer dem Task zugewiesen ist und Daten empfangen wurden
     if(local_portnbr == usart->receive_status[i].portnbr)
      {
        *nbr_of_bytes = *usart->receive_status[i].memptr;
        return(TRUE);
      }
   }
   
   return(FALSE);
}

