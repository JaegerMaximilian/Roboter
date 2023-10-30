/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      RECEIVETASK.c
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

#define RRTLAN_RECEIVETASK_EXTERN

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "rrt_serialconfig.h"
#include "rrt_dataLinkLayer.h"
#include "global.h"
#include "rrt_receivetask.h"
#include "multitask.h"
#include "define.h"

/**************************************************************************
***   FUNKTIONNAME: InitReceivetask                                     ***
***   FUNKTION: initialisiert den Empfängertask                         ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitReceivetask(void)
{
   SET_CYCLE(SERIAL_RECEIVE_TASKNBR, 1);
   SET_TASK(SERIAL_RECEIVE_TASKNBR, CYCLE);
   SET_TASK_HANDLE(SERIAL_RECEIVE_TASKNBR, ReceiveTask); 
}

/**************************************************************************
***   FUNCTIONNAME:        ReceiveTask                                  ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char ReceiveTask(void)
{  
 uint8_t message_nbr;

 #ifdef _SERIAL_INTERFACE_C0 
  
  message_nbr = 0;
  
  while((USART_RXBufferData_Available(&USART_data_C0)) && (message_nbr < NUM_MESSAGES_PER_INTERFACE))
   { 
     Receive_DataLink_Data(&USART_data_C0);
     message_nbr++;
   } 
      
 #endif
 
 #ifdef _SERIAL_INTERFACE_C1
  
  message_nbr = 0;
  
  while((USART_RXBufferData_Available(&USART_data_C1)) && (message_nbr < NUM_MESSAGES_PER_INTERFACE))
   { 
     Receive_DataLink_Data(&USART_data_C1);
     message_nbr++;
   } 
      
 #endif 

 #ifdef _SERIAL_INTERFACE_D0  
   
   message_nbr = 0;
  
  while((USART_RXBufferData_Available(&USART_data_C0)) && (message_nbr < NUM_MESSAGES_PER_INTERFACE))
   { 
     Receive_DataLink_Data(&USART_data_C0);
     message_nbr++;
   }
        
 #endif  
 
 #ifdef _SERIAL_INTERFACE_D1
   
   message_nbr = 0;
  
   while((USART_RXBufferData_Available(&USART_data_D1)) && (message_nbr < NUM_MESSAGES_PER_INTERFACE))
   { 
     Receive_DataLink_Data(&USART_data_D1);
     message_nbr++;
   } 
     
 #endif 
 
 #ifdef _SERIAL_INTERFACE_E0 
   
   message_nbr = 0;
  
   while((USART_RXBufferData_Available(&USART_data_E0)) && (message_nbr < NUM_MESSAGES_PER_INTERFACE))
   { 
     Receive_DataLink_Data(&USART_data_E0);
     message_nbr++;
   } 
   
     
 #endif 
 
 #ifdef _SERIAL_INTERFACE_E1
   
   message_nbr = 0;
  
   while((USART_RXBufferData_Available(&USART_data_E1)) && (message_nbr < NUM_MESSAGES_PER_INTERFACE))
   { 
     Receive_DataLink_Data(&USART_data_E1);
     message_nbr++;
   } 
     
 #endif  
 
 #ifdef _SERIAL_INTERFACE_F0 
   
   message_nbr = 0;
  
   while((USART_RXBufferData_Available(&USART_data_F0)) && (message_nbr < NUM_MESSAGES_PER_INTERFACE))
   { 
     Receive_DataLink_Data(&USART_data_F0);
     message_nbr++;
   }   
     
 #endif
      
 return(ENABLE);
}