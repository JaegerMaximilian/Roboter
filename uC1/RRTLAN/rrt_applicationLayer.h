/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER
Modul:     EVERYONE
File:      APPLICATIONLAYER.H
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


#ifndef RRTLAN_APPLICATIONLAYER_H
#define RRTLAN_APPLICATIONLAYER_H


#ifndef RRTLAN_APPLICATIONLAYER_EXTERN
#define RRTLAN_APPLICATIONLAYER_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
#define NBR_HEADER_BYTES    10 

RRTLAN_APPLICATIONLAYER_EXTERN port_app_alloc_t port_alloc[NUM_MESSAGE_STATUS];

/**************************************************************************
***                          Prototypen-Definition                      ***
**************************************************************************/
uint8_t Send_Application_Data(USART_data_t* usart, uint8_t dest_portnbr, uint8_t* first_data_byte, uint8_t nbr_of_bytes);
void Receive_Application_Data(USART_data_t* usart, uint8_t local_portnbr, uint8_t* receiveArray);
uint8_t Received_AppData_Available(USART_data_t* usart, uint8_t local_portnbr, uint8_t* nbr_of_bytes);
                    


#endif


