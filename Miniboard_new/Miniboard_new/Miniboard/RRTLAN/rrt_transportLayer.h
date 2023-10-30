/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER
Modul:     EVERYONE
File:      TRANSPORTLAYER.H
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


#ifndef RRTLAN_TRANSPORTLAYER_H
#define RRTLAN_TRANSPORTLAYER_H


#pragma used+     //warning entfällt

#ifndef RRTLAN_TRANSPORTLAYER_EXTERN
#define RRTLAN_TRANSPORTLAYER_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/


/**************************************************************************
***                          Prototypen-Definition                      ***
**************************************************************************/
void InitPortApp(void); 
void Port_App_allocation(uint8_t portnbr, uint8_t appnbr);
void Send_Transport_Data(USART_data_t* usart, uint8_t dest_portnbr, uint8_t* memptr);
uint8_t Receive_transport_Data(USART_data_t* usart);                   
#pragma used-


#endif


