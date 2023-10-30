/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER
Modul:     EVERYONE
File:      DATALINKLAYER.H
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


#ifndef RRTLAN_DATALINKLAYER_H
#define RRTLAN_DATALINKLAYER_H

#include "rrt_serialconfig.h" 

#pragma used+     //warning entfällt

#ifndef RRTLAN_DATALINKLAYER_EXTERN
   #define RRTLAN_DATALINKLAYER_EXTERN extern
#endif

//Receiver states
#define _FIND_STX_I              0
#define _FIND_STX_II             1
#define _WAIT_FOR_FIRST_BLOCK    2
#define _MEMORY_ALLOC            3
#define _RECEIVE_DATA            4
#define _FIND_ETX_I              5
#define _FIND_ETX_II             6
#define _CHECK_SUM               7
#define _MESSAGE_TYPE            8
#define _TRANSFER_DATA           9

#define _MESSAGE_TYPE_DATA       8
#define _MESSAGE_TYPE_ACK        16

/**************************************************************************
***                          Prototypen-Definition                      ***
**************************************************************************/
void InitTransmissionStatus(USART_data_t* usart);
void Receive_DataLink_Data(USART_data_t* usart);
void Send_DataLink_Data(USART_data_t* usart, uint8_t* memptr);
void Send_DataLink_ACK(USART_data_t* usart);
void Check_transmission_status(USART_data_t* usart);
void BuildCRC16(uint8_t buff_length, uint8_t* memptr);
void crcInit(void);
crc crc_calc(uint8_t* memptr,unsigned int nBytes);

#pragma used-

#endif



