/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      TRANSMITTINGTASK.H
Version :  V 1.0
Date    :  25.02.2008
Author  :  MICHAEL ZAUNER

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


#ifndef RRTLAN_RECEIVEDATA_H
#define RRTLAN_RECEIVEDATA_H

#pragma used+     //warning entfällt

#ifndef RRTLAN_RECEIVEDATA_EXTERN
#define RRTLAN_RECEIVEDATA_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                               ***
**************************************************************************/
 

/**************************************************************************
***                          Prototypen-Definition                           ***
**************************************************************************/
void InitReceiveData(void);
uint8_t VelMsgTask(void);
uint8_t PosMsgTask(void);
uint8_t ParameterMsgTask(void);
unsigned char ReceiveDataTask3(void);
unsigned char ReceiveDataTask4(void);
unsigned char ReceiveDataTask5(void);
unsigned char ReceiveDataTask6(void);
unsigned char ReceiveDataTask7(void);
unsigned char ReceiveDataTask8(void);
                    
#pragma used-


#endif


