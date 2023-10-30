/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      TIMEOUTMANAGER.H
Version :  V 1.0
Date    :  25.02.2008
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


#ifndef RRTLAN_TIMEOUTMANAGER_H
#define RRTLAN_TIMEOUTMANAGER_H

#include "rrt_serialconfig.h" 

#pragma used+     //warning entfällt

#ifndef RRTLAN_TIMEOUTMANAGER_EXTERN
#define RRTLAN_TIMEOUTMANAGER_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
 

/**************************************************************************
***                          Prototypen-Definition                      ***
**************************************************************************/
void InitTimeoutManager(void);
void Check_Timeout(USART_data_t* usart);
unsigned char TimeoutManager(void);
                    
#pragma used-


#endif


