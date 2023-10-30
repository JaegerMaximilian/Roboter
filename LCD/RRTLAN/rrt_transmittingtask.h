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


#ifndef RRTLAN_TRANSMITTINGTASK_H
#define RRTLAN_TRANSMITTINGTASK_H

#include "rrt_usart_driver.h"

#ifndef RRTLAN_TRANSMITTINGTASK_EXTERN
	#define RRTLAN_TRANSMITTINGTASK_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                               ***
**************************************************************************/
 

/**************************************************************************
***                          Prototypen-Definition                           ***
**************************************************************************/
void InitTransmit(void);
uint8_t TransmitTask1(void);
void debugMsg(char *text);


#endif


