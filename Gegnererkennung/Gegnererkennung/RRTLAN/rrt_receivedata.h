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


#ifndef RRTLAN_RECEIVEDATA_EXTERN
#define RRTLAN_RECEIVEDATA_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
// *************************************
// Motion Status
// *************************************
#define MOTION_OK             1
#define MOTION_ERROR_IR       2
#define MOTION_ERROR_SCHLEPP  3

#define GEGNER_ON          1
#define GEGNER_OFF         2 

/**************************************************************************
***                          Prototypen-Definition                      ***
**************************************************************************/
void InitReceiveData(void);
uint8_t rrtlanRoboPos_Task(void);


#endif


