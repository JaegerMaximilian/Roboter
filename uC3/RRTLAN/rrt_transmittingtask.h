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
unsigned char TransmitTask1(void);
unsigned char TransmitTask2(void);
void debugMsg(char *text);
void sendSensors(uint16_t pressureleft, uint16_t pressureright, float posleft, float posright, uint16_t galh, uint16_t galv);
void sendEnemyData_to_Pathplaner_RRTLAN();
void sendEnemyDataRaw_to_WIFI_RRTLAN();

#endif


