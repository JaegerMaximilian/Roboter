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
 

/**************************************************************************
***                          Prototypen-Definition                      ***
**************************************************************************/
void InitReceiveData();
uint8_t rrtlanLift_Task();
uint8_t rrtlanServo_Task();
uint8_t rrtlanDigiOut_Task();
uint8_t rrtlanEnemyDataRaw_from_WIFI_Task();
uint8_t rrtlanInitLift_Task();
unsigned char rrtlanMotorPosition_Task();
#endif


