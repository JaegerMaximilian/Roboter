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
***                        Variablen-Definition                               ***
**************************************************************************/
RRTLAN_RECEIVEDATA_EXTERN float winkel_x, winkel_y;
RRTLAN_RECEIVEDATA_EXTERN int16_t xPos, yPos, phiPos, vIst;
RRTLAN_RECEIVEDATA_EXTERN uint8_t statusAntrieb;
RRTLAN_RECEIVEDATA_EXTERN uint8_t SpielFarbe, Strategie;

RRTLAN_RECEIVEDATA_EXTERN uint16_t rearpressureleft;
RRTLAN_RECEIVEDATA_EXTERN uint16_t rearpressureright;
RRTLAN_RECEIVEDATA_EXTERN float liftposrearleft;
RRTLAN_RECEIVEDATA_EXTERN float liftposrearright;
RRTLAN_RECEIVEDATA_EXTERN uint16_t galh;
RRTLAN_RECEIVEDATA_EXTERN uint16_t galv;

/**************************************************************************
***                          Prototypen-Definition                           ***
**************************************************************************/
void InitReceiveData(void);
unsigned char rrtlanDebugMsg_uC2Task(void);
unsigned char rrtlanSensor_uC2Task(void);
unsigned char rrtlanDebugMsg_GegnerTask(void);
unsigned char rrtlanEnemyPos_GegnerTask(void);
unsigned char rrtlanAntrieb_uC2Task(void);
unsigned char rrtlanPos_uC2Task(void);
unsigned char rrtlanDebugMsg_LCDTask(void);
unsigned char rrtlanEinstellung_LCDTask(void);
unsigned char rrtlan_WIFI_Mosi_Task(void);
uint8_t rrtlanEnemyData_to_Pathplaner_Task();
uint8_t rrtlanEnemyDataRaw_to_WIFI_Task();

#endif


