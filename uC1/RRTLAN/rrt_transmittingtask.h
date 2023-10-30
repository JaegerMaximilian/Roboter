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
#include "ki.h"
#include "wifi.h"

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
void debugMsg(char *text);
void setServo_RRTLAN(USART_data_t *controller, uint8_t nbrServo, int16_t winkelServo);
void setDigitalOut_RRTLAN(USART_data_t *controller, uint8_t nbrDiOut, uint8_t value);
void VelocityCommand_RRTLAN(USART_data_t *controller, uint8_t nbr, float vel);
void PositionCommand_RRTLAN(USART_data_t *controller, uint8_t nbr, float vel, float s0);
void VoltageCommand_RRTLAN(USART_data_t *controller, uint8_t port, float vol, uint8_t nbrMot);
void setAntrieb_RRTLAN(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint16_t rSoll, uint16_t xSoll, uint16_t ySoll, uint16_t sSoll, uint8_t type, uint8_t gegnerErkennung);
void setAntriebClothoid_RRTLAN(int16_t vMax, element_t *wP, uint8_t nOP);
void setPosition_RRTLAN(uint16_t x0, uint16_t y0, uint16_t phi0);
void setDataLCD_RRTLAN(uint8_t points, uint8_t time);
void setDataWIFI_RRTLAN(uint8_t eP);
void sendPosDataEnemy_from_WIFI_RRLAN(uint16_t xPos, uint16_t yPos, uint16_t phiPos, EnemyDataRaw_t *enemy);
void startLiftInit_RRTLAN(USART_data_t *controller);
void setSchleppfehler_RRLAN(uint8_t schleppdistance);
void sendPos2EnemyDetection_RRLAN();
 
#endif


