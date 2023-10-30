/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      SENSOR.H
Version :  V 1.0
Date    :  13.12.2010
Author  :  MICHAEL ZAUNER

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : XMega256A3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024

                 Copyright (c) 2010 by FH-Wels
                     All Rights Reserved.  
****************************************************************/


#ifndef _SENSOR_H
#define _SENSOR_H

#ifndef _SENSOR_EXTERN
   #define _SENSOR_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
_SENSOR_EXTERN uint8_t uC1sensorenPortA, uC1sensorenPortF;
_SENSOR_EXTERN uint8_t uC2sensorenPortB;
_SENSOR_EXTERN uint8_t uC3sensorenPortB, uC3sensorenPortC;
_SENSOR_EXTERN uint16_t usRearRight, usRearLeft, usFrontLeft, usFrontRight;
_SENSOR_EXTERN uint16_t usRightRear, usLeftRear, usLeftFront, usRightFront;
_SENSOR_EXTERN float posLiftRear;

_SENSOR_EXTERN uint8_t irVL1, irVL2,irVL3,irVR1,irVR2,irVR3,refVL,refVR;
/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void InitSensor(void);
unsigned char SensorTask(void);
                    

#endif


