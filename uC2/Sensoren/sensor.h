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
_SENSOR_EXTERN uint8_t uC2sensorenPortF;

/* ******************** */
/* digital sensors µC 2 */
/* ******************** */
/* reflex sensor */
#define REFLEX_1			((uC2sensorenPortB & 0x10) ? 1 : 0)
#define REFLEX_2			((uC2sensorenPortB & 0x20) ? 1 : 0)
#define REFLEX_3			((uC2sensorenPortB & 0x40) ? 1 : 0)
#define REFLEX_4			((uC2sensorenPortB & 0x80) ? 1 : 0)
#define REFLEX_TOTAL		((0x01 * REFLEX_1) + (0x02 * REFLEX_2) + (0x04 * REFLEX_3) + (0x08 * REFLEX_4))



/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void InitSensor(void);
unsigned char SensorTask(void);
                    

#endif


