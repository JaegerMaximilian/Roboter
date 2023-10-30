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
_SENSOR_EXTERN uint8_t uC3sensorenPortB, uC3sensorenPortC;
_SENSOR_EXTERN uint8_t uC3sensorenPortF;
_SENSOR_EXTERN uint16_t Gal_V, Gal_H;

/* ******************** */
/* digital sensors µC 3 */
/* ******************** */
/* lift rear: IR and colour front */
#define COLOUR_REAR_1		((uC3sensorenPortB & 0x20) ? 1 : 0)
#define COLOUR_REAR_2		((uC3sensorenPortB & 0x40) ? 1 : 0)
#define IR_REAR_1			((uC3sensorenPortB & 0x01) ? 1 : 0)
#define IR_REAR_2			((uC3sensorenPortB & 0x02) ? 1 : 0)
#define IR_REAR_3			((uC3sensorenPortB & 0x04) ? 1 : 0)
#define IR_REAR_4			((uC3sensorenPortB & 0x08) ? 1 : 0)
#define IR_REAR_5			((uC3sensorenPortB & 0x10) ? 1 : 0)
/* reserve */
#define IR_RESERVE_1		((uC3sensorenPortC & 0x10) ? 1 : 0)
#define IR_RESERVE_2		((uC3sensorenPortC & 0x20) ? 1 : 0)
#define IR_RESERVE_3		((uC3sensorenPortC & 0x40) ? 1 : 0)
#define IR_RESERVE_4		((uC3sensorenPortC & 0x80) ? 1 : 0)



/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void InitSensor(void);
unsigned char SensorTask(void);
                    

#endif


