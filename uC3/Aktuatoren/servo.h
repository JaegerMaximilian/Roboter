/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      SERVO.H
Version :  V 1.0
Date    :  10.12.2010
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


#ifndef _SERVO_H
#define _SERVO_H


#include <avr/io.h>


#ifndef _SERVO_EXTERN
   #define _SERVO_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
// without trajectory -> 10000° per sec => given angle is controlled in one second
#define WITH_OUT_TRAJECTORY		10000.0

struct Servo
{
   float minT;                    // minimale Pulsweite (in s)
   float maxT;                    // maximale Pulsweite (in s)
   float T;                       // Periodendauer 
   float Ticks;                   // Timer Ticks für die Periodendauer
   float maxPhi;                  // maximaler Stellwinkel (in °)
   float Phi;                     // Stellwinkel
   uint8_t tmrNbr;
   TC0_t * qTimer0;                // verwendeter Timer
   TC1_t * qTimer1;
   unsigned char Channel;         // PWM-Channel
   unsigned int Pwm;              // PWM-Timerwert
	float phiPerSec;					// change in [°] per second, e.g. 1° per 20 ms
	float actualPhi;					// actual calculated angle tn
	float oldPhi;						// old angle -> from tn-1
};

typedef struct Servo servo_t; 

_SERVO_EXTERN servo_t se_arm, se_pince_nez2, se_pince_nez3, se_pince_nez4, se_pince_nez5;

/* servo positions */
#define SE_ARM_0			5
#define SE_PINCE_NEZ2_0		10
#define SE_PINCE_NEZ3_0		60 // 55
#define SE_PINCE_NEZ4_0		30 // 25
#define SE_PINCE_NEZ5_0		40 // 35

// #define SE_PINCE_NEZ1_0			67
// #define SE_PINCE_NEZ2_0			67
// #define SE_PINCE_NEZ3_0			67
// #define SE_PINCE_NEZ4_0			67
// #define SE_PINCE_NEZ5_0			67


/* servo number */
#define SE_ARM_FRONTLEFT_NBR		8
#define SE_ARM_FRONTRIGHT_NBR		9
#define SE_ARM_REARLEFT_NBR			10
#define SE_ARM_REARRIGHT_NBR		11
/*#define SE_PINCE_NEZ5_NBR			15*/

/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void InitServo(void);
unsigned char ServoTask(void);
void SetServo(servo_t *servo);

#endif


