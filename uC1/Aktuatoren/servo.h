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

//_SERVO_EXTERN servo_t se_res1, se_res2;
//_SERVO_EXTERN servo_t se_siloLeft1, se_sileLeft2, se_siloRight1, se_siloRight2;
//_SERVO_EXTERN servo_t se_suckerLeft, se_suckerRight;

_SERVO_EXTERN servo_t se_array[8];

/* number of servos from µC1 */
#define SE_MAX_uC1_SERVOS			8

/* offset of servos from µC2 */
#define SE_uC2_OFFSET				SE_MAX_uC1_SERVOS
#define SE_MAX_uC2_SERVOS			(0 + SE_uC2_OFFSET)

/* offset of servos from µC2 */
#define SE_uC3_OFFSET				(SE_MAX_uC2_SERVOS)
#define SE_MAX_uC3_SERVOS			(4 + SE_uC3_OFFSET)

/* number for internal servos - µC1 */
#define SE_VL_NBR	0
#define SE_VR_NBR	1
#define SE_H_NBR	2
#define SE_K1_NBR	3
#define SE_K2_NBR	4
#define SE_IMP_NBR	6
#define SE_RES1_NBR	5
#define SE_RES2_NBR	7

// /* servo number µC 2 */
// #define SE_PINCE_NEZ1_NBR	(0 + SE_uC2_OFFSET)
// #define SE_FLAG1_NBR		(1 + SE_uC2_OFFSET)
// #define SE_FLAG2_NBR		(2 + SE_uC2_OFFSET)
// 

 /* servo number µC 3 */
 //#define SE_ARM_FRONTLEFT_NBR			(0 + SE_uC3_OFFSET)
 //#define SE_ARM_FRONTRIGHT_NBR			(1 + SE_uC3_OFFSET)
 //#define SE_ARM_REARLEFT_NBR			(2 + SE_uC3_OFFSET)
 //#define SE_ARM_REARRIGHT_NBR			(3 + SE_uC3_OFFSET)
/* #define SE_PINCE_NEZ5_NBR			(4 + SE_uC3_OFFSET)*/

/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void InitServo(void);
unsigned char ServoTask(void);
void SetServo(servo_t *servo);

#endif


