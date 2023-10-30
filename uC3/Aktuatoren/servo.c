/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      SERVO.c
Version :  V 1.0
Date    :  10.12.2010
Author  :  ZAUNER MICHAEL

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

#define _SERVO_EXTERN

#include <avr/io.h>   
#include "multitask.h"
#include "servo.h"
#include "ports.h"
#include "global.h"
#include <util/delay.h>
#include "define.h" 
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "timer.h"
#include "rrt_receivedata.h"

/**************************************************************************
***   FUNKTIONNAME: InitServo                                           ***
***   FUNKTION: initialisiert die Servoansteuerung                      ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitServo(void)
{
	// zyklischer Task - Zykluszeit: 20 ms
	SET_CYCLE(SERVO_TASKNBR, 2000);
	SET_TASK(SERVO_TASKNBR, CYCLE);
	SET_TASK_HANDLE(SERVO_TASKNBR, ServoTask); 

	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######            P I N C E   N E Z    1             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_arm.minT = 0.0008;
	se_arm.maxT = 0.00215;
	se_arm.T = 0.02;
	se_arm.Ticks = 10000.0;
	se_arm.maxPhi = 135.0;
	se_arm.oldPhi = SE_ARM_0;
	se_arm.actualPhi = SE_ARM_0;
	se_arm.Phi = SE_ARM_0;
	se_arm.qTimer0 = &TCF0;
	se_arm.Channel = CH_A;
	se_arm.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_pince_nez1.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_arm.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second


	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######            P I N C E   N E Z    2             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_pince_nez2.minT = 0.0008;
	se_pince_nez2.maxT = 0.00215;
	se_pince_nez2.T = 0.02;
	se_pince_nez2.Ticks = 10000.0;
	se_pince_nez2.maxPhi = 135.0;
	se_pince_nez2.oldPhi = SE_PINCE_NEZ2_0;
	se_pince_nez2.actualPhi = SE_PINCE_NEZ2_0;
	se_pince_nez2.Phi = SE_PINCE_NEZ2_0;
	se_pince_nez2.qTimer0 = &TCF0;
	se_pince_nez2.Channel = CH_B;
	se_pince_nez2.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_pince_nez2.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_pince_nez2.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second


	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######            P I N C E   N E Z    3             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_pince_nez3.minT = 0.0008;
	se_pince_nez3.maxT = 0.00215;
	se_pince_nez3.T = 0.02;
	se_pince_nez3.Ticks = 10000.0;
	se_pince_nez3.maxPhi = 135.0;
	se_pince_nez3.oldPhi = SE_PINCE_NEZ3_0;
	se_pince_nez3.actualPhi = SE_PINCE_NEZ3_0;
	se_pince_nez3.Phi = SE_PINCE_NEZ3_0;
	se_pince_nez3.qTimer0 = &TCF0;
	se_pince_nez3.Channel = CH_C;
	se_pince_nez3.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_pince_nez3.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_pince_nez3.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second


	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######            P I N C E   N E Z    4             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_pince_nez4.minT = 0.0008;
	se_pince_nez4.maxT = 0.00215;
	se_pince_nez4.T = 0.02;
	se_pince_nez4.Ticks = 10000.0;
	se_pince_nez4.maxPhi = 135.0;
	se_pince_nez4.oldPhi = SE_PINCE_NEZ4_0;
	se_pince_nez4.actualPhi = SE_PINCE_NEZ4_0;
	se_pince_nez4.Phi = SE_PINCE_NEZ4_0;
	se_pince_nez4.qTimer0 = &TCF0;
	se_pince_nez4.Channel = CH_D;
	se_pince_nez4.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_pince_nez4.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_pince_nez4.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######            P I N C E   N E Z    5             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_pince_nez5.minT = 0.0008;
	se_pince_nez5.maxT = 0.00215;
	se_pince_nez5.T = 0.02;
	se_pince_nez5.Ticks = 10000.0;
	se_pince_nez5.maxPhi = 135.0;
	se_pince_nez5.oldPhi = SE_PINCE_NEZ5_0;
	se_pince_nez5.actualPhi = SE_PINCE_NEZ5_0;
	se_pince_nez5.Phi = SE_PINCE_NEZ5_0;
	se_pince_nez5.qTimer0 = &TCE0;
	se_pince_nez5.Channel = CH_A;
	se_pince_nez5.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_pince_nez5.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_pince_nez5.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

}

/**************************************************************************
***   FUNCTIONNAME:        ServoTask                                   ***
***   FUNCTION:            Servoansteuerung                            ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char ServoTask(void)
{
	SET_CYCLE(SERVO_TASKNBR, 20);
	SET_TASK(SERVO_TASKNBR, CYCLE);
   
	SetServo(&se_arm);
	SetServo(&se_pince_nez2);
	SetServo(&se_pince_nez3);
	SetServo(&se_pince_nez4);
	SetServo(&se_pince_nez5);
   
	return(CYCLE);
}

/**************************************************************************
***   FUNCTIONNAME:        SetServo                                     ***
***   FUNCTION:            setzt eine Servo                             ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   Pointer auf Servostruktur                    ***
**************************************************************************/
void SetServo(servo_t *servo)
{
	float tPwm;
	
	// ***************************************
	// given angle isn't reached (angle - given angle != 0)
	// => add (if actual angle is below given angle) or sub (if actual angle is above given angle)
	// ***************************************
	if (fabs(servo->oldPhi - servo->Phi) > 0.001)
	{
		servo->actualPhi += ((servo->actualPhi < servo->Phi) ? (servo->phiPerSec * servo->T) : -(servo->phiPerSec * servo->T));
	}
	
	// ***************************************
	// saturate angle
	// => if the angle comes from below the the given angle and is now bigger than this
	// => set the actual angle to the given angle
	// ***************************************
	if ((servo->actualPhi > servo->Phi) && ((servo->oldPhi - servo->Phi) < 0.0))
		servo->actualPhi = servo->Phi;
	
	// ***************************************
	// saturate angle
	// => if the angle comes from above the the given angle and is now smaller than this
	// => set the actual angle to the given angle
	// ***************************************
	if ((servo->actualPhi < servo->Phi) && ((servo->oldPhi - servo->Phi) > 0.0))
		servo->actualPhi = servo->Phi;
	
	// store the actual angle
	servo->oldPhi = servo->actualPhi;
	
	// calculate the needed period time
	tPwm = servo->minT + (servo->maxT - servo->minT) * servo->actualPhi / servo->maxPhi;
	
	// saturate the periode
	if(tPwm > servo->maxT)
		tPwm = servo->maxT;
	if(tPwm < servo->minT)
		tPwm = servo->minT;
	
	// calculate the PWM
	servo->Pwm = (unsigned int)(tPwm * servo->Ticks / servo->T);
	
	// set the PWM to the output
	if(servo->tmrNbr == 0)
		SetPWM0(servo->qTimer0, servo->Channel, servo->Pwm);
	else
		SetPWM1(servo->qTimer1, servo->Channel, servo->Pwm);
}