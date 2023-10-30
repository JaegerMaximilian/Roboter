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
	// #######                     A R M                     ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_pince_nez1.minT = 0.0008;
	se_pince_nez1.maxT = 0.00215;
	se_pince_nez1.T = 0.02;
	se_pince_nez1.Ticks = 10000.0;
	se_pince_nez1.maxPhi = 135.0;
	se_pince_nez1.oldPhi = SE_PINCE_NEZ1_0;
	se_pince_nez1.actualPhi = SE_PINCE_NEZ1_0;
	se_pince_nez1.Phi = SE_PINCE_NEZ1_0;
	se_pince_nez1.qTimer0 = &TCC0;
	se_pince_nez1.Channel = CH_A;
	se_pince_nez1.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_arm.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_pince_nez1.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######                   F L A G   1                 ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_flag1.minT = 0.0008;
	se_flag1.maxT = 0.00215;
	se_flag1.T = 0.02;
	se_flag1.Ticks = 10000.0;
	se_flag1.maxPhi = 135.0;
	se_flag1.oldPhi	 = SE_FLAG1_0;
	se_flag1.actualPhi = SE_FLAG1_0;
	se_flag1.Phi = SE_FLAG1_0;
	se_flag1.qTimer0 = &TCC0;
	se_flag1.Channel = CH_B;
	se_flag1.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	se_flag1.phiPerSec = 100.0;			// -> change in angle -> 1.0° per tick (50.0° per second)
	//se_flag1.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######                   F L A G   2                 ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_flag2.minT = 0.0008;
	se_flag2.maxT = 0.00215;
	se_flag2.T = 0.02;
	se_flag2.Ticks = 10000.0;
	se_flag2.maxPhi = 135.0;
	se_flag2.oldPhi	= SE_FLAG2_0;
	se_flag2.actualPhi = SE_FLAG2_0;
	se_flag2.Phi = SE_FLAG2_0;
	se_flag2.qTimer0 = &TCC0;
	se_flag2.Channel = CH_C;
	se_flag2.tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_flag2.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_flag2.phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

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
   
	SetServo(&se_pince_nez1);
	SetServo(&se_flag1);
	SetServo(&se_flag2);
   
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