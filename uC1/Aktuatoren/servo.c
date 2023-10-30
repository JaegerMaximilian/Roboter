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
#include "command.h"

/**************************************************************************
***   FUNKTIONNAME: InitServo                                           ***
***   FUNKTION: initialisiert die Servoansteuerung                      ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitServo(void)
{
	// zyklischer Task - Zykluszeit: 20 ms
	SET_CYCLE(SERVO_TASKNBR, 50);
	SET_TASK(SERVO_TASKNBR, CYCLE);
	SET_TASK_HANDLE(SERVO_TASKNBR, ServoTask); 

	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######                Servo vorne links              ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_VL_NBR].minT = 0.0008;
	se_array[SE_VL_NBR].maxT = 0.00215;
	se_array[SE_VL_NBR].T = 0.014;
	se_array[SE_VL_NBR].Ticks = 7142.0;
	se_array[SE_VL_NBR].maxPhi = 135.0;
	se_array[SE_VL_NBR].oldPhi = SE_VL;
	se_array[SE_VL_NBR].actualPhi = SE_VL;
	se_array[SE_VL_NBR].Phi = SE_VL;
	se_array[SE_VL_NBR].qTimer0 = &TCD0;
	se_array[SE_VL_NBR].Channel = CH_A;
	se_array[SE_VL_NBR].tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_res1.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_VL_NBR].phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_VL_NBR]));

	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######                Servo vorne rechts             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_VR_NBR].minT = 0.0008;
	se_array[SE_VR_NBR].maxT = 0.00215;
	se_array[SE_VR_NBR].T = 0.02;
	se_array[SE_VR_NBR].Ticks = 10000.0;
	se_array[SE_VR_NBR].maxPhi = 135.0;
	se_array[SE_VR_NBR].oldPhi	 = SE_VR;
	se_array[SE_VR_NBR].actualPhi = SE_VR;
	se_array[SE_VR_NBR].Phi		 = SE_VR;
	se_array[SE_VR_NBR].qTimer1 = &TCD1;
	se_array[SE_VR_NBR].Channel = CH_A;
	se_array[SE_VR_NBR].tmrNbr = 1;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_res2.phiPerSec.phiPerSec = 50.0;			// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_VR_NBR].phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_VR_NBR]));

	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######             Servo hinten		                ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_H_NBR].minT = 0.0008;
	se_array[SE_H_NBR].maxT = 0.00215;
	se_array[SE_H_NBR].T = 0.02;
	se_array[SE_H_NBR].Ticks = 10000.0;
	se_array[SE_H_NBR].maxPhi = 135.0;
	se_array[SE_H_NBR].oldPhi	= SE_HL; //CHECK_ROBOT_TYPE(SE_SILO_LEFT1_0,SE_S_SILO_LEFT1_0);
	se_array[SE_H_NBR].actualPhi = SE_HL; //CHECK_ROBOT_TYPE(SE_HL,SE_S_SILO_LEFT1_0);
	se_array[SE_H_NBR].Phi = SE_HL; //CHECK_ROBOT_TYPE(SE_HL,SE_S_SILO_LEFT1_0);
	se_array[SE_H_NBR].qTimer0 = &TCD0;
	se_array[SE_H_NBR].Channel = CH_B;
	se_array[SE_H_NBR].tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_siloLeft1.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_H_NBR].phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_H_NBR]));


	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######             Servo Kirsche1	                 ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_K1_NBR].minT = 0.0008;
	se_array[SE_K1_NBR].maxT = 0.00215;
	se_array[SE_K1_NBR].T = 0.02;
	se_array[SE_K1_NBR].Ticks = 10000.0;
	se_array[SE_K1_NBR].maxPhi = 135.0;
	se_array[SE_K1_NBR].oldPhi	= SE_HR;
	se_array[SE_K1_NBR].actualPhi = SE_HR;
	se_array[SE_K1_NBR].Phi = SE_HR;
	se_array[SE_K1_NBR].qTimer1 = &TCD1;
	se_array[SE_K1_NBR].Channel = CH_B;
	se_array[SE_K1_NBR].tmrNbr = 1;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_sileLeft2.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_K1_NBR].phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_K1_NBR]));


	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######            Servo Kirsche2 		             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_K2_NBR].minT = 0.0008;
	se_array[SE_K2_NBR].maxT = 0.00215;
	se_array[SE_K2_NBR].T =  0.02;
	se_array[SE_K2_NBR].Ticks = 10000.0;
	se_array[SE_K2_NBR].maxPhi = 135.0;
	se_array[SE_K2_NBR].oldPhi = 125; //CHECK_ROBOT_TYPE(SE_SILO_RIGHT1_0,SE_S_SILO_RIGHT1_0);
	se_array[SE_K2_NBR].actualPhi = 124; //CHECK_ROBOT_TYPE(SE_RES1,SE_S_SILO_RIGHT1_0);
	se_array[SE_K2_NBR].Phi = 130; //CHECK_ROBOT_TYPE(SE_RES1,SE_S_SILO_RIGHT1_0);
	se_array[SE_K2_NBR].qTimer0 = &TCD0;
	se_array[SE_K2_NBR].Channel = CH_C;
	se_array[SE_K2_NBR].tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_siloRight1.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_K2_NBR].phiPerSec = 60;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_K2_NBR]));


	//cmd_SetServo(SE_K2_NBR, 10);
	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######             Reserve 1 			             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_RES1_NBR].minT = 0.001;
	se_array[SE_RES1_NBR].maxT = 0.002;
	se_array[SE_RES1_NBR].T =  0.02;
	se_array[SE_RES1_NBR].Ticks = 10000.0;
	se_array[SE_RES1_NBR].maxPhi = 135.0;
	se_array[SE_RES1_NBR].oldPhi = 0; //CHECK_ROBOT_TYPE(SE_RES2,SE_S_SILO_RIGHT2_0);
	se_array[SE_RES1_NBR].actualPhi = 0; //CHECK_ROBOT_TYPE(SE_RES2,SE_S_SILO_RIGHT2_0);
	se_array[SE_RES1_NBR].Phi = 0; //CHECK_ROBOT_TYPE(SE_RES2,SE_S_SILO_RIGHT2_0);
	se_array[SE_RES1_NBR].qTimer0 = &TCD0;
	se_array[SE_RES1_NBR].Channel = CH_D;
	se_array[SE_RES1_NBR].tmrNbr = 0;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_siloRight2.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_RES1_NBR].phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_RES1_NBR]));


	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######            Reserve 2				             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_RES2_NBR].minT = 0.0008;
	se_array[SE_RES2_NBR].maxT = 0.00215;
	se_array[SE_RES2_NBR].T = 0.02;
	se_array[SE_RES2_NBR].Ticks = 10000.0;
	se_array[SE_RES2_NBR].maxPhi = 135.0;
	se_array[SE_RES2_NBR].oldPhi = SE_RES3; //CHECK_ROBOT_TYPE(SE_RES3,SE_S_SUCKER_RIGHT_0);
	se_array[SE_RES2_NBR].actualPhi = SE_RES3; //CHECK_ROBOT_TYPE(SE_RES3,SE_S_SUCKER_RIGHT_0);
	se_array[SE_RES2_NBR].Phi = SE_RES3; //CHECK_ROBOT_TYPE(SE_RES3,SE_S_SUCKER_RIGHT_0);
	se_array[SE_RES2_NBR].qTimer1 = &TCE1;
	se_array[SE_RES2_NBR].Channel = CH_B;
	se_array[SE_RES2_NBR].tmrNbr = 1;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_suckerRight.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_RES2_NBR].phiPerSec = WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_RES2_NBR]));


	// ############################################################
	// ############################################################
	// #######                                               ######
	// #######             Impeller 			             ######
	// #######                                               ######
	// ############################################################
	// ############################################################
	se_array[SE_IMP_NBR].minT = 0.001;
	se_array[SE_IMP_NBR].maxT = 0.002;
	se_array[SE_IMP_NBR].T = 0.02;
	se_array[SE_IMP_NBR].Ticks = 10000.0;
	se_array[SE_IMP_NBR].maxPhi = 135.0;
	se_array[SE_IMP_NBR].oldPhi = 0; //CHECK_ROBOT_TYPE(SE_RES4,SE_S_SUCKER_LEFT_0);
	se_array[SE_IMP_NBR].actualPhi = 0; //CHECK_ROBOT_TYPE(SE_RES4,SE_S_SUCKER_LEFT_0);
	se_array[SE_IMP_NBR].Phi = 0; //CHECK_ROBOT_TYPE(SE_RES4,SE_S_SUCKER_LEFT_0);
	se_array[SE_IMP_NBR].qTimer1 = &TCE1;
	se_array[SE_IMP_NBR].Channel = CH_A;
	se_array[SE_IMP_NBR].tmrNbr = 1;
	// 1.0 phiPerTick: 1.0 * 50.0Hz (1/T = 1/20ms) = 50.0
	// se_suckerLeft.phiPerSec = 50.0;					// -> change in angle -> 1.0° per tick (50.0° per second)
	se_array[SE_IMP_NBR].phiPerSec = 50;// WITH_OUT_TRAJECTORY;		// -> given angle is controlled in one second

	SetServo(&(se_array[SE_IMP_NBR]));

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
   
	SetServo(&se_array[SE_VL_NBR]);
	SetServo(&se_array[SE_VR_NBR]);
	SetServo(&se_array[SE_H_NBR]);
	SetServo(&se_array[SE_K1_NBR]);
	SetServo(&se_array[SE_K2_NBR]);
	SetServo(&se_array[SE_IMP_NBR]);
	SetServo(&se_array[SE_RES1_NBR]);
	SetServo(&se_array[SE_RES2_NBR]);
	
   
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