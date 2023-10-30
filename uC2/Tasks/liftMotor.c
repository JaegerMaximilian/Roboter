//* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      motor driver source file.
 *
 *      This file contains the function to deal with up to two motors
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. 
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *
 * \par Schematic Miniboard V3.0:
 *      Hardware details are shown in the schematic Miniboard V3.0, page 1, sector B6/7 and page 2, sector A/B 1-4
 *
 * \par Documentation
 *      The file provide functions to control motors 
 *
 * \author
 *      Michael Zauner
 *      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
 *      Support email: roboracing@fh-wels.at
 *
 * $Revision: 1 $
 * $Date: 2012-08-23  $  \n
 *
 * Copyright (c) 2012, RRT (University of Applied Sciences Upper Austria) All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of RRT may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RRT (University of Applied Sciences Upper Austria) 
 * "AS IS" AND ANY EXPRESS OR IMPLIED  * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/ 

#define _LIFT_MOTOR_EXTERN


#include <avr/io.h>   
#include "multitask.h"
#include "motor.h"
#include "define.h" 
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "global.h"
#include "qdec_driver.h"
#include "timer.h"
#include "usart.h"
#include "accumulator.h"
#include "adc_driver.h"
#include "liftMotor.h"
#include "anaPos.h"



//#define  _DEBUG_MOTOR_DATA_

/* ************************************************************** */
/*! \brief Initialize the motor control.
 *
 *  Motor control initialization function.
 *  Sets up the ports, timers, qdecs, limits, odometry and controllers 
 *
 */
/* ************************************************************** */
void liftMotor_init(uint8_t pos_enable, float pos)
{
	/* cyclic task - cycle time: 10 ms */
	SET_CYCLE(LIFT_MOTOR_TASKNBR, 10);
	SET_TASK(LIFT_MOTOR_TASKNBR, CYCLE);
	SET_TASK_HANDLE(LIFT_MOTOR_TASKNBR, liftMotorTask);
   

	/* ############################################################
	* ############################################################
	* #######                                               ######
	* #######             M O T O R   R E A R               ######
	* #######                                               ######
	* ############################################################
	* ############################################################ */
	/* ********************************************* 
	* common settings for trajectory planning 
	* ********************************************* */
	//float pos = 0.0;
	//for (uint8_t i = 0; i < 100; i++)
	//{
		//processAnaPos(&posRear);
		//pos += posRear.pos;
	//}
	//
	//pos /= 100.0;
	
	liftRear.Traj.Xs = pos;												/* start value of the motor [m] */
	liftRear.Traj.Vstart = 0.0;										/* start velocity of the trajectory ramp [m/s] */
	liftRear.Traj.Vend = 0.0;											/* end velocity of the trajectory ramp [m/s] */
	liftRear.Traj.Ramp = SIN_2_RAMP;									/* ramp type -> sin^2 or linear */
	liftRear.Traj.AmaxUs = 1.5;										/* maximum acceleration at up slope [m^2/s] */
	liftRear.Traj.AmaxDs = 1.5;										/* maximum acceleration at down slope [m^2/s] */
	liftRear.Traj.State = _MOTION_READY;							/* reset motion status */

   
	/* ********************************************* 
	* odometry  
	* ********************************************* */
	liftRear.Odo.IncsPerRot = 12.0;									/* set increments per rotation */
	liftRear.Odo.Gear = 74.83;									/* set gear: 19:1 -> motor, 30:15 -> bevel gear */
	liftRear.Odo.DisPerRot = 0.02 * M_PI;							/* set driven distance per rotation [m] */
	liftRear.Odo.Dis = liftRear.Traj.Xs;								/* set start position in actual state -> see common settings */ 
	liftRear.Odo.Dis_1 = liftRear.Traj.Xs;								/* set start position in last state -> see common settings */   
	liftRear.Odo.IncTotal = (liftRear.Traj.Xs *						/* set total increments for start position */
							liftRear.Odo.Gear * 
							liftRear.Odo.IncsPerRot * 
							4.0) / liftRear.Odo.DisPerRot; 
	liftRear.Odo.invDir = true;										/* input value negate or not */ 


	/* ********************************************* 
	* QDEC  
	* ********************************************* */
	liftRear.Qdec.qPort = &PORTE;										/* port of the QDEC channel */
	liftRear.Qdec.qPin = 6;												/* A pin of the QDEC channel */
	liftRear.Qdec.invIO = false;										/* input value negate or not */
	liftRear.Qdec.qEvMux = 4;											/* event channel multiplex channel */
	liftRear.Qdec.qPinInput = EVSYS_CHMUX_PORTE_PIN6_gc;		/* set input for event system */
	liftRear.Qdec.useIndex = false;									/* use of index pin enable/disable */
	liftRear.Qdec.qIndexState = EVSYS_QDIRM_00_gc;				/* initialize event channel as QDEC */
	liftRear.Qdec.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
	liftRear.Qdec.qTimer0 = &TCE0;									/* set timer */
	liftRear.Qdec.qEventChannel = TC_EVSEL_CH4_gc;				/* set event channel */
	liftRear.Qdec.lineCount = (uint16_t) (liftRear.Odo.IncsPerRot *			/* set line counter */ 
							liftRear.Odo.Gear);	
	if(liftRear.Qdec.lineCount > 16383)
	{
		liftRear.Qdec.encDivider = 4;	
		liftRear.Qdec.lineCount /= liftRear.Qdec.encDivider;							
	}	
	else
	{
		liftRear.Qdec.encDivider = 1;	
	}		
	InitQdec(&liftRear);   

 
	/* ********************************************* 
	* PWM 
	* ********************************************* */
	PORTE.DIR |= 0x30;													/* initialize RIN pin -> set DIR to output */
	PORTE.OUTCLR = 0x30;													/* initialize RIN pin -> set OUT to zero -> OUT2 = 1 */
	liftRear.Pwm.Driver = BD6232HFP;								/* set driver type */
	liftRear.Pwm.qPort = &PORTE;										/* set PWM port */
	liftRear.Pwm.qPin = 4;												/* set PWM pin */
	liftRear.Pwm.nbrTimer = 1;										/* set timer number -> timer0 or timer1 */
	liftRear.Pwm.qTimer1 = &TCE1;									/* set PWM timer */
	liftRear.Pwm.Channel = CH_A;										/* set PWM channel */
	liftRear.Pwm.Frequence = 20000;									/* set PWM frequency [Hz] */
	liftRear.Pwm.invU = 1.0;											/* set voltage multiplier -> 1.0 ... not negated, -1.0 ... negated */
	liftRear.Pwm.qDIRport = &PORTE;									/* set FIN port */
	liftRear.Pwm.qDIRpin = 5;

	InitPwm(&liftRear);   


	/* ********************************************* 
	* velocity control
	* ********************************************* */
	liftRear.Vel.controlOn = 1;										/* velocity control on(1)/off(0) */
	liftRear.Vel.Kp = 100;												/* set Kp vel = 15*/
	liftRear.Vel.Ki = 300;									      /* set Ki vel = 250*/
	liftRear.Vel.Sat_min = -20.0;										/* set minimum saturation -> negativ accumulator voltage */
	liftRear.Vel.Sat_max = 20.0;										/* set maximum saturation -> positiv accumulator voltage */

	/* ********************************************* 
	* position control
	* ********************************************* */
	liftRear.Pos.controlOn = pos_enable;										/* position control on(1)/off(0) */
	liftRear.Pos.Kp = 20.0;											/* set Kp */
	liftRear.Pos.Ki = 0.0;											/* set Ki */
	liftRear.Pos.Sat_min = -2.0;										/* set minimum saturation -> maximum negative velocity */
	liftRear.Pos.Sat_max = 2.0;										/* set maximum saturation -> positiv negative velocity */
 
 	/* ********************************************* 
		* current limiter
		* ********************************************* */
	liftRear.curLim.enable = 0;
	liftRear.curLim.setUp_currentLimiter = 0;
	liftRear.curLim.I_max = 0.0;
	liftRear.curLim.deltaTauLim = 0.1;
	liftRear.curLim.deltaTauUnLim = 0.1;
	liftRear.curLim.I_channel_HS = 3;
	liftRear.curLim.I_channel_LS = 2;
	liftRear.curLim.adc = adca_read;
	liftRear.curLim.I_offset_HS = 0;
	liftRear.curLim.I_offset_LS = 0;
}



/* ************************************************************** */
/*! \brief Motor control.
 *
 *  Motor control task.
 *  calculates the corresponding trajectory and motor voltage
 *  task is executed all 10 ms 
 *
 */
/* ************************************************************** */
uint8_t liftMotorTask()
{
   SET_CYCLE(LIFT_MOTOR_TASKNBR, 10);
   SET_TASK(LIFT_MOTOR_TASKNBR, CYCLE);

   
   /* ********************************************* 
    * motor
    * ********************************************* */
	controlMotion(&liftRear);
 
   return(CYCLE);
}