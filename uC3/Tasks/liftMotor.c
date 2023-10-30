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



//#define  _DEBUG_MOTOR_DATA_

/* ************************************************************** */
/*! \brief Initialize the motor control.
 *
 *  Motor control initialization function.
 *  Sets up the ports, timers, qdecs, limits, odometry and controllers 
 *
 */
/* ************************************************************** */
void liftMotor_init(uint8_t pos_enable, float pos, uint8_t mot_nr)
{
	/* cyclic task - cycle time: 10 ms */
	SET_CYCLE(LIFT_MOTOR_TASKNBR, 10);
	SET_TASK(LIFT_MOTOR_TASKNBR, CYCLE);
	SET_TASK_HANDLE(LIFT_MOTOR_TASKNBR, liftMotorTask);
   
if (mot_nr == LIFT_REAR_RIGHT_NBR)
{
	/* ############################################################
	* ############################################################
	* #######                                               ######
	* #######         M O T O R   L I F T   H R             ######
	* #######                                               ######
	* ############################################################
	* ############################################################ */
	/* ********************************************* 
	* common settings for trajectory planning 
	* ********************************************* */
	liftHR.Traj.Xs = pos;												/* start value of the motor [m] */
	liftHR.Traj.Vstart = 0.0;										/* start velocity of the trajectory ramp [m/s] */
	liftHR.Traj.Vend = 0.0;											/* end velocity of the trajectory ramp [m/s] */
	liftHR.Traj.Ramp = SIN_2_RAMP;									/* ramp type -> sin^2 or linear */
	liftHR.Traj.AmaxUs = 1.5;										/* maximum acceleration at up slope [m^2/s] */
	liftHR.Traj.AmaxDs = 1.5;										/* maximum acceleration at down slope [m^2/s] */
	liftHR.Traj.State = _MOTION_READY;							/* reset motion status */

   
	/* ********************************************* 
	* odometry  
	* ********************************************* */
	liftHR.Odo.IncsPerRot = 12.0;									/* set increments per rotation */
	liftHR.Odo.Gear = 74.83;									/* set gear: 19:1 -> motor, 30:15 -> bevel gear */
	liftHR.Odo.DisPerRot = 0.05;							/* set driven distance per rotation [m] */
	liftHR.Odo.Dis = liftHR.Traj.Xs;								/* set start position in actual state -> see common settings */ 
	liftHR.Odo.Dis_1 = liftHR.Traj.Xs;								/* set start position in last state -> see common settings */   
	liftHR.Odo.IncTotal = (liftHR.Traj.Xs *						/* set total increments for start position */
							liftHR.Odo.Gear * 
							liftHR.Odo.IncsPerRot * 
							4.0) / liftHR.Odo.DisPerRot; 
	liftHR.Odo.invDir = true;	// *****									/* input value negate or not */ 


	/* ********************************************* 
	* QDEC  
	* ********************************************* */
	liftHR.Qdec.qPort = &PORTB;										/* port of the QDEC channel */
	liftHR.Qdec.qPin = 2;												/* A pin of the QDEC channel */
	liftHR.Qdec.invIO = false;										/* input value negate or not */
	liftHR.Qdec.qEvMux = 2;											/* event channel multiplex channel */
	liftHR.Qdec.qPinInput = EVSYS_CHMUX_PORTB_PIN2_gc;		/* set input for event system */
	liftHR.Qdec.useIndex = false;									/* use of index pin enable/disable */
	liftHR.Qdec.qIndexState = EVSYS_QDIRM_00_gc;				/* initialize event channel as QDEC */
	liftHR.Qdec.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
	liftHR.Qdec.qTimer0 = &TCC0;									/* set timer */
	liftHR.Qdec.qEventChannel = TC_EVSEL_CH2_gc;				/* set event channel */
	liftHR.Qdec.lineCount = (uint16_t) (liftHR.Odo.IncsPerRot *			/* set line counter */ 
							liftHR.Odo.Gear);	
	if(liftHR.Qdec.lineCount > 16383)
	{
		liftHR.Qdec.encDivider = 4;	
		liftHR.Qdec.lineCount /= liftHR.Qdec.encDivider;							
	}	
	else
	{
		liftHR.Qdec.encDivider = 1;	
	}		
	InitQdec(&liftHR);   

 
	/* ********************************************* 
	* PWM 
	* ********************************************* */
	PORTE.DIR |= 0x0C;													/* initialize RIN pin -> set DIR to output */
	PORTE.OUTCLR = 0x0C;													/* initialize RIN pin -> set OUT to zero -> OUT2 = 1 */
	liftHR.Pwm.Driver = BD6232HFP;								/* set driver type */
	liftHR.Pwm.qPort = &PORTE;										/* set PWM port */
	liftHR.Pwm.qPin = 2;												/* set PWM pin */
	liftHR.Pwm.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
	liftHR.Pwm.qTimer0 = &TCE0;									/* set PWM timer */
	liftHR.Pwm.Channel = CH_C;										/* set PWM channel */
	liftHR.Pwm.Frequence = 20000;									/* set PWM frequency [Hz] */
	liftHR.Pwm.invU = -1.0;											/* set voltage multiplier -> 1.0 ... not negated, -1.0 ... negated */
	liftHR.Pwm.qDIRport = &PORTE;									/* set FIN port */
	liftHR.Pwm.qDIRpin = 3;

	InitPwm(&liftHR);   


	/* ********************************************* 
	* velocity control
	* ********************************************* */
	liftHR.Vel.controlOn = 1;										/* velocity control on(1)/off(0) */
	liftHR.Vel.Kp = 150;												/* set Kp vel = 15*/
	liftHR.Vel.Ki = 500;									      /* set Ki vel = 250*/
	liftHR.Vel.Sat_min = -20.0;										/* set minimum saturation -> negativ accumulator voltage */
	liftHR.Vel.Sat_max = 20.0;										/* set maximum saturation -> positiv accumulator voltage */

	/* ********************************************* 
	* position control
	* ********************************************* */
	liftHR.Pos.controlOn = pos_enable;										/* position control on(1)/off(0) */
	liftHR.Pos.Kp = 20.0;											/* set Kp */
	liftHR.Pos.Ki = 0.0;											/* set Ki */
	liftHR.Pos.Sat_min = -2.0;										/* set minimum saturation -> maximum negative velocity */
	liftHR.Pos.Sat_max = 2.0;										/* set maximum saturation -> positiv negative velocity */
 
 	/* ********************************************* 
		* current limiter
		* ********************************************* */
	liftHR.curLim.enable = 0;
	liftHR.curLim.setUp_currentLimiter = 0;
	liftHR.curLim.I_max = 0.0;
	liftHR.curLim.deltaTauLim = 0.1;
	liftHR.curLim.deltaTauUnLim = 0.1;
	liftHR.curLim.I_channel_HS = 3;
	liftHR.curLim.I_channel_LS = 2;
	liftHR.curLim.adc = adca_read;
	liftHR.curLim.I_offset_HS = 0;
	liftHR.curLim.I_offset_LS = 0;
	}
	
	if (mot_nr == LIFT_REAR_LEFT_NBR)
{
	/* ############################################################
	* ############################################################
	* #######                                               ######
	* #######         M O T O R   L I F T   H L             ######
	* #######                                               ######
	* ############################################################
	* ############################################################ */
	/* ********************************************* 
	* common settings for trajectory planning 
	* ********************************************* */
	liftHL.Traj.Xs = pos;								/* start value of the motor [m] */
	liftHL.Traj.Vstart = 0.0;										/* start velocity of the trajectory ramp [m/s] */
	liftHL.Traj.Vend = 0.0;											/* end velocity of the trajectory ramp [m/s] */
	liftHL.Traj.Ramp = SIN_2_RAMP;									/* ramp type -> sin^2 or linear */
	liftHL.Traj.AmaxUs = 1.5;										/* maximum acceleration at up slope [m^2/s] */
	liftHL.Traj.AmaxDs = 1.5;										/* maximum acceleration at down slope [m^2/s] */
	liftHL.Traj.State = _MOTION_READY;							/* reset motion status */

   
	/* ********************************************* 
	* odometry  
	* ********************************************* */
	liftHL.Odo.IncsPerRot = 12.0;									/* set increments per rotation */
	liftHL.Odo.Gear = 74.83;									/* set gear: 19:1 -> motor, 30:15 -> bevel gear */
	liftHL.Odo.DisPerRot = 0.05;							/* set driven distance per rotation [m] */
	liftHL.Odo.Dis = liftHL.Traj.Xs;								/* set start position in actual state -> see common settings */ 
	liftHL.Odo.Dis_1 = liftHL.Traj.Xs;								/* set start position in last state -> see common settings */   
	liftHL.Odo.IncTotal = (liftHL.Traj.Xs *						/* set total increments for start position */
							liftHL.Odo.Gear * 
							liftHL.Odo.IncsPerRot * 
							4.0) / liftHL.Odo.DisPerRot; 
	liftHL.Odo.invDir = true;  // *****										/* input value negate or not */ 


	/* ********************************************* 
	* QDEC  
	* ********************************************* */
	liftHL.Qdec.qPort = &PORTB;										/* port of the QDEC channel */
	liftHL.Qdec.qPin = 0;												/* A pin of the QDEC channel */
	liftHL.Qdec.invIO = false;										/* input value negate or not */
	liftHL.Qdec.qEvMux = 0;											/* event channel multiplex channel */
	liftHL.Qdec.qPinInput = EVSYS_CHMUX_PORTB_PIN0_gc;		/* set input for event system */
	liftHL.Qdec.useIndex = false;									/* use of index pin enable/disable */
	liftHL.Qdec.qIndexState = EVSYS_QDIRM_00_gc;				/* initialize event channel as QDEC */
	liftHL.Qdec.nbrTimer = 1;										/* set timer number -> timer0 or timer1 */
	liftHL.Qdec.qTimer1 = &TCC1;									/* set timer */
	liftHL.Qdec.qEventChannel = TC_EVSEL_CH0_gc;				/* set event channel */
	liftHL.Qdec.lineCount = (uint16_t) (liftHL.Odo.IncsPerRot *			/* set line counter */ 
							liftHL.Odo.Gear);	
	if(liftHL.Qdec.lineCount > 16383)
	{
		liftHL.Qdec.encDivider = 4;	
		liftHL.Qdec.lineCount /= liftHL.Qdec.encDivider;							
	}	
	else
	{
		liftHL.Qdec.encDivider = 1;	
	}		
	InitQdec(&liftHL);   

 
	/* ********************************************* 
	* PWM 
	* ********************************************* */
	PORTE.DIR |= 0x03;													/* initialize RIN pin -> set DIR to output */
	PORTE.OUTCLR = 0x03;													/* initialize RIN pin -> set OUT to zero -> OUT2 = 1 */
	liftHL.Pwm.Driver = BD6232HFP;								/* set driver type */
	liftHL.Pwm.qPort = &PORTE;										/* set PWM port */
	liftHL.Pwm.qPin = 0;												/* set PWM pin */
	liftHL.Pwm.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
	liftHL.Pwm.qTimer0 = &TCE0;									/* set PWM timer */
	liftHL.Pwm.Channel = CH_A;										/* set PWM channel */
	liftHL.Pwm.Frequence = 20000;									/* set PWM frequency [Hz] */
	liftHL.Pwm.invU = -1.0;											/* set voltage multiplier -> 1.0 ... not negated, -1.0 ... negated */
	liftHL.Pwm.qDIRport = &PORTE;									/* set FIN port */
	liftHL.Pwm.qDIRpin = 1;

	InitPwm(&liftHL);   


	/* ********************************************* 
	* velocity control
	* ********************************************* */
	liftHL.Vel.controlOn = 1;										/* velocity control on(1)/off(0) */
	liftHL.Vel.Kp = 150;												/* set Kp vel = 15*/
	liftHL.Vel.Ki = 500;									      /* set Ki vel = 250*/
	liftHL.Vel.Sat_min = -20.0;										/* set minimum saturation -> negativ accumulator voltage */
	liftHL.Vel.Sat_max = 20.0;										/* set maximum saturation -> positiv accumulator voltage */

	/* ********************************************* 
	* position control
	* ********************************************* */
	liftHL.Pos.controlOn = pos_enable;										/* position control on(1)/off(0) */
	liftHL.Pos.Kp = 20.0;											/* set Kp */
	liftHL.Pos.Ki = 0.0;											/* set Ki */
	liftHL.Pos.Sat_min = -0.1;										/* set minimum saturation -> maximum negative velocity */
	liftHL.Pos.Sat_max = 0.1;										/* set maximum saturation -> positiv negative velocity */
 
 	/* ********************************************* 
		* current limiter
		* ********************************************* */
	liftHL.curLim.enable = 0;
	liftHL.curLim.setUp_currentLimiter = 0;
	liftHL.curLim.I_max = 0.0;
	liftHL.curLim.deltaTauLim = 0.1;
	liftHL.curLim.deltaTauUnLim = 0.1;
	liftHL.curLim.I_channel_HS = 3;
	liftHL.curLim.I_channel_LS = 2;
	liftHL.curLim.adc = adca_read;
	liftHL.curLim.I_offset_HS = 0;
	liftHL.curLim.I_offset_LS = 0;
  
}
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
	controlMotion(&liftHR);
	controlMotion(&liftHL);
	

 
   return(CYCLE);
}