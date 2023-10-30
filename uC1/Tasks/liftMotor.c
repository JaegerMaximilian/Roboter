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
#include "command.h"



//#define  _DEBUG_MOTOR_DATA_

#define KP_VEL		400.0
#define KI_VEL		1000.0

#define KP_POS		40.0

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
   
if (mot_nr == LIFT_FRONT_RIGHT_NBR)
{
	/* ############################################################
	* ############################################################
	* #######                                               ######
	* #######         M O T O R   L I F T   V R             ######
	* #######                                               ######
	* ############################################################
	* ############################################################ */
	/* ********************************************* 
	* common settings for trajectory planning 
	* ********************************************* */
	liftVR.Traj.Xs = pos;												/* start value of the motor [m] */
	liftVR.Traj.Vstart = 0.0;										/* start velocity of the trajectory ramp [m/s] */
	liftVR.Traj.Vend = 0.0;											/* end velocity of the trajectory ramp [m/s] */
	liftVR.Traj.Ramp = SIN_2_RAMP;									/* ramp type -> sin^2 or linear */
	liftVR.Traj.AmaxUs = 15.0;										/* maximum acceleration at up slope [m^2/s] */
	liftVR.Traj.AmaxDs = 15.0;										/* maximum acceleration at down slope [m^2/s] */
	liftVR.Traj.State = _MOTION_READY;							/* reset motion status */

   
	/* ********************************************* 
	* odometry  
	* ********************************************* */
	liftVR.Odo.IncsPerRot = 12.0;									/* set increments per rotation */
	liftVR.Odo.Gear = 74.83;									/* set gear: 19:1 -> motor, 30:15 -> bevel gear */
	liftVR.Odo.DisPerRot = 0.05;							/* set driven distance per rotation [m] */
	liftVR.Odo.Dis = liftVR.Traj.Xs;								/* set start position in actual state -> see common settings */ 
	liftVR.Odo.Dis_1 = liftVR.Traj.Xs;								/* set start position in last state -> see common settings */   
	liftVR.Odo.IncTotal = (liftVR.Traj.Xs *						/* set total increments for start position */
							liftVR.Odo.Gear * 
							liftVR.Odo.IncsPerRot * 
							4.0) / liftVR.Odo.DisPerRot; 
	liftVR.Odo.invDir = true;	// *****									/* input value negate or not */ 


	/* ********************************************* 
	* QDEC  
	* ********************************************* */
	liftVR.Qdec.qPort = &PORTB;										/* port of the QDEC channel */
	liftVR.Qdec.qPin = 2;												/* A pin of the QDEC channel */
	liftVR.Qdec.invIO = false;										/* input value negate or not */
	liftVR.Qdec.qEvMux = 2;											/* event channel multiplex channel */
	liftVR.Qdec.qPinInput = EVSYS_CHMUX_PORTB_PIN2_gc;		/* set input for event system */
	liftVR.Qdec.useIndex = false;									/* use of index pin enable/disable */
	liftVR.Qdec.qIndexState = EVSYS_QDIRM_00_gc;				/* initialize event channel as QDEC */
	liftVR.Qdec.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
	liftVR.Qdec.qTimer0 = &TCC0;									/* set timer */
	liftVR.Qdec.qEventChannel = TC_EVSEL_CH2_gc;				/* set event channel */
	liftVR.Qdec.lineCount = (uint16_t) (liftVR.Odo.IncsPerRot *			/* set line counter */ 
							liftVR.Odo.Gear);	
	if(liftVR.Qdec.lineCount > 16383)
	{
		liftVR.Qdec.encDivider = 4;	
		liftVR.Qdec.lineCount /= liftVR.Qdec.encDivider;							
	}	
	else
	{
		liftVR.Qdec.encDivider = 1;	
	}		
	InitQdec(&liftVR);   

 
	/* ********************************************* 
	* PWM 
	* ********************************************* */
	PORTE.DIR |= 0x03;													/* initialize RIN pin -> set DIR to output */
	PORTE.OUTCLR = 0x03;													/* initialize RIN pin -> set OUT to zero -> OUT2 = 1 */
	liftVR.Pwm.Driver = BD6232HFP;								/* set driver type */
	liftVR.Pwm.qPort = &PORTE;										/* set PWM port */
	liftVR.Pwm.qPin = 0;												/* set PWM pin */
	liftVR.Pwm.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
	liftVR.Pwm.qTimer0 = &TCE0;									/* set PWM timer */
	liftVR.Pwm.Channel = CH_A;										/* set PWM channel */
	liftVR.Pwm.Frequence = 20000;									/* set PWM frequency [Hz] */
	liftVR.Pwm.invU = 1.0;											/* set voltage multiplier -> 1.0 ... not negated, -1.0 ... negated */
	liftVR.Pwm.qDIRport = &PORTE;									/* set FIN port */
	liftVR.Pwm.qDIRpin = 1;

	InitPwm(&liftVR);   


	/* ********************************************* 
	* velocity control
	* ********************************************* */
	liftVR.Vel.controlOn = 1;										/* velocity control on(1)/off(0) */
	liftVR.Vel.Kp = KP_VEL;												/* set Kp vel = 15*/
	liftVR.Vel.Ki = KI_VEL;									      /* set Ki vel = 250*/
	liftVR.Vel.Sat_min = -20.0;										/* set minimum saturation -> negativ accumulator voltage */
	liftVR.Vel.Sat_max = 20.0;										/* set maximum saturation -> positiv accumulator voltage */

	/* ********************************************* 
	* position control
	* ********************************************* */
	liftVR.Pos.controlOn = pos_enable;										/* position control on(1)/off(0) */
	liftVR.Pos.Kp = KP_POS;											/* set Kp */
	liftVR.Pos.Ki = 0.0;											/* set Ki */
	liftVR.Pos.Sat_min = -2.0;										/* set minimum saturation -> maximum negative velocity */
	liftVR.Pos.Sat_max = 2.0;										/* set maximum saturation -> positiv negative velocity */
 
 	/* ********************************************* 
		* current limiter
		* ********************************************* */
	liftVR.curLim.enable = 0;
	liftVR.curLim.setUp_currentLimiter = 0;
	liftVR.curLim.I_max = 0.0;
	liftVR.curLim.deltaTauLim = 0.1;
	liftVR.curLim.deltaTauUnLim = 0.1;
	liftVR.curLim.I_channel_HS = 3;
	liftVR.curLim.I_channel_LS = 2;
	liftVR.curLim.adc = adca_read;
	liftVR.curLim.I_offset_HS = 0;
	liftVR.curLim.I_offset_LS = 0;
	}
	
	if (mot_nr == LIFT_FRONT_LEFT_NBR)
{
	/* ############################################################
	* ############################################################
	* #######                                               ######
	* #######         M O T O R   L I F T   V L             ######
	* #######                                               ######
	* ############################################################
	* ############################################################ */
	/* ********************************************* 
	* common settings for trajectory planning 
	* ********************************************* */
	liftVL.Traj.Xs = pos;								/* start value of the motor [m] */
	liftVL.Traj.Vstart = 0.0;										/* start velocity of the trajectory ramp [m/s] */
	liftVL.Traj.Vend = 0.0;											/* end velocity of the trajectory ramp [m/s] */
	liftVL.Traj.Ramp = SIN_2_RAMP;									/* ramp type -> sin^2 or linear */
	liftVL.Traj.AmaxUs = 15.0;										/* maximum acceleration at up slope [m^2/s] */
	liftVL.Traj.AmaxDs = 15.0;										/* maximum acceleration at down slope [m^2/s] */
	liftVL.Traj.State = _MOTION_READY;							/* reset motion status */

   
	/* ********************************************* 
	* odometry  
	* ********************************************* */
	liftVL.Odo.IncsPerRot = 12.0;									/* set increments per rotation */
	liftVL.Odo.Gear = 74.83;									/* set gear: 19:1 -> motor, 30:15 -> bevel gear */
	liftVL.Odo.DisPerRot = 0.05;							/* set driven distance per rotation [m] */
	liftVL.Odo.Dis = liftVL.Traj.Xs;								/* set start position in actual state -> see common settings */ 
	liftVL.Odo.Dis_1 = liftVL.Traj.Xs;								/* set start position in last state -> see common settings */   
	liftVL.Odo.IncTotal = (liftVL.Traj.Xs *						/* set total increments for start position */
							liftVL.Odo.Gear * 
							liftVL.Odo.IncsPerRot * 
							4.0) / liftVL.Odo.DisPerRot; 
	liftVL.Odo.invDir = true;  // *****										/* input value negate or not */ 


	/* ********************************************* 
	* QDEC  
	* ********************************************* */
	liftVL.Qdec.qPort = &PORTB;										/* port of the QDEC channel */
	liftVL.Qdec.qPin = 0;												/* A pin of the QDEC channel */
	liftVL.Qdec.invIO = false;										/* input value negate or not */
	liftVL.Qdec.qEvMux = 0;											/* event channel multiplex channel */
	liftVL.Qdec.qPinInput = EVSYS_CHMUX_PORTB_PIN0_gc;		/* set input for event system */
	liftVL.Qdec.useIndex = false;									/* use of index pin enable/disable */
	liftVL.Qdec.qIndexState = EVSYS_QDIRM_00_gc;				/* initialize event channel as QDEC */
	liftVL.Qdec.nbrTimer = 1;										/* set timer number -> timer0 or timer1 */
	liftVL.Qdec.qTimer1 = &TCC1;									/* set timer */
	liftVL.Qdec.qEventChannel = TC_EVSEL_CH0_gc;				/* set event channel */
	liftVL.Qdec.lineCount = (uint16_t) (liftVL.Odo.IncsPerRot *			/* set line counter */ 
							liftVL.Odo.Gear);	
	if(liftVL.Qdec.lineCount > 16383)
	{
		liftVL.Qdec.encDivider = 4;	
		liftVL.Qdec.lineCount /= liftVL.Qdec.encDivider;							
	}	
	else
	{
		liftVL.Qdec.encDivider = 1;	
	}		
	InitQdec(&liftVL);   

 
	/* ********************************************* 
	* PWM 
	* ********************************************* */
	PORTE.DIR |= 0x0C;													/* initialize RIN pin -> set DIR to output */
	PORTE.OUTCLR = 0x0C;													/* initialize RIN pin -> set OUT to zero -> OUT2 = 1 */
	liftVL.Pwm.Driver = BD6232HFP;								/* set driver type */
	liftVL.Pwm.qPort = &PORTE;										/* set PWM port */
	liftVL.Pwm.qPin = 2;												/* set PWM pin */
	liftVL.Pwm.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
	liftVL.Pwm.qTimer0 = &TCE0;									/* set PWM timer */
	liftVL.Pwm.Channel = CH_C;										/* set PWM channel */
	liftVL.Pwm.Frequence = 20000;									/* set PWM frequency [Hz] */
	liftVL.Pwm.invU = 1.0;											/* set voltage multiplier -> 1.0 ... not negated, -1.0 ... negated */
	liftVL.Pwm.qDIRport = &PORTE;									/* set FIN port */
	liftVL.Pwm.qDIRpin = 3;

	InitPwm(&liftVL);   


	/* ********************************************* 
	* velocity control
	* ********************************************* */
	liftVL.Vel.controlOn = 1;										/* velocity control on(1)/off(0) */
	liftVL.Vel.Kp = KP_VEL;												/* set Kp vel = 15*/
	liftVL.Vel.Ki = KI_VEL;									      /* set Ki vel = 250*/
	liftVL.Vel.Sat_min = -20.0;										/* set minimum saturation -> negativ accumulator voltage */
	liftVL.Vel.Sat_max = 20.0;										/* set maximum saturation -> positiv accumulator voltage */

	/* ********************************************* 
	* position control
	* ********************************************* */
	liftVL.Pos.controlOn = pos_enable;										/* position control on(1)/off(0) */
	liftVL.Pos.Kp = KP_POS;											/* set Kp */
	liftVL.Pos.Ki = 0.0;											/* set Ki */
	liftVL.Pos.Sat_min = -0.1;										/* set minimum saturation -> maximum negative velocity */
	liftVL.Pos.Sat_max = 0.1;										/* set maximum saturation -> positiv negative velocity */
 
 	/* ********************************************* 
		* current limiter
		* ********************************************* */
	liftVL.curLim.enable = 0;
	liftVL.curLim.setUp_currentLimiter = 0;
	liftVL.curLim.I_max = 0.0;
	liftVL.curLim.deltaTauLim = 0.1;
	liftVL.curLim.deltaTauUnLim = 0.1;
	liftVL.curLim.I_channel_HS = 3;
	liftVL.curLim.I_channel_LS = 2;
	liftVL.curLim.adc = adca_read;
	liftVL.curLim.I_offset_HS = 0;
	liftVL.curLim.I_offset_LS = 0;
  
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
	controlMotion(&liftVR);
	controlMotion(&liftVL);

	

 
   return(CYCLE);
}