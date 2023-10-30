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

#define _MOTOR_EXTERN


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
#include "accumulator.h"
#include "LEDcontrol.h"
#include "usart.h"




/* actual accumulator voltage */
float uBat;


/* ************************************************************** */
/*! \brief Initialize the motor control.
 *
 *  Motor control initialization function.
 *  Sets up the ports, timers, qdecs, limits, odometry and controllers 
 *
 */
/* ************************************************************** */
void Motor_init()
{
   /* cyclic task - cycle time: 10 ms */
   SET_CYCLE(MOTOR_TASKNBR, 10);
   SET_TASK(MOTOR_TASKNBR, CYCLE);
   SET_TASK_HANDLE(MOTOR_TASKNBR, MotorTask);
   

   /* ############################################################
    * ############################################################
    * #######                                               ######
    * #######              M O T O R   L E F T              ######
    * #######                                               ######
    * ############################################################
    * ############################################################ */
   /* ********************************************* 
    * common settings for trajectory planning 
    * ********************************************* */
   motorLeft.Traj.Xs = 0.0;											/* start value of the motor [m] */
   motorLeft.Traj.Vstart = 0.0;										/* start velocity of the trajectory ramp [m/s] */
   motorLeft.Traj.Vend = 0.0;											/* end velocity of the trajectory ramp [m/s] */
   motorLeft.Traj.Ramp = LIN_RAMP;									/* ramp type -> sin^2 or linear */
   motorLeft.Traj.AmaxUs = 2.0;										/* maximum acceleration at up slope [m^2/s] */
   motorLeft.Traj.AmaxDs = 2.0;										/* maximum acceleration at down slope [m^2/s] */
   motorLeft.Traj.State = _MOTION_READY;							/* reset motion status */

   
   /* ********************************************* 
    * odometry  
    * ********************************************* */
   motorLeft.Odo.IncsPerRot = 3.0;								/* set increments per rotation */
   motorLeft.Odo.Gear = 75.0;											/* set gear */
   motorLeft.Odo.DisPerRot = 0.028*M_PI;									/* set driven distance per rotation [m] */
   motorLeft.Odo.Dis = motorLeft.Traj.Xs;                   /* set start position in actual state -> see common settings */ 
   motorLeft.Odo.Dis_1 = motorLeft.Traj.Xs;                 /* set start position in last state -> see common settings */   
   motorLeft.Odo.IncTotal = (motorLeft.Traj.Xs *				/* set total increments for start position */
                             motorLeft.Odo.Gear * 
							        motorLeft.Odo.IncsPerRot * 
									  4.0) / 
									  motorLeft.Odo.DisPerRot; 
   motorLeft.Odo.invDir = true;										/* input value negate or not */ 


   /* ********************************************* 
    * QDEC  
    * ********************************************* */
   motorLeft.Qdec.qPort = &PORTD;									/* port of the QDEC channel */
   motorLeft.Qdec.qPin = 2;											/* A pin of the QDEC channel */
   motorLeft.Qdec.invIO = false;										/* input value negate or not */
   motorLeft.Qdec.qEvMux = 2;											/* event channel multiplex channel */
   motorLeft.Qdec.qPinInput = EVSYS_CHMUX_PORTD_PIN2_gc;		/* set input for event system */
   motorLeft.Qdec.useIndex = false;									/* use of index pin enable/disable */
   motorLeft.Qdec.qIndexState = EVSYS_QDIRM_00_gc;				/* initialize event channel as QDEC */
   motorLeft.Qdec.nbrTimer = 0;										/* set timer number -> timer0 or timer1 */
   motorLeft.Qdec.qTimer0 = &TCD0;									/* set timer */
   motorLeft.Qdec.qEventChannel = TC_EVSEL_CH2_gc;				/* set event channel */
   motorLeft.Qdec.lineCount = motorLeft.Odo.IncsPerRot *		/* set line counter */ 
                              motorLeft.Odo.Gear;									
   InitQdec(&motorLeft);   

 
   /* ********************************************* 
    * PWM 
    * ********************************************* */
   PORTD.DIR |= 0x10;													/* initialize RIN pin -> set DIR to output */
   PORTD.OUTCLR = 0x10;													/* initialize RIN pin -> set OUT to zero -> OUT2 = 1 */
   motorLeft.Pwm.Driver = BD6232HFP;								/* set driver type */
   motorLeft.Pwm.qPort = &PORTD;										/* set PWM port */
   motorLeft.Pwm.qPin = 4;												/* set PWM pin */
   motorLeft.Pwm.nbrTimer = 1;										/* set timer number -> timer0 or timer1 */
   motorLeft.Pwm.qTimer1 = &TCD1;									/* set PWM timer */
   motorLeft.Pwm.Channel = CH_A;										/* set PWM channel */
   motorLeft.Pwm.Frequence = 20000;									/* set PWM frequency [Hz] */	
   motorLeft.Pwm.invU = 1.0;											/* set voltage multiplier -> 1.0 ... not negated, -1.0 ... negated */
   motorLeft.Pwm.qDIRport = &PORTD;									/* set FIN port */
   motorLeft.Pwm.qDIRpin = 6;											/* set FIN pin */
   InitPwm(&motorLeft);   


   /* ********************************************* 
    * velocity control
    * ********************************************* */
   motorLeft.Vel.controlOn = 0;										/* velocity control on(1)/off(0) */
   motorLeft.Vel.Kp = 120.0;											/* set Kp */
   motorLeft.Vel.Ki = 800.0;											/* set Ki */
   motorLeft.Vel.Sat_min = -12.0;									/* set minimum saturation -> negativ accumulator voltage */
   motorLeft.Vel.Sat_max = 12.0;										/* set maximum saturation -> positiv accumulator voltage */

   /* ********************************************* 
    * position control
    * ********************************************* */
   motorLeft.Pos.controlOn = 0;										/* position control on(1)/off(0) */
   motorLeft.Pos.Kp = 25.0;												/* set Kp */
   motorLeft.Pos.Ki = 0.0;												/* set Ki */
   motorLeft.Pos.Sat_min = -2.0;									   /* set minimum saturation -> maximum negative velocity */
   motorLeft.Pos.Sat_max = 2.0;										/* set maximum saturation -> positiv negative velocity */
   


   /* ############################################################
    * ############################################################
    * #######                                               ######
    * #######             M O T O R   R I G H T             ######
    * #######                                               ######
    * ############################################################
    * ############################################################ */
   /* ********************************************* 
    * common settings for trajectory planning 
    * ********************************************* */
   motorRight.Traj.Xs = 0.0;											/* start value of the motor [m] */
   motorRight.Traj.Vstart = 0.0;										/* start velocity of the trajectory ramp [m/s] */
   motorRight.Traj.Vend = 0.0;										/* end velocity of the trajectory ramp [m/s] */
   motorRight.Traj.Ramp = LIN_RAMP;									/* ramp type -> sin^2 or linear */
   motorRight.Traj.AmaxUs = 2.0;										/* maximum acceleration at up slope [m^2/s] */
   motorRight.Traj.AmaxDs = 2.0;										/* maximum acceleration at down slope [m^2/s] */
   motorRight.Traj.State = _MOTION_READY;							/* reset motion status */

   
   /* ********************************************* 
    * odometry  
    * ********************************************* */
   motorRight.Odo.IncsPerRot = 3.0;								/* set increments per rotation */
   motorRight.Odo.Gear = 75.0;										/* set gear */
   motorRight.Odo.DisPerRot = 0.028 * M_PI;									/* set driven distance per rotation [m] */
   motorRight.Odo.Dis = motorRight.Traj.Xs;                 /* set start position in actual state -> see common settings */ 
   motorRight.Odo.Dis_1 = motorRight.Traj.Xs;               /* set start position in last state -> see common settings */   
   motorRight.Odo.IncTotal = (motorRight.Traj.Xs *				/* set total increments for start position */
                             motorRight.Odo.Gear * 
							        motorRight.Odo.IncsPerRot * 
									  4.0) / 
									  motorRight.Odo.DisPerRot; 
   motorRight.Odo.invDir = false;									/* input value negate or not */ 


   /* ********************************************* 
    * QDEC  
    * ********************************************* */
   motorRight.Qdec.qPort = &PORTD;									/* port of the QDEC channel */
   motorRight.Qdec.qPin = 0;											/* A pin of the QDEC channel */
   motorRight.Qdec.invIO = false;									/* input value negate or not */
   motorRight.Qdec.qEvMux = 0;										/* event channel multiplex channel */
   motorRight.Qdec.qPinInput = EVSYS_CHMUX_PORTD_PIN0_gc;	/* set input for event system */
   motorRight.Qdec.useIndex = false;								/* use of index pin enable/disable */
   motorRight.Qdec.qIndexState = EVSYS_QDIRM_00_gc;			/* initialize event channel as QDEC */
   motorRight.Qdec.nbrTimer = 1;										/* set timer number -> timer0 or timer1 */
   motorRight.Qdec.qTimer1 = &TCC1;									/* set timer */
   motorRight.Qdec.qEventChannel = TC_EVSEL_CH0_gc;			/* set event channel */
   motorRight.Qdec.lineCount = motorRight.Odo.IncsPerRot *	/* set line counter */ 
                               motorRight.Odo.Gear;									
   InitQdec(&motorRight);   

 
   /* ********************************************* 
    * PWM 
    * ********************************************* */
   PORTD.DIR |= 0x20;													/* initialize RIN pin -> set DIR to output */
   PORTD.OUTCLR = 0x20;													/* initialize RIN pin -> set OUT to zero -> OUT2 = 1 */
   motorRight.Pwm.Driver = BD6232HFP;								/* set driver type */
   motorRight.Pwm.qPort = &PORTD;									/* set PWM port */
   motorRight.Pwm.qPin = 5;											/* set PWM pin */
   motorRight.Pwm.nbrTimer = 1;										/* set timer number -> timer0 or timer1 */
   motorRight.Pwm.qTimer1 = &TCD1;									/* set PWM timer */
   motorRight.Pwm.Channel = CH_B;									/* set PWM channel */
   motorRight.Pwm.Frequence = 20000;								/* set PWM frequency [Hz] */	
   motorRight.Pwm.invU = -1.0;										/* set voltage multiplier -> 1.0 ... not negated, -1.0 ... negated */
   motorRight.Pwm.qDIRport = &PORTD;								/* set FIN port */
   motorRight.Pwm.qDIRpin = 7;										/* set FIN pin */
   InitPwm(&motorRight);   


   /* ********************************************* 
    * velocity control
    * ********************************************* */
   motorRight.Vel.controlOn = 0;										/* velocity control on(1)/off(0) */
   motorRight.Vel.Kp = 120.0;											/* set Kp */
   motorRight.Vel.Ki = 800.0;											/* set Ki */
   motorRight.Vel.Sat_min = -12.0;									/* set minimum saturation -> negativ accumulator voltage */
   motorRight.Vel.Sat_max = 12.0;									/* set maximum saturation -> positiv accumulator voltage */

   /* ********************************************* 
    * position control
    * ********************************************* */
   motorRight.Pos.controlOn = 0;										/* position control on(1)/off(0) */
   motorRight.Pos.Kp = 5.0;											/* set Kp */
   motorRight.Pos.Ki = 0.0;											/* set Ki */
   motorRight.Pos.Sat_min = -2.0;									/* set minimum saturation -> maximum negative velocity */
   motorRight.Pos.Sat_max = 2.0;										/* set maximum saturation -> positiv negative velocity */
  
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
uint8_t MotorTask()
{
   SET_CYCLE(MOTOR_TASKNBR, 10);
   SET_TASK(MOTOR_TASKNBR, CYCLE);

   /* ********************************************* 
    * update accumulator voltage
   // ********************************************* */
   uBat = getAccumulatorVoltage();
   motorLeft.Vel.Sat_min = -uBat;
   motorLeft.Vel.Sat_max = uBat;
   motorRight.Vel.Sat_min = -uBat;
   motorRight.Vel.Sat_max = uBat;
   
   
   /* ********************************************* 
    * left motor
    * ********************************************* */
	
//	motorLeft.Vel.Y = 7.0;
	controlMotion(&motorLeft);
	

   /* ********************************************* 
    * right motor
    * ********************************************* */
//	motorRight.Vel.Y = 7.0;
	controlMotion(&motorRight);
      
 
   return(CYCLE);
}


/* ************************************************************** */
/*! \brief sign function for float.
 *
 *  returns the sign of a given float number.
 *
 *  \param a float number.
 *
 *  \retval -1.0  If a is less 0.0.
 *  \retval  1.0  If a is greater or equal 0.0. 
 *  
*/
/* ************************************************************** */
float fsign(float a) 
{
	if(a < 0.0)
		return(-1.0);
	else
		return(1.0);
}



/* ************************************************************** */
/*! \brief Generate a trajectory for the position control.
 *
 *  Calculate the position trajectory -> way points, velocity,... .
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void genTraj_pos(Motor_t *motorPoint)
{
   float fS0, fVmax_1, fSign;  
   int16_t siZeitKorrektur;

   
   /* ****************************************************************
      ****    after motion: pos -> give out the endpoint          ****
      ****                  speed -> give out zero                ****
      **************************************************************** */
   if(motorPoint->Traj.State == _MOTION_READY)
   {  
      motorPoint->Traj.Sn = 0.0;
      motorPoint->Traj.Sn_1 = 0.0;
      motorPoint->Traj.Vn = motorPoint->Traj.Vend;  
   } 
   
   /* ****************************************************************
      ****   new motion: calculate up- and down-slope time and    ****
      ****               the maximum velocity                     ****
      **************************************************************** */
   if(motorPoint->Traj.State == _NEW_MOTION)
   {                                 
      /* *********************************
         calculate motion
         ********************************* */
      fS0 = motorPoint->Traj.S0 - motorPoint->Traj.Xs;
      motorPoint->Traj.S0 = fS0;
      /* *****************************************
         adapt sign of way and velocity
         ***************************************** */

      if(((motorPoint->Traj.S0 < 0.0) && (motorPoint->Traj.Vmax > 0.0)) || 
         ((motorPoint->Traj.S0 > 0.0) && (motorPoint->Traj.Vmax < 0.0)))
         motorPoint->Traj.Vmax *= (-1.0);
      /* *****************************************
         velocity limitation
         ***************************************** */
      if(fabs(motorPoint->Traj.Vmax/fS0) > SPEED_LIMIT_FACTOR) 
         motorPoint->Traj.Vmax = fS0 * SPEED_LIMIT_FACTOR;
                  
      /* *****************************************
         velocity limitation (if velocity is to high for the given acceleration) 
         ***************************************** */
      fVmax_1 = sqrt(RAMP_LIMIT*fabs(fS0)*motorPoint->Traj.AmaxUs+pow(fabs(motorPoint->Traj.Vstart),2.0));
      if(fabs(motorPoint->Traj.Vmax) > fVmax_1) 
         motorPoint->Traj.Vmax = fVmax_1 * (float)fsign(motorPoint->Traj.Vmax);
      /* *****************************************
         velocity limitation (if velocity is to high for the given acceleration) 
         ***************************************** */
      fVmax_1 = sqrt(RAMP_LIMIT*fabs(fS0)*motorPoint->Traj.AmaxDs+pow(fabs(motorPoint->Traj.Vstart),2.0));
      if(fabs(motorPoint->Traj.Vmax) > fVmax_1) 
        motorPoint->Traj.Vmax = fVmax_1 * (float)fsign(motorPoint->Traj.Vmax);
      /* *****************************************
         calculate up- and down-slope for a given acceleration
         ***************************************** */
      motorPoint->Traj.Tus = fabs(motorPoint->Traj.Vmax - motorPoint->Traj.Vstart) / motorPoint->Traj.AmaxUs;
      motorPoint->Traj.Tds = fabs(motorPoint->Traj.Vmax - motorPoint->Traj.Vend) / motorPoint->Traj.AmaxDs;
                     
      /* *****************************************
         fit up- and down-slope to T_ABTAST
         ***************************************** */
      if(motorPoint->Traj.Tus > 0.0001)
      {
         siZeitKorrektur = (signed int)(motorPoint->Traj.Tus / T_ABTAST);
         motorPoint->Traj.Tus = T_ABTAST * ((float)siZeitKorrektur + 1); 
      }
                     
      if(motorPoint->Traj.Tds > 0.0001)
      {
         siZeitKorrektur = (signed int)(motorPoint->Traj.Tds / T_ABTAST);
         motorPoint->Traj.Tds = T_ABTAST * ((float)siZeitKorrektur + 1);
      } 
                        
                        
      /* *****************************************
         calculate tmax
         ***************************************** */
      motorPoint->Traj.Tmax = (fabs(fS0) - 
                             (motorPoint->Traj.Tus * (fabs(motorPoint->Traj.Vmax) + fabs(motorPoint->Traj.Vstart)) / 2.0) - 
                             (motorPoint->Traj.Tds * (fabs(motorPoint->Traj.Vmax) + fabs(motorPoint->Traj.Vend)) / 2.0)) / 
                             fabs(motorPoint->Traj.Vmax); 
                                                    
      siZeitKorrektur = (signed int)(motorPoint->Traj.Tmax / T_ABTAST);
      motorPoint->Traj.Tmax = T_ABTAST * ((float)siZeitKorrektur + 1);

      /* *****************************************
         set parameter in the calc-struct
         ***************************************** */
      /* recalculate Vmax (according to the recalculation with T_ABTAST !!!!) */
      fSign = (float)fsign(motorPoint->Traj.Vmax);
      motorPoint->Traj.Vmax = ((2 * fabs(fS0) - 
                              fabs(motorPoint->Traj.Vstart) *  motorPoint->Traj.Tus -  
                              fabs(motorPoint->Traj.Vend) *  motorPoint->Traj.Tds) / 
                              (motorPoint->Traj.Tus + motorPoint->Traj.Tds + 2 * motorPoint->Traj.Tmax)) * fSign; 
                      
      motorPoint->Traj.State = _UP_SLOPE;
   } 
  
   
   /* generate the way(angle) - velocity profil */
   switch(motorPoint->Traj.State)                                                 
   {  
      /* ****************************************************************
            set up slope 
         **************************************************************** */
      case _UP_SLOPE:
      {  
         /* **********************************************
            calculate the way/velocity
            ********************************************** */
         if(motorPoint->Traj.Ramp == LIN_RAMP)
         {
            motorPoint->Traj.Sn_1 = motorPoint->Traj.Sn;
            motorPoint->Traj.Sn = motorPoint->Traj.Vstart * motorPoint->Traj.T + 
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vstart) *
                                  pow(motorPoint->Traj.T, 2.0) / 2 / motorPoint->Traj.Tus; 
                              
            motorPoint->Traj.Vn = motorPoint->Traj.Vstart + 
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vstart) *
                                  motorPoint->Traj.T / motorPoint->Traj.Tus;
            
            /* saturate velocity to Vmax */
            if(fabs(motorPoint->Traj.Vn) >= fabs(motorPoint->Traj.Vmax)) 
               motorPoint->Traj.Vn = motorPoint->Traj.Vmax;
         }
         else if(motorPoint->Traj.Ramp == SIN_2_RAMP)
         {
            motorPoint->Traj.Sn_1 = motorPoint->Traj.Sn;
            motorPoint->Traj.Sn = motorPoint->Traj.Vstart * motorPoint->Traj.T + 
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vstart) *
                                  (motorPoint->Traj.T / 2.0 - motorPoint->Traj.Tus * 
                                  sin(M_PI * motorPoint->Traj.T / motorPoint->Traj.Tus) / 2.0 / M_PI);
                              
            motorPoint->Traj.Vn = motorPoint->Traj.Vstart + 
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vstart) / 2.0  *
                                  (1.0 - cos(M_PI * motorPoint->Traj.T / motorPoint->Traj.Tus));
         }         
         /* saturate velocity to Vmax */
         if(fabs(motorPoint->Traj.Vn) >= fabs(motorPoint->Traj.Vmax)) 
            motorPoint->Traj.Vn = motorPoint->Traj.Vmax;

         /* **********************************************
            save way/angle points
            ********************************************** */
         motorPoint->Traj.Sold = motorPoint->Traj.Sn;
         motorPoint->Traj.Vold = motorPoint->Traj.Vn;         
         

         if(((motorPoint->Traj.T >= motorPoint->Traj.Tus) && (motorPoint->Traj.Ramp == LIN_RAMP)) ||
            ((motorPoint->Traj.T >= (motorPoint->Traj.Tus - 0.1 * T_ABTAST)) && (motorPoint->Traj.Ramp == SIN_2_RAMP)))      
         {
            if(motorPoint->Traj.Tmax > T_MAX_MIN)
            {
               motorPoint->Traj.State = _MAX_SPEED;
               motorPoint->Traj.SI = motorPoint->Traj.Sn;
               motorPoint->Traj.Tmax -= (motorPoint->Traj.T - motorPoint->Traj.Tus);
            }
            else
            {
               motorPoint->Traj.State = _DOWN_SLOPE; 
               motorPoint->Traj.SII = motorPoint->Traj.Sn;
               motorPoint->Traj.Tds -= (motorPoint->Traj.T - motorPoint->Traj.Tus);
            }
            motorPoint->Traj.T = 0.0;  
         }

         motorPoint->Traj.T += T_ABTAST;
         
         break;
      } 
      
      /* ****************************************************************
            set down slope 
         **************************************************************** */
      case _DOWN_SLOPE:
      {
         /* **********************************************
            calculate the way/velocity
            ********************************************** */
         if(motorPoint->Traj.Ramp == LIN_RAMP)
         {
            motorPoint->Traj.Sn_1 = motorPoint->Traj.Sn;
            motorPoint->Traj.Sn = motorPoint->Traj.SII + 
                                  motorPoint->Traj.Vmax * motorPoint->Traj.T - 
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vend) *
                                  pow(motorPoint->Traj.T, 2.0) / 2 / motorPoint->Traj.Tds; 
            /* saturate way to s0 */
            if(fabs(motorPoint->Traj.Sn) >= fabs(motorPoint->Traj.S0)) 
               motorPoint->Traj.Sn = motorPoint->Traj.S0;

            motorPoint->Traj.Vn = motorPoint->Traj.Vmax - 
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vend) *
                                  motorPoint->Traj.T / motorPoint->Traj.Tds;  
         }   
         else if(motorPoint->Traj.Ramp == SIN_2_RAMP)
         {
            motorPoint->Traj.Sn_1 = motorPoint->Traj.Sn;
            motorPoint->Traj.Sn = motorPoint->Traj.SII +
                                  motorPoint->Traj.Vmax * motorPoint->Traj.T -
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vend) *
                                  (motorPoint->Traj.T / 2.0 - motorPoint->Traj.Tds * 
                                  sin(M_PI * motorPoint->Traj.T / motorPoint->Traj.Tds) / 2.0 / M_PI);
                              
            motorPoint->Traj.Vn = motorPoint->Traj.Vmax - 
                                  (motorPoint->Traj.Vmax - motorPoint->Traj.Vend) / 2.0  *
                                  (1.0 - cos(M_PI * motorPoint->Traj.T / motorPoint->Traj.Tds));
         }         

            /* saturate velocity to Vend */
         if(fabs(motorPoint->Traj.Vn) <= fabs(motorPoint->Traj.Vend)) 
            motorPoint->Traj.Vn = motorPoint->Traj.Vend;
         if((fabs(motorPoint->Traj.Vend) < 0.001) && ((motorPoint->Traj.Vn * (float)fsign(motorPoint->Traj.Vold)) < 0.0))
            motorPoint->Traj.Vn = motorPoint->Traj.Vend;
            /* saturate velocity to Vmax */
         if(fabs(motorPoint->Traj.Vn) >= fabs(motorPoint->Traj.Vmax)) 
            motorPoint->Traj.Vn = motorPoint->Traj.Vmax;

         /* **********************************************
            save way/angle points
            ********************************************** */
         motorPoint->Traj.Sold = motorPoint->Traj.Sn;
         motorPoint->Traj.Vold = motorPoint->Traj.Vn;

         if(((motorPoint->Traj.T >= motorPoint->Traj.Tds) && (motorPoint->Traj.Ramp == LIN_RAMP)) ||
            ((motorPoint->Traj.T >= (motorPoint->Traj.Tds - 0.1 * T_ABTAST)) && (motorPoint->Traj.Ramp == SIN_2_RAMP)))      
         {
            /* update the calc-struct index */
            motorPoint->Traj.Tus = 0.0;
            motorPoint->Traj.Tds = 0.0;
            motorPoint->Traj.Tmax = 0.0;
            motorPoint->Traj.Vstart = 0.0;
            motorPoint->Traj.Vend = 0.0;
            motorPoint->Traj.Vmax = 0.0;
            motorPoint->Traj.Sold = 0.0;
            motorPoint->Traj.SI = 0.0;
            motorPoint->Traj.SII = 0.0;
            motorPoint->Traj.Vold = 0.0;
            
            motorPoint->Traj.Status = MOTION_OK;  
            
            motorPoint->Traj.State = _MOTION_READY;
            motorPoint->Traj.T = 0.0;  
         }

         motorPoint->Traj.T += T_ABTAST;

         break;
      }
      /* ****************************************************************
            set maximum velocity 
         **************************************************************** */
      case _MAX_SPEED:
      {
         /* **********************************************
            calculate the way/velocity
            ********************************************** */
         motorPoint->Traj.Sn_1 = motorPoint->Traj.Sn;
         motorPoint->Traj.Sn = motorPoint->Traj.SI + 
                               motorPoint->Traj.Vmax * motorPoint->Traj.T; 

         motorPoint->Traj.Vn = motorPoint->Traj.Vmax;
           
         /* **********************************************
            save way/angle points
            ********************************************** */
         motorPoint->Traj.Sold = motorPoint->Traj.Sn;
         motorPoint->Traj.Vold = motorPoint->Traj.Vn;

#ifdef LIN_RAMP
         if(motorPoint->Traj.T >= motorPoint->Traj.Tmax)
#else
         if(motorPoint->Traj.T >= (motorPoint->Traj.Tmax - 0.1 * T_ABTAST))
#endif          
         {
            if(motorPoint->Traj.Tds > T_DOWNSLOPE_MIN) 
            {
               motorPoint->Traj.State = _DOWN_SLOPE; 
               motorPoint->Traj.SII = motorPoint->Traj.Sn;
               motorPoint->Traj.Tds -= (motorPoint->Traj.T - motorPoint->Traj.Tmax);
            }
            else
            {
					/* update the calc-struct index */
               motorPoint->Traj.Tus = 0.0;
               motorPoint->Traj.Tds = 0.0;
               motorPoint->Traj.Tmax = 0.0;
               motorPoint->Traj.Vstart = 0.0;
               motorPoint->Traj.Vend = 0.0;
               motorPoint->Traj.Vmax = 0.0;
               motorPoint->Traj.Sold = 0.0;
               motorPoint->Traj.SI = 0.0;
               motorPoint->Traj.SII = 0.0;
               motorPoint->Traj.Vold = 0.0;
               
					/* saturate way to s0 */
               if(fabs(motorPoint->Traj.Sn) >= fabs(motorPoint->Traj.S0)) 
                  motorPoint->Traj.Sn = motorPoint->Traj.S0; 
               
               motorPoint->Traj.Status = MOTION_OK; 
               
               motorPoint->Traj.State = _MOTION_READY;
            }
            motorPoint->Traj.T = 0.0;  
         }

         motorPoint->Traj.T += T_ABTAST;

         break;
      }
   }  

   // set way profil 
   motorPoint->Traj.Xs += (motorPoint->Traj.Sn - motorPoint->Traj.Sn_1);
   // set velocity profil
   motorPoint->Traj.Xv = motorPoint->Traj.Vn;
}


/* ************************************************************** */
/*! \brief Generate a trajectory for the velocity control.
 *
 *  Calculate the velocity trajectory -> velocity,... .
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void genTraj_vel( Motor_t * motorPoint ) 
{
	/* save old velocity */
	motorPoint->Traj.Vn_1 = motorPoint->Traj.Vn;
	
	/* if velocity to go is higher than actual velocity 
	   -> increase velocity                             */ 
	if ((motorPoint->Traj.V0 - motorPoint->Traj.Vn_1) > (motorPoint->Traj.AmaxUs * T_ABTAST))
	{
		motorPoint->Traj.Vn += motorPoint->Traj.AmaxUs * T_ABTAST;
		/* if new velocity value is higher than velocity to go 
		   -> saturate velocity                                */
		if (motorPoint->Traj.Vn > motorPoint->Traj.V0)
		{
			motorPoint->Traj.Vn = motorPoint->Traj.V0;
		}
		 
	} 
	/* if velocity to go is lower than actual velocity 
	   -> decrease velocity                             */ 
	else if ((motorPoint->Traj.Vn_1 - motorPoint->Traj.V0) > (motorPoint->Traj.AmaxDs * T_ABTAST))
	{
		motorPoint->Traj.Vn -= motorPoint->Traj.AmaxDs * T_ABTAST;
		/* if new velocity value is lower than velocity to go 
		   -> saturate velocity                                */
		if (motorPoint->Traj.Vn < motorPoint->Traj.V0)
		{
			motorPoint->Traj.Vn = motorPoint->Traj.V0;
		}
		 
	} 
	/* otherwise -> set velocity to velocity to go */
	else
	{
		motorPoint->Traj.Vn = motorPoint->Traj.V0;
	}
	
	/* set reference value */ 
	motorPoint->Traj.Xv = motorPoint->Traj.Vn;
}



/* ************************************************************** */
/*! \brief Handle with the odometry.
 *
 *  Calculate the current velocity and position from the dedicated motor.
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void Odometrie(Motor_t *motorPoint)
{
   int32_t dInc;
   
   /* **********************************************       
      save last increment count and read out the actual count
      ********************************************** */       
   motorPoint->Odo.Inc_1 = motorPoint->Odo.Inc; 
   if(motorPoint->Qdec.nbrTimer == 0)
      motorPoint->Odo.Inc = (long int)motorPoint->Qdec.qTimer0->CNT;
   else
      motorPoint->Odo.Inc = (long int)motorPoint->Qdec.qTimer1->CNT;
    
   /* **********************************************       
      counter overflow
      **********************************************        */
   if((motorPoint->Odo.Inc < ((motorPoint->Odo.IncsPerRot * motorPoint->Odo.Gear * 4) / 3)) && 
      (motorPoint->Odo.Inc_1 > ((motorPoint->Odo.IncsPerRot * motorPoint->Odo.Gear * 8) / 3)))
     dInc = motorPoint->Odo.Inc - motorPoint->Odo.Inc_1 + (motorPoint->Odo.IncsPerRot * motorPoint->Odo.Gear * 4);
   /* **********************************************       
      counter underflow
      **********************************************        */
   else if((motorPoint->Odo.Inc_1 < ((motorPoint->Odo.IncsPerRot * motorPoint->Odo.Gear * 4) / 3)) && 
           (motorPoint->Odo.Inc > ((motorPoint->Odo.IncsPerRot * motorPoint->Odo.Gear * 8) / 3)))
     dInc = motorPoint->Odo.Inc - motorPoint->Odo.Inc_1 - (motorPoint->Odo.IncsPerRot * motorPoint->Odo.Gear * 4);
   /* **********************************************       
      otherwise...
     **********************************************        */
   else
     dInc = motorPoint->Odo.Inc - motorPoint->Odo.Inc_1;
   
   /* **********************************************       
      calculate the actual counter value
      ********************************************** */       
   if(motorPoint->Odo.invDir == false)
      motorPoint->Odo.IncTotal += dInc;
   else                      
      motorPoint->Odo.IncTotal -= dInc;

   /* **********************************************       
      calculate the actual position
      ********************************************** */       
   motorPoint->Odo.Dis_1 = motorPoint->Odo.Dis;
   motorPoint->Odo.Dis = (float)(motorPoint->Odo.IncTotal) * motorPoint->Odo.DisPerRot / 
                          motorPoint->Odo.IncsPerRot / motorPoint->Odo.Gear / 4.0;

   /* **********************************************       
      calculate the actual velocity
      ********************************************** */       
   motorPoint->Odo.Vel = (motorPoint->Odo.Dis - motorPoint->Odo.Dis_1) / T_ABTAST; 
}



/* ************************************************************** */
/*! \brief Initialize the QDEC.
 *
 *  total initialization from the dedicated ODEC
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void InitQdec(Motor_t *motorPoint)
{
	/* QDEC initialization
      nbrTimer = 0 -> TC0
	   nbrTimer = 1 -> TC1 */          
	/* TC0 */
    if(motorPoint->Qdec.nbrTimer == 0)
    {
       QDEC_Total_Setup0(motorPoint->Qdec.qPort,
                        motorPoint->Qdec.qPin,
                        motorPoint->Qdec.invIO,
                        motorPoint->Qdec.qEvMux,
                        motorPoint->Qdec.qPinInput,
                        motorPoint->Qdec.useIndex,
                        motorPoint->Qdec.qIndexState,
                        motorPoint->Qdec.qTimer0,
                        motorPoint->Qdec.qEventChannel,
                        motorPoint->Qdec.lineCount);
    }
	 /* TC1 */
    else
    {
       QDEC_Total_Setup1(motorPoint->Qdec.qPort,
                        motorPoint->Qdec.qPin,
                        motorPoint->Qdec.invIO,
                        motorPoint->Qdec.qEvMux,
                        motorPoint->Qdec.qPinInput,
                        motorPoint->Qdec.useIndex,
                        motorPoint->Qdec.qIndexState,
                        motorPoint->Qdec.qTimer1,
                        motorPoint->Qdec.qEventChannel,
                        motorPoint->Qdec.lineCount);
    
    }   
}




/* ************************************************************** */
/*! \brief Initialize the PWM channel.
 *
 *  Total initialization from the dedicated PWM channel.
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void InitPwm(Motor_t *motorPoint)
{
   /* ************************************
      set pin as output
      ************************************ */
   motorPoint->Pwm.qPort->DIR |= (0x01 << motorPoint->Pwm.qPin);

   /* ************************************
      set PWM frequency
      ************************************ */
   if(motorPoint->Pwm.Frequence > 490)
   {
		 if (motorPoint->Pwm.nbrTimer == 0)
		 {
				motorPoint->Pwm.qTimer0->CTRLA = 0x01;              
				motorPoint->Pwm.qTimer0->PER = SYSTEM_CLOCK / (unsigned long)motorPoint->Pwm.Frequence;         
		 }
		 else
		 {
				motorPoint->Pwm.qTimer1->CTRLA = 0x01;              
				motorPoint->Pwm.qTimer1->PER = SYSTEM_CLOCK / (unsigned long)motorPoint->Pwm.Frequence;         
		 }		 
      
   } 
   else if(motorPoint->Pwm.Frequence > 245)
   {
		if (motorPoint->Pwm.nbrTimer == 0)
		{
				motorPoint->Pwm.qTimer0->CTRLA = 0x02;              
				motorPoint->Pwm.qTimer0->PER = SYSTEM_CLOCK / 2 / (unsigned long)motorPoint->Pwm.Frequence;         
		}
		else
		{
				motorPoint->Pwm.qTimer1->CTRLA = 0x02;              
				motorPoint->Pwm.qTimer1->PER = SYSTEM_CLOCK / 2 / (unsigned long)motorPoint->Pwm.Frequence;         
		}		 
   } 
   else if(motorPoint->Pwm.Frequence > 123)
   {
		if (motorPoint->Pwm.nbrTimer == 0)
		{
				motorPoint->Pwm.qTimer0->CTRLA = 0x03;              
				motorPoint->Pwm.qTimer0->PER = SYSTEM_CLOCK / 4 / (unsigned long)motorPoint->Pwm.Frequence;         
		}
		else
		{
				motorPoint->Pwm.qTimer1->CTRLA = 0x03;              
				motorPoint->Pwm.qTimer1->PER = SYSTEM_CLOCK / 4 / (unsigned long)motorPoint->Pwm.Frequence;         
		}		 
   } 
   else if(motorPoint->Pwm.Frequence > 62)
   {
		if (motorPoint->Pwm.nbrTimer == 0)
		{
				motorPoint->Pwm.qTimer0->CTRLA = 0x04;              
				motorPoint->Pwm.qTimer0->PER = SYSTEM_CLOCK / 8 / (unsigned long)motorPoint->Pwm.Frequence;         
		}
		else
		{
				motorPoint->Pwm.qTimer1->CTRLA = 0x04;              
				motorPoint->Pwm.qTimer1->PER = SYSTEM_CLOCK / 8 / (unsigned long)motorPoint->Pwm.Frequence;         
		}		 
   } 
   else if(motorPoint->Pwm.Frequence > 7)
   {
		if (motorPoint->Pwm.nbrTimer == 0)
		{
				motorPoint->Pwm.qTimer0->CTRLA = 0x05;              
				motorPoint->Pwm.qTimer0->PER = SYSTEM_CLOCK / 64 / (unsigned long)motorPoint->Pwm.Frequence;         
		}
		else
		{
				motorPoint->Pwm.qTimer1->CTRLA = 0x05;              
				motorPoint->Pwm.qTimer1->PER = SYSTEM_CLOCK / 64 / (unsigned long)motorPoint->Pwm.Frequence;         
		}		 
   } 
   else if(motorPoint->Pwm.Frequence > 2)
   {
		if (motorPoint->Pwm.nbrTimer == 0)
		{
				motorPoint->Pwm.qTimer0->CTRLA = 0x06;              
				motorPoint->Pwm.qTimer0->PER = SYSTEM_CLOCK / 256 / (unsigned long)motorPoint->Pwm.Frequence;         
		}
		else
		{
				motorPoint->Pwm.qTimer1->CTRLA = 0x06;              
				motorPoint->Pwm.qTimer1->PER = SYSTEM_CLOCK / 256 / (unsigned long)motorPoint->Pwm.Frequence;         
		}		 
   } 
   
   /* ************************************
      set start PWM value for no motion
      ************************************ */
   if((motorPoint->Pwm.nbrTimer == 0) && (motorPoint->Pwm.Driver == LMD18200))
   {
		motorPoint->Pwm.PWM = motorPoint->Pwm.qTimer0->PER / 2;
   }	
   else if((motorPoint->Pwm.nbrTimer == 1) && (motorPoint->Pwm.Driver == LMD18200))
   {
			motorPoint->Pwm.PWM = motorPoint->Pwm.qTimer1->PER / 2;
   }
   else if(motorPoint->Pwm.Driver == BD6232HFP)
      motorPoint->Pwm.PWM = 0;

   /* ************************************
      set value to the channel
	   ************************************ */
	if (motorPoint->Pwm.nbrTimer == 0)
	{
		motorPoint->Pwm.qTimer0->CTRLB |= 0x03 | (0x10 << motorPoint->Pwm.Channel);
		if(motorPoint->Pwm.Channel == CH_A)
			motorPoint->Pwm.qTimer0->CCA = motorPoint->Pwm.PWM;         
		else if(motorPoint->Pwm.Channel == CH_B)
			motorPoint->Pwm.qTimer0->CCB = motorPoint->Pwm.PWM;          
		else if(motorPoint->Pwm.Channel == CH_C)
			motorPoint->Pwm.qTimer0->CCC = motorPoint->Pwm.PWM;         
		else if(motorPoint->Pwm.Channel == CH_D)
			motorPoint->Pwm.qTimer0->CCD = motorPoint->Pwm.PWM;       
	}
	else
	{
		motorPoint->Pwm.qTimer1->CTRLB |= 0x03 | (0x10 << motorPoint->Pwm.Channel);
		if(motorPoint->Pwm.Channel == CH_A)
			motorPoint->Pwm.qTimer1->CCA = motorPoint->Pwm.PWM;         
		else if(motorPoint->Pwm.Channel == CH_B)
			motorPoint->Pwm.qTimer1->CCB = motorPoint->Pwm.PWM;          
	}		 

}



/* ************************************************************** */
/*! \brief Set motor voltage.
 *
 *  Set dedicated voltage considering the battery voltage. 
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void SetU_Motor(Motor_t *motorPoint)
{
   /* ***************************************
      saturate the voltage to the battery voltage 
      ***************************************  */
   if(motorPoint->Vel.Y < -uBat)
      motorPoint->Vel.Y = -uBat;   
   if(motorPoint->Vel.Y > uBat)
      motorPoint->Vel.Y = uBat; 

   /* ***************************************
      invert/no invert the voltage 
      ***************************************  */
   motorPoint->Vel.Y *= motorPoint->Pwm.invU;
   
   
   /* ***************************************
      LMD18200 
      ***************************************  */
   if(motorPoint->Pwm.Driver == LMD18200)
   {
		if(motorPoint->Pwm.nbrTimer == 0)
		{
			motorPoint->Pwm.PWM = (uint16_t)((float)(motorPoint->Pwm.qTimer0->PER / 2.0) * 
		                         (1 +  motorPoint->Vel.Y / motorPoint->Vel.Sat_max));
		}
		else
		{
			motorPoint->Pwm.PWM = (uint16_t)((float)(motorPoint->Pwm.qTimer1->PER / 2.0) * 
		                         (1 +  motorPoint->Vel.Y / motorPoint->Vel.Sat_max));
		}						
   }
   /* ***************************************
      BD63HFP
      ***************************************  */
   if(motorPoint->Pwm.Driver == BD6232HFP)
   {
      if(motorPoint->Vel.Y < 0.0)
      {
			motorPoint->Pwm.qDIRport->OUTCLR = (0x01 << motorPoint->Pwm.qDIRpin);
			
         if(fabs(motorPoint->Vel.Y) > motorPoint->Vel.Sat_max)
            motorPoint->Vel.Y = -motorPoint->Vel.Sat_max;
			if(motorPoint->Pwm.nbrTimer == 0)
			{
				motorPoint->Pwm.PWM = (uint16_t)(((float)(motorPoint->Pwm.qTimer0->PER)) * 
                               (fabs(motorPoint->Vel.Y) / motorPoint->Vel.Sat_max));
			}
			else
			{
				motorPoint->Pwm.PWM = (uint16_t)(((float)(motorPoint->Pwm.qTimer1->PER)) * 
                               (fabs(motorPoint->Vel.Y) / motorPoint->Vel.Sat_max));
			}							
      }
      else
      {
			motorPoint->Pwm.qDIRport->OUTSET = (0x01 << motorPoint->Pwm.qDIRpin);

         if(motorPoint->Vel.Y > motorPoint->Vel.Sat_max)
            motorPoint->Vel.Y = motorPoint->Vel.Sat_max;
			
			if (motorPoint->Pwm.nbrTimer == 0)
			{
				motorPoint->Pwm.PWM = (uint16_t)(((float)(motorPoint->Pwm.qTimer0->PER)) * 
                               (1.0 - (motorPoint->Vel.Y / motorPoint->Vel.Sat_max)));
			}	
			else
			{
				motorPoint->Pwm.PWM = (uint16_t)(((float)(motorPoint->Pwm.qTimer1->PER)) * 
                               (1.0 - (motorPoint->Vel.Y / motorPoint->Vel.Sat_max)));
			}						
      }
   }
   
   /* ***************************************
      set PWM 
      *************************************** */
   if (motorPoint->Pwm.nbrTimer == 0)
   {
		SetPWM0(motorPoint->Pwm.qTimer0, motorPoint->Pwm.Channel, motorPoint->Pwm.PWM);
   }
   else
   {
		SetPWM1(motorPoint->Pwm.qTimer1, motorPoint->Pwm.Channel, motorPoint->Pwm.PWM);
   }
}



/* ************************************************************** */
/*! \brief Velocity control.
 *
 *  Complete the velocity controller. 
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void controlVel(Motor_t *motorPoint)
{
   /* e */
   float fReglerAbweichung;
   
   /* **********************
      calculate e
      ********************** */
   fReglerAbweichung = motorPoint->Traj.Xv + motorPoint->Pos.Y - motorPoint->Odo.Vel;
   
   /* **********************
       calculate y
      ********************** */
   motorPoint->Vel.Integrator += fReglerAbweichung * motorPoint->Vel.Ki * T_ABTAST;
   motorPoint->Vel.Y = fReglerAbweichung * motorPoint->Vel.Kp + motorPoint->Vel.Integrator;
	
	if(motorPoint->Vel.Integrator >= motorPoint->Vel.Sat_max)
		motorPoint->Vel.Integrator = motorPoint->Vel.Sat_max;
	else if(motorPoint->Vel.Integrator <= motorPoint->Vel.Sat_min)
		motorPoint->Vel.Integrator = motorPoint->Vel.Sat_min;

   /* ***************************************
      saturate y
      *************************************** */
   if(motorPoint->Vel.Y > motorPoint->Vel.Sat_max) 
      motorPoint->Vel.Y = motorPoint->Vel.Sat_max;
   else if(motorPoint->Vel.Y < motorPoint->Vel.Sat_min)
      motorPoint->Vel.Y = motorPoint->Vel.Sat_min;         
}



/* ************************************************************** */
/*! \brief Position control.
 *
 *  Complete the position controller. 
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void controlPos(Motor_t *motorPoint)
{
   /* e */
   float fReglerAbweichung;
   
  /* **********************
      calculate e
     ********************** */
   fReglerAbweichung = motorPoint->Traj.Xs - motorPoint->Odo.Dis;
   
  /* **********************
      calculate y
     ********************** */
   motorPoint->Pos.Integrator += fReglerAbweichung * motorPoint->Pos.Ki * T_ABTAST;
   motorPoint->Pos.Y = fReglerAbweichung * motorPoint->Pos.Kp + motorPoint->Pos.Integrator;

   /* ***************************************
      saturate y
      *************************************** */
   if(motorPoint->Pos.Y > motorPoint->Pos.Sat_max) 
      motorPoint->Pos.Y = motorPoint->Pos.Sat_max;
   else if(motorPoint->Pos.Y < motorPoint->Pos.Sat_min)
      motorPoint->Pos.Y = motorPoint->Pos.Sat_min;         
}


/* ************************************************************** */
/*! \brief Set position command.
 *
 *  Initialize the position control. 
 *
 *  \param motorPoint pointer to motor.
 *  \param Vmax maximum velocity.
 *  \param S0 distance to drive.
 *
*/
/* ************************************************************** */
void setMotion(Motor_t *motorPoint, float Vmax, float S0)
{
   motorPoint->Traj.S0 = S0;  
   motorPoint->Traj.Vmax = Vmax;  
   motorPoint->Traj.State = _NEW_MOTION; 
}


/* ************************************************************** */
/*! \brief Set velocity command.
 *
 *  Initialize the velocity control. 
 *
 *  \param motorPoint pointer to motor.
 *  \param V0 velocity to drive.
 *
*/
/* ************************************************************** */
void setVelocity(Motor_t *motorPoint, float V0)
{
	motorPoint->Traj.V0 = V0; 
	motorPoint->Pos.Y = 0.0;
}

/* ************************************************************** */
/*! \brief Control the motion.
 *
 *  Control the whole motion, what mean generate the trajectory, 
 *  calculate the odometry data, position and velocity control and
 *  set the motor voltage. 
 *
 *  \param motorPoint pointer to motor.
 *
*/
/* ************************************************************** */
void controlMotion(Motor_t *motorPoint)
{
	char text[100];
	
	
   /* ************************************
      generate the trajectory (s, v)
      ************************************ */
   if (motorPoint->Pos.controlOn != 0)
   {
		genTraj_pos(motorPoint);
   }
   else if (motorPoint->Vel.controlOn != 0)
   {
		genTraj_vel(motorPoint);
   }
   
   /* ************************************
      calculate the odometry data (s, v)
      ************************************ */
   Odometrie(motorPoint);         
   
   /* ************************************
      position control
      ************************************ */
   if(motorPoint->Pos.controlOn != 0)
   {
		controlPos(motorPoint);
   }	
   
   /* ************************************
      velocity control
      ************************************ */
   if(motorPoint->Vel.controlOn != 0)
   {
		controlVel(motorPoint);
   }

   /* ************************************
      set motor voltage
      ************************************ */
   SetU_Motor(motorPoint);   

 		//sprintf(text,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \r", motorRight.Traj.Vn, motorRight.Odo.Vel, motorRight.Vel.Y, motorLeft.Traj.Vn, motorLeft.Odo.Vel, motorLeft.Vel.Y);																
  		//sprintf(text,"%4.3f;%4.3f \r", motorLeft.Traj.Xs, motorLeft.Odo.Dis);																
//		writeString_usart(&usartC1, text);

}