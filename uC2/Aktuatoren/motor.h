//* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      motor driver header file.
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


#ifndef _MOTOR_H
#define _MOTOR_H


/* intern/extern switch */
#ifndef _MOTOR_EXTERN
   #define _MOTOR_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */

/* ****************************
   QDEC
   **************************** */
struct qdecMotor
{
   PORT_t * qPort;               /*!< QDEC port */
   uint8_t qPin;                 /*!< pin of A signal (B signal = Pin_A + 1) */
   uint8_t invIO;                /*!< invert pin -> true ... invert; false ... invert not */
   uint8_t qEvMux;               /*!< Event multiplexer (only 0, 2 or 4 allowed) */
   EVSYS_CHMUX_t qPinInput;      /*!< input of event system */
   uint8_t useIndex;             /*!< use index pin -> true ... use it; false ... don't use it */
   EVSYS_QDIRM_t qIndexState;    /*!< status to trigger the indexe */ 
   uint8_t nbrTimer;             /*!< timer number (0 ... TC0; 1 ... TC1) */
   TC0_t * qTimer0;              /*!< pointer to QDEC TC0 */
   TC1_t * qTimer1;              /*!< pointer to QDEC TC1 */
   TC_EVSEL_t qEventChannel;     /*!< used event channel */
   uint16_t lineCount;           /*!< counts per rotation */
	uint16_t encDivider;
};

typedef struct qdecMotor qdecMotor_t; 



/* ****************************
   PWM 
   **************************** */
struct pwmMotor
{
   uint8_t Driver;               /*!< motor driver (LMD18200, BD6232HFP) */
   PORT_t * qPort;               /*!< PWM port */
   uint8_t qPin;                 /*!< PWM pin */
   uint8_t nbrTimer;             /*!< timer number (0 ... TC0; 1 ... TC1) */
   TC0_t * qTimer0;              /*!< pointer to used timer (TC0) */
   TC1_t * qTimer1;              /*!< pointer to used timer (TC1) */
   uint8_t Channel;              /*!< used PWM channel */
   uint8_t Channel2;
   uint16_t Frequence;           /*!< PWM frequency */
   uint16_t PWM;                 /*!< PWM value */
	uint16_t PWM2;  
   float invU;                   /*!< -1 ... skip sign of U, 1 ... don't skip sign */
   PORT_t * qDIRport;				/*!< direction port */
   uint8_t qDIRpin;					/*!< direction pin */
   uint8_t qDIRpin2;
	USART_t * usart;					/*!< pointer to usart (is need for sabertooth module) */
};

typedef struct pwmMotor pwmMotor_t;


/* ****************************
  odometry
  **************************** */
struct odoMotor
{
   float IncsPerRot;             /*!< increments per rotation (encoder) */
   float Gear;                   /*!< gear ratio */
   float DisPerRot;              /*!< distance per rotation (on shaft) */
   int32_t Inc;                  /*!< actual counter value at t */
   int32_t Inc_1;                /*!< old counter value at t-1 */
   int32_t IncTotal;					/*!< total increments */
   uint8_t invDir;               /*!< invert counter direction (true) or not (false) */
   float Dis;                    /*!< actual distance at t */
   float Dis_1;                  /*!< actual distance at t-1 */
   float Vel;                    /*!< actual velocity */
};

typedef struct odoMotor odoMotor_t; 


/* ****************************
   closed loop controller
   **************************** */
struct controlMotor_s
{
   uint8_t controlOn;            /*!< closed loop controller on (true) or off (false) */
   float Kp;                     /*!< KP of controller */
   float Ki;                     /*!< KI of controller */
   float Integrator;             /*!< integrator value */
   float Sat_min;                /*!< minimum saturation */
   float Sat_max;                /*!< maximum saturation */
   float X;                      /*!< input (r) */
   float Y;                      /*!< output (y) */
};

typedef struct controlMotor_s controlMotor_t;


/* *******************************************
   trajectory generation  
   ******************************************* */
#define _MOTION_READY       0
#define _UP_SLOPE           1
#define _DOWN_SLOPE         2
#define _MAX_SPEED          3      
#define _NEW_MOTION         4

/* maximum ratio between veocity and way */
#define SPEED_LIMIT_FACTOR    10.0

/* limit of the ramp (maximum percent of ramp 0.9 <-> 90%) */
#define RAMP_LIMIT            0.9

/* minimum up slope time (in sec) */
#define T_UPSLOPE_MIN      0.001
/* maximum down slope time (in sec) */
#define T_DOWNSLOPE_MIN    0.001
/* maximum time (in sec) */
#define T_MAX_MIN          0.001

/* sampling time */
#define T_ABTAST     0.01

/* type of ramp -> sin² or linear */
#define LIN_RAMP     0
#define SIN_2_RAMP   1

#define MOTION_OK		1

struct genTra
{
   float S0;                     /*!< way to go (r) */
   float Sn;                     /*!< actual way point at t */
   float Sn_1;                   /*!< old way point at t-1 */
   float V0;							/*!< velocity to go (r) */
   float Vn;							/*!< actual velocity at t */
   float Vn_1;							/*!< old velocity at t-1 */
   float Sold;                   /*!< old way point (after up slope,...) */
   float Vold;                   /*!< old velocity (after up slope,...) */
   float Tus;                    /*!< up slope time */
   float Tds;                    /*!< down slope time */
   float Tmax;                   /*!< time of maximum velocity */
   float Vmax;                   /*!< maximum velocity */
   float Vstart;                 /*!< start velocity */
   float Vend;                   /*!< end velocity */
   float SI;                     /*!< way after up slope */
   float SII;                    /*!< way after Tmax */
   float T;                      /*!< actual time */
   float AmaxUs;                 /*!< maximum acceleration at up slope */
   float AmaxDs;                 /*!< maximum acceleration at down slope */
   float Xs;                     /*!< actual way point for controller */
   float Xv;                     /*!< actual velocity for controller */
   unsigned char State;          /*!< actual state (up slope,...) */
   unsigned char Status;
   unsigned char Ramp;           /*!< ramp type of velocity (linear or sin²) */
};

typedef struct genTra genTraj_t;


/* types of motor driver */
#define LMD18200				1
#define BD6232HFP				2
#define BTN7970				3
#define SABERTOOTH_2X60		4

struct curLimit
{
	uint8_t enable;
	uint8_t setUp_currentLimiter;
	float I_max;						/*!< maximum motor current */
	float deltaTauLim;				/*!< delta that minimize/maximize the saturation in over current */
	float deltaTauUnLim;				/*!< delta that minimize/maximize the saturation in over current */
	uint8_t I_channel_HS;			/*!< ADC channel of motor plus */
	uint8_t I_channel_LS;			/*!< ADC channel of motor minus */
	uint16_t I_offset_HS;
	uint16_t I_offset_LS;
	uint16_t (*adc)(uint8_t,uint8_t);
};
typedef struct curLimit curLimit_t;

/* ****************************
   motor struct (with odometry, trajectory planning, velocity- and position-controller, QDEC and PWM) */
// ****************************
struct Motor
{
   qdecMotor_t Qdec;
   pwmMotor_t Pwm;
   odoMotor_t Odo;
   genTraj_t Traj;      
   controlMotor_t Vel;
   controlMotor_t Pos;
	curLimit_t curLim;
}; 

typedef struct Motor Motor_t;



/* motors */
_MOTOR_EXTERN Motor_t liftRear;


/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
uint8_t liftMotorTask();
uint8_t setUp_curLim(Motor_t *motorPoint);
float getMotorCurrent(Motor_t *motorPoint);
void curentLimiter(Motor_t *motorPoint, float current, float uBat);
void genTraj_pos(Motor_t *motorPoint);
void genTraj_vel(Motor_t * motorPoint); 
void Odometrie(Motor_t *motorPoint);
void InitQdec(Motor_t *motorPoint);
void InitPwm(Motor_t *motorPoint);
void SetU_Motor(Motor_t *motorPoint);
void controlVel(Motor_t *motorPoint);
void controlPos(Motor_t *motorPoint);
void setMotion(Motor_t *motorPoint, float Vmax, float S0);
void setVelocity(Motor_t *motorPoint, float V0);
void controlMotion(Motor_t *motorPoint);
void resetOdometry(Motor_t * motorPoint, float pos);
void resetTrajectory(Motor_t * motorPoint);
float getAngleMot(Motor_t * motor_point);
                    

#endif


