/*
 * Arm.h
 *
 * Created: 15/12/2020 09:56:22
 *  Author: danny
 */ 


#ifndef ARM_H_
#define ARM_H_


#ifndef _ARM_EXTERN_
	#define _ARM_EXTERN_ extern
#endif

#include <avr/io.h>
#include "motor.h"
#include "PSE541.h"

#define CS_WAIT_FOR_INIT	140
#define CS_INIT_STATE_0		0
#define CS_INIT_STATE_1		5
#define CS_INIT_STATE_2		10
#define CS_INIT_STATE_3		11
#define CS_READY_STATE		15
#define CS_SUCK_CUP			20
#define CS_STORE_CUP		25
#define CS_LOCK1_CUP		30
#define CS_LOCK2_CUP		35
#define CS_WAIT_POS			40
#define CS_WAIT_READY_PLACE 100
#define CS_READY_PLACE		110
#define CS_PLACE_TOP_0		115
#define CS_PLACE_TOP_1		120
#define CS_PLACE_TOP_2		130


#define CS_RED			1
#define CS_GREEN		0
#define CS_EMPTY		2
#define CS_STACKSIZE	4

//op_Mode
#define CS_COLLECT		0
#define CS_PLACE		1
#define CS_IDLE			2
#define CS_LOCK			3

#define CS_TIME_OUT		200		// 400 ... 4 sec



typedef struct  
{
	uint8_t State;
// 	uint8_t *ColorSensorPort;
// 	uint8_t ColorSensorMask;
// 	uint8_t *IRSensorPort;
// 	uint8_t IRSensorMask;
	uint8_t *RefSwitchPort;
	uint8_t RefSwitchMask;
	uint8_t VacNumber;
	uint8_t MotNumber;
	uint8_t Cupstack[CS_STACKSIZE];
	uint8_t hexagonNumber;
	uint8_t ServoArm;
	uint8_t ServoSilo2;
	uint8_t ServoSucker;
	uint8_t ServoArmPos0;
	uint8_t ServoArmPos1;
	float ServoArmPosRef;
// 	uint8_t ServoSilo2Pos0;
// 	uint8_t ServoSilo2Pos1;
	uint8_t ServoSuckerPos0;
	uint8_t ServoSuckerPos1;
	uint8_t ServoSuckerPos2;
	uint8_t WaitStateCycles;
	uint8_t NextState;
	uint8_t taskNbr;
	uint8_t opMode;  // opperation Mode: 0 .. collect; 
	Motor_t *motor;
	PSE541_t *pressure;
	uint16_t timeOut;
		
} Arm_t;


/*#define CHECK_COLOR(a) (((*(a->ColorSensorPort)) & a->ColorSensorMask) ? CS_RED : CS_GREEN)*/
#define CHECK_REF(a) (((*(a->RefSwitchPort)) & a->RefSwitchMask) ? 0 : 1)
/*#define CHECK_IR(a) (((*(a->IRSensorPort)) & a->IRSensorMask) ? 1 : 0)*/


_ARM_EXTERN_ Arm_t ArmLeft, ArmRight;
_ARM_EXTERN_ uint8_t WaitForInit;

void InitArms();
uint8_t ArmLeftTask();
uint8_t ArmRightTask();

/* **************************** */
/*         V A C U U M          */
/* **************************** */
/* vacuum-pump-number */
#define VACUUM_REAR_LEFT_NBR	0
#define VACUUM_REAR_RIGHT_NBR	1

/* vacuum commands */
#define CMD_VACUUM_ON		1
#define CMD_VACUUM_OFF		0

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* **************************** */
/*         M O T O R S          */
/* **************************** */
/* motor-numbers */
#define LIFT_FRONT_LEFT_NBR		0
#define LIFT_FRONT_RIGHT_NBR	1
#define LIFT_REAR_LEFT_NBR		2
#define LIFT_REAR_RIGHT_NBR		3

/* motor-positions [m] */
#define LIFT_FRONT_POS_0		0.05
#define LIFT_FRONT_POS_1		0.010
#define LIFT_FRONT_POS_2		0.145 //0.155
#define LIFT_FRONT_POS_3		0.190 //190
#define LIFT_FRONT_POS_SHUF		0.055
#define LIFT_FRONT_POS_REF		0.008
/* motor-speed [m/s] */
#define LIFT_FRONT_SPEED		0.18   //0.12 = standard

#define LIFT_BACK_POS_0			50
#define LIFT_BACK_POS_1			75
#define LIFT_BACK_POS_2			100

#define SE_HL_NBR	2
#define SE_HR_NBR	3

#endif /* ARM_H_ */