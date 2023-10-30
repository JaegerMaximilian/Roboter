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


#define CS_INIT_STATE_00	0
#define CS_INIT_STATE_0		2
#define CS_INIT_STATE_1		5
#define CS_INIT_STATE_2		10
#define CS_INIT_STATE_3		12
#define CS_INIT_STATE_4		13
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
	float	ServoArmPosRef;
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

void InitArms();
uint8_t ArmLeftTask();
uint8_t ArmRightTask();

#endif /* ARM_H_ */