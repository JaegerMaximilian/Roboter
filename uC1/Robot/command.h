/*
 * command.h
 *
 * Created: 27.10.2020 10:59:13
 *  Author: P20087
 */ 


#ifndef COMMAND_H_
#define COMMAND_H_

#include "sensor.h"
#include "ki.h"
#include "Pfadplanung.h"
#include "global.h"
#include "rrt_receivedata.h"
#include "define.h"

#ifndef _COMMAND_EXTERN
#define _COMMAND_EXTERN	extern
#endif

#define CHECK_ROBOT_TYPE(b, c) ((RobotType_RAM == MASTER_ROBOT) ? b : c)

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* **************************** */
/*         M O T O R S          */
/* **************************** */
/* motor-numbers */

#define LIFT_FRONT_LEFT_NBR		0
#define LIFT_FRONT_RIGHT_NBR	1
#define LIFT_REAR_NBR			2


/* motor-positions [m] */
#define LIFT_FRONT_POS_0		0.05
#define LIFT_FRONT_POS_1		0.010
#define LIFT_FRONT_POS_2		0.145 //0.155
#define LIFT_FRONT_POS_3		0.190 //190
#define LIFT_FRONT_POS_SHUF		0.055
#define LIFT_FRONT_POS_REF		0.008

#define LIFT_TOLERANCE			0.002
#define LIFT_REL_POS_HUB		0.015

/* motor-speed [m/s] */
#define LIFT_FRONT_SPEED		0.22   //0.12 = standard  0.4
#define ANGULAR_VELOCITY		300	//120


// Angel Cakebuilder
#define ANGEL_CAKEBUILD			35
#define ANGEL_CAKEBUILD_CHERRY	40

#define LIFT_BACK_POS_0			50
#define LIFT_BACK_POS_1			75
#define LIFT_BACK_POS_2			100


	
/* **************************** */
/*         V A C U U M          */
/* **************************** */
/* vacuum-pump-number */
#define VACUUM_FRONT_LEFT_NBR	0
#define VACUUM_FRONT_RIGHT_NBR	1
#define VACUUM_REAR_LEFT_NBR	4
#define VACUUM_REAR_RIGHT_NBR	5
#define MV_REAR_LEFT_NBR		6
#define MV_REAR_RIGHT_NBR		7

/* vacuum commands */
#define CMD_VACUUM_ON		1
#define CMD_VACUUM_OFF		0

/* **************************** */
/*         S E R V O S          */
/* **************************** */
/* 0 < SERVOPOS < 135 */
/* servo positions (inactive) [°] */

#define SE_VR			67
#define SE_HL			135
#define SE_S_SILO_LEFT1_0	95
#define SE_HR			67
#define SE_S_SILO_LEFT2_0	85
#define SE_RES1	0
#define SE_S_SILO_RIGHT1_0	0
#define SE_RES2	0
#define SE_S_SILO_RIGHT2_0	0
#define SE_RES4	30 //30
#define SE_S_SUCKER_LEFT_0  50 //
#define SE_RES3	125 //90
#define SE_S_SUCKER_RIGHT_0	70 // 70
#define SE_PINCE_NEZ1_0		122
#define SE_PINCE_NEZ2_0		10
#define SE_PINCE_NEZ3_0		60 // 55
#define SE_PINCE_NEZ4_0		30 // 25
#define SE_PINCE_NEZ5_0		40 // 35
#define SE_ARM_LEFT_0		5
#define SE_VL		0
#define SE_FLAG1_0			93 // 25
#define SE_FLAG2_0		    80 // 35

/* servo positions (active) [°] */
/* SE_S stand for slave robot - some positions need robot individual settings*/

#define SE_RES2_1			67
#define SE_SILO_LEFT1_1		65 //70
#define SE_S_SILO_LEFT1_1	68 //68
#define SE_SILO_LEFT2_1		60 //65
#define SE_S_SILO_LEFT2_1	60 //35
#define SE_SILO_RIGHT1_1	62
#define SE_S_SILO_RIGHT1_1	20 //60
#define SE_SILO_RIGHT2_1	65 //50
#define SE_S_SILO_RIGHT2_1	20 //65
#define SE_SUCKER_LEFT_1	70 //65 
#define SE_S_SUCKER_LEFT_1  100 //
#define SE_SUCKER_RIGHT_1   87
#define SE_S_SUCKER_RIGHT_1 05	//05
#define SE_SUCKER_LEFT_2	55 //45 //neutral position
#define SE_S_SUCKER_LEFT_2	80 //neutral position slave robot
#define SE_SUCKER_RIGHT_2	100 //neutral position
#define SE_S_SUCKER_RIGHT_2	40 //neutral position slave robot

#define SE_PINCE_NEZ1_1		30
#define SE_PINCE_NEZ2_1		85
#define SE_PINCE_NEZ3_1		130
#define SE_PINCE_NEZ4_1		100
#define SE_PINCE_NEZ5_1		110

#define SE_ARM_LEFT_1		70
#define SE_ARM_RIGHT_1		66
#define SE_ARM_RIGHT_2		110
#define SE_FLAG1_1			2
#define SE_FLAG2_1		    40 

/* ******************** */
/* Ultrasonic Sensor to base coordinate system */
/* ******************** */
#define OFFSET_FRONT_X		140
#define OFFSET_FRONT_Y		55
#define OFFSET_REAR_X		85
#define OFFSET_REAR_Y		80

/* ******************** */
/* digital sensors µC 1 */
/* ******************** */
/* lift left: IR and colour front */
#define COLOUR_FRONT_LEFT	((uC1sensorenPortA & 0x10) ? 1 : 0)
#define IR_FRONT_LEFT		((uC1sensorenPortF & 0x02) ? 1 : 0)
/* lift right: IR and colour front */
#define COLOUR_FRONT_RIGHT	((uC1sensorenPortA & 0x20) ? 1 : 0)
#define IR_FRONT_RIGHT		((uC1sensorenPortF & 0x01) ? 1 : 0)
/* end switches cupstacker */
#define REF_SWITCH_LEFT ((uC1sensorenPortA & 0x80) ? 1 : 0)
#define REF_SWITCH_RIGHT ((uC1sensorenPortA & 0x40) ? 1 : 0)


/* ******************** */
/* digital sensors µC 2 */
/* ******************** */
/* reflex sensor */
#define REFLEX_1			((uC2sensorenPortB & 0x80) ? 1 : 0)
#define REFLEX_2			((uC2sensorenPortB & 0x40) ? 1 : 0)
#define REFLEX_3			((uC2sensorenPortB & 0x20) ? 1 : 0)
#define REFLEX_4			((uC2sensorenPortB & 0x10) ? 1 : 0)
#define REFLEX_TOTAL		((0x01 * REFLEX_1) + (0x02 * REFLEX_2) + (0x04 * REFLEX_3) + (0x08 * REFLEX_4))
#define DO_LED1_NBR			3
#define DO_LED2_NBR			1
#define DO_LED3_NBR			2
#define DO_POLWENDER_NBR	0

/* ******************** */
/* digital sensors µC 3 */
/* ******************** */
/* lift rear: IR and colour front */
#define COLOUR_REAR_1		((uC3sensorenPortC & 0x40) ? 1 : 0)
#define COLOUR_REAR_2		((uC3sensorenPortC & 0x20) ? 1 : 0)
#define IR_REAR_1			((uC3sensorenPortB & 0x01) ? 1 : 0)
#define IR_REAR_2			((uC3sensorenPortB & 0x02) ? 1 : 0)
#define IR_REAR_3			((uC3sensorenPortB & 0x04) ? 1 : 0)
#define IR_REAR_4			((uC3sensorenPortB & 0x08) ? 1 : 0)
#define IR_REAR_5			((uC3sensorenPortB & 0x10) ? 1 : 0)
/* reserve */
#define IR_RESERVE_1		((uC3sensorenPortC & 0x10) ? 1 : 0)
#define IR_RESERVE_2		((uC3sensorenPortB & 0x40) ? 1 : 0)
#define IR_RESERVE_3		((uC3sensorenPortB & 0x20) ? 1 : 0)
#define COLOUR_NORTH_SOUTH	((uC3sensorenPortC & 0x80) ? 1 : 0)

/* ******************** */
/* Roboter Typen */
/* ******************** */
#define MASTER_ROBOT 1
#define SLAVE_ROBOT 2

_COMMAND_EXTERN robot_t master, slave;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void cmd_SetServo(uint8_t nbr, int16_t angle);
void cmd_SetMotorPos(uint8_t nbr, float vel, float pos);
void cmd_SetMotorVel(uint8_t nbr, float vel);
void cmd_CtrlVacuum(uint8_t nbr, uint8_t val);
void cmd_Drive(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint16_t rSoll, uint16_t xSoll, uint16_t ySoll, uint16_t sSoll, uint8_t type, uint8_t gegnerErkennung, element_t *wP, uint8_t nOP, uint8_t AmaxUs,uint8_t AmaxDs);
void cmd_DriveToPoint_ABS(int16_t vStart, int16_t vEnd, int16_t vMax, uint16_t xSoll, uint16_t ySoll, uint8_t gegnerErkennung);
void cmd_DriveToPos_REL(int16_t vStart, int16_t vEnd, int16_t vMax, uint16_t sSoll, uint8_t gegnerErkennung);
void cmd_TurnAroundAngle_ABS(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint8_t gegnerErkennung);
void cmd_TurnAroundAngle_REL(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint8_t gegnerErkennung);
void cmd_DriveArc(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint16_t rSoll, uint8_t gegnerErkennung);
void cmd_DrivePath(int16_t vMax,uint8_t gegnerErkennung, point_t *wP, uint8_t nOP);
void cmd_SetDigitalOut(uint8_t nbr, uint8_t val);

#endif /* COMMAND_H_ */