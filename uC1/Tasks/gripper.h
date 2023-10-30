/*
* gripper.h
*
* Created: 26.01.2023 08:40:37
*  Author: perne
*/


#ifndef GRIPPER_H_
#define GRIPPER_H_

#include <avr/io.h>
#include "motor.h"

#ifndef _GRIPPER_EXTERN_
#define _GRIPPER_EXTERN_ extern
#endif

#define GR_INIT_STATE_0		0
#define GR_INIT_STATE_1		10
#define GR_INIT_STATE_2		20
#define GR_INIT_STATE_3		30
#define GR_INIT_STATE_4		40
#define GR_READY_STATE		50
#define GR_READY_PLACE		60
#define GR_READY_GRIP1		70
#define GR_READY_GRIP2		75
#define GR_READY_GRIP3		76
#define GR_READY_OPMODE1	80
#define GR_READY_TAKEOFF	90
#define GR_READY_GRIPNEXT1	100
#define GR_READY_GRIPNEXT1a	101
#define GR_READY_GRIPNEXT2	110
#define GR_READY_GRIPNEXT3	120
#define GR_READY_GRIPNEXT4	130
#define GR_READY_OPMODE2	140
#define GR_READY_GRIPFULL	150
#define GR_READY_GRIPFULL2	160
#define GR_READY_GRIPFULL2a	161
#define GR_READY_GRIPFULL3	170
#define GR_READY_GRIPFULL4	180
#define GR_READY_OPMODE3	190
#define GR_READY_Ready	 	191

#define GR_CAKEBUILDER_START	195
#define GR_CAKE1_BUILD1			200
#define GR_CAKE1_OPEN			210
#define GR_CAKE1_CLOSE			220
#define GR_CAKE1_LIFT			230
#define GR_CAKE1_TURN			240
#define GR_CAKE1_SINK			250
#define GR_CAKE2_OPEN			215
#define GR_CAKE2_CLOSE			225
#define GR_CAKE2_LIFT			235
#define GR_CAKE2_TURN			245
#define GR_CAKE3_OPEN			255
#define GR_CAKE3_SINK			256
#define GR_CAKE4_BUILD2			260
#define GR_CAKE4_CLOSE			270
#define GR_CAKE4_LIFT			280
#define GR_CAKE4_TURN			290
#define GR_CAKE4_SINK			300
#define GR_CAKE5_OPEN			310
#define GR_CAKE5_CLOSE			320
#define GR_CAKE5_LIFT			330
#define GR_CAKE5_TURN			340
#define GR_CAKE6_OPEN			350
#define GR_CAKE6_SINK			351
#define GR_CAKE7_BUILD3			360
#define GR_CAKE7_CLOSE			370
#define GR_CAKE7_LIFT			380
#define GR_CAKE7_TURN			390
#define GR_CAKE7_SINK			400
#define GR_CAKE8_OPEN			410
#define GR_CAKE8_CLOSE			420
#define GR_CAKE8_LIFT			430
#define GR_CAKE8_TURN			440
#define GR_CAKE8_SINK			445
#define GR_CAKE9_OPEN			450
#define GR_CAKE9_CHERRYPOS		460
#define GR_GRIPCHERRY_POS		470
#define GR_CHERRY1_TURN			480
#define GR_CHERRY1_CAKEBUILD3	490
#define GR_CHERRY1_OPEN			500
#define GR_CHERRY1_CLOSE		510
#define GR_CHERRY1_CHERRYPOS	520
#define GR_CHERRY2_TURN			530
#define GR_CHERRY2_CAKEBUILD3	540
#define GR_CHERRY2_OPEN			550
#define GR_CHERRY2_CLOSE		560
#define GR_CHERRY2_CHERRYPOS	570
#define GR_CHERRY3_TURN			580
#define GR_CHERRY3_CAKEBUILD3	590
#define GR_CHERRY3_OPEN			600
#define GR_CHERRY3_CLOSE		700
#define GR_CHERRY3_CHERRYPOS	800
#define GR_CAKE_READY			810
#define GR_3CAKE				820
#define GR_3CAKE_OPEN			830
#define GR_3CAKE_CLOSE			840
#define GR_3CAKE_LIFT			850
#define GR_3CAKE_LIFT2			851
#define GR_3CAKE_TURN			860
#define GR_3CAKE_SINK			870
#define GR_3CAKE2_OPEN			880
#define GR_3CAKE2_CLOSE			890
#define GR_3CAKE2_LIFT			900
#define GR_3CAKE2_LIFT2			901
#define GR_3CAKE2_TURN			910
#define GR_3CAKE2_SINK			920
#define GR_3CAKE3_OPEN			930
#define GR_RECONCILE			940
#define GR_RECONCILE2			950

#define GR_SORT0				953
#define GR_SORT1				955
#define GR_SORT2				960
#define GR_SORT3				970
#define GR_SORT4				980
#define GR_SORT5				990
#define GR_SORT6				1000
#define GR_SORT7				1010
#define GR_SORT8				1020
#define GR_SORT9				1030
#define GR_SORT10				1040
#define GR_SORT11				1050
#define GR_SORT12				1060
#define GR_SORT13				1070
#define GR_SORT14				1080
#define GR_SORT15				1085
#define GR_SORT16				1090
#define GR_SORT17				1100
#define GR_SORT18				1110
#define GR_SORT19				1120
#define GR_SORT20				1130
#define GR_SORT21				1140
#define GR_SORT22				1145
#define GR_SORT23				1150
#define GR_SORT24				1160
#define GR_SORT25				1170
#define GR_SORT26				1180
#define GR_SORT27				1190
#define GR_SORT28				1200
#define GR_SORT29				1210
#define GR_SORT30				1220
#define GR_SORT31				1230
#define GR_SORT32				1240
















#define GR_MOTION_READY				10000


#define NO_CAKE 0
#define BROWN_CAKE 1
#define YELLOW_CAKE 2
#define PURPLE_CAKE 3

typedef struct
{
	uint8_t *IRSensorPort1;
	uint8_t *IRSensorPort2;
	uint8_t *IRSensorPort3;
	uint8_t *RefSwitchPort;
	uint8_t ServoPosGripClose;
	uint8_t ServoPosGripOpen;
	uint8_t ServoPosGripRef;
	uint8_t ServoPosGripPushOpenLeft;
	uint8_t ServoPosGripPushCloseLeft;
	uint8_t ServoPosGripPushOpenRight;
	uint8_t ServoPosGripPushCloseRight;
	uint8_t ServoPosGripPush;
	uint8_t ServoPosCherryOpen;
	uint8_t ServoPosCherryClose;
	uint8_t ServoGripperPosCherry;
	uint8_t ServoGripperNbr;
	uint8_t ServoCherryNbr;
	Motor_t *motor;
	uint8_t MotNumber;
	uint8_t WaitStateCycles;
	uint16_t NextState;
	uint16_t State;
	uint8_t taskNbr;
	uint8_t opModeFeedback;
	uint8_t opMode;  // opperation Mode: 0 .. collect;
	uint8_t cakecount;
	uint8_t NbrElemets;
	uint8_t ServoNumberPusherLeft;
	uint8_t ServoNumberPusherRight;
	float MotorPosRef;
	float MotorPosUP;
	float MotorPosBUILD1;			// Position Brown
	float MotorPosBUILD2;			// Position Yellow
	float MotorPosBUILD3;			// Position Purple
	float MotorPosBUILD1sink;			// Position Brown
	float MotorPosBUILD2sink;			// Position Yellow
	float MotorPosBUILD3sink;			// Position Purple
	float MotorPosDrive;
	
	float MotorPosCherry;
	uint32_t CakeLayerType;
	
} gripper_t;


#define CHECK_REF(a) (((*(a->RefSwitchPort)) & a->RefSwitchMask) ? 0 : 1)
#define CHECK_IR(a) (((*(a->IRSensorPort)) & a->IRSensorMask) ? 1 : 0)

uint16_t StateBuilder ;
uint8_t CakeFinished;
uint8_t countcake1;					//KI gibt vor wie viele Kuchen mit welcher Höhe gebaut werden
uint8_t countcake2;
uint8_t countcake3;
uint8_t countcake1_gripper;			// countcake wird auf eine interne Variable gespeichert
uint8_t countcake2_gripper;
uint8_t countcake3_gripper;
float heightcake_left;				// Höhe des zu bauenden Kuchens
float heightcake_right;
float height_together;
uint16_t nextStateBuilder ;
uint8_t ReadyBuilder;
uint8_t cakerest;
uint8_t cakemarker;					// Anzahl der gebauten Kuchen
uint8_t cakedirektion;
uint8_t cakeheight_1;				//Höhe der gebauten Kuchen
uint8_t cakeheight_2;
uint8_t cakeheight_3;
float cherryheight_1;				//Höhe des Greifers wenn er Kirschen ablegt
float cherryheight_2;
float cherryheight_3;

_GRIPPER_EXTERN_ gripper_t Gripper, Gripper_Right, Gripper_Left;

uint8_t GripperLeftTask();

uint8_t GripperRightTask();

uint8_t CakebuilderTask();

uint8_t Position_erreicht(float sollPosition, uint8_t Lift);

void InitGripper();

#endif /* GRIPPER_H_ */