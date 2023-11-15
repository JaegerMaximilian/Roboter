/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
*
* \brief
*      Includes debug-task, here the software parts can be tested.
*
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
*
* \par Documentation
*      The file includes the debug-task.
*
* \author
*      Michael Zauner
*      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
*      Support email: roboracing@fh-wels.at
*
* $Revision: 1 $
* $Date: 2012-09-11  $  \n
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

#define _DEBUG_EXTERN


#include <avr/io.h>
#include "multitask.h"
#include "debug.h"
#include "ports.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "usart.h"
#include <stdint.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "adc_driver.h"
#include "accumulator.h"
#include "sensor.h"
#include "anaPos.h"
#include "servo.h"
#include "motor.h"
#include "rrt_receivetask.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "Pfadplanung.h"
#include "PSE541.h"
#include "rrt_transmittingtask.h"
#include "ki.h"
#include "Arm.h"
#include "command.h"
#include "wifi.h"
#include "enemyDetection.h"
#include "gripper.h"
#include "nextion.h"

uint8_t CANRec = 0;
uint8_t punkte_Kirschen_Zusatz =0;

extern uint16_t KI_State;


/* ************************************************************** */
/*! \brief Initialize debug-task.
*
*  Function initialize the debug-task
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
void InitDebug(void)
{
	/* cyclic task - cycle time: 500 ms */
	SET_CYCLE(DEBUG_TASKNBR, 500);
	SET_TASK(DEBUG_TASKNBR, CYCLE);
	SET_TASK_HANDLE(DEBUG_TASKNBR, DebugTask);
	
	//se_array[SE_IMP_NBR].Phi = 0;
	//se_array[SE_IMP_NBR].Phi = 0;
	
	
}


//typedef struct
//{
//int16_t Xpos;					// X-Position der Elemente
//int16_t Ypos;					// Y-Position der Elemente
//} element_t;

/* ************************************************************** */
/*! \brief Debug-task.
*
*  Here different functions can be tested
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
uint8_t DebugTask(void)
{
	static uint8_t State = 0;

	static uint8_t Points = 0;
	static uint8_t Time = 0;
	uint8_t text1[150];//, text2[150], data[50] = {0,1,2,3,4,5,6,7};
	//static convData_t d;
	//
	//point_t Start, Ziel;
	//segment_t A,B,C;
	//matrixpoint_t p;
	
	//d.uint32 = 0x40480000;
	//data[0] = CAN_HARDWARE_ID;
	//data[1] = d.uint8[0];
	//data[2] = d.uint8[1];
	//data[3] = d.uint8[2];
	//data[4] = d.uint8[3];
	//data[5] = 4;
	//data[6] = (uint8_t) (CAN_LB_RECEIVE_OFFSET);
	//data[7] = (uint8_t) ((CAN_LB_RECEIVE_OFFSET) >> 8);
	///* send hardware ID cyclic */
	//CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 8, CAN_LB_RECEIVE_OFFSET, data);

	
	SET_CYCLE(DEBUG_TASKNBR, 500);
	Points++;
	Time++;
	//if(Time>100)
	//{
		//Time=0;
	//}
	
	NextionSetPoints(spielZeit);
	NextionSetTime(spielZeit);
	//    LED_PORT.OUT ^= ledBit;
	// //   ledBit = (ledBit != 0x40) ? ledBit << 1 : 0x20;
	//    ledBit = (ledBit != 0x80) ? ledBit << 1 : 0x20;

	TOGGLE_PIN(LED_PORT1, LED1);
// 	TOGGLE_PIN(LED_PORT1, LED2);
// 	TOGGLE_PIN(LED_PORT1, LED3);
// 	TOGGLE_PIN(LED_PORT2, LED4);
	
	
	//	se_array[SE_RES1_NBR].Phi = 0;
	//	se_array[SE_RES2_NBR].Phi = 130;
	
	
	//Home position reached
	uint8_t homePosition[50];
// 	sprintf(homePosition, "#12H%d\n\r*", HomePositionReached);
// 	writeString_usart(&WIFI_IF, homePosition);
	
	//uint8_t points_MasterRobot[30];
	//uint8_t punkte_Master=Punkte+punkte_Kirschen_Zusatz+Punkte_Kirschen;
	//sprintf(points_MasterRobot, "#12p%03d*", punkte_Master);
	//writeString_usart(&WIFI_IF, points_MasterRobot);
	
	// 		volatile point_t start, ziel, pointList5000[20];
	// 		uint8_t pointNr;
	//
	//
	// 		PATH_SET_OBSTICAL(0,0,5,4,10);
	// 		PATH_ENABLE_OBSTACLE(0);
	// 		// Kirschen
	// 		PATH_SET_OBSTICAL(1,21,5,26,10);
	// 		PATH_ENABLE_OBSTACLE(1);
	//
	// 		start.Xpos = 2700;
	// 		start.Ypos = 1700;
	//
	// 		ziel.Xpos = 2700;
	// 		ziel.Ypos = 300;
	//
	// 		PATH_DriveToAbsPos(start, ziel, &(pointList5000[0]), &pointNr);
	
	
	

	
	
	//sprintf( "#Opmode: %d ; Feedback: %d \n\r*",Gripper.opMode ,Gripper.opModeFeedback );
	//writeString_usart(&WIFI_IF, homePosition);
	//se_array[SE_IMP_NBR].phiPerSec = 150;// WITH_OUT_TRAJECTORY;
	//
	//cmd_SetDigitalOut(DO_POLWENDER_NBR, 1);
	//se_array[SE_IMP_NBR].Phi = 40;
	////
	//se_array[SE_K2_NBR].Phi = 4;
	//cmd_SetServo(SE_IMP_NBR, 100);
	// stellung 10 Grad ->open stellung 32 Grad
	//sprintf(text1, "#µC1: VL: %d, VR: %d \r\n*", refVL, refVR);
	//writeString_usart(&usartC0, text1);
	
	//se_array[SE_RES2_NBR].Phi = 50;
	// se_array[SE_RES1_NBR].Phi = 100;
	
	//sprintf(text1, "#µC1: VL: %d, VR: %d \r\n*", END_VL, END_VR);
	//writeString_usart(&usartC0, text1);
	
	//if (Gripper_Left.opModeFeedback == 1)
	//{
	//Gripper_Left.opMode = 2;
	//}
	//
	//if (Gripper_Left.opModeFeedback == 2 && Gripper_Right.opModeFeedback==1)
	//{
	//ReadyBuilder = 20;
	//countcake1=2;
	//countcake2=1;
	//countcake3=0;
	//Gripper_Left.CakeLayerType =63;
	//Gripper_Right.CakeLayerType = 0;
	//}
	//

	//
	//sprintf(text1, "#IR_VR1: %d, IR_VR2: %d, IR_VR3: %d, IR_VL1: %d, IR_VL2: %d, IR_VL3: %d, IR_K1: %d, IR_K2: %d\r\n*", IR_VR1, IR_VR2, IR_VR3, irVL1, IR_VL2, IR_VL3, IR_K1, IR_K2);
	//writeString_usart(&WIFI_IF, text1);
	
	//sprintf(text1, "#XPOS: %d, YPOS: %d, PHI:%d \r\n*", xPos, yPos, phiPos);
	//writeString_usart(&WIFI_IF, text1);
	
	//sprintf(text1, "#START: %d\r\n*", START_SCHNUR);
	////writeString_usart(&WIFI_IF, text1);
	//cmd_SetDigitalOut(DO_POLWENDER_NBR, 1);
	//cmd_SetServo(SE_IMP_NBR,70);
	//
	//	se_array[SE_RES1_NBR].Phi = 60;
	//	se_array[SE_RES1_NBR].Phi = 135;

	// 	Arm_t *ps1, *ps2;
	//
	// 	ps1 = &ArmLeft;
	// 	ps2 = &ArmRight;
	//
	//sprintf(text1, "End_VL: %d, End_VR: %d\r\n", CHECK_REF(ps1), CHECK_REF(ps2));
	//writeString_usart(&usartF0, text1);
	
	
	//sprintf(text1, "%ld;%d;%d;%d\r\n", (uint32_t)KI_State,xPos, yPos, otherRobot.Xpos);
	//writeString_usart(&usartF0, text1);
	
	// Punkte = spielZeit;

	//  		sprintf(text1, "%d;%d;%.3f;%.3f;%d;%d;\r\n", rearpressureleft, rearpressureright, liftposrearleft, liftposrearright, pressureFrontLeft.pressure, pressureFrontRight.pressure);
	//  	   	writeString_usart(&usartF0, text1);
	
	//  		sprintf(text1, "%d;%d;\r\n", galv, galh);
	//  		writeString_usart(&usartF0, text1);

	//   		sprintf(text1, "%d;%d;\r\n", galv, galh);
	//   		writeString_usart(&usartF0, text1);

	
	//se_array[SE_IMP_NBR].Phi = 100;
	//se_array[SE_RES2_NBR].Phi = 80;
	//
	//se_array[SE_VL_NBR].Phi = 75;
	//setVelocity(&liftVL, 0.01);
	//setVelocity(&liftVR, 0.01);
	
	//sprintf(text1, "#µC1: POS %.3f, SOLL_POS %.3f %.2f\r\n*", liftVL.Odo.Dis,liftVR.Traj.Xs,liftVR.Pos.Y);
	//writeString_usart(&usartC0, text1);
	//
	
	//sprintf(text1, "#status %d \r\n*", statusAntrieb);
	//writeString_usart(&usartC0, text1);
	//
	//if (Gripper.opModeFeedback == 1)
	//{
	//Gripper.opMode = 2;
	//}
	//else if (Gripper.opModeFeedback == 2)
	//{
	//Gripper.opMode=3;
	//}
	//else if (Gripper.opModeFeedback==3)
	//{
	//ReadyBuilder=10;
	//}
	//
	//cmd_SetServo(SE_RES1_NBR, Gripper.ServoPosGripPushOpenRight);
	//cmd_SetServo(SE_RES2_NBR, Gripper.ServoPosGripPushOpenLeft);
	//if (Gripper.opModeFeedback == 1)
	//{
	//Gripper.opMode = 2;
	//}
	//if (Gripper.opModeFeedback == 2)
	//{
	//Gripper.opMode = 3;
	//}
	//if (Gripper.opModeFeedback == 3)
	//{
	//ReadyBuilder = 10;
	//}
	//
	//if (Gripper_Right.opModeFeedback == 1)
	//{
	//Gripper_Right.opMode = 2;
	//}
	//
	//if (Gripper_Right.opModeFeedback == 2 && Gripper_Left.opModeFeedback == 1)
	//{
	//ReadyBuilder = 11;
	//}
	//
	//
	//if (Gripper_Left.opModeFeedback == 2)
	//{
	//Gripper_Left.opMode = 3;
	//}
	//
	//if (Gripper_Right.opModeFeedback == 1)
	//{
	//Gripper_Right.opMode = 2;
	//}
	//
	//if (Gripper_Right.opModeFeedback == 2)
	//{
	//Gripper_Right.opMode = 3;
	//}
	
	//setVelocity(&liftVL, 0.01);
	//setVelocity(&liftVR, 0.01);
	
	//cmd_SetServo(SE_RES2_NBR, 123);			RES2-> links
	//cmd_SetServo(SE_RES1_NBR, 55);
	
	// cmd_SetDigitalOut(DO_LED1_NBR, 1)
	
	//se_array[SE_K1_NBR].Phi = 119;
	//se_array[SE_RES1_NBR].Phi = 130;
	
	if(State == 0)
	{
		State = 1;
		
		//se_array[SE_VL_NBR].Phi = 120;
		//se_array[SE_VR_NBR].Phi = 120;
		//se_array[SE_H_NBR].Phi = 120;
		//se_array[SE_RES1_NBR].Phi = 120;
		//se_array[SE_RES2_NBR].Phi = 120;
		//se_array[SE_K1_NBR].Phi = 120;
		//se_array[SE_K2_NBR].Phi = 120;
		//se_array[SE_IMP_NBR].Phi = 120;
		//setMotion(&liftVR, 0.10, 0.100);
		//setMotion(&liftVL, 0.10, 0.100);
		//  		setVelocity(&liftVL, 0.05);
		//   		setVelocity(&liftVR, 0.05);
		//cmd_SetMotorPos(LIFT_REAR_NBR,0.1,0.05);
		// cmd_SetDigitalOut(DO_LED1_NBR, 1);
		//cmd_SetDigitalOut(DO_LED2_NBR, 1);
		//cmd_SetDigitalOut(DO_LED3_NBR, 1);
		//cmd_SetDigitalOut(DO_POLWENDER_NBR, 1);
	}
	else
	{
		State = 0;
		//se_array[SE_VL_NBR].Phi = 60;
		//se_array[SE_VR_NBR].Phi = 60;
		//se_array[SE_H_NBR].Phi = 60;
		//se_array[SE_RES1_NBR].Phi = 60;
		//se_array[SE_RES2_NBR].Phi = 60;
		//se_array[SE_K1_NBR].Phi = 60;
		//se_array[SE_K2_NBR].Phi = 60;
		//se_array[SE_IMP_NBR].Phi = 60;
		//setMotion(&liftVR, 0.10, 0.050);
		//setMotion(&liftVL, 0.10, 0.050);
		//  	 	setVelocity(&liftVL, -0.05);
		//  	 	setVelocity(&liftVR, -0.05);
		//cmd_SetDigitalOut(DO_LED1_NBR, 0);
		//cmd_SetDigitalOut(DO_LED2_NBR, 0);
		//cmd_SetDigitalOut(DO_LED3_NBR, 0);
		//cmd_SetDigitalOut(DO_POLWENDER_NBR, 0);
		//cmd_SetMotorPos(LIFT_REAR_NBR,0.1,0.1);
	}
	
	//	Punkte = (++Punkte > 100) ? 0 : Punkte;
	

	
	if(Punkte_Kirschen > 0)
	{
		punkte_Kirschen_Zusatz = 5;
	}
	else
	{
		punkte_Kirschen_Zusatz = 0;
	}
	
	setDataLCD_RRTLAN(Punkte+Punkte_Kirschen +punkte_Kirschen_Zusatz, spielZeit);
	
	
	//	sprintf(text1, "OtherRobot:(%d:%d)/(%d:%d)/(%d:%d)/(%d:%d)\n", EnemyDataRaw.x[0], EnemyDataRaw.y[0],EnemyDataRaw.x[1], EnemyDataRaw.y[1]
	//	,EnemyDataRaw.x[2], EnemyDataRaw.y[2],EnemyDataRaw.x[3], EnemyDataRaw.y[3]);
	//	writeString_usart(&usartF0, text1);

	//sprintf(text1, "Other:(%d:%d)/(%d:%d)/(%d:%d)/(%d:%d) Own:(%d:%d)/(%d:%d)/(%d:%d)/(%d:%d) Fusion:(%d:%d)/(%d:%d)/(%d:%d)/(%d:%d)\n", EnemyDataRaw.x[0], EnemyDataRaw.y[0],EnemyDataRaw.x[1], EnemyDataRaw.y[1]
	//,EnemyDataRaw.x[2], EnemyDataRaw.y[2],EnemyDataRaw.x[3], EnemyDataRaw.y[3],dynamicObstacles[0].pos.Xpos, dynamicObstacles[0].pos.Ypos,dynamicObstacles[1].pos.Xpos, dynamicObstacles[1].pos.Ypos
	//,dynamicObstacles[2].pos.Xpos, dynamicObstacles[2].pos.Ypos,dynamicObstacles[3].pos.Xpos, dynamicObstacles[3].pos.Ypos,enemyRobot[0].Xpos,enemyRobot[0].Ypos,enemyRobot[1].Xpos,
	//enemyRobot[1].Ypos, enemyRobot[2].Xpos, enemyRobot[2].Ypos, enemyRobot[3].Xpos, enemyRobot[3].Ypos);
	//writeString_usart(&usartF0, text1);
	

	//sprintf(text1, "(%d:%d)/(%d:%d)/(%d:%d)/(%d:%d)\n", dynamicObstacles[0].pos.Xpos, dynamicObstacles[0].pos.Ypos,dynamicObstacles[1].pos.Xpos, dynamicObstacles[1].pos.Ypos
	//,dynamicObstacles[2].pos.Xpos, dynamicObstacles[2].pos.Ypos,dynamicObstacles[3].pos.Xpos, dynamicObstacles[3].pos.Ypos);
	//writeString_usart(&usartF0, text1);

	// //Daten vom WIFI werden vom uC1 auf den uC3 gesendet
	//sendPosDataEnemy_from_WIFI_RRLAN(xPos, yPos, phiPos/10, &EnemyDataRaw);
	//// Ultraschall vorne
	//float xFrontRight;
	//float xFrontLeft;
	//float yFrontRight;
	//float yFrontLeft;
	//
	//
	//
	//xFrontRight = (float)xPos + (float)(OFFSET_FRONT_X + usRightFront) * cos(DEG2RAD(phiPos/10)) + (float)OFFSET_FRONT_Y * sin(DEG2RAD(phiPos/10));
	//yFrontRight = (float)yPos - (float)OFFSET_FRONT_Y * cos(DEG2RAD(phiPos/10)) + (OFFSET_FRONT_X + usRightFront) * sin(DEG2RAD(phiPos/10));
	//
	//xFrontLeft = (float)xPos + (float)(OFFSET_FRONT_X + usLeftFront) * cos(DEG2RAD(phiPos/10)) - (float)OFFSET_FRONT_Y * sin(DEG2RAD(phiPos/10));
	//yFrontLeft = (float)yPos + (float)OFFSET_FRONT_Y * cos(DEG2RAD(phiPos/10)) + (float)(OFFSET_FRONT_X + usLeftFront) * sin(DEG2RAD(phiPos/10));
	
	//sprintf(text1, "R:(%d:%d:%d) P1:(%.0f:%.0f::%d) P2:(%.0f:%.0f::%d)\n", xPos,yPos,phiPos/10, xFrontRight, yFrontRight,usRightFront,xFrontLeft, yFrontLeft,usLeftFront);
	//writeString_usart(&usartF0, text1);
	
	
	//// Servoarme hinten hinaufschwenken
	//cmd_SetServo(SE_PINCE_NEZ1_NBR, SE_PINCE_NEZ1_1);
	//cmd_SetServo(SE_PINCE_NEZ2_NBR, SE_PINCE_NEZ2_1);
	//cmd_SetServo(SE_PINCE_NEZ3_NBR, SE_PINCE_NEZ3_1);
	//cmd_SetServo(SE_PINCE_NEZ4_NBR, SE_PINCE_NEZ4_1);
	//cmd_SetServo(SE_PINCE_NEZ5_NBR, SE_PINCE_NEZ5_1);
	//
	//
	//
	//
	//static uint8_t cupColor[5]; // 0 = noch nicht initialisiert, 1 = Güner Becher, 2 = Roter Becher
	//#define GREENCUP 1
	//#define REDCUP 2
	//cupColor[0] = REDCUP;
	//cupColor[4] = GREENCUP;
	//if (COLOUR_REAR_1 == 1 && COLOUR_REAR_2 == 0)
	//{
	//cupColor[1] = REDCUP;
	//cupColor[2] = GREENCUP;
	//cupColor[3] = GREENCUP;
	//}
	//else if (COLOUR_REAR_1 == 0 && COLOUR_REAR_2 == 1)
	//{
	//cupColor[1] = GREENCUP;
	//cupColor[2] = REDCUP;
	//cupColor[3] = GREENCUP;
	//}
	//else
	//{
	//cupColor[1] = GREENCUP;
	//cupColor[2] = GREENCUP;
	//cupColor[3] = REDCUP;
	//}
	//
	//uint8_t ir_Cups_Rear[5] = {IR_REAR_1,IR_REAR_2,IR_REAR_3,IR_REAR_4,IR_REAR_5};
	//
	//sprintf(text1, "1: %d:%d 2: %d:%d 3: %d:%d 4: %d:%d 5: %d:%d \n", ir_Cups_Rear[0],cupColor[0], ir_Cups_Rear[1],cupColor[1],ir_Cups_Rear[2],cupColor[2],ir_Cups_Rear[3],cupColor[3],ir_Cups_Rear[4],cupColor[4]);
	//writeString_usart(&WIFI_IF, text1);
	//
	
	//sprintf(text1, "%d / %d / %d / %d / %d \n", cupColor[0],cupColor[1],cupColor[2],cupColor[3],cupColor[4]);
	//sprintf(text1, "Rear1:%d / Rear2:%d\n", COLOUR_REAR_1,COLOUR_REAR_2);
	//writeString_usart(&usartF0, text1);

	// 	Ultraschall hinten
	// 		float xRearRight;
	// 		float xRearLeft;
	// 		float yRearRight;
	// 		float yRearLeft;
	//
	//
	//
	// 		xRearLeft = (float)xPos - (float)(OFFSET_REAR_X + usLeftRear) * cos(DEG2RAD(phiPos/10)) - (float)OFFSET_REAR_Y * sin(DEG2RAD(phiPos/10));
	// 		yRearLeft = (float)yPos + (float)OFFSET_REAR_Y * cos(DEG2RAD(phiPos/10)) - (float)(OFFSET_REAR_X + usLeftRear) * sin(DEG2RAD(phiPos/10));
	//
	// 		xRearRight = (float)xPos - (float)(OFFSET_REAR_X + usRightRear) * cos(DEG2RAD(phiPos/10)) + (float)OFFSET_REAR_Y * sin(DEG2RAD(phiPos/10));
	// 		yRearRight = (float)yPos - (float)OFFSET_REAR_Y * cos(DEG2RAD(phiPos/10)) - (float)(OFFSET_REAR_X + usRightRear) * sin(DEG2RAD(phiPos/10));
	//
	//		sprintf(text1, "R:(%d:%d:%d) P3:(%.0f:%.0f::%d) P4:(%.0f:%.0f::%d)\n", xPos,yPos,phiPos/10, xRearLeft, yRearLeft,usLeftRear, xRearRight, yRearRight,usRightRear);
	//
	//

	//	sprintf(text1, "R:(%d:%d:%d) P1:(%.0f:%.0f::%d) P2:(%.0f:%.0f::%d)\n", xPos,yPos,phiPos/10, xFrontLeftEnemy, yFrontLeftEnemy,usLeftFront,xFrontRightEnemy, yFrontRightEnemy,usRightFront);
	//sprintf(text1, "R:(%d:%d:%d) Enemy:(%.0f:%.0f)\n", xPos,yPos,phiPos/10, xPosEnemy, yPosEnemy);







	//		sprintf(text1, "R:(%d:%d:%d) P3:(%.0f:%.0f::%d) P4:(%.0f:%.0f::%d)\n", xPos,yPos,phiPos/10, xRearLeft, yRearLeft,usLeftRear, xRearRight, yRearRight,usRightRear);
	//
	//
	
	
	//sprintf(text1, "%d; %d; %d; %d;\n", usRightRear, usLeftRear, usLeftFront, usRightFront);
	//writeString_usart(&usartF0, text1);
	//sprintf(text1, "%d:%d:%d\n", xPos,yPos,phiPos/10);
	//		sprintf(text1, "R:(%d:%d:%d)\n", cupStackerLeft.opMode,KI_State,phiPos/10);
	
	
	//sprintf(text1, "%d; %d; %d; %d;\n", usRightRear, usLeftRear, usLeftFront, usRightFront);
	// 		sprintf(text1, "R:(%d:%d:%d)\n", xPos,yPos,phiPos/10);
	//	sprintf(text1, "Pressure:SuckerPos:(%d:%.1f) : %d\n", pressureFrontLeft.pressure,se_array[SE_SUCKER_LEFT_NBR].Phi,RobotType_RAM);
	
	// 	if (RobotType_RAM == MASTER_ROBOT)
	// 	{
	// 		setDataWIFI_RRTLAN(5);
	// 	}
	//

	//sprintf(text1, "M:(%d:%d) S:(%d:%d) EndposSlave:%d \n", master.Xpos,master.Ypos, slave.Xpos, slave.Ypos, EndPositionSlave);
	//writeString_usart(&usartF0, text1);

	#define DELAY_SCH 20
	
	//cmd_SetServo(SE_ARM_RIGHT_NBR, SE_ARM_RIGHT_1);

	// 	//sprintf(text1, "%d\n", COLOUR_NORTH_SOUTH);
	//  	if (State == 0)
	//  	{
	//  	//	State = 1;
	//  		//cmd_SetServo(SE_ARM_RIGHT_NBR, SE_ARM_RIGHT_1);
	//
	//  		//
	//  		//// Lift nach unten => 0,9 muss noch ausgemessen werden
	//  		////			cmd_SetMotorPos(LIFT_BACK_NBR, 0.12, 0.75); // 0,75 muss noch genauer eingestellt werden
	//  		//
	//  		//// 		SET_PIN(LED_PORT,LED1);
	//  		////  		cmd_SetServo(SE_FLAG1_NBR, SE_FLAG1_0);
	//  		////
	//  		//// Becherreihe mit Servoarm klemmen
	//  		////			cmd_SetServo(SE_PINCE_NEZ1_NBR, SE_PINCE_NEZ1_0);
	//  		//
	//  		switch(startup)
	//  		{
	//  		case 0:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ2_NBR, SE_PINCE_NEZ2_0);
	//  		SET_CYCLE(DEBUG_TASKNBR, DELAY_SCH);
	//  		startup = 1;
	//  		break;
	//  		}
	//  		case 1:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ3_NBR, SE_PINCE_NEZ3_0);
	//  		SET_CYCLE(DEBUG_TASKNBR, DELAY_SCH);
	//  		startup = 2;
	//  		break;
	//  		}
	//  		case 2:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ4_NBR, SE_PINCE_NEZ4_0);
	//  		SET_CYCLE(DEBUG_TASKNBR, DELAY_SCH);
	//  		startup = 3;
	//  		break;
	//  		}
	//  		case 3:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ5_NBR, SE_PINCE_NEZ5_0);
	//  		SET_CYCLE(DEBUG_TASKNBR, 500);
	//  		startup = 0;
	//  		State = 1;
	//  		break;
	//  		}
	//  		}		 		//cmd_SetServo(SE_ARM_NBR, SE_ARM_1);
	//
	//  	}
	//  	else
	//  	{
	//  			//		cmd_SetServo(SE_ARM_RIGHT_NBR, SE_ARM_RIGHT_0);
	//  	//	State = 0;
	//
	//  		////	 		 cmd_SetServo(SE_FLAG1_NBR, SE_FLAG1_1);
	//  		////		 CLR_PIN(LED_PORT,LED1);
	//  		//// Servoarme hinten hinaufschwenken
	//  		////			cmd_SetServo(SE_PINCE_NEZ1_NBR, SE_PINCE_NEZ1_1);
	//  		switch(startup)
	//  		{
	//
	//  		case 0:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ2_NBR, SE_PINCE_NEZ2_1);
	//  		SET_CYCLE(DEBUG_TASKNBR, DELAY_SCH);
	//  		startup = 1;
	//  		break;
	//  		}
	//  		case 1:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ3_NBR, SE_PINCE_NEZ3_1);
	//  		SET_CYCLE(DEBUG_TASKNBR, DELAY_SCH);
	//  		startup = 2;
	//  		break;
	//  		}
	//  		case 2:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ4_NBR, SE_PINCE_NEZ4_1);
	//  		SET_CYCLE(DEBUG_TASKNBR, DELAY_SCH);
	//  		startup = 3;
	//  		break;
	//  		}
	//  		case 3:
	//  		{
	//  		cmd_SetServo(SE_PINCE_NEZ5_NBR, SE_PINCE_NEZ5_1);
	//  		SET_CYCLE(DEBUG_TASKNBR, 500);
	//  		startup = 0;
	//  		State = 0;
	//  		break;
	//  		}		//cmd_SetServo(SE_ARM_NBR, SE_ARM_0);
	//  		}
	//  	}
	
	
	
	// 	Start.Xpos = 510;
	// 	Start.Ypos = 340;
	//
	// 	Ziel.Xpos = 2780;
	// 	Ziel.Ypos = 1790;
	//
	// 	enemyRobot_1.Xpos = 1500;
	// 	enemyRobot_1.Ypos = 1000;
	//
	// 	enemyRobot_2.Xpos = 2040;
	// 	enemyRobot_2.Ypos = 300;
	//
	// 	smallRobot.Xpos = 500;
	// 	smallRobot.Ypos = 1510;
	//
	// sysTime = 0;
	// SET_PIN(LED_PORT, LED3);
	// 	PATH_DriveToAbsPos(100, Start, Ziel, 0);

	// 	A.o.Xpos = 0;
	// 	A.o.Ypos = 0;
	// 	A.v.x = 10;
	// 	A.v.y = 10;
	//
	// 	p.Xpos = 10;
	// 	p.Ypos = 10;
	//
	// 	B.o.Xpos = 0;
	// 	B.o.Ypos = 3;
	// 	B.v.x = 10;
	// 	B.v.y = 2;
	//
	// 	C.o.Xpos = 11;
	// 	C.o.Ypos = 10;
	// 	C.v.x = 3;
	// 	C.v.y = 4;

	
	
	// 	sprintf(text1, "AxB: %d :: AxC: %d\n", PATH_CalcIntersection(A.o,p,B), PATH_CalcIntersection(A.o,p,C));
	// 	writeString_usart(&usartF0, text1);
	//
	//
	Arm_t *a;
	Arm_t *b;
	
	//	a = &cupStackerRight;
	//b = &cupStackerLeft;
	
	//cmd_SetServo(SE_SILO_LEFT1_NBR, SE_SILO_LEFT1_1);
	//cmd_SetServo(SE_SILO_LEFT2_NBR, SE_SILO_LEFT2_1);
	//cmd_SetServo(SE_SILO_RIGHT1_NBR, SE_SILO_RIGHT1_1);
	//cmd_SetServo(SE_SILO_RIGHT2_NBR, SE_SILO_RIGHT2_1);
	//cmd_SetServo(SE_SUCKER_LEFT_NBR, SE_SUCKER_LEFT_1);
	//cmd_SetServo(SE_SUCKER_RIGHT_NBR, SE_SUCKER_RIGHT_1);
	//
	//sprintf(text1, "R: %d (%d) [%d] :: L: %d (%d) [%d]\n", CHECK_COLOR(a), CHECK_IR(a), a->pressure->pressure, CHECK_COLOR(b), CHECK_IR(b), b->pressure->pressure);
	//writeString_usart(&usartF0, text1);


	
	
	
	//  	for (uint8_t i = 0; i < PATH_GRID_DIM_Y; i++)
	//  	{
	//  		for (uint8_t j = 0; j < PATH_GRID_DIM_X; j++)
	//  		{
	//  			sprintf(text1, "%d;", path.occupancyGrid[j][i]);
	//  			writeString_usart(&usartF0, text1);
	//  		}
	//
	//  		sprintf(text1, "\n");
	//  		writeString_usart(&usartF0, text1);
	//  	}

	//CLR_PIN(LED_PORT, LED3);

	//   	SET_VACUUM_PUMP_LEFT;
	//   	SET_MV_LEFT;
	// 	SET_VACUUM_PUMP_RIGHT
	// 	SET_MV_RIGHT;


	//     	sprintf(text1,"pR: %d mbar - pL: %d\r\n", pressureFrontRight.pressure, pressureFrontLeft.pressure);
	//     	writeString_usart(&usartF0, text1);

	//      	sprintf(text1,"%.1f (%.1f) - %.1f (%.1f)\r\n", posFrontLeft.pos, (liftVL.Odo.Dis*1000.0), posFrontRight.pos, (liftVR.Odo.Dis * 1000.0));
	//      	writeString_usart(&usartF0, text1);



	///* ******************** */
	///* digital sensors µC 2 */
	///* ******************** */
	///* reflex sensor */
	//#define REFLEX_1			((uC2sensorenPortB & 0x80) ? 1 : 0)
	//#define REFLEX_2			((uC2sensorenPortB & 0x40) ? 1 : 0)
	//#define REFLEX_3			((uC2sensorenPortB & 0x20) ? 1 : 0)
	//#define REFLEX_4			((uC2sensorenPortB & 0x10) ? 1 : 0)
	//#define REFLEX_TOTAL		((0x01 * REFLEX_1) + (0x02 * REFLEX_2) + (0x04 * REFLEX_3) + (0x08 * REFLEX_4))
	//
	///* ******************** */
	///* digital sensors µC 3 */
	///* ******************** */
	///* lift rear: IR and colour front */
	//#define COLOUR_REAR_1		((uC3sensorenPortB & 0x20) ? 1 : 0)
	//#define COLOUR_REAR_2		((uC3sensorenPortB & 0x40) ? 1 : 0)
	//#define IR_REAR_1			((uC3sensorenPortB & 0x01) ? 1 : 0)
	//#define IR_REAR_2			((uC3sensorenPortB & 0x02) ? 1 : 0)
	//#define IR_REAR_3			((uC3sensorenPortB & 0x04) ? 1 : 0)
	//#define IR_REAR_4			((uC3sensorenPortB & 0x08) ? 1 : 0)
	//#define IR_REAR_5			((uC3sensorenPortB & 0x10) ? 1 : 0)
	// #define IR_RESERVE_1		((uC3sensorenPortC & 0x10) ? 1 : 0)
	// #define IR_RESERVE_2		((uC3sensorenPortC & 0x20) ? 1 : 0)
	// #define IR_RESERVE_3		((uC3sensorenPortC & 0x40) ? 1 : 0)
	// #define IR_RESERVE_4		((uC3sensorenPortC & 0x80) ? 1 : 0)
	// sprintf(text1,"LIR:%d LC:%d :: RIR:%d RC:%d\r\n",IR_FRONT_LEFT, COLOUR_FRONT_LEFT, IR_FRONT_RIGHT, COLOUR_FRONT_RIGHT);
	// writeString_usart(&usartF0, text1);
	// sprintf(text1,"CR1:%d CR2:%d :: IR1:%d IR2:%d IR3:%d IR4:%d IR5:%d :: RES1:%d RES2:%d RES3:%d RES4:%d\r\n",
	//                         COLOUR_REAR_1,
	// 						COLOUR_REAR_2,
	// 						IR_REAR_1,
	// 						IR_REAR_2,
	// 						IR_REAR_3,
	// 						IR_REAR_4,
	// 						IR_REAR_5,
	// 						IR_RESERVE_1,
	// 						IR_RESERVE_2,
	// 						IR_RESERVE_3,
	// 						IR_RESERVE_4);
	// writeString_usart(&usartF0, text1);
	//sprintf(text1,"IR1:%d IR2:%d IR3:%d IR4:%d IR5:%d\r\n",
	//IR_REAR_1,
	//IR_REAR_2,
	//IR_REAR_3,
	//IR_REAR_4,
	//IR_REAR_5);
	//writeString_usart(&usartF0, text1);

	//  if (State == 0)
	//  {
	// 	 element_t wayPoints[5];
	//
	// 	 wayPoints[0].Xpos = 500;
	// 	 wayPoints[0].Ypos = 0;
	// 	 wayPoints[1].Xpos = 750;
	// 	 wayPoints[1].Ypos = 750;
	// 	 wayPoints[2].Xpos = 300;
	// 	 wayPoints[2].Ypos = 600;
	// 	 wayPoints[3].Xpos = 0;
	// 	 wayPoints[3].Ypos = 0;
	//
	//  	State = 1;
	//
	//
	
	//	setAntriebClothoid_RRTLAN(400, wayPoints, 4);
	
	//   	SET_VACUUM_PUMP_LEFT;
	//   	SET_MV_LEFT;
	//
	//   	SET_VACUUM_PUMP_RIGHT
	//   	SET_MV_RIGHT;
	//
	//
	//
	//
	//    	se_array[SE_RES1_NBR].Phi = 45.0;
	//    	se_array[SE_RES2_NBR].Phi = 45.0;
	//    	se_array[SE_SILO_LEFT1_NBR].Phi = 45.0;
	//    	se_array[SE_SILO_LEFT2_NBR].Phi = 45.0;
	//    	se_array[SE_SILO_RIGHT1_NBR].Phi = 45.0;
	//    	se_array[SE_SILO_RIGHT2_NBR].Phi = 45.0;
	//    	se_array[SE_SUCKER_LEFT_NBR].Phi = 45.0;
	//    	se_array[SE_SUCKER_RIGHT_NBR].Phi = 45.0;
	//
	//setServo_RRTLAN(&MCU2, SE_ARM_NBR, 45);
	//setServo_RRTLAN(&MCU2, SE_FLAG1_NBR, 45);
	//setServo_RRTLAN(&MCU2, SE_FLAG2_NBR, 45);
	
	//   	setDigitalOut_RRTLAN(&MCU2, DO_LED1_UC2_NBR, 1);
	//   	setDigitalOut_RRTLAN(&MCU2, DO_LED2_UC2_NBR, 1);
	//   	setDigitalOut_RRTLAN(&MCU2, DO_LED3_UC2_NBR, 1);
	//
	//PositionCommand_RRTLAN(&MCU2, 0.05, 0.05);
	
	
	//setDigitalOut_RRTLAN(&MCU3, DO_LED1_UC3_NBR, 1);
	//setDigitalOut_RRTLAN(&MCU3, DO_LED2_UC3_NBR, 1);
	//setDigitalOut_RRTLAN(&MCU3, DO_LED3_UC3_NBR, 1);
	//
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ1_NBR, 45);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ2_NBR, 45);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ3_NBR, 45);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ4_NBR, 45);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ5_NBR, 45);
	
	
	//	setMotion(&liftVR, 0.50, 0.08);
	
	//  }
	//  else
	//  {
	//	State = 0;
	
	//    	se_array[SE_RES1_NBR].Phi = 90.0;
	//    	se_array[SE_RES2_NBR].Phi = 90.0;
	//    	se_array[SE_SILO_LEFT1_NBR].Phi = 90.0;
	//    	se_array[SE_SILO_LEFT2_NBR].Phi = 90.0;
	//    	se_array[SE_SILO_RIGHT1_NBR].Phi = 90.0;
	//    	se_array[SE_SILO_RIGHT2_NBR].Phi = 90.0;
	//    	se_array[SE_SUCKER_LEFT_NBR].Phi = 90.0;
	//    	se_array[SE_SUCKER_RIGHT_NBR].Phi = 90.0;
	//
	//   	CLR_VACUUM_PUMP_RIGHT;
	//   	CLR_MV_RIGHT;
	//
	//   	CLR_VACUUM_PUMP_LEFT;
	//   	CLR_MV_LEFT;
	
	
	//setServo_RRTLAN(&MCU2, SE_ARM_NBR, 90);
	//setServo_RRTLAN(&MCU2, SE_FLAG1_NBR, 90);
	//setServo_RRTLAN(&MCU2, SE_FLAG2_NBR, 90);
	
	//   	setDigitalOut_RRTLAN(&MCU2, DO_LED1_UC2_NBR, 0);
	//   	setDigitalOut_RRTLAN(&MCU2, DO_LED2_UC2_NBR, 0);
	//   	setDigitalOut_RRTLAN(&MCU2, DO_LED3_UC2_NBR, 0);
	
	//PositionCommand_RRTLAN(&MCU2, -0.05, 0.0);
	//
	
	//setDigitalOut_RRTLAN(&MCU3, DO_LED1_UC3_NBR, 0);
	//setDigitalOut_RRTLAN(&MCU3, DO_LED2_UC3_NBR, 0);
	//setDigitalOut_RRTLAN(&MCU3, DO_LED3_UC3_NBR, 0);
	//
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ1_NBR, 90);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ2_NBR, 90);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ3_NBR, 90);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ4_NBR, 90);
	//setServo_RRTLAN(&MCU3, SE_PINCE_NEZ5_NBR, 90);
	
	
	//	setMotion(&liftVR, 0.50, 0.100);
	
	// }

	


	//    	sprintf(text1,"Hello World! %2.1fV - %.0f :: %.0f\r\n", getAccumulatorVoltage(), posFrontLeft.pos, posFrontRight.pos);
	//    	writeString_usart(&usartF0, text1);

	//	rpLidar_sendRequest(&rpLidar, RPLIDAR_GET_HEALTH);


	// 	sabertooth_2x60_sendCommand(&(sabertoothModules[0]),sabertoothModules[0].adr, SABERTOOTH_2X60_DRIVE_FORWARD_MOTOR_2, State);
	// 	State = ((++State > 127) ? 0 : State);
	// 	setVelocity(&motorLeft, 0.2);
	// 	setVelocity(&motorRight, 0.2);

	return(CYCLE);
}
