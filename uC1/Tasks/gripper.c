/*
* gripper.c
*
* Created: 26.01.2023 08:40:19
*  Author: perne
*/

#define _GRIPPER_EXTERN_

#include "gripper.h"
#include "servo.h"
#include "command.h"
#include "multitask.h"
#include "define.h"
#include "liftMotor.h"
#include "motor.h"
#include "ports.h"
#include "stddef.h"
#include "avr/delay.h"
#include "usart.h"
#include <stdio.h>


void InitGripper()
{
	
	Gripper.IRSensorPort1 = &irVL1;
	Gripper.IRSensorPort2 = &irVL2;
	Gripper.IRSensorPort3 = &irVL3;
	Gripper.RefSwitchPort = &refVL;
	Gripper.MotorPosCherry = CHECK_ROBOT_TYPE(0.077,0.077);
	Gripper.MotorPosRef = CHECK_ROBOT_TYPE(0.001,0.001);
	Gripper.MotorPosUP = CHECK_ROBOT_TYPE(0.078,0.068);
	Gripper.MotorPosBUILD1 = CHECK_ROBOT_TYPE(0.019,0.019);
	Gripper.MotorPosBUILD2 = CHECK_ROBOT_TYPE(0.033,0.033);
	Gripper.MotorPosBUILD3 = CHECK_ROBOT_TYPE(0.058,0.058);
	Gripper.MotorPosBUILD1sink = CHECK_ROBOT_TYPE(0.019,0.019);
	Gripper.MotorPosBUILD2sink = CHECK_ROBOT_TYPE(0.033,0.033);
	Gripper.MotorPosBUILD3sink = CHECK_ROBOT_TYPE(0.058,0.058);
	Gripper.MotorPosDrive = CHECK_ROBOT_TYPE(0.078,0.078);
	Gripper.motor = &liftVL;
	Gripper.MotNumber = LIFT_FRONT_LEFT_NBR;
	Gripper.ServoNumberPusherLeft = SE_RES2_NBR;
	Gripper.ServoNumberPusherRight = SE_RES1_NBR;
	Gripper.ServoPosGripClose = CHECK_ROBOT_TYPE(75,75);		//122
	Gripper.ServoPosGripOpen = CHECK_ROBOT_TYPE(95,95);		//105
	Gripper.ServoPosGripPushOpenLeft = CHECK_ROBOT_TYPE(115,115);
	Gripper.ServoPosGripPushCloseLeft = CHECK_ROBOT_TYPE(70,70);
	Gripper.ServoPosGripPushOpenRight = CHECK_ROBOT_TYPE(70,70);
	Gripper.ServoPosGripPushCloseRight = CHECK_ROBOT_TYPE(135,135);
	Gripper.ServoPosGripPush = CHECK_ROBOT_TYPE(67,67);
	Gripper.ServoPosGripRef = CHECK_ROBOT_TYPE(67,67);
	Gripper.ServoGripperPosCherry = CHECK_ROBOT_TYPE(75,75);
	Gripper.ServoPosCherryClose = CHECK_ROBOT_TYPE(119,119);
	Gripper.ServoPosCherryOpen = CHECK_ROBOT_TYPE(85,85);
	Gripper.ServoGripperNbr = SE_VL_NBR;
	Gripper.opMode = 0;						// Einerstelle Aufnehmen;  Zehnerstelle Ablegen   zB: 10 heist Bauteile werden Abgelegt
	Gripper.cakecount=0;
	Gripper.opModeFeedback = 0;
	Gripper.NbrElemets = 0;
	Gripper.CakeLayerType = 0;					// BROWN -> 01; YELLOW -> 10; PURPPLE -> 11		EXAMPLE=1111111010101 (Die unteren drei sind Braun, oberen drei Purple)
	Gripper.State = GR_INIT_STATE_0;
	Gripper.taskNbr = GRIPPER_TASKNBR;
	
	// set init Statebuilder
	StateBuilder = GR_CAKEBUILDER_START;
	nextStateBuilder = 0;
	ReadyBuilder = 0;
	countcake1=0;
	countcake2=0;
	countcake3=0;
	heightcake_left=0;
	heightcake_right=0;
	height_together=0;
	cakerest=0;
	cakemarker=0;
	cakedirektion=3;
	cherryheight_1=Gripper.MotorPosUP;
	cherryheight_2=Gripper.MotorPosUP;
	cherryheight_3=Gripper.MotorPosUP;
	cakeheight_1=0;
	cakeheight_2=0;
	cakeheight_3=0;
	
	
	// 	//set servos to init positions

	cmd_SetServo(SE_VR_NBR, Gripper.ServoGripperPosCherry);
	cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryClose);
	cmd_SetServo(SE_RES2_NBR, Gripper.ServoPosGripPushCloseLeft);
	cmd_SetServo(SE_RES1_NBR, Gripper.ServoPosGripPushCloseRight);
	
	/* cyclic task - cycle time: 100 ms */
	SET_CYCLE(GRIPPER_TASKNBR,100);
	SET_TASK(GRIPPER_TASKNBR, CYCLE);
	SET_TASK_HANDLE(GRIPPER_TASKNBR, GripperRightTask);
	
	/* cyclic task - cycle time: 100 ms */
	SET_CYCLE(CAKEBUILDER_TASKNBR,100);
	SET_TASK(CAKEBUILDER_TASKNBR, CYCLE);
	SET_TASK_HANDLE(CAKEBUILDER_TASKNBR, CakebuilderTask);
	

}

void Gripper_StateMachine (gripper_t *ps)
{
	uint8_t text1[100];
	switch (ps->State)
	{

		case GR_INIT_STATE_0:
		{
			cmd_SetServo(ps->MotNumber, ps->ServoGripperPosCherry);
			liftMotor_init(0,0,ps->MotNumber);
			cmd_SetMotorVel(ps->MotNumber,-0.018);
			ps->State = GR_INIT_STATE_1;
			break;
		}
		case GR_INIT_STATE_1: //drive lift down until switch hits
		{
			if (*(ps->RefSwitchPort))
			{
				cmd_SetMotorVel(ps->MotNumber,0);
				ps->State = GR_INIT_STATE_2;
				SET_CYCLE(ps->taskNbr, 100);			//settle time
			}
			break;
		}
		case GR_INIT_STATE_2:
		{
			liftMotor_init(1,0.0,ps->MotNumber);
			cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,ps->MotorPosRef);
			ps->State=GR_READY_PLACE;
			SET_CYCLE(ps->taskNbr, 1000);			//settle time

			break;
		}
		
		
		case GR_READY_PLACE:
		{
			se_array[SE_VR_NBR].phiPerSec = 30.0;
			se_array[SE_VL_NBR].phiPerSec = 30.0;
			ps->State=GR_READY_GRIP1;
			
			break;
		}
		
		case GR_READY_GRIP1:
		{
			
			if (*(ps->IRSensorPort1) == 1 || *(ps->IRSensorPort2) == 1 || *(ps->IRSensorPort3) == 1  )
			{
				SET_CYCLE(ps->taskNbr, 200);
				ps->State=GR_READY_GRIP2;
				
			}
			break;
		}
		
		case GR_READY_GRIP2:
		{
			sprintf(text1,"#GR_READY_GRIP2\r\n*");
			writeString_usart(&usartC0, text1);
			
			se_array[SE_VR_NBR].phiPerSec = 80.0;
			se_array[SE_VL_NBR].phiPerSec = 80.0;
			
			cmd_SetServo(ps->MotNumber, ps->ServoPosGripClose);
			SET_CYCLE(ps->taskNbr, 800);
			ps->State=GR_READY_GRIP3;
			break;
		}
		
		case GR_READY_GRIP3:
		{
			sprintf(text1, "#GR_READY_GRIP3\r\n*");
			writeString_usart(&usartC0, text1);
			
			
			cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,ps->MotorPosUP);
			cmd_SetServo(ps->ServoNumberPusherLeft, ps->ServoPosGripPushOpenLeft);
			cmd_SetServo(ps->ServoNumberPusherRight, ps->ServoPosGripPushOpenRight);
			
			ps->State=GR_READY_OPMODE1;
			break;
		}
		
		case GR_READY_OPMODE1:
		{
			sprintf(text1, "#GR_READY_OPMODE1 \r\n*");
			writeString_usart(&usartC0, text1);
			
			
			
			if (ps->motor->Odo.Dis >= (ps->MotorPosUP - 0.004))
			{
				ps->opModeFeedback = 1;
				
				sprintf(text1, "#gripper 1  \r\n*");
				writeString_usart(&usartC0, text1);
				

				if (ps->opMode == 2)
				{
					ps->State=GR_READY_TAKEOFF;
					SET_CYCLE(ps->taskNbr, 100);
				}
			}
			break;
		}
		
		case GR_READY_TAKEOFF:
		{
			
			cmd_SetServo(ps->MotNumber, ps->ServoPosGripOpen);
			
			SET_CYCLE(ps->taskNbr, 1000);
			ps->State=GR_READY_GRIPNEXT1;

			break;
		}
		
		case GR_READY_GRIPNEXT1:
		{
			
			cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,ps->MotorPosRef);
			//cmd_SetServo(ps->ServoNumberPusher, ps->ServoPosGripPushOpen);
			ps->State=GR_READY_GRIPNEXT1a;
			SET_CYCLE(ps->taskNbr, 500);
			
			
			break;
		}
		
		case GR_READY_GRIPNEXT1a:
		{
			cmd_Drive(0,0,MIN_VELOCITY,0,0,0,0, 40 ,POS_REL,OFF,NULL,NULL);
			
			ps->State= GR_MOTION_READY;
			ps->NextState=GR_READY_GRIPNEXT2;
			
			
			break;
		}
		
		case GR_READY_GRIPNEXT2:
		{
			
			if (ps->motor->Odo.Dis <= (ps->MotorPosRef + 0.004))
			{
				cmd_SetServo(ps->MotNumber, ps->ServoPosGripClose);
				ps->State=GR_READY_GRIPNEXT3;
				SET_CYCLE(ps->taskNbr, 500);
			}
			
			break;
		}
		
		case GR_READY_GRIPNEXT3:
		{
			cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,ps->MotorPosUP);
			
			
			
			ps->State=GR_READY_GRIPNEXT4;
			
			break;
		}
		
		case GR_READY_GRIPNEXT4:
		{
			
			if (ps->motor->Odo.Dis >= (ps->MotorPosUP - 0.004))
			{
				ps->opModeFeedback = 2;
				
				ps->State=GR_READY_OPMODE2;
				SET_CYCLE(ps->taskNbr, 200);
				sprintf(text1,"#GR_READY_GRIPNEXT4\r\n*");
				writeString_usart(&usartC0, text1);
			}
			
			break;
		}
		
		case GR_READY_OPMODE2:
		{
			if (ps->motor->Odo.Dis <= (ps->MotorPosUP + 0.004))
			{
				ps->opModeFeedback = 2;
				
				if (ps->opMode == 3)
				{
					ps->State=GR_READY_GRIPFULL;
					
					sprintf(text1,"#ps->opMode 3 \r\n*");
					writeString_usart(&usartC0, text1);
				}

			}
			break;
		}
		
		case GR_READY_GRIPFULL:
		{
			cmd_SetServo(ps->MotNumber, ps->ServoPosGripOpen);
			SET_CYCLE(ps->taskNbr, 1000);
			
			sprintf(text1,"#GR_READY_GRIPFULL \r\n*");
			writeString_usart(&usartC0, text1);
			
			ps->State=GR_READY_GRIPFULL2;
			
			break;
		}
		
		case GR_READY_GRIPFULL2:
		{
			cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,ps->MotorPosRef);
			
			sprintf(text1,"#GR_READY_GRIPFULL2 \r\n*");
			writeString_usart(&usartC0, text1);
			
			ps->State=GR_READY_GRIPFULL2a;
			SET_CYCLE(ps->taskNbr, 1000);

			break;
		}
		
		case GR_READY_GRIPFULL2a:
		{
			cmd_Drive(0,0,MIN_VELOCITY,0,0,0,0, 40 ,POS_REL,OFF,NULL,NULL);
			
			ps->State= GR_MOTION_READY;
			ps->NextState=GR_READY_GRIPFULL3;
			
			
			break;
		}
		
		case GR_READY_GRIPFULL3:
		{
			
			if (ps->motor->Odo.Dis <= (ps->MotorPosRef + 0.004))
			{
				cmd_SetServo(ps->MotNumber, ps->ServoPosGripClose);
				ps->State=GR_READY_GRIPFULL4;
				SET_CYCLE(ps->taskNbr, 1000);
			}
			
			break;
		}
		
		case GR_READY_GRIPFULL4:
		{
			cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED, 0.02);
			
			ps->State=GR_READY_OPMODE3;
			
			break;
		}
		
		case GR_READY_OPMODE3:
		{
			
			//cmd_SetServo(ps->ServoNumberPusher, ps->ServoPosGripPushClose);
			if (ps->motor->Odo.Dis >= (0.02 - 0.002))
			{
				ps->opModeFeedback = 3;
				cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,ps->MotorPosDrive);
				ps->State=GR_READY_Ready;
				
			}
			break;
		}
		
		case GR_READY_Ready:
		{
			break;
		}
		
		// MOTION READY
		case GR_MOTION_READY:
		{
			if(statusAntrieb == MOTION_OK)
			{
				ps->State = ps->NextState;
			}
			break;
		}
		
	}
	

}

void Cake_Builder ()
{
	uint8_t text1[100];
	//##################################################################################################
	// cake builder
	//##################################################################################################
	
	switch (StateBuilder)
	{
		
		case GR_CAKEBUILDER_START:
		{
			if (ReadyBuilder == 10)
			{
				
				
				StateBuilder = GR_CAKE1_BUILD1;
				
				cmd_SetServo(SE_RES1_NBR, Gripper.ServoPosGripPushCloseRight);
				cmd_SetServo(SE_RES2_NBR, Gripper.ServoPosGripPushCloseLeft);
				
				
				
			}
			else if (ReadyBuilder == 11)
			{
				
				if (SpielFarbe==BLUE_1||SpielFarbe==BLUE_2||SpielFarbe==BLUE_3||SpielFarbe==BLUE_4||SpielFarbe==BLUE_5 )
				{
					StateBuilder = GR_SORT0;
				} 
				else 
				{
					StateBuilder = GR_SORT2;
				}
				
				
				cmd_SetServo(SE_RES1_NBR, Gripper.ServoPosGripPushCloseRight);
				cmd_SetServo(SE_RES2_NBR, Gripper.ServoPosGripPushCloseLeft);
				
				
				
			}
			
			else if (ReadyBuilder == 20)
			{
				StateBuilder = GR_3CAKE;
				cmd_SetServo(SE_RES1_NBR, Gripper.ServoPosGripPushCloseRight);
				cmd_SetServo(SE_RES2_NBR, Gripper.ServoPosGripPushCloseLeft);
			}
			else if (ReadyBuilder == 30)
			{
				StateBuilder = GR_RECONCILE;
				cmd_SetServo(SE_RES1_NBR, Gripper.ServoPosGripPushCloseRight);
				cmd_SetServo(SE_RES2_NBR, Gripper.ServoPosGripPushCloseLeft);
			}
			
			break;
		}
		cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD+20,0,0,0,0,TURN_REL,OFF,NULL,NULL);
		case GR_SORT0:
		{
			cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD+20,0,0,0,0,TURN_REL,OFF,NULL,NULL);
			
			StateBuilder= GR_MOTION_READY;
			nextStateBuilder=GR_SORT1;
			break;
		}
		case GR_SORT1:
		{
			
			cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryClose);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			
			StateBuilder = GR_SORT2;
			
			break;
		}
		
		case GR_SORT2:
		{
			if (Position_erreicht(Gripper.MotorPosRef,0))
			{
				
				
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				StateBuilder = GR_SORT3;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		
		
		case GR_SORT3:
		{
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 );
			
			StateBuilder = GR_SORT4;
			
			break;
		}
		
		case GR_SORT4:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3,0))
			{
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,500);
				
				StateBuilder = GR_SORT5;
			}
			break;
		}
		case GR_SORT5:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_SORT6;
			
			break;
		}
		
		case GR_SORT6:
		{
			
			if (Position_erreicht(Gripper.MotorPosBUILD3+ LIFT_REL_POS_HUB,0))
			{
			
			cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD+20,0,0,0,0,TURN_REL,OFF,NULL,NULL);
			
			StateBuilder= GR_MOTION_READY;
			nextStateBuilder=GR_SORT7;
			}
			
			break;
		}
		
		case GR_SORT7:
		{
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			
			StateBuilder = GR_SORT8;
			
			break;
		}
		
		case GR_SORT8:
		{
			if (Position_erreicht(Gripper.MotorPosRef,0))
			{
				
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				
				StateBuilder = GR_SORT9;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		
		case GR_SORT9:
		{
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 );
			
			StateBuilder = GR_SORT10;
			
			break;
		}
		
		case GR_SORT10:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3,0))
			{
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,500);
				
				StateBuilder = GR_SORT11;
			}
			break;
		}
		case GR_SORT11:
		{
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_SORT12;
			
			break;
		}
		
		case GR_SORT12:
		{
			
			
			
			if (Position_erreicht(Gripper.MotorPosBUILD3+LIFT_REL_POS_HUB,0))
			{
			cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD+20,0,0,0,0,TURN_REL,OFF,NULL,NULL);
			
			StateBuilder= GR_MOTION_READY;
			nextStateBuilder=GR_SORT13;
			}
			
			break;
		}
		
		case GR_SORT13:
		{
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			
			StateBuilder = GR_SORT14;
			
			break;
		}
		
		case GR_SORT14:
		{
			if (Position_erreicht(Gripper.MotorPosRef,0))
			{
				
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				
				StateBuilder = GR_SORT15;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		
	
		case GR_SORT15:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_SORT16;
			
			break;
		}
		
		case GR_SORT16:
		{
			
			
			if (Position_erreicht(Gripper.MotorPosBUILD3+LIFT_REL_POS_HUB,0))
			{
			
			cmd_Drive(0,0,ANGULAR_VELOCITY ,2*ANGEL_CAKEBUILD+40,0,0,0,0,TURN_REL,OFF,NULL,NULL);
			
			StateBuilder= GR_MOTION_READY;
			nextStateBuilder=GR_SORT17;
			
			}
			break;
		}
		
		case GR_SORT17:
		{
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			
			StateBuilder = GR_SORT18;
			
			break;
		}
		
		case GR_SORT18:
		{
			if (Position_erreicht(Gripper.MotorPosRef,0))
			{
				
				
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,500);
				
				
				
				StateBuilder = GR_SORT19;
				
				
			}
			
			break;
		}
		
		case GR_SORT19:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_SORT20;
			
			break;
		}
		
		case GR_SORT20:
		{
			
			
			if (Position_erreicht(Gripper.MotorPosBUILD3+LIFT_REL_POS_HUB,0))
			{
			
			cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD+19,0,0,0,0,TURN_REL,OFF,NULL,NULL);
			
			StateBuilder= GR_MOTION_READY;
			nextStateBuilder=GR_SORT21;
			
			}
			break;
		}
		
		case GR_SORT21:
		{
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3sink );
			
			StateBuilder = GR_SORT22;
			
			break;
		}
		
		case GR_SORT22:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3sink,0))
			{
				
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				
				StateBuilder = GR_SORT23;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		case GR_SORT23:
		{
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			
			StateBuilder = GR_SORT24;
			
			break;
		}
		
		case GR_SORT24:
		{
			
			
			if (Position_erreicht(Gripper.MotorPosRef,0))
			{
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,500);
				StateBuilder = GR_SORT25;
				
				
			}
			break;
		}
		
		case GR_SORT25:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_SORT26;
			
			break;
		}
		
		case GR_SORT26:
		{
			
			
			if (Position_erreicht(Gripper.MotorPosBUILD3+LIFT_REL_POS_HUB,0))
			{
				
				cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD+19,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_SORT27;
				
			}
			break;
		}
		
		case GR_SORT27:
		{
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3sink );
			
			StateBuilder = GR_SORT28;
			
			break;
		}
		
		case GR_SORT28:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3sink,0))
			{
				
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				
				StateBuilder = GR_SORT29;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		case GR_SORT29:
		{
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			
			StateBuilder = GR_SORT30;
			
			break;
		}
		
		case GR_SORT30:
		{
			
			
			if (Position_erreicht(Gripper.MotorPosRef,0))
			{
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,500);
				StateBuilder = GR_SORT31;
				
				
			}
			break;
		}
		
		case GR_SORT31:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1);
			
			StateBuilder = GR_SORT32;
			
			break;
		}
		
		case GR_SORT32:
		{
			if (SpielFarbe==GREEN_1 || SpielFarbe==GREEN_2 ||SpielFarbe==GREEN_3 ||SpielFarbe==GREEN_4 ||SpielFarbe==GREEN_5)
			{
				if (Position_erreicht(Gripper.MotorPosBUILD1,0))
				{
					
					cmd_Drive(0,0,ANGULAR_VELOCITY ,2*ANGEL_CAKEBUILD+38,0,0,0,0,TURN_REL,OFF,NULL,NULL);
					
					StateBuilder= GR_MOTION_READY;
					nextStateBuilder=GR_CAKE1_OPEN;
					
				}
			} 
			else
			{
				if (Position_erreicht(Gripper.MotorPosBUILD1,0))
				{
					
					cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD+19,0,0,0,0,TURN_REL,OFF,NULL,NULL);
					
					StateBuilder= GR_MOTION_READY;
					nextStateBuilder=GR_CAKE1_OPEN;
					
				}
			}
			
			
			break;
		}
		
		case GR_CAKE1_BUILD1:
		{
			sprintf(text1,"#GR_CAKE1_BUILD1\r\n*");
			writeString_usart(&usartC0, text1);
			cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryClose);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1);
			
			StateBuilder = GR_CAKE1_OPEN;
			
			break;
		}
		
		case GR_CAKE1_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1,0))
			{
				sprintf(text1,"#GR_CAKE1_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				Punkte += 1;
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				StateBuilder = GR_CAKE1_CLOSE;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		case GR_CAKE1_CLOSE:
		{
			
			sprintf(text1,"#GR_CAKE1_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			
			cmd_SetServo(0, Gripper.ServoPosGripClose);
			SET_CYCLE(CAKEBUILDER_TASKNBR,500);
			
			StateBuilder = GR_CAKE1_LIFT;
			
			break;
		}
		
		case GR_CAKE1_LIFT:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_CAKE1_TURN;
			
			break;
		}
		
		case GR_CAKE1_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1,1))
			{
				sprintf(text1,"#GR_CAKE1_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CAKE1_SINK;
			}
			
			break;
		}
		
		case GR_CAKE1_SINK:
		{
			sprintf(text1,"#GR_CAKE1_SINK\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1sink);
			
			StateBuilder = GR_CAKE2_OPEN;
			
			break;
		}
		
		case GR_CAKE2_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1sink,0))
			{
				sprintf(text1,"#GR_CAKE2_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				Punkte += 1;
				
				StateBuilder = GR_CAKE2_CLOSE;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		case GR_CAKE2_CLOSE:
		{
			sprintf(text1,"#GR_CAKE2_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetServo(0, Gripper.ServoPosGripClose);
			SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			
			StateBuilder = GR_CAKE2_LIFT;
			
			break;
		}
		
		case GR_CAKE2_LIFT:
		{
			sprintf(text1,"#GR_CAKE2_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_CAKE2_TURN;
			
			break;
		}
		
		case GR_CAKE2_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1,1))
			{
				sprintf(text1,"#GR_CAKE2_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CAKE3_SINK;
			}
			
			break;
		}
		
		case GR_CAKE3_SINK:
		{
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1sink);
			
			StateBuilder = GR_CAKE3_OPEN;
			
			break;
		}
		
		case GR_CAKE3_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1sink,0))
			{
				sprintf(text1,"#GR_CAKE3_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				Punkte += 2;
				
				StateBuilder = GR_CAKE4_BUILD2;
			}
			
			break;
		}
		
		case GR_CAKE4_BUILD2:
		{
			sprintf(text1,"#GR_CAKE4_BUILD2\r\n*");
			writeString_usart(&usartC0, text1);
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD2);
			
			StateBuilder = GR_CAKE4_CLOSE;
			
			break;
		}
		
		case GR_CAKE4_CLOSE:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD2,0))
			{
				sprintf(text1,"#GR_CAKE4_CLOSE\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,500);
				
				StateBuilder = GR_CAKE4_LIFT;
			}
			
			break;
		}
		
		case GR_CAKE4_LIFT:
		{
			sprintf(text1,"#GR_CAKE4_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD2 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_CAKE4_TURN;
			
			break;
		}
		
		case GR_CAKE4_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD2,1))
			{
				sprintf(text1,"#GR_CAKE4_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CAKE4_SINK;
			}
			
			
			break;
		}
		
		case GR_CAKE4_SINK:
		{
			sprintf(text1,"#GR_CAKE4_SINK\r\n*");
			writeString_usart(&usartC0, text1);
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD2sink);
			
			StateBuilder = GR_CAKE5_OPEN;
			
			break;
		}
		
		case GR_CAKE5_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD2sink,0))
			{
				sprintf(text1,"#GR_CAKE5_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				Punkte += 1;
				
				StateBuilder = GR_CAKE5_CLOSE;
			}
			
			break;
		}
		
		case GR_CAKE5_CLOSE:
		{
			sprintf(text1,"#GR_CAKE5_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetServo(0, Gripper.ServoPosGripClose);
			SET_CYCLE(CAKEBUILDER_TASKNBR,500);
			
			StateBuilder = GR_CAKE5_LIFT;
			
			break;
		}
		
		case GR_CAKE5_LIFT:
		{
			sprintf(text1,"#GR_CAKE5_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD2 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_CAKE5_TURN;
			
			break;
		}
		
		case GR_CAKE5_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD2 + LIFT_REL_POS_HUB ,0))
			{
				sprintf(text1,"#GR_CAKE5_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CAKE6_SINK;
			}
			
			break;

		}
		
		case GR_CAKE6_SINK:
		{
			sprintf(text1,"#GR_CAKE6_SINK\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD2sink);
			
			StateBuilder = GR_CAKE6_OPEN;
			
			break;
		}
		
		case GR_CAKE6_OPEN:
		{
			
			if (Position_erreicht(Gripper.MotorPosBUILD2sink,0))
			{
				sprintf(text1,"#GR_CAKE6_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				Punkte += 6;
				
				StateBuilder = GR_CAKE7_BUILD3;
			}
			
			
			
			break;
		}
		case GR_CAKE7_BUILD3:
		{
			sprintf(text1,"#GR_CAKE7_BUILD3\r\n*");
			writeString_usart(&usartC0, text1);
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3);
			
			StateBuilder = GR_CAKE7_CLOSE;
			
			break;
		}
		case GR_CAKE7_CLOSE:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3,0))
			{
				sprintf(text1,"#GR_CAKE7_CLOSE\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,500);
				
				StateBuilder = GR_CAKE7_LIFT;
			}
			
			break;
		}
		case GR_CAKE7_LIFT:
		{
			sprintf(text1,"#GR_CAKE4_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_CAKE7_TURN;
			
			break;
		}
		case GR_CAKE7_TURN:
		{
			sprintf(text1,"#GR_CAKE7_TURN\r\n*");
			writeString_usart(&usartC0, text1);
			if (Position_erreicht(Gripper.MotorPosBUILD3 +LIFT_REL_POS_HUB,0))
			{
				cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CAKE7_SINK;
			}
			break;
		}
		
		case GR_CAKE7_SINK:
		{
			sprintf(text1,"#GR_CAKE6_SINK\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3);
			
			StateBuilder = GR_CAKE8_OPEN;
			
			break;
		}

		case GR_CAKE8_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3,0))
			{
				sprintf(text1,"#GR_CAKE8_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				Punkte += 5;
				
				StateBuilder = GR_CAKE8_CLOSE;
			}
			break;
		}
		case GR_CAKE8_CLOSE:
		{
			sprintf(text1,"#GR_CAKE8_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetServo(0, Gripper.ServoPosGripClose);
			SET_CYCLE(CAKEBUILDER_TASKNBR,500);
			
			StateBuilder = GR_CAKE8_LIFT;
			
			break;
		}
		case GR_CAKE8_LIFT:
		{
			sprintf(text1,"#GR_CAKE8_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder = GR_CAKE8_TURN;
			
			break;
		}
		case GR_CAKE8_TURN:
		{
			sprintf(text1,"#GR_CAKE8_TURN\r\n*");
			writeString_usart(&usartC0, text1);
			
			if (Position_erreicht(Gripper.MotorPosBUILD3+ LIFT_REL_POS_HUB,0))
			{
				cmd_Drive(0,0,-ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CAKE8_SINK;
			}
			
			break;
		}
		case GR_CAKE8_SINK:
		{
			sprintf(text1,"#GR_CAKE6_SINK\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3);
			
			StateBuilder = GR_CAKE9_OPEN;
			
			break;
		}
		case GR_CAKE9_OPEN:
		{
			sprintf(text1,"#GR_CAKE9_OPEN\r\n*");
			writeString_usart(&usartC0, text1);
			if (Position_erreicht(Gripper.MotorPosBUILD3,0))
			{
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				Punkte += 5;
				
				StateBuilder = GR_CAKE9_CHERRYPOS;
			}
			break;
		}
		case GR_CAKE9_CHERRYPOS:
		{
			sprintf(text1,"#GR_CAKE9_CHERRYPOS\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosCherry);
			
			StateBuilder = GR_GRIPCHERRY_POS;
			
			break;
		}
		
		//####################  CHERRY   ############################
		case GR_GRIPCHERRY_POS:
		{
			sprintf(text1,"#GR_GRIPCHERRY_POS\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosCherry);
			cmd_SetServo(0, Gripper.ServoGripperPosCherry);
			SET_CYCLE(CAKEBUILDER_TASKNBR, 300);
			
			StateBuilder = GR_CHERRY1_TURN;
			
			break;
		}
		
		case GR_CHERRY1_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosCherry,0))
			{
				sprintf(text1,"#GR_CHERRY1_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,ANGULAR_VELOCITY , 18 ,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CHERRY1_CAKEBUILD3;
			}
			
			break;
		}
		
		case GR_CHERRY1_CAKEBUILD3:
		{
			sprintf(text1,"#GR_CHERRY1_CAKEBUILD3\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 - 0.003);
			
			StateBuilder = GR_CHERRY1_OPEN;
			
			break;
		}
		
		case GR_CHERRY1_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3 - 0.003,0))
			{
				sprintf(text1,"#GR_CHERRY1_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryOpen);
				Punkte += 3;
				SET_CYCLE(CAKEBUILDER_TASKNBR, 400);
				
				StateBuilder = GR_CHERRY1_CLOSE;
			}
			
			break;
		}
		
		case GR_CHERRY1_CLOSE:
		{
			sprintf(text1,"#GR_CHERRY1_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryClose);
			
			StateBuilder = GR_CHERRY1_CHERRYPOS;
			
			break;
		}
		
		case GR_CHERRY1_CHERRYPOS:
		{
			sprintf(text1,"#GR_CHERRY1_CHERRYPOS\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosCherry);
			
			StateBuilder = GR_CHERRY2_TURN;
			
			break;
		}
		case GR_CHERRY2_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosCherry,0))
			{
				sprintf(text1,"#GR_CHERRY2_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CHERRY2_CAKEBUILD3;
			}
			
			break;
		}
		case GR_CHERRY2_CAKEBUILD3:
		{
			sprintf(text1,"#GR_CHERRY2_CAKEBUILD3\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 - 0.003);
			
			StateBuilder = GR_CHERRY2_OPEN;
			
			break;
		}
		case GR_CHERRY2_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3 - 0.003,0))
			{
				sprintf(text1,"#GR_CHERRY2_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryOpen);
				Punkte += 3;
				SET_CYCLE(CAKEBUILDER_TASKNBR, 400);
				
				StateBuilder = GR_CHERRY2_CLOSE;
			}
			break;
		}
		
		case GR_CHERRY2_CLOSE:
		{
			sprintf(text1,"#GR_CHERRY2_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryClose);
			
			StateBuilder = GR_CHERRY2_CHERRYPOS;
			
			break;
		}
		case GR_CHERRY2_CHERRYPOS:
		{
			sprintf(text1,"#GR_CHERRY2_CHERRYPOS\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosCherry);
			
			StateBuilder = GR_CHERRY3_TURN;
			
			break;
		}
		case GR_CHERRY3_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosCherry,0))
			{
				sprintf(text1,"#GR_CHERRY3_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder=GR_CHERRY3_CAKEBUILD3;
			}
			
			break;
		}
		case GR_CHERRY3_CAKEBUILD3:
		{
			sprintf(text1,"#GR_CHERRY3_CAKEBUILD3\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 - 0.003);
			
			StateBuilder = GR_CHERRY3_OPEN;
			
			break;
		}
		case GR_CHERRY3_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3 - 0.003,0))
			{
				sprintf(text1,"#GR_CHERRY3_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryOpen);
				Punkte += 3;
				SET_CYCLE(CAKEBUILDER_TASKNBR, 400);
				
				StateBuilder = GR_CHERRY3_CLOSE;
			}
			
			break;
		}
		case GR_CHERRY3_CLOSE:
		{
			sprintf(text1,"#GR_CHERRY3_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetServo(SE_K1_NBR, Gripper.ServoPosCherryClose);
			
			StateBuilder = GR_CHERRY3_CHERRYPOS;
			
			break;
		}
		case GR_CHERRY3_CHERRYPOS:
		{
			sprintf(text1,"#GR_CHERRY3_CHERRYPOS\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosCherry);
			
			StateBuilder = GR_CAKE_READY;
			
			break;
		}
		
		case GR_CAKE_READY:
		{
			if (Position_erreicht(Gripper.MotorPosCherry,0))
			{
				
				CakeFinished = TRUE;
			}
			
			break;
		}
		
		
		// MOTION READY
		case GR_MOTION_READY:
		{
			if(statusAntrieb == MOTION_OK)
			{
				StateBuilder = nextStateBuilder;
			}
			break;
		}
		
		case GR_3CAKE:
		{
			
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1);
			
			StateBuilder = GR_3CAKE_OPEN;
			
			break;
		}
		case  GR_3CAKE_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1,0))
			{
				sprintf(text1,"#GR_CAKE1_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				Punkte += 1;
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				StateBuilder = GR_3CAKE_LIFT;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		case  GR_3CAKE_LIFT:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3);
			
			StateBuilder =  GR_3CAKE_CLOSE;
			
			break;
		}
		
		case  GR_3CAKE_CLOSE:
		{
			
			sprintf(text1,"#GR_CAKE1_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			
			if (Position_erreicht(Gripper.MotorPosBUILD3,1))
			{
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				StateBuilder = GR_3CAKE_LIFT2;
			}
			break;
		}
		
		
		case  GR_3CAKE_LIFT2:
		{
			sprintf(text1,"#GR_CAKE1_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder =  GR_3CAKE_TURN;
			
			break;
		}
		
		case  GR_3CAKE_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3,1))
			{
				sprintf(text1,"#GR_CAKE1_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder= GR_3CAKE_SINK;
			}
			
			break;
		}
		
		case  GR_3CAKE_SINK:
		{
			sprintf(text1,"#GR_CAKE1_SINK\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1);
			
			StateBuilder =  GR_3CAKE_OPEN;
			
			break;
		}
		
		case  GR_3CAKE2_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1,0))
			{
				sprintf(text1,"#GR_CAKE2_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				
				Punkte += 1;
				
				StateBuilder =  GR_3CAKE2_LIFT;
				
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
			}
			
			break;
		}
		
		case  GR_3CAKE2_LIFT:
		{
			sprintf(text1,"#GR_CAKE2_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3);
			
			StateBuilder =  GR_3CAKE2_CLOSE;
			
			break;
		}
		
		case  GR_3CAKE2_CLOSE:
		{
			sprintf(text1,"#GR_CAKE2_CLOSE\r\n*");
			writeString_usart(&usartC0, text1);
			if (Position_erreicht(Gripper.MotorPosBUILD3,1))
			{
				cmd_SetServo(0, Gripper.ServoPosGripClose);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				StateBuilder =  GR_3CAKE2_LIFT2;
			}
			break;
		}
		
		case  GR_3CAKE2_LIFT2:
		{
			sprintf(text1,"#GR_CAKE2_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD3 + LIFT_REL_POS_HUB);
			
			StateBuilder =  GR_3CAKE_TURN;
			
			break;
		}
		
		case  GR_3CAKE2_TURN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD3,1))
			{
				sprintf(text1,"#GR_CAKE2_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_Drive(0,0,ANGULAR_VELOCITY ,ANGEL_CAKEBUILD,0,0,0,0,TURN_REL,OFF,NULL,NULL);
				
				StateBuilder= GR_MOTION_READY;
				nextStateBuilder= GR_3CAKE2_SINK;
			}
			
			break;
		}
		
		case  GR_3CAKE2_SINK:
		{
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosBUILD1);
			
			StateBuilder =  GR_3CAKE3_OPEN;
			
			break;
		}
		
		case  GR_3CAKE3_OPEN:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1,0))
			{
				sprintf(text1,"#GR_CAKE3_OPEN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				
				Punkte += 2;
				
				StateBuilder =GR_CAKE9_CHERRYPOS;
			}
			
			break;
		}
		
		//Kuchen zwischen Ablegen
		
		case  GR_RECONCILE:
		{
			sprintf(text1,"#GR_CAKE5_LIFT\r\n*");
			writeString_usart(&usartC0, text1);
			
			cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			
			StateBuilder = GR_RECONCILE2;
			
			break;
		}
		
		case GR_RECONCILE2:
		{
			if (Position_erreicht(Gripper.MotorPosBUILD1 ,0))
			{
				sprintf(text1,"#GR_CAKE5_TURN\r\n*");
				writeString_usart(&usartC0, text1);
				
				cmd_SetServo(0, Gripper.ServoPosGripOpen);
				SET_CYCLE(CAKEBUILDER_TASKNBR,200);
				CakeFinished = TRUE;
				StateBuilder=GR_CAKEBUILDER_START;
				ReadyBuilder =0;
				Gripper.opMode=0;
				Gripper.opModeFeedback=0;
				cmd_SetMotorPos(0,LIFT_FRONT_SPEED,Gripper.MotorPosRef);
			}
			
			break;

		}
		
	}
	
}





/**************************************************************************
***   FUNKTIONNAME: SetNextStep                                         ***
***   FUNKTION: Setzt die nächsten Schritte                             ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: Current ... aktueller State                   ***
***                       Next ... nächster State -> bei OK             ***
***                       Error ... nächster State -> bei Error         ***
**************************************************************************/
void SetNextStep(unsigned int Current, unsigned int Next, unsigned int Error)
{
	KI_State = 65000;
	KI_StateNext = Next;
	KI_StateOld = Current;
	KI_StateError = Error;
}


/**************************************************************************
***   FUNKTIONNAME: Position erreicht                                   ***
***   FUNKTION: Setzt die nächsten Schritte                             ***
**************************************************************************/

uint8_t Position_erreicht(float sollPosition, uint8_t Lift)
{
	if (Lift)
	{
		if (Gripper.motor->Odo.Dis >= (sollPosition + LIFT_REL_POS_HUB - LIFT_TOLERANCE) && Gripper.motor->Odo.Dis <= (sollPosition + LIFT_REL_POS_HUB + LIFT_TOLERANCE))
		{
			return(TRUE);
			
		}
		else
		{
			return(FALSE);
		}
	}

	else
	{
		if (Gripper.motor->Odo.Dis >= (sollPosition - LIFT_TOLERANCE) && Gripper.motor->Odo.Dis <= (sollPosition + LIFT_TOLERANCE))
		{
			return(TRUE);
		}
		else
		{
			return(FALSE);
		}
		
		
	}
}





//##############################################################################################
// Aufruf State Machine
//##############################################################################################

uint8_t GripperRightTask()
{
	SET_CYCLE(GRIPPER_TASKNBR, 10);
	Gripper_StateMachine(&Gripper);
	
	return(CYCLE);
}

uint8_t CakebuilderTask()
{
	SET_CYCLE(CAKEBUILDER_TASKNBR, 10);
	Cake_Builder();
	
	return(CYCLE);
}
