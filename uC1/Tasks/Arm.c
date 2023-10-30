/*
* cupStacker.c
*
* Created: 15/12/2020 09:56:07
*  Author: danny
*/

#define _ARM_EXTERN_

#include "Arm.h"
#include "sensor.h"
#include "servo.h"
#include "command.h"
#include "multitask.h"
#include "define.h"
#include "liftMotor.h"


float cupPos[2] = {LIFT_FRONT_POS_3, LIFT_FRONT_POS_2};


void InitArms()
{
// 	cupStackerLeft.ColorSensorPort = &uC1sensorenPortA;
// 	cupStackerLeft.ColorSensorMask = 0x10;
// 	cupStackerLeft.IRSensorPort=&uC1sensorenPortF;
// 	cupStackerLeft.IRSensorMask= 0x02;
	ArmLeft.VacNumber=VACUUM_FRONT_LEFT_NBR;
	ArmLeft.MotNumber=LIFT_FRONT_LEFT_NBR;
	ArmLeft.RefSwitchPort=&uC1sensorenPortF;
	ArmLeft.RefSwitchMask=0x01;
	ArmLeft.taskNbr=ARM_LEFT_TASKNBR;
	ArmLeft.hexagonNumber = 0;
	ArmLeft.motor = &liftVL;
	ArmLeft.pressure = &pressureFrontLeft;
	ArmLeft.ServoSucker = SE_VL_NBR;
	ArmLeft.ServoSuckerPos0 = CHECK_ROBOT_TYPE(SE_RES4,SE_S_SUCKER_LEFT_0);
	ArmLeft.ServoSuckerPos1 = CHECK_ROBOT_TYPE(SE_SUCKER_LEFT_1,SE_S_SUCKER_LEFT_1);
	ArmLeft.ServoSuckerPos2 = CHECK_ROBOT_TYPE(SE_SUCKER_LEFT_2,SE_S_SUCKER_LEFT_2);
	ArmLeft.ServoArmPosRef = 0.065;
 	ArmLeft.ServoArm = SE_K1_NBR;
 	ArmLeft.ServoArmPos0 = SE_HR;
 	ArmLeft.ServoArmPos1 = CHECK_ROBOT_TYPE(SE_SILO_LEFT2_1,SE_S_SILO_LEFT2_1);
// 	ArmLeft.ServoSilo2 = SE_HL_NBR;
// 	ArmLeft.ServoSilo2Pos0 = SE_HL;
// 	ArmLeft.ServoSilo2Pos1 = CHECK_ROBOT_TYPE(SE_SILO_LEFT1_1,SE_S_SILO_LEFT1_1);
	
// 	ArmRight.ColorSensorPort=&uC1sensorenPortA;
// 	ArmRight.ColorSensorMask=0x20;
// 	ArmRight.IRSensorPort=&uC1sensorenPortF;
// 	ArmRight.IRSensorMask=0x02;
	ArmRight.VacNumber=VACUUM_FRONT_RIGHT_NBR;
	ArmRight.MotNumber=LIFT_FRONT_RIGHT_NBR;
	ArmRight.RefSwitchPort=&uC1sensorenPortF;
	ArmRight.RefSwitchMask=0x02;
	ArmRight.taskNbr=ARM_RIGHT_TASKNBR;
	ArmRight.hexagonNumber = 0;
	ArmRight.motor = &liftVR;
	ArmRight.pressure = &pressureFrontRight;
	ArmRight.ServoSucker = SE_VR_NBR;
	ArmRight.ServoSuckerPos0 = CHECK_ROBOT_TYPE(SE_RES3,SE_S_SUCKER_RIGHT_0);
	ArmRight.ServoSuckerPos1 = CHECK_ROBOT_TYPE(SE_SUCKER_RIGHT_1,SE_S_SUCKER_RIGHT_1);
	ArmRight.ServoSuckerPos2=CHECK_ROBOT_TYPE(SE_SUCKER_RIGHT_2,SE_S_SUCKER_RIGHT_2);
	ArmRight.ServoArmPosRef = 0.03;
 	ArmRight.ServoArm = SE_K1_NBR;
 	ArmRight.ServoArmPos0 = SE_HR;
 	ArmRight.ServoArmPos1 = CHECK_ROBOT_TYPE(SE_SILO_LEFT2_1,SE_S_SILO_LEFT2_1);
// 	ArmRight.ServoSilo2 = SE_RES1_NBR;
// 	ArmRight.ServoSilo2Pos0 = SE_RES1;
// 	ArmRight.ServoSilo2Pos1 = CHECK_ROBOT_TYPE(SE_SILO_RIGHT1_1,SE_S_SILO_RIGHT1_1);
	
	//set values in Stacks to empty
	for (uint8_t i = 0; i<CS_STACKSIZE;i++)
	{
		ArmRight.Cupstack[i]=CS_EMPTY;
		ArmLeft.Cupstack[i]=CS_EMPTY;
	}
	
// 	//set servos to init positions
// 	cmd_SetServo(SE_HR_NBR, ArmLeft.ServoArmPos0);
// 	cmd_SetServo(SE_RES2_NBR,ArmRight.ServoArmPos0);
// 	cmd_SetServo(SE_RES3_NBR,ArmLeft.ServoSuckerPos0);
// 	cmd_SetServo(SE_RES4_NBR,ArmRight.ServoSuckerPos0);
// 	cmd_CtrlVacuum(VACUUM_FRONT_LEFT_NBR,CMD_VACUUM_OFF);
// 	cmd_CtrlVacuum(VACUUM_FRONT_RIGHT_NBR,CMD_VACUUM_OFF);
// 
// 	
 	/* cyclic task - cycle time: 100 ms */
 	SET_CYCLE(ARM_LEFT_TASKNBR,100);
 	SET_TASK(ARM_LEFT_TASKNBR, CYCLE);
 	SET_TASK_HANDLE(ARM_LEFT_TASKNBR, ArmLeftTask);
 
 	/* cyclic task - cycle time: 100 ms */
 	SET_CYCLE(ARM_RIGHT_TASKNBR,100);
 	SET_TASK(ARM_RIGHT_TASKNBR, CYCLE);
 	SET_TASK_HANDLE(ARM_RIGHT_TASKNBR, ArmRightTask);

}

//ps ... pointer structure
void Arm_StateMachine(Arm_t *ps)
{
 	//switch (ps->State)
 	//{
		//case CS_INIT_STATE_00:
		//{			
			//cmd_SetServo(SE_ARM_FRONTRIGHT_NBR, 70);
			//
			//SET_CYCLE(ps->taskNbr, 50);
			//ps->State = CS_INIT_STATE_0;
			//break;
		//}
		//case CS_INIT_STATE_0:
		//{
			//cmd_SetServo(SE_ARM_FRONTLEFT_NBR, -20);
			//
			//SET_CYCLE(ps->taskNbr, 1000);
			//ps->State = CS_INIT_STATE_1;
			//break;
		//}
 		//case CS_INIT_STATE_1:
 		//{
			//cmd_SetServo(SE_VR_NBR, 67);
 			//liftMotor_init(0,0,ps->MotNumber);
 			//cmd_SetMotorVel(ps->MotNumber,-0.03);
 			//ps->State = CS_INIT_STATE_2;
 			//break;
 		//}
 		//case CS_INIT_STATE_2: //drive lift down until switch hits
 		//{
 			//if (CHECK_REF(ps))
 			//{
 				//cmd_SetMotorVel(ps->MotNumber,0);
 				//ps->State = CS_INIT_STATE_3;
 				//SET_CYCLE(ps->taskNbr, 100);			//settle time
 			//}
 			//break;
 		//}
 		//case CS_INIT_STATE_3:
 		//{ 			
  			//liftMotor_init(1,LIFT_FRONT_POS_REF,ps->MotNumber);
  			//cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,ps->ServoArmPosRef);
  			//ps->State=CS_INIT_STATE_4;
  			//SET_CYCLE(ps->taskNbr, 1000);			//settle time
 //
 			//break;
 		//}
		//case CS_INIT_STATE_4:
		//{
			//if(liftVL.Odo.Dis >= (ArmLeft.ServoArmPosRef - 0.002) && liftVR.Odo.Dis >= (ArmRight.ServoArmPosRef - 0.002))
			//{
				//cmd_SetServo(SE_ARM_FRONTRIGHT_NBR, -90);
				//cmd_SetServo(SE_ARM_FRONTLEFT_NBR, -50);
				//ps->State=CS_READY_STATE;
			//}
			//SET_CYCLE(ps->taskNbr, 10);
			//break;
		//}
//// 		
//// 		
//// 		
//// 		case CS_READY_STATE:
//// 		{
//// 			/* collect */
//// 			if ((ps->opMode == CS_COLLECT) && (ps->hexagonNumber < 3))
//// 			{
//// 				/* red cup */
//// 				if ((CHECK_IR(ps)) && (CHECK_COLOR(ps) == CS_RED))
//// 				{
//// 					ps->Cupstack[ps->hexagonNumber] = CS_RED;
//// 					cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,LIFT_FRONT_POS_1);
//// 					/* turn on vacuum, only if less than 2 cups are selected */
//// 					if (ps->hexagonNumber < 2)
//// 					{
//// 						cmd_CtrlVacuum(ps->VacNumber, 1);
//// 						ps->State = CS_SUCK_CUP;
//// 					}
//// 					else
//// 					{
//// 						ps->hexagonNumber = 3;
//// 					}
//// 					
//// 					
//// 				}
//// 				/*   green cup   */
//// 				else if ((CHECK_IR(ps)) && (CHECK_COLOR(ps) == CS_GREEN))
//// 				{
//// 					ps->Cupstack[ps->hexagonNumber] = CS_GREEN;
//// 					cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,LIFT_FRONT_POS_1);
//// 					/* turn on vacuum, only if less than 2 cups are selected */
//// 					if (ps->hexagonNumber < 2)
//// 					{
//// 						cmd_CtrlVacuum(ps->VacNumber, 1);
//// 						ps->State = CS_SUCK_CUP;
//// 					}
//// 					else
//// 					{
//// 						ps->hexagonNumber = 3;
//// 					}
//// 					
//// 				}
//// 			}
//// 			/* place */
//// 			else if (ps->opMode == CS_PLACE)
//// 			{
//// 				
//// 				cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,LIFT_FRONT_POS_0); /* lift sleeve to free cup*/
//// 				if (ps->hexagonNumber==3)
//// 				{
//// 					ps->hexagonNumber=2;
//// 					ps->State=CS_WAIT_READY_PLACE;
//// 					SET_CYCLE(ps->taskNbr,500);
//// 				}
//// 				else if (ps->hexagonNumber==2)
//// 				{
//// 					cmd_SetServo(ps->ServoArm,ps->ServoArmPos0);
//// 					ps->hexagonNumber=1;
//// 					ps->State=CS_WAIT_READY_PLACE;
//// 					SET_CYCLE(ps->taskNbr,500);
//// 				}
//// 				else if (ps->hexagonNumber==1)
//// 				{
//// 					cmd_SetServo(ps->ServoArm,ps->ServoArmPos1);
//// 					ps->State=CS_PLACE_TOP_0;
//// 					SET_CYCLE(ps->taskNbr,750);
//// 				}
//// 				else
//// 				{
//// 					ps->State=CS_WAIT_READY_PLACE;
//// 				}
//// 				
//// 			}
//// 			//idle    wait for further instructions from KI
//// 			else if (ps->opMode == CS_IDLE)
//// 			{
//// 
//// 			}
//// 			break;
//// 		}
//// 		
//// 		case CS_PLACE_TOP_0:
//// 		{
//// 			cmd_SetServo(ps->ServoSilo2,ps->ServoSilo2Pos0);
//// 			ps->hexagonNumber=0;
//// 			SET_CYCLE(ps->taskNbr,500);
//// 			ps->State=CS_PLACE_TOP_1;
//// 			break;
//// 		}
//// 		case CS_PLACE_TOP_1:
//// 		{
//// 			cmd_SetServo(ps->ServoArm,ps->ServoArmPos0);
//// 			SET_CYCLE(ps->taskNbr,500);
//// 			ps->State=CS_WAIT_READY_PLACE;
//// 			break;
//// 		}
//// 		
//// 		/* disables pick-up mode and idles until KI triggers ps->opMode to place */
//// 		case CS_WAIT_READY_PLACE:
//// 		{
//// 			ps->State=CS_READY_STATE;
//// 			ps->opMode=CS_IDLE;
//// 			break;
//// 		}
//// 		
//// 		/* swing sucker servo in once position is reached */
//// 		case CS_SUCK_CUP:
//// 		{
//// 			if (ps->motor->Odo.Dis < (LIFT_FRONT_POS_1 + 0.005))
//// 			{
//// 				cmd_SetServo(ps->ServoSucker, ps->ServoSuckerPos1); //active position
//// 				ps->timeOut = 0;
//// 				ps->State = CS_STORE_CUP;
//// 			}
//// 			break;
//// 		}
//// 		
//// 		/* sucker servo to neutral position and lift to respective storage position */
//// 		case CS_STORE_CUP:
//// 		{
//// 			if (ps->pressure->pressure < 700)
//// 			{
//// 				cmd_SetServo(ps->ServoSucker, ps->ServoSuckerPos2);
//// 				cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,cupPos[ps->hexagonNumber]);
//// 				ps->State = CS_LOCK1_CUP;
//// 			}
//// 			else
//// 			{
//// 				// after timeout -> go to initial state of cupstacker
//// 				if (++(ps->timeOut) > CS_TIME_OUT)
//// 				{
//// 					cmd_CtrlVacuum(ps->VacNumber, 0);
//// 					cmd_SetServo(ps->ServoSucker, ps->ServoSuckerPos0);
//// 					cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,LIFT_FRONT_POS_0);
//// 					ps->State = CS_READY_STATE;
//// 					
//// 				}
//// 				
//// 			}
//// 			
//// 			break;
//// 		}
//// 		
//// 		/* lock cup and give time to settle
//// 		"if CupNumber==0 set ServoSilo2 to Pos1, else set ServoSilo1 to Pos1" */
//// 		case CS_LOCK1_CUP:
//// 		{
//// 			if (ps->motor->Odo.Dis > (cupPos[ps->hexagonNumber] - 0.001))
//// 			{
//// 				cmd_SetServo(((ps->hexagonNumber == 0) ? ps->ServoSilo2 : ps->ServoArm), ((ps->hexagonNumber == 0) ? ps->ServoSilo2Pos1 : ps->ServoArmPos1));
//// 				SET_CYCLE(ps->taskNbr, 500);
//// 				ps->State = CS_LOCK2_CUP;
//// 			}
//// 			
//// 			break;
//// 		}
//// 		/* after settle time, turns vac off and servo sucker goes back to inactive position */
//// 		case CS_LOCK2_CUP:
//// 		{
//// 			cmd_CtrlVacuum(ps->VacNumber, 0);
//// 			cmd_SetServo(ps->ServoSucker, ps->ServoSuckerPos0);
//// 			ps->State = CS_WAIT_POS;
//// 			SET_CYCLE(ps->taskNbr, 200);
//// 			break;
//// 		}
//// 		/* cupNumber count goes up, lift goes back to neutral position */
//// 		case CS_WAIT_POS:
//// 		{
//// 			(ps->hexagonNumber)++;
//// 			cmd_SetMotorPos(ps->MotNumber,LIFT_FRONT_SPEED,LIFT_FRONT_POS_0);
//// 			ps->State = CS_READY_STATE;
//// 			break;
//// 		}
//// 		
//}
}

uint8_t ArmLeftTask()
{
	SET_CYCLE(ARM_LEFT_TASKNBR, 10);
	Arm_StateMachine(&ArmLeft);
	
	return(CYCLE);
}
uint8_t ArmRightTask()
{
	SET_CYCLE(ARM_RIGHT_TASKNBR, 10);
	Arm_StateMachine(&ArmRight);
	
	return(CYCLE);
}



