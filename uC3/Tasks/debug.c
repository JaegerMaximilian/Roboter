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
#include "rrt_transmittingtask.h"
#include "sensor.h"
#include "UNDK20.h"
#include "servo.h"
#include "PSE541.h"
#include "motor.h"
#include "dynamixel.h"


 
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
}





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
	static uint8_t startup = 1;
	static uint8_t ledBit = 0x20;
//	uint8_t data[6] = {1,2,3,4,5,6};
		uint8_t ret;
		uint8_t *data1;
	
   uint8_t text1[150], text2[150], data[50] = {0,1,2,3,4,5,6,7};
	static convData_t d;
	
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

   
   SET_CYCLE(DEBUG_TASKNBR, 1500);
  

   
//     sprintf(text1, "µC3 :: %d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d \r", COLOUR_REAR_1, COLOUR_REAR_2, 
// 					IR_REAR_1, IR_REAR_2, IR_REAR_3, IR_REAR_4, IR_REAR_5, IR_RESERVE_1, 
// 					IR_RESERVE_2, IR_RESERVE_3, IR_RESERVE_4);
//      sprintf(text1, "µC3 :: %d:%d:%d:%d:%d \r", us1.distance, us2.distance, us3.distance, us4.distance, us5.distance);
//  	debugMsg(text1);

//    LED_PORT.OUT ^= ledBit;
// //   ledBit = (ledBit != 0x40) ? ledBit << 1 : 0x20;
//    ledBit = (ledBit != 0x80) ? ledBit << 1 : 0x20;

	TOGGLE_PIN(LED_PORT, LED1);
	TOGGLE_PIN(LED_PORT, LED2);
	TOGGLE_PIN(LED_PORT, LED3);

 	//sprintf(text1,"Hello World! %2.1f, ES: %d\r\n", getAccumulatorVoltage(), PORTF.IN & 0x03);
	//sprintf(text1,"Druck_Links: %d, Druck_rechts: %d\r\n", pressureRearLeft.pressure, pressureRearRight.pressure);
	//sprintf(text1,"Gal_V: %d, Gal_H: %d\r\n", Gal_V, Gal_H);
 	//debugMsg(text1);
	 
//  	 if(State == 0)
//  	 {
//  		 State = 1;
//  		 SET_MV_LEFT;
//  		 SET_MV_RIGHT;
//  		 SET_VACUUM_PUMP_LEFT;
//  		 SET_VACUUM_PUMP_RIGHT;
//   		 setVelocity(&liftHL, 0.05);
//   		 setVelocity(&liftHR, 0.05);
//  		 AX_setLEDControl(&(ax_servo[0]),0);
//  		 AX_setTorqueControl(&(ax_servo[0]),1);
//  		 AX_setMaximumTorqueRAM(&(ax_servo[0]),100);
//  		 AX_setPositionSpeed_direct(&(ax_servo[0]), DEG2RAD(10),DEG2RAD(360));
// 		 AX_setLEDControl(&(ax_servo[1]),0);
// 		 AX_setTorqueControl(&(ax_servo[1]),1);
// 		 AX_setMaximumTorqueRAM(&(ax_servo[1]),100);
// 		 AX_setPositionSpeed_direct(&(ax_servo[1]), DEG2RAD(10),DEG2RAD(360));
// 		 AX_setLEDControl(&(ax_servo[2]),0);
// 		 AX_setTorqueControl(&(ax_servo[2]),1);
// 		 AX_setMaximumTorqueRAM(&(ax_servo[2]),100);
// 		 AX_setPositionSpeed_direct(&(ax_servo[2]), DEG2RAD(10),DEG2RAD(360));
// 		 AX_setLEDControl(&(ax_servo[3]),0);
// 		 AX_setTorqueControl(&(ax_servo[3]),1);
// 		 AX_setMaximumTorqueRAM(&(ax_servo[3]),100);
// 		 AX_setPositionSpeed_direct(&(ax_servo[3]), DEG2RAD(10),DEG2RAD(360));
//  	 }
//  	 else
//  	 {
//  		 State = 0;
//  		 CLR_MV_LEFT;
//  		 CLR_MV_RIGHT;
//  		 CLR_VACUUM_PUMP_LEFT;
//  		 CLR_VACUUM_PUMP_RIGHT;
//  		 setVelocity(&liftHL, -0.05);
//   		 setVelocity(&liftHR, -0.05);
//  		 AX_setLEDControl(&(ax_servo[0]),1);
//  		 AX_setPositionSpeed_direct(&(ax_servo[0]), DEG2RAD(-10),DEG2RAD(360));
// 		 AX_setLEDControl(&(ax_servo[1]),1);
// 		 AX_setPositionSpeed_direct(&(ax_servo[1]), DEG2RAD(-10),DEG2RAD(360));
// 		 AX_setLEDControl(&(ax_servo[2]),1);
// 		 AX_setPositionSpeed_direct(&(ax_servo[2]), DEG2RAD(-10),DEG2RAD(360));
// 		 AX_setLEDControl(&(ax_servo[3]),1);
// 		 AX_setPositionSpeed_direct(&(ax_servo[3]), DEG2RAD(-10),DEG2RAD(360));
// 	 }
	 

//	rpLidar_sendRequest(&rpLidar, RPLIDAR_GET_HEALTH);


	
	//CANRec = 0;
	//
	//CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 2, 0, &data[0]);
	//CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 2, 0, &data[2]);
// 	CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 2, 0, &data[4]);
// 	CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 2, 0, &data[6]);
	

// 	sabertooth_2x60_sendCommand(&(sabertoothModules[0]),sabertoothModules[0].adr, SABERTOOTH_2X60_DRIVE_FORWARD_MOTOR_2, State);
// 	State = ((++State > 127) ? 0 : State);
// 	setVelocity(&motorLeft, 0.2);
// 	setVelocity(&motorRight, 0.2);
	
// 	if (State == 0)
// 	{
// 		State = 1;
// 		AX_setSpeed_direct(&(ax_servo[1]), 10.0);
// 	}
// 	else if (State == 1)
// 	{
// 		State = 2;
// 		AX_setSpeed_direct(&(ax_servo[1]), 0.0);
// 	}
// 	else if (State == 2)
// 	{
// 		State = 3;
// 		AX_setSpeed_direct(&(ax_servo[1]), -10.0);
// 	}
// 	else if (State == 3)
// 	{
// 		State = 0;
// 		AX_setSpeed_direct(&(ax_servo[1]), 0.0);
// 	}
// 	if (State == 0)
// 	{
// 		d.int8[0] = 90;
// 		d.int8[1] = 90;
// 		d.int8[2] = 90;
// 		d.int8[3] = 90;
// 		
//  		data[0] = RRTLAN_DYN_PRO_GOAL_POSITION_GROUP1;
//  		data[1] = CAN_REGISTER_WRITE;
//  		data[2] = d.uint8[0];
//  		data[3] = d.uint8[1];
//  		data[4] = d.uint8[2];
//  		data[5] = d.uint8[3];
//  		CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 6, 0x400, data);
// 
//  		data[0] = RRTLAN_DYN_PRO_GOAL_POSITION_GROUP2;
//  		data[1] = CAN_REGISTER_WRITE;
//  		data[2] = d.uint8[0];
//  		data[3] = d.uint8[1];
//  		data[4] = d.uint8[2];
//  		data[5] = d.uint8[3];
//  		CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 6, 0x400, data);
// 		 
// 		SET_CYCLE(DEBUG_TASKNBR, 10);
// 		State = 1;
// 	}
// 	else if (State == 1)
// 	{
// 
// 		data[0] = RRTLAN_DYN_PRO_GOAL_POSITION_GROUP3;
// 		data[1] = CAN_REGISTER_WRITE;
// 		data[2] = 90;
// 		data[3] = 90;
// 		data[4] = 90;
// 		data[5] = 90;
// 		CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 6, 0x400, data);
// 
// 		State = 2;
// 	} 
// 	else if (State == 2)
// 	{
//  		data[0] = RRTLAN_DYN_PRO_GOAL_POSITION_GROUP1;
//  		data[1] = CAN_REGISTER_WRITE;
//  		data[2] = 0;
//  		data[3] = 0;
//  		data[4] = 0;
//  		data[5] = 0;
//  		CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 6, 0x400, data);
// 		 
//  		data[0] = RRTLAN_DYN_PRO_GOAL_POSITION_GROUP2;
//  		data[1] = CAN_REGISTER_WRITE;
//  		data[2] = 0;
//  		data[3] = 0;
//  		data[4] = 0;
//  		data[5] = 0;
//  		CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 6, 0x400, data);
// 		 
// 		 SET_CYCLE(DEBUG_TASKNBR, 10);
// 		 
// 		 State = 3;
// 	}
// 	else if (State == 3)
// 	{
// 		data[0] = RRTLAN_DYN_PRO_GOAL_POSITION_GROUP3;
// 		data[1] = CAN_REGISTER_WRITE;
// 		data[2] = 0;
// 		data[3] = 0;
// 		data[4] = 0;
// 		data[5] = 0;
// 		CAN_mcp2515_sendMessage(&CAN_mcp2515_SPIC, 6, 0x400, data);
// 
// 		State = 0;
// 		 
// 	}
// 	
	
	
	


   
	//RRTLAN_TransportLayer_addNewMsg(&recList, &test, data1, 10);
	//RRTLAN_TransportLayer_addNewMsg(&recList, &test, data2, 10);
	//RRTLAN_TransportLayer_addNewMsg(&recList, &test, data3, 10);
	//
	//RRTLAN_TransportLayer_removeMsg(&recList, &(recList.last));
	//RRTLAN_TransportLayer_removeMsg(&recList, &(recList.last));
	//RRTLAN_TransportLayer_removeMsg(&recList, &(recList.last));
	//RRTLAN_TransportLayer_removeMsg(&recList, &(recList.last));
	
// 	sprintf(text1, "Hello World!\r\n");
// 	writeString_usart(&RRTLAN_PHYSICALLAYER_INTERFACE_SERIAL_1, text1);
	
	///* RRTLAN timestamp message - test scenario */
	//data[0] = 0x01;
	//data[1] = (uint8_t)(RRTLAN_MY_ID);
	//data[2] = (uint8_t)(RRTLAN_MY_ID >> 8);;
	//data[3] = 0x00;
	//data[4] = 0x0C;
	//data[5] = 0x05;
	//data[6] = 0x0A;
	//data[7] = 0x11;
	//data[8] = 0x22;
	//data[9] = 0x33;
	//data[10] = 0x44;

	
	///* RRTLAN write - test scenario */
	//data[0] = 0x01;
	//data[1] = (uint8_t)(RRTLAN_MY_ID);
	//data[2] = (uint8_t)(RRTLAN_MY_ID >> 8);;
	//data[3] = 0x00;
	//data[4] = 0x0C;
	//data[5] = 0x00;
	//data[6] = 0x03;
	//
	//data[7] = 0x11;
	//data[8] = 0x22;
	//data[9] = 0x33;
	//data[10] = 0x44;
	//
	///* valid data */
	///* hwid */
	//data[11] = 0x00;
	//data[12] = 0x00;
	//data[13] = 0x48;
	//data[14] = 0x40;
	///* reg adr */
	//data[15] = 0x03;
	//data[16] = 0x00;
	///* value */
	//d.uint32 += 51;
	//data[17] = d.uint8[0];
	//data[18] = d.uint8[1];
	//data[19] = d.uint8[2];
	//data[20] = d.uint8[3];
	
	///* invalid hwid */
	///* hwid */
	//data[21] = 0xFF;
	//data[22] = 0x00;
	//data[23] = 0x04;
	//data[24] = 0x08;
	///* reg adr */
	//data[25] = 0x06;
	//data[26] = 0x00;
	///* value */
	//data[27] = 0x11;
	//data[28] = 0x33;
	//data[29] = 0x55;
	//data[30] = 0x77;
//
	///* invalid read only */
	///* hwid */
	//data[31] = 0x00;
	//data[32] = 0x00;
	//data[33] = 0x04;
	//data[34] = 0x08;
	///* reg adr */
	//data[35] = 0x03;
	//data[36] = 0x00;
	///* value */
	//data[37] = 0x11;
	//data[38] = 0x33;
	//data[39] = 0x55;
	//data[40] = 0x77;
//
	///* invalid reg adr */
	///* hwid */
	//data[41] = 0x00;
	//data[42] = 0x00;
	//data[43] = 0x04;
	//data[44] = 0x08;
	///* reg adr */
	//data[45] = 0xFF;
	//data[46] = 0x00;
	///* value */
	//data[47] = 0x11;
	//data[48] = 0x33;
	//data[49] = 0x55;
	//data[50] = 0x77;

	
	///* RRTLAN get modules - test scenario */
	//data[0] = 0x01;
	//data[1] = (uint8_t)(RRTLAN_MY_ID);
	//data[2] = (uint8_t)(RRTLAN_MY_ID >> 8);;
	//data[3] = 0x00;
	//data[4] = 0x0C;
	//data[5] = 0x00;
	//data[6] = 0x05;
	//
	
	
	///* RRTLAN read - test scenario */
	//data[0] = 0x01;
	//data[1] = (uint8_t)(RRTLAN_MY_ID);
	//data[2] = (uint8_t)(RRTLAN_MY_ID >> 8);;
	//data[3] = 0x00;
	//data[4] = 0x0C;
	//data[5] = 0x00;
 	///* read */
 	//data[6] = 0x02;
	//
	//data[7] = 0x11;
	//data[8] = 0x22;
	//data[9] = 0x33;
	//data[10] = 0x44;
//
	///* read ID */
	//data[11] = 0x01;
	//data[12] = 0x00;
	//
	///* valid data */
	///* hwid */
	//data[13] = 0x00;
	//data[14] = 0x08;
	//data[15] = 0x20;
	//data[16] = 0x04;
	///* reg adr */
	//data[17] = 0x01;
	//data[18] = 0x00;
	//
	///* invalid hwid */
	///* hwid */
	//data[19] = 0xFF;
	//data[20] = 0x00;
	//data[21] = 0x04;
	//data[22] = 0x08;
	///* reg adr */
	//data[23] = 0x06;
	//data[24] = 0x00;
	//
	///* valid read only */
	///* hwid */
	//data[25] = 0x00;
	//data[26] = 0x00;
	//data[27] = 0x04;
	//data[28] = 0x08;
	///* reg adr */
	//data[29] = 0x03;
	//data[30] = 0x00;
	//
	///* invalid reg adr */
	///* hwid */
	//data[31] = 0x00;
	//data[32] = 0x00;
	//data[33] = 0x04;
	//data[34] = 0x08;
	///* reg adr */
	//data[35] = 0xFF;
	//data[36] = 0x00;
	//
	

	///* RRTLAN cyclic read - test scenario */
	//data[0] = 0x01;
	//data[1] = (uint8_t)(RRTLAN_MY_ID);
	//data[2] = (uint8_t)(RRTLAN_MY_ID >> 8);;
	//data[3] = 0x01;
	//data[4] = 0x00;
	//data[5] = 0x00;
	//data[6] = 0x08;
	//
	//data[7] = 0x11;
	//data[8] = 0x22;
	//data[9] = 0x33;
	//data[10] = 0x44;

//static uint8_t k = 0;
//
	///* read ID */
	//data[11] = 0x01 + k; // k++;
	//data[12] = 0x00;
	//
	///*  cycle time 500 ms */
	//data[13] = 15;
	//data[14] = 0;
	//
	///* valid data */
	///* hwid */
	//data[15] = 0x00;
	//data[16] = 0x00;
	//data[17] = 0x18;
	//data[18] = 0x20;
	///* reg adr */
	//data[19] = 0x03;
	//data[20] = 0x00;
	//
	///* hwid */
	//data[21] = 0x00;
	//data[22] = 0x00;
	//data[23] = 0x18;
	//data[24] = 0x20;
	///* reg adr */
	//data[25] = 0x04;
	//data[26] = 0x00;


	///* RRTLAN ack - test scenario */
	//data[0] = 0x01;
	//data[1] = (uint8_t)(RRTLAN_MY_ID);
	//data[2] = (uint8_t)(RRTLAN_MY_ID >> 8);
	//data[3] = 0x00;
	//data[4] = 0x0C;
	//data[5] = 0x00;
	//data[6] = 0x01;
	//
	//data[7] = 0x11;
	//data[8] = 0x22;
	//data[9] = 0x33;
	//data[10] = 0x44;
//
	///* read ID */
	//data[11] = 0x01;
//
	///* allocate memory for the Acknowledge message */
	//cli();
	//data1 = (uint8_t *) malloc(RRTLAN_ACK_MSG_LENGTH);
	//sei();
					//
	///* if memory was allocated -> ad the message to the queue */
	//if ((data1 != NULL) && (State == 0))
	//{
		//convData_t sys_time;
		//sys_time.uint32 = RRTLAN_getSystemTime();
						//
		///* Acknowledge message */
		//data1[0] = RRTLAN_CURRENT_VERSION;
		//data1[1] = (uint8_t)(RRTLAN_MY_ID);
		//data1[2] = (uint8_t)(RRTLAN_MY_ID >> 8);
		//data1[3] = 0x00;
		//data1[4] = 0x0C;
		//data1[5] = 0x01;
		//data1[6] = RRTLAN_READ_MSG_TYPE;
						//
		//data1[7] = sys_time.uint8[0];
		//data1[8] = sys_time.uint8[1];
		//data1[9] = sys_time.uint8[2];
		//data1[10] = sys_time.uint8[3];
						//
		//data1[11] = 0x00;
//
		///* if the message couldn't connected to the queue -> free data memory */
		//RRTLAN_TransportLayer_addNewMsg(&transList, &trans_msg, data1, RRTLAN_ACK_MSG_LENGTH, RRTLAN_RESEND_NBR, &RRTLAN_PHYSICALLAYER_INTERFACE_SERIAL_2);
	//}

// ***************************** ************************************

		

	///* allocate memory for the Acknowledge message */
	//data1 = RRTLAN_GetTransBuffer();
					//
	///* if memory was allocated -> ad the message to the queue */
	//if (data1 != NULL)
	//{
		//convData_t sys_time;
		//sys_time.uint32 = RRTLAN_getSystemTime();
						//
		///* Acknowledge message */
		//data1[0] = RRTLAN_CURRENT_VERSION;
		//data1[1] = (uint8_t) 0x1600;
		//data1[2] = (uint8_t) 0x1600 >> 8;
		//data1[3] = (uint8_t) RRTLAN_MY_ID;
		//data1[4] = (uint8_t) RRTLAN_MY_ID >> 8;
		//data1[5] = RRTLAN_NO_ACK_ID;
		//data1[6] = RRTLAN_ACK_MSG_TYPE;
						//
		//data1[7] = sys_time.uint8[0];
		//data1[8] = sys_time.uint8[1];
		//data1[9] = sys_time.uint8[2];
		//data1[10] = sys_time.uint8[3];
						//
		//data1[11] = 1;
//
		///* confirm message -> increment in index of the receive buffer, set data length and pointer to usart */
		//RRTLAN_ConfirmTransMessage(RRTLAN_ACK_MSG_LENGTH, RRTLAN_NO_RESEND, &RRTLAN_PHYSICALLAYER_INTERFACE_SERIAL_1);
	//}

 	//if (State == 0)
 	//{
		 ////uint8_t text[100];
////sprintf(text,"Get CAN Message! -> %d\r\n", CANRec++);
////writeString_usart(&usartE1, text);
		//
		 //
//		RRTLAN_Datalink_Layer_SendData_serial(&RRTLAN_PHYSICALLAYER_INTERFACE_SERIAL_1, data, 27);
		 //
 	//}
//
	//State = 1;
	
	//if (startup++ > 10)
	//{
		//State = 0;
		//startup = 0;
	//}
	

	//SET_PIN(BREMSEN_PORT, BREMSE_LINKS);
	//SET_PIN(BREMSEN_PORT, BREMSE_RECHTS);
//
//
	//if (State == 0)
	//{
		//State = 1;
		//setVelocity(&motorRight, 0.2);
		//setVelocity(&motorLeft, 0.2);
	//} 
	//else if (State == 1)
	//{
		//State = 2;
		//setVelocity(&motorRight, 0.0);
		//setVelocity(&motorLeft, 0.0);
	//}
	//else if (State == 2)
	//{
		//State = 3;
		//setVelocity(&motorRight, -0.2);
		//setVelocity(&motorLeft, -0.2);
	//}
	//else
	//{
		//State = 0;
		//setVelocity(&motorRight, 0.0);
		//setVelocity(&motorLeft, 0.0);
	//}



	//if (State == 0)
	//{
		//State = 1;
		//calcLookupTableBytewise(POLYNOMIAL_32);
	//}

	//mcp2515_Send_CAN_Message(&mcp2515_CAN,CAN_KS_TRANSMIT_OFFSET,0,0,SINGLE,5,data);
	//if (startup == 0)
	//{
		//startup = 1;
		//mcp2515_Send_CAN_Message(&mcp2515_CAN,0,0,0,SINGLE,1,data);
	//}
	//if (startup < 10)
	//{
		//convData_t d2b;
		//
		//startup++;
		//
		///* store register */
		//d2b.uint32 = 0x40480000;
		//
		///* msg -> reg_adr - DATA:0 - DATA:1 - DATA:2 - DATA:3 */
		//data[0] = CAN_HARDWARE_ID;
		//data[1] = d2b.uint8[0];
		//data[2] = d2b.uint8[1];
		//data[3] = d2b.uint8[2];
		//data[4] = d2b.uint8[3];
		//data[5] = 4;
		///* send hardware ID cyclic */
		//mcp2515_Send_CAN_Message(&mcp2515_CAN, 0x711, 0, 0, SINGLE, 6, data);
//
//
		///* store register */
		//d2b.uint32 = 0x4048C000;
		//
		///* msg -> reg_adr - DATA:0 - DATA:1 - DATA:2 - DATA:3 */
		//data[0] = CAN_HARDWARE_ID;
		//data[1] = d2b.uint8[0];
		//data[2] = d2b.uint8[1];
		//data[3] = d2b.uint8[2];
		//data[4] = d2b.uint8[3];
		//data[5] = 4;
		///* send hardware ID cyclic */
		//mcp2515_Send_CAN_Message(&mcp2515_CAN, 0x715, 0, 0, SINGLE, 6, data);
	//}


//
//
	//sprintf(text1,"%.3f:%.3f - %.3f:%.3f - %.3f:%.3f - %d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\r\n", nbb_RC_values.leftJoystick[JOYSTICK_UP_DOWN],
	//nbb_RC_values.leftJoystick[JOYSTICK_LEFT_RIGHT],
	//nbb_RC_values.rightJoystick[JOYSTICK_UP_DOWN],
	//nbb_RC_values.rightJoystick[JOYSTICK_LEFT_RIGHT],
	//nbb_RC_values.centerJoystick[JOYSTICK_UP_DOWN],
	//nbb_RC_values.centerJoystick[JOYSTICK_LEFT_RIGHT],
	//RC_NOT_AUS,
	//RC_HUPE,
	//RC_LANGSAM,
	//RC_LICHT,
	//RC_FLIPPER,
	//RC_CAM,
	//RC_REF,
	//RC_RESET,
	//RC_ARM,
	//RC_DRIVE,
	//RC_AUTO,
	//RC_FORWARD);
	//writeString_usart(&usartD0, text1);
	
//	sprintf(text1, "Hello World!\r\n");
	//writeString_usart(&usartE1, text1);
	//writeString_usart(&usartD0, text1);
	
	// 	AX_Ping(&(ax_servo[0]));
// 	AX_Buffer[4] = 0;
	
	//if (State == 0)
	//{
		//State = 1;
	////	AX_setLEDControl(&(ax_servo[0]), 0);
//// 		ret = AX_setReadCommand(&(ax_servo[0]),3,1);
//// 	sprintf(text1,"%d:%d:%d:%d:%d - ID: %d %d (%d)\r\n", AX_Buffer[0], AX_Buffer[1], AX_Buffer[2], AX_Buffer[3], AX_Buffer[4], AX_Buffer[5], AX_Buffer[5], ret);
//
////	writeString_usart(&usartD0, text1);
		//setVelocity(&motorLeft, 0.2);
		//setVelocity(&motorRight, 0.2);
	//} 
	//else if(State == 1)
	//{
		//State = 0;
		////AX_setLEDControl(&(ax_servo[0]), 1);
		//setVelocity(&motorLeft, -0.2);
		//setVelocity(&motorRight, -0.2);
	//}
 
//	if (parameter[TORQUE_1].changed == 1)
	//{
		//parameter[TORQUE_1].changed = 0;
		//AX_setMaximumTorqueRAM(&(ax_servo[0]), (uint8_t)parameter[TORQUE_1].RAM);
	//}
   //
	//if (parameter[TORQUE_2].changed == 1)
	//{
		//parameter[TORQUE_2].changed = 0;
		//AX_setMaximumTorqueRAM(&(ax_servo[1]), (uint8_t)parameter[TORQUE_2].RAM);
	//}
//
	//if (parameter[TORQUE_3].changed == 1)
	//{
		//parameter[TORQUE_3].changed = 0;
		//AX_setMaximumTorqueRAM(&(ax_servo[2]), (uint8_t)parameter[TORQUE_3].RAM);
	//}
	//
	//error = AX_setReadCommand(&(ax_servo[0]),AX_PRESENT_POSITION_L, 2);
	//phi_1 = (uint16_t)AX_Buffer[5] + ((uint16_t)AX_Buffer[6]) * 256;
	//error = AX_setReadCommand(&(ax_servo[1]),AX_PRESENT_POSITION_L, 2);
	//phi_2 = (uint16_t)AX_Buffer[5] + ((uint16_t)AX_Buffer[6]) * 256;
	//error = AX_setReadCommand(&(ax_servo[2]),AX_PRESENT_POSITION_L, 2);
	//phi_3 = (uint16_t)AX_Buffer[5] + ((uint16_t)AX_Buffer[6]) * 256;
	//
//
 	//sprintf(text1,"%d - %d - %d\r\n", phi_1, phi_2,phi_3);
 	//writeString_usart(&usartD0, text1);
//
	//if (abs((int16_t)phi_1 - (int16_t)phi_1_alt) > 5)
	//{
		//AX_setPositionSpeed_direct(&(ax_servo[0]), AX_invers_transformAngle(phi_1),1.0);
		//phi_1_alt = phi_1;
	//}
//
	//if (abs((int16_t)phi_2 - (int16_t)phi_2_alt) > 5)
	//{
		//if (phi_2 > 800)
		//{
			//phi_2 = 800;
		//}
		//if (phi_2 < 250)
		//{
			//phi_2 = 250;
		//}
		//AX_setPositionSpeed_direct(&(ax_servo[1]), AX_invers_transformAngle(phi_2),1.0);
		//phi_2_alt = phi_2;
	//}
//
	//if (abs((int16_t)phi_3 - (int16_t)phi_3_alt) > 5)
	//{
		//if (phi_3 > 600)
		//{
			//phi_3 = 600;
		//}
		//if (phi_3 < 400)
		//{
			//phi_3 = 400;
		//}
		//AX_setPositionSpeed_direct(&(ax_servo[2]), AX_invers_transformAngle(phi_3),1.0);
		//phi_3_alt = phi_3;
	//}
	//
	//
	//
	//
	//
	//if (State == 0)
	//{
		//State = 1;
		////AX_setReadCommand(&(ax_servo[0]),AX_PRESENT_POSITION_L, 2);
		////phi_1 = (uint16_t)AX_Buffer[5] + ((uint16_t)AX_Buffer[6]) * 256;
		////AX_setReadCommand(&(ax_servo[1]),AX_PRESENT_POSITION_L, 2);
		////phi_2 = (uint16_t)AX_Buffer[5] + ((uint16_t)AX_Buffer[6]) * 256;
		////AX_setReadCommand(&(ax_servo[2]),AX_PRESENT_POSITION_L, 2);
		////phi_3 = (uint16_t)AX_Buffer[5] + ((uint16_t)AX_Buffer[6]) * 256;
////
	////sprintf(text1,"%d - %d - %d\r\n", phi_1, phi_2,phi_3);
	////writeString_usart(&usartD0, text1);
////
		////
		////AX_setOperatingMode(&(ax_servo[0]), AX_JOINT_MODE);
////
		////AX_setMaximumTorqueRAM(&(ax_servo[0]), 10);
		////AX_setPositionSpeed_direct(&(ax_servo[0]), AX_invers_transformAngle(phi_1),10.0);
		////AX_setOperatingMode(&(ax_servo[1]), AX_JOINT_MODE);
////
		////AX_setMaximumTorqueRAM(&(ax_servo[1]), 5);
		////AX_setPositionSpeed_direct(&(ax_servo[1]), AX_invers_transformAngle(phi_2),10.0);
		////AX_setOperatingMode(&(ax_servo[2]), AX_JOINT_MODE);
////
		////AX_setMaximumTorqueRAM(&(ax_servo[2]), 5);
		////AX_setPositionSpeed_direct(&(ax_servo[2]), AX_invers_transformAngle(phi_3),10.0);
		////
		////AX_setTorqueControl(&(ax_servo[0]), AX_MODE_TORQUE_FREE_RUN);
		////AX_setTorqueControl(&(ax_servo[1]), AX_MODE_TORQUE_FREE_RUN);
		////AX_setTorqueControl(&(ax_servo[2]), AX_MODE_TORQUE_FREE_RUN);		
////AX_setReadCommand(&(ax_servo[0]),AX_TORQUE_ENABLE, 1);
////sprintf(text1,"%d\r\n", AX_Buffer[5]);
////writeString_usart(&usartD0, text1);
////AX_setReadCommand(&(ax_servo[1]),AX_TORQUE_ENABLE, 1);
////sprintf(text1,"%d\r\n", AX_Buffer[5]);
////writeString_usart(&usartD0, text1);
////AX_setReadCommand(&(ax_servo[0]),AX_TORQUE_ENABLE, 1);
////sprintf(text1,"%d\r\n", AX_Buffer[5]);
////writeString_usart(&usartD0, text1);
//
//
	//} 
	//else
	//{
	////	State = 0;
	//}
	//
	////


   return(CYCLE);
}
