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
#include "DIPswitch.h"
#include "LEDcontrol.h"
#include <stdint.h>
#include "accumulator.h"
#include "motor.h"
#include "dynamixel.h"
#include <util/delay.h>
#include "QTR_1RC.h"
#include "StartDevice.h"
 



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
   static uint8_t State = 0, count = 0;
   char text[100];
	uint8_t ret;
	static uint8_t led = 0, pos_change = 0;
	static float pos = 20.0;
   
   SET_CYCLE(DEBUG_TASKNBR, 500);
   
//	count = (++count < 2 ? count : 0);
	
	TOGGLE_LED;
	
//	sprintf(text,"BL %d , FL %d , BR %d , FR %d \r\n", groundSensors[GROUNDS_BACK_LEFT].status, groundSensors[GROUNDS_FRONT_LEFT].status, groundSensors[GROUNDS_BACK_RIGHT].status, groundSensors[GROUNDS_FRONT_RIGHT].status);
	sprintf(text,"Hello World! - %.1fV - %d\r\n", getAccumulatorVoltage(), StartDevice_ReadOut());
	writeString_usart(&usartC1, text);	
	
	
	//
	//if(++pos_change >= 1)
	//{
		//pos_change = 0;
		//led = ((led == 0) ? 1: 0);
		//pos = ((pos > 0.0) ? -15.0 : 15.0);
	//}
	//
	//sprintf(text,"Start Transmission -> (%d)!\r", ax_servo[count].ID);
	//writeString_usart(&usartC1, text);
	
//	ret = AX_Ping(&(ax_servo[count]));
//	ret = AX_setWriteCommand(ax_servo + count, 0, 12);
	//ret = AX_setLEDControl(ax_servo + count, led);
	//
	//_delay_us(100);
	//ret = AX_setReadCommand(ax_servo + count, 36, 8);
	//
	//_delay_us(100);
	
	//switch (State)
	//{
		//case 0:
		//{
			//State = 1;
			//
			//ret = AX_setOperatingMode(ax_servo + 0, AX_WHEEL_MODE);
			//ret = AX_setOperatingMode(ax_servo + 1, AX_WHEEL_MODE);
			//
			//SET_CYCLE(DEBUG_TASKNBR, 100);
		//
			//break;
		//}		
		//case 1:
		//{
			//State = 1;
			//
			//ret = AX_setSpeed_direct(ax_servo + 0, 70);
			//ret = AX_setSpeed_direct(ax_servo + 1, 70);
			//ret = AX_setReadCommand(ax_servo + 0, AX_PRESENT_POSITION_L, 2);
//
			//SET_CYCLE(DEBUG_TASKNBR, 100);
			//
			//
			//break;
		//}
		//case 2:
		//{
			//State = 1;
			//
			//ret = AX_setSpeed_direct(ax_servo + 0, -70);
			//ret = AX_setSpeed_direct(ax_servo + 1, -70);
//
			//ret = AX_setReadCommand(ax_servo + count, AX_PRESENT_POSITION_L, 2);
			//
			//SET_CYCLE(DEBUG_TASKNBR, 500);
			//
			//break;
		//}
	//}
//
	////ret = AX_setPositionSpeed_notDirect(ax_servo + 0, pos, 10);
	////_delay_us(100);
	////ret = AX_setPositionSpeed_notDirect(ax_servo + 1, pos, 10);
	////_delay_us(100);
	////ret = AX_Action(ax_servo);
//
	//
	////ret = AX_setLEDControl(ax_servo + 1, led);
	////ret = AX_setPositionSpeed_direct(ax_servo + 0, pos, 10);
	////ret = AX_setPositionSpeed_direct(ax_servo + 1, pos, 10);
	////ret = AX_setId(ax_servo + count, 9);
	////ret = AX_Reset(ax_servo + count);
//
	//sprintf(text,"Msg -> %d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d.%d : Pos -> %d\r\r", AX_Buffer[AX_START_1],
																												//AX_Buffer[AX_START_2],
																												//AX_Buffer[AX_ID],
																												//AX_Buffer[AX_LENGTH],
																												//AX_Buffer[AX_ERROR],
																												//AX_Buffer[AX_FIRST_PARAMETER],
																												//AX_Buffer[AX_FIRST_PARAMETER + 1],
																												//AX_Buffer[AX_FIRST_PARAMETER + 2],
																												//AX_Buffer[AX_FIRST_PARAMETER + 3],
																												//AX_Buffer[AX_FIRST_PARAMETER + 4],
																												//AX_Buffer[AX_FIRST_PARAMETER + 5],
																												//AX_Buffer[AX_FIRST_PARAMETER + 6],
																												//AX_Buffer[AX_FIRST_PARAMETER + 7],
																												//AX_Buffer[AX_FIRST_PARAMETER + 8],
																												//AX_Buffer[AX_FIRST_PARAMETER] + AX_Buffer[AX_FIRST_PARAMETER + 1] * 256);
	//writeString_usart(&usartC1, text);
//
	//CLR_LED;
	//
   //if (count++ > 10)
   //{
		//count = 0;
		//if(State == 0)
		//{
			////setVelocity(&motorLeft, 1.0);
			//
////			setMotion(&motorLeft,1.5,0.80);
			//setVelocity(&motorLeft, 1.0);
			//
			//State = 10;
		//}   
		//else if(State == 10)
		//{
		////	setMotion(&motorLeft,1.5,0.36);
			//setVelocity(&motorLeft, 1.0);
//
			//State = 0;
		//}   
   //}
   //
   //
		//if (DIPswitch_ReadOut_total() == 1)
		//{
			//setVelocity(&motorLeft, 1.0);
			//setVelocity(&motorRight, 1.0);
			//SET_LED;
		//}
		//else if (DIPswitch_ReadOut_total() == 2)
		//{
			//setVelocity(&motorLeft, -1.0);
			//setVelocity(&motorRight, -1.0);
			//TOGGLE_LED;
		//}
		//else 
		//{
			//setVelocity(&motorLeft, 0.0);
			//setVelocity(&motorRight, 0.0);
			//CLR_LED;
		//}			
   //
		//TOGGLE_LED;
//
		//sprintf(text,"PORTE: 3 2 1 0 -> %d %d %d %d :: %d; Akku: %2.2f \r", DIPswitch_ReadOut_individual(3),
		//DIPswitch_ReadOut_individual(2),
		//DIPswitch_ReadOut_individual(1),
		//DIPswitch_ReadOut_individual(0),
		//DIPswitch_ReadOut_total(),
		//getAccumulatorVoltage());
		//writeString_usart(&usartC1, text);
																			//
		//sprintf(text,"RIGHT: %ld   LEFT: %ld \r", motorRight.Odo.IncTotal , motorLeft.Odo.IncTotal);																
		//sprintf(text,"RIGHT -> dis: %4.3f  vel: %4.3f   LEFT -> dis: %4.3f  vel: %4.3f \r", motorRight.Odo.Dis , motorRight.Odo.Vel , motorLeft.Odo.Dis, motorLeft.Odo.Vel);																


//   return(DISABLE);
   return(CYCLE);
}
