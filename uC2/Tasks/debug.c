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
#include "motor.h"
#include "cupLift.h"
#include "anaPos.h"
#include "sensor.h"



 
 uint8_t CANRec = 0;
 
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

   
   SET_CYCLE(DEBUG_TASKNBR, 500);

//    LED_PORT.OUT ^= ledBit;
// //   ledBit = (ledBit != 0x40) ? ledBit << 1 : 0x20;
//    ledBit = (ledBit != 0x80) ? ledBit << 1 : 0x20;

// 	if (CHECK_REF)
// 	{
// 		SET_PIN(LED_PORT, LED1);
// 	} 
// 	else
// 	{
// 		CLR_PIN(LED_PORT, LED1);
// 	}

	TOGGLE_PIN(LED_PORT, LED1);
	TOGGLE_PIN(LED_PORT, LED2);
	TOGGLE_PIN(LED_PORT, LED3);
	
	//State = END_K2;
//
  	//sprintf(text1, "#µC2  ES: %d\r\n*", END_K2);
  	//debugMsg(text1);
	  //

	//setVelocity(&liftRear, -0.01);
	//setVelocity(&liftRear, -0.03);


if (State == 0)
{
	State = 1;
	
 //	 	se_arm.Phi = 45.0;
// 	 	se_flag1.Phi = 45.0;
// 	 	se_flag2.Phi = 45.0;

	//SET_PIN(DO_PORT, DO_LED1_NBR);
	//SET_PIN(DO_PORT, DO_LED2_NBR);
	//SET_PIN(DO_PORT, DO_LED3_NBR);
	//SET_PIN(DO_PORT, DO_POLWENDER_NBR);

		//setMotion(&liftRear, 0.05, 0.05);
	//		setVelocity(&liftRear, 0.05);

}
else
{
	State = 0;

 //	 	se_arm.Phi = 120.0;
// 	 	se_flag1.Phi = 90.0;
// 	 	se_flag2.Phi = 90.0;
	//CLR_PIN(DO_PORT, DO_LED1_NBR);
	//CLR_PIN(DO_PORT, DO_LED2_NBR);
	//CLR_PIN(DO_PORT, DO_LED3_NBR);	
	//CLR_PIN(DO_PORT, DO_POLWENDER_NBR);
		//setMotion(&liftRear, 0.05, 0.1);
	//	setVelocity(&liftRear, -0.05);

}

   return(CYCLE);
}
