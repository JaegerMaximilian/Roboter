/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Check start device.
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
 *      The file includes the task that waits for start signal and initialize 
 *      the chosen strategy.
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

#define _START_EXTERN


#include <avr/io.h>   
#include "multitask.h"
#include "start.h"
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
#include "StartDevice.h"
 



/* ************************************************************** */
/*! \brief Initialize start-task.
 *
 *  Function initialize the start-task 
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
void InitStart(void)
{
   /* cyclic task - cycle time: 5 ms */
   SET_CYCLE(START_TASKNBR, 5);
   SET_TASK(START_TASKNBR, CYCLE);
   SET_TASK_HANDLE(START_TASKNBR, StartTask); 
}



/* ************************************************************** */
/*! \brief Start-task.
 *
 *  Task that waits for start signal and initialize 
 *  the chosen strategy. 
 *
 *  \version 11.09.2012
 *
 */
/* ************************************************************** */
uint8_t StartTask(void)
{
   uint8_t DIPvalue;
   
   SET_CYCLE(START_TASKNBR, 5);
   
   
   /* check start device
      true ... start robot
	   false ... wait */
   //if(StartDevice_ReadOut() == 1)
   //{
		InitStrategy0(); 
		return (DISABLE);
   //}
   
   //return(CYCLE);
}
