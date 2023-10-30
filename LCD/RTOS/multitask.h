/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Multitasking system header file.
 *
 *      This file contains the cooperative multitasking system with all components.
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
 * \par Documentation
 *      The file provide functions for the multitasking system. 
 *
 * \author
 *      Michael Zauner
 *      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
 *      Support email: roboracing@fh-wels.at
 *
 * $Revision: 1 $
 * $Date: 2012-08-23  $  \n
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


#ifndef _MULTITASK_H
#define _MULTITASK_H

#include <stdint.h>

/* intern/extern switch */
#ifndef _MULTITASK_EXTERN
   #define _MULTITASK_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* maximum number of tasks */                                      
#define MAX_TASK     30

/* disable task */
#define DISABLE      0x00
/* bit0 indicates that the task is enabled */ 
#define ENABLE       0x01                              
/* bit1 indicates that the task is a cyclic task */  
#define CYCLE        0x02


/* caller adress */
/* system call */
#define SYSTEM_CALL     0xFF
/* cycle call */
#define CYCLE_CALL      0xFE
                                                                          
/*! macro for setting the task */ 
#define SET_TASK(a, b) sMultitasking[a].ucStatus |= b
/*! macro for detecting if the task is enabled */
#define TASK_IS_ENABLED(a) (sMultitasking[a].ucStatus & ENABLE)
/*! macro to disable a task */
#define DISABLE_TASK(a) sMultitasking[a].ucStatus = DISABLE
/*! macro to setting the cycle */
#define SET_CYCLE(a, b) sMultitasking[a].uiIntervall = (b - 1)
#define IS_CYCLE_TASK(a) (sMultitasking[a].ucStatus & CYCLE)
#define IS_DISABLE_TASK(a) (sMultitasking[a].ucStatus & (CYCLE | ENABLE))
/*! macro to set the caller */
#define SET_CALLER(a, b) sMultitasking[a].ucCaller = b 
#define CALLER_IS(a) sMultitasking[a].ucCaller 
/*! macro to set the task handler */
#define SET_TASK_HANDLE(a, b) sMultitasking[a].fpTaskHandle = b
/*! macro to write to the mailbox */ 
#define WRITE_TO_MAILBOX(a, b, c) sMultitasking[a].ucMailBox[b] = c
/*! macro to read from the mailbox */
#define READ_FROM_MAILBOX(a, b) sMultitasking[a].ucMailBox[b]


/* task structure */
struct Multitasking
{
   /*!< state of the task */
   uint8_t ucStatus;
   /*!< interval time (in ms) for cyclic tasks */
   uint16_t uiIntervall;
   /*!< task caller */
   uint8_t ucCaller;
	/*!< mailbox for communication between tasks */ 
   uint8_t ucMailBox[5];
   /*!< task handler -> pointer to function */
   uint8_t (*fpTaskHandle)(void);                     
};

typedef struct Multitasking tsMultitasking; 

_MULTITASK_EXTERN tsMultitasking sMultitasking[MAX_TASK];
_MULTITASK_EXTERN unsigned char errorTask;


/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void InitMultitasking(void);
void MultitaskingSystem(void);


#endif


