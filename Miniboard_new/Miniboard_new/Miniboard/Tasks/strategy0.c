/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
*
* \brief
*      Strategy 0.
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
*      Execute strategy 0 -> .
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

#define _STRATEGY_0_EXTERN


#include <avr/io.h>
#include "multitask.h"
#include "strategy0.h"
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
#include "QTR_1RC.h"

#define LED1_RIGHT		0x01
#define LED2_RIGHT		0x02
#define LED3_RIGHT		0x04
#define LED4_RIGHT		0x08
#define LED5_RIGHT		0x10
#define LED6_RIGHT		0x20

#define LED1_LEFT		0x01
#define LED2_LEFT		0x02
#define LED3_LEFT		0x04
#define LED4_LEFT		0x08
#define LED5_LEFT		0x10
#define LED6_LEFT		0x20

#define FAN_RIGHT		0x04
#define FAN_LEFT		0x08

uint8_t ltMuster1_Right[6] = {LED1_RIGHT,
	LED1_RIGHT + LED2_RIGHT,
	LED1_RIGHT + LED2_RIGHT + LED3_RIGHT,
	LED1_RIGHT + LED2_RIGHT + LED3_RIGHT + LED4_RIGHT,
	LED1_RIGHT + LED2_RIGHT + LED3_RIGHT + LED4_RIGHT + LED5_RIGHT,
LED1_RIGHT + LED2_RIGHT + LED3_RIGHT + LED4_RIGHT + LED5_RIGHT + LED6_RIGHT};

uint8_t ltMuster1_Left[6] = {LED6_RIGHT,
	LED6_RIGHT + LED5_RIGHT,
	LED6_RIGHT + LED5_RIGHT + LED4_RIGHT,
	LED6_RIGHT + LED5_RIGHT + LED4_RIGHT + LED3_RIGHT,
	LED6_RIGHT + LED5_RIGHT + LED4_RIGHT + LED3_RIGHT + LED2_RIGHT,
LED6_RIGHT + LED5_RIGHT + LED4_RIGHT + LED3_RIGHT + LED2_RIGHT + LED1_RIGHT};
uint8_t ltMuster2_Right[6]=	{LED6_LEFT,
	LED6_LEFT + LED5_LEFT,
	LED6_LEFT + LED5_LEFT + LED4_LEFT,
	LED6_LEFT + LED5_LEFT + LED4_LEFT + LED3_LEFT,
	LED6_LEFT + LED5_LEFT + LED4_LEFT + LED3_LEFT + LED2_LEFT,
LED6_LEFT + LED5_LEFT + LED4_LEFT + LED3_LEFT + LED2_LEFT + LED1_LEFT};

uint8_t ltMuster2_LEFT[6]=	{LED1_LEFT,
	LED1_LEFT + LED2_LEFT,
	LED1_LEFT + LED2_LEFT + LED3_LEFT,
	LED1_LEFT + LED2_LEFT + LED3_LEFT + LED4_LEFT,
	LED1_LEFT + LED2_LEFT + LED3_LEFT + LED4_LEFT + LED5_LEFT,
LED1_LEFT + LED2_LEFT + LED3_LEFT + LED4_LEFT + LED5_LEFT + LED6_LEFT};

uint8_t allLEDs=LED1_LEFT + LED2_LEFT + LED3_LEFT + LED4_LEFT + LED5_LEFT + LED6_LEFT;

uint8_t ltMuster3_Right[6]	={LED1_RIGHT,
	LED2_RIGHT,
	LED3_RIGHT,
	LED4_RIGHT,
	LED5_RIGHT,
LED6_RIGHT};
uint8_t ltMuster3_Left[6] = {LED6_RIGHT,
	LED5_RIGHT,
	LED4_RIGHT,
	LED3_RIGHT,
	LED2_RIGHT,
LED1_RIGHT };

uint8_t ltMuster4_Right[6]	={LED6_LEFT,
	LED5_LEFT,
	LED4_LEFT,
	LED3_LEFT,
	LED2_LEFT,
LED1_RIGHT};
uint8_t ltMuster4_Left[6] = {LED1_LEFT,
	LED2_LEFT,
	LED3_LEFT,
	LED4_LEFT,
	LED5_LEFT,
LED6_LEFT };
PORT_t *Led1[6];
PORT_t *Led2[6];


/* ************************************************************** */
/*! \brief Initialize strategy0-task.
*
*  Function initialize the strategy0-task
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
void InitStrategy0(void)
{
	/* cyclic task - cycle time: 5 ms */
	SET_CYCLE(STRATEGY_0_TASKNBR, 50);
	SET_TASK(STRATEGY_0_TASKNBR, CYCLE);
	SET_TASK_HANDLE(STRATEGY_0_TASKNBR, Strategy0Task);
	
	SET_CYCLE(SHAKE_TASKNBR, 200);
	SET_TASK(SHAKE_TASKNBR, CYCLE);
	SET_TASK_HANDLE(SHAKE_TASKNBR, ShakeTask);
	
	
	
	//Led1[0] = &PORTA;
	//Led2[1] = &PORTA;
	//Led2[2] = &PORTA;
	//Led2[3] = &PORTA;
	//Led2[4] = &PORTA;
	//Led2[5] = &PORTA;
	//
	//Led2[0] = &PORTA;
	//Led2[1] = &PORTA;
	//Led2[2] = &PORTB;
	//Led2[3] = &PORTB;
	//Led2[4] = &PORTC;
	//Led2[5] = &PORTC;
}


void setLEDBar_Right(uint8_t led)
{
	if (led & 0x01)
	{
		SET_PIN(PORTA, 0);
	}
	else
	{
		CLR_PIN(PORTA, 0);
	}
	
	if (led & 0x02)
	{
		SET_PIN(PORTA, 1);
	}
	else
	{
		CLR_PIN(PORTA, 1);
	}
	
	if (led & 0x04)
	{
		SET_PIN(PORTA, 2);
	}
	else
	{
		CLR_PIN(PORTA, 2);
	}
	
	if (led & 0x08)
	{
		SET_PIN(PORTA, 3);
	}
	else
	{
		CLR_PIN(PORTA, 3);
	}
	
	if (led & 0x10)
	{
		SET_PIN(PORTA, 4);
	}
	else
	{
		CLR_PIN(PORTA, 4);
	}
	
	if (led & 0x20)
	{
		SET_PIN(PORTA, 5);
	}
	else
	{
		CLR_PIN(PORTA, 5);
	}
	
}

void setLEDBar_Left(uint8_t led)
{
	if (led & 0x01)
	{
		SET_PIN(PORTA, 7);
	}
	else
	{
		CLR_PIN(PORTA, 7);
	}
	
	if (led & 0x02)
	{
		SET_PIN(PORTA, 6);
	}
	else
	{
		CLR_PIN(PORTA, 6);
	}
	
	if (led & 0x20)
	{
		SET_PIN(PORTB, 2);
	}
	else
	{
		CLR_PIN(PORTB, 2);
	}
	
	if (led & 0x10)
	{
		SET_PIN(PORTB, 3);
	}
	else
	{
		CLR_PIN(PORTB, 3);
	}
	
	if (led & 0x04)
	{
		SET_PIN(PORTC, 0);
	}
	else
	{
		CLR_PIN(PORTC, 0);
	}
	
	if (led & 0x08)
	{
		SET_PIN(PORTC, 1);
	}
	else
	{
		CLR_PIN(PORTC, 1);
	}
}

void setHula(uint8_t fan)
{
	if (fan & 0x04)
	{
		SET_PIN(PORTC, 2);
	}
	else
	{
		CLR_PIN(PORTC, 2);
	}
	
	if (fan & 0x08)
	{
		SET_PIN(PORTC, 3);
	}
	else
	{
		CLR_PIN(PORTC, 3);
	}
}




/* ************************************************************** */
/*! \brief Strategy0-task.
*
*  Execute strategy0 -> .
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
uint8_t Strategy0Task(void)
{
	static uint8_t muster = 1;
	static uint8_t enable = 0;
	static uint8_t lengthArray;
	static uint8_t i=0;
	
	SET_CYCLE(STRATEGY_0_TASKNBR, 150);
	
	
	switch(muster)
	{
		case 1:
		lengthArray = sizeof(ltMuster1_Right);
		setLEDBar_Right(ltMuster1_Right[i]);
		setLEDBar_Left(ltMuster1_Left[i]);
		if (i==lengthArray-1)
		{
			muster=2;
		}
		break;
		
		case 2:
		lengthArray = sizeof(ltMuster2_Right);
		setLEDBar_Right(ltMuster2_Right[i]);
		setLEDBar_Left(ltMuster2_LEFT[i]);
		if (i==lengthArray-1)
		{
			muster=3;
		}
		break;
		
		case 3:
		lengthArray = sizeof(ltMuster3_Right);
		setLEDBar_Right(ltMuster3_Right[i]);
		setLEDBar_Left(ltMuster3_Left[i]);
		if (i==lengthArray-1)
		{
			muster=4;
		}
		break;
		
		case 4:
		lengthArray = sizeof(ltMuster4_Right);
		setLEDBar_Right(ltMuster4_Right[i]);
		setLEDBar_Left(ltMuster4_Left[i]);
		if (i==lengthArray-1)
		{
			muster=5;
		}
		break;
		
		case 5:
		lengthArray = 11;
		
		if(enable==0)
		{
			setLEDBar_Right(allLEDs);
			setLEDBar_Left(0x00);

			enable=1;
		}
		else
		{
			setLEDBar_Right(0x00);
			setLEDBar_Left(allLEDs);

			enable=0;
		}
		
		
		if (i==10)
		{
			muster=1;
		}
		
		break;
		
		
	}
	i++;
	
	if(i>=lengthArray)
	{
		i=0;
	}
	

	return(CYCLE);
}


/* ************************************************************** */
/*! \brief Strategy0-task.
*
*  Execute strategy0 -> .
*
*  \version 11.09.2012
*
*/
/* ************************************************************** */
uint8_t ShakeTask(void)
{
	static shakeState = 0;
	
	SET_CYCLE(SHAKE_TASKNBR, 200);
	
	if (shakeState == 0)
	{
		shakeState = 1;
		setHula(0xFF);
	} 
	else
	{
		shakeState = 0;
		setHula(0x00);
	}
	
	
	return(CYCLE);
	
}