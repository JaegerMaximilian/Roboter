/*
 * TimeCount.c
 *
 * Created: 12.08.2024 17:57:54
 *  Author: marku
 */ 
#include <avr/io.h>
#include "rplidar.h"
#include "define.h"
#include "global.h"
#include "multitask.h"
#include "usart.h"
#include <string.h>
#include "timer.h"
#include <stdio.h>
#include <math.h>
#include "obstacle.h"
#include "TimeCount.h"

void TimeCount_Init()
{
	/* cyclic task - cycle time: 2 ms */
	SET_CYCLE(TIMECOUNT_TASKNBR, 2);
	SET_TASK(TIMECOUNT_TASKNBR, CYCLE);
	SET_TASK_HANDLE(TIMECOUNT_TASKNBR, TimeCountTask);
}

uint8_t TimeCountTask()
{
	
	/* set cycle-time to 1 ms */
	
	timeCount++;
	SET_CYCLE(TIMECOUNT_TASKNBR, 2);
	return(CYCLE);
}

