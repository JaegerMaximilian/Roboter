/*
* KiWatch.c
*
* Created: 15.11.2023 20:09:49
*  Author: marku
*/

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>

#include "kiWatch.h"
#include "ki.h"
#include "define.h"
#include "global.h"
#include "command.h"
#include "Pfadplanung.h"
#include "multitask.h"
#include "logger.h"

void InitKiWatch(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_TASK(KI_WATCH_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_TASKNBR, 200);
	SET_TASK_HANDLE(KI_WATCH_TASKNBR, KiWatchTask);
}

/**************************************************************************
***   FUNCTIONNAME:        KiWatchTask                                  ***
***   FUNCTION:            überwacht ob Aufgaben bereits erledigt sind  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiWatchTask(void)
{
	// zyklischer Task - Zykluszeit: 200 ms
	SET_CYCLE(KI_WATCH_TASKNBR, 100);
	
	static uint8_t CounterArea1000 = 0;
	static uint8_t CounterArea2000 = 0;
	static uint8_t CounterArea3000 = 0;
	static uint8_t CounterArea4000 = 0;
	static uint8_t CounterArea5000 = 0;
	static uint8_t CounterArea6000 = 0;
	
	static uint8_t InArea1000 = 0;
	static uint8_t InArea2000 = 0;
	static uint8_t InArea3000 = 0;
	static uint8_t InArea4000 = 0;
	static uint8_t InArea5000 = 0;
	static uint8_t InArea6000 = 0;
	
	char text1[200];
	InArea1000 = Path_IsInArea(1250,250,1750,750);
	InArea2000 = Path_IsInArea(750, 450, 1250, 950);
	InArea3000 = Path_IsInArea(750, 1050, 1250, 1550);
	InArea4000 = Path_IsInArea(1250, 1250, 1750, 1750);
	InArea5000 = Path_IsInArea(1750, 1050, 2250, 1550);
	InArea6000 = Path_IsInArea(1750, 450, 2250, 950);
	
	//Position 1000
	if(InArea1000)
	{
		if(CounterArea1000 < MaxCountInArea)
		{
			CounterArea1000++;
		}
		
	}
	else
	{
		if(CounterArea1000 > 0)
		{
			CounterArea1000--;
		}

	}
	
	if(CounterArea1000 >= MaxCountInArea)
	{
		KI_Task[1].Status = DID;
	}
	
	//Position 2000
	if(InArea2000)
	{
		if(CounterArea2000 < MaxCountInArea)
		{
			CounterArea2000++;
		}
		
	}
	else
	{
		if(CounterArea2000 > 0)
		{
			CounterArea2000--;
		}

	}
	
	if(CounterArea2000 >= MaxCountInArea)
	{
		KI_Task[2].Status = DID;
	}
	
	//Position 3000
	if(InArea3000)
	{
		if(CounterArea3000 < MaxCountInArea)
		{
			CounterArea3000++;
		}
		
	}
	else
	{
		if(CounterArea3000 > 0)
		{
			CounterArea3000--;
		}

	}
	
	if(CounterArea3000 >= MaxCountInArea)
	{
		KI_Task[3].Status = DID;
	}
	
	//Position 4000
	if(InArea4000)
	{
		if(CounterArea4000 < MaxCountInArea)
		{
			CounterArea4000++;
		}
		
	}
	else
	{
		if(CounterArea4000 > 0)
		{
			CounterArea4000--;
		}

	}
	
	if(CounterArea4000 >= MaxCountInArea)
	{
		KI_Task[4].Status = DID;
	}
	
	//Position 5000
	if(InArea5000)
	{
		if(CounterArea5000 < MaxCountInArea)
		{
			CounterArea5000++;
		}
		
	}
	else
	{
		if(CounterArea5000 > 0)
		{
			CounterArea5000--;
		}

	}
	if(CounterArea5000 >= MaxCountInArea)
	{
		KI_Task[5].Status = DID;
	}
	
	//Position 6000
	if(InArea6000)
	{
		if(CounterArea6000 < MaxCountInArea)
		{
			CounterArea6000++;
		}
		
	}
	else
	{
		if(CounterArea6000 > 0)
		{
			CounterArea6000--;
		}

	}
	
	if(CounterArea6000 >= MaxCountInArea)
	{
		KI_Task[6].Status = DID;
	}
	
	
	return(CYCLE);
}