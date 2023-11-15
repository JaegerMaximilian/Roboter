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

void InitKiWatch(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_TASK(KI_WATCH_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_TASKNBR, 100);
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
	// zyklischer Task - Zykluszeit: 100 ms
	SET_CYCLE(KI_WATCH_TASKNBR, 100);


	
	return (CYCLE);
}