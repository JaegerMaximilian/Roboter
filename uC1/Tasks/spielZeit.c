/*
* spielZeit.c
*
* Created: 10.12.2019 12:40:48
*  Author: P20087
*/

#define _SPIELZEIT_EXTERN


#include <avr/io.h>   
#include "multitask.h"
#include "define.h" 
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include "global.h"
#include "usart.h"
#include "spielZeit.h"


/* ************************************************************** */
/*! \brief Initialize the motor control.
 *
 *  Motor control initialization function.
 *  Sets up the ports, timers, qdecs, limits, odometry and controllers 
 *
 */
/* ************************************************************** */
void spielZeit_init()
{
   /* cyclic task - cycle time: 10 ms */
   SET_CYCLE(SPIELZEIT_TASKNBR, 100);
   SET_TASK(SPIELZEIT_TASKNBR, CYCLE);
   SET_TASK_HANDLE(SPIELZEIT_TASKNBR, spielZeitTask);
   
   /* set playing time to 100 sec */
   spielzeit_100telSek = 10000; //10000 1sec = 100
}



/* ************************************************************** */
/*! \brief Motor control.
 *
 *  Motor control task.
 *  calculates the corresponding trajectory and motor voltage
 *  task is executed all 10 ms 
 *
 */
/* ************************************************************** */
uint8_t spielZeitTask()
{
	SET_CYCLE(SPIELZEIT_TASKNBR, 10);

	/* update playing time (1 sec - interval) */
	if (spielzeit_100telSek > 0)
	{
		spielzeit_100telSek--;
	}
	spielZeit_10telSek = spielzeit_100telSek/10;
	

	return(CYCLE);
}