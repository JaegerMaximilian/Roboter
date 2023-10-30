/*
 * cupLift.c
 *
 * Created: 10.02.2021 09:57:56
 *  Author: P20087
 */ 

#define _CUPLIFT_EXTERN_

#include "cupLift.h"
#include "sensor.h"
#include "multitask.h"
#include "define.h"
#include "liftMotor.h"
#include "motor.h"


/* ************************************************************** */
/*! \brief initialize function for cup lift initializion.
 *
 *  initialize the cup lift init task.
 *  
*/
/* ************************************************************** */
void InitCupLift()
{
	/* cyclic task - cycle time: 100 ms */
	SET_CYCLE(CUPLIFT_TASKNBR, 100);
	SET_TASK(CUPLIFT_TASKNBR, CYCLE);
	SET_TASK_HANDLE(CUPLIFT_TASKNBR, CupLiftTask);
}



/* ************************************************************** */
/*! \brief cup lift init task.
 *
 *  handle cup lift initializion.
 *
 *  \ret task status.
 *  
*/
/* ************************************************************** */
uint8_t CupLiftTask()
{
	/* state variable */
	static uint8_t cupLiftState = 0;
	
	/* set task cycle to 10 ms */
	SET_CYCLE(CUPLIFT_TASKNBR, 10);

	/* initializion state-machine */
	switch (cupLiftState)
	{
		/* state 1: start reference motion */
		case CL_INIT_STATE_0:
		{
			/* initialize lift motor: velocity controlled, offest position -> 0 */
			liftMotor_init(0,0.0);
			/* start motion: 0.03 m/s downward */
			setVelocity(&liftRear, -0.03);
			/* switch state */
			cupLiftState = CL_INIT_STATE_1;
			break;
		}
		/* state 2: drive lift down until switch hits */
		case CL_INIT_STATE_1:
		{
			/* switch is hit */
			if (CHECK_REF)
			{
				/* stop lift motor */
				setVelocity(&liftRear, 0.0);
				/* switch state */
				cupLiftState = CL_INIT_STATE_2;
				/* set short delay -> 100 ms */
				SET_CYCLE(CUPLIFT_TASKNBR, 100);	
			}
			break;
		}
		/* state 3: drive lift to refernce position */
		case CL_INIT_STATE_2:
		{
			/* initialize lift motor: position controlled, offest position -> 0.02 m */
			liftMotor_init(1, CUPLIFT_POS_REF);
			/* drive lift to refernce position -> 0.075 m */
			setMotion(&liftRear, -0.1, CUPLIFT_POS_0);
			/* disable task -> initializion is finished */
			return(DISABLE);
		}
	}
	
	return(CYCLE);
}
