/*
 * cupLift.h
 *
 * Created: 10.02.2021 09:57:56
 *  Author: P20087
 */ 


#ifndef CUPLIFT_H_
#define CUPLIFT_H_

#include <avr/io.h>
#include "anaPos.h"
#include "ports.h"

#ifndef _CUPLIFT_EXTERN_
	#define _CUPLIFT_EXTERN_ extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* states for init-stte-machine */
#define CL_INIT_STATE_0		0
#define CL_INIT_STATE_1		10
#define CL_INIT_STATE_2		20

/* macro to check reference-switch */
#define CHECK_REF (END_K2)

/* position when reference-switch is hit -> 0.00 m */
#define CUPLIFT_POS_REF		0.00
/* refernce position -> 0.12 m */
#define CUPLIFT_POS_0		0.05


/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void InitCupLift();
uint8_t CupLiftTask();



#endif /* CUPLIFT_H_ */