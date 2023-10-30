/*
 * liftMotor.h
 *
 * Created: 10.12.2019 12:15:19
 *  Author: P20087
 */ 


#ifndef LIFTMOTOR_H_
#define LIFTMOTOR_H_

#ifndef _LIFT_MOTOR_EXTERN
	#define _LIFT_MOTOR_EXTERN extern
#endif



/**************************************************************************
***            Variablen-Definition                                     ***
**************************************************************************/
#define LIFT_REAR_RIGHT_NBR 3
#define LIFT_REAR_LEFT_NBR 2

/**************************************************************************
***            Prototypen-Definition                                    ***
**************************************************************************/
void liftMotor_init(uint8_t pos_enable, float pos, uint8_t mot_nr);
uint8_t liftMotorTask();


#endif /* LIFTMOTOR_H_ */