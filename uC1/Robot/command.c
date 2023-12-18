/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
*
* \brief
*      Includes all robot-commands. The serveral functions split the commands to
*      the different microcontroller. So an unique interface is given to the user.
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
*      The file includes the robot-commands.
*
* \author
*      Michael Zauner
*      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
*      Support email: roboracing@fh-wels.at
*
* $Revision: 1 $
* $Date: 2020-10-27  $  \n
*
* Copyright (c) 2020, RRT (University of Applied Sciences Upper Austria) All rights reserved.
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

#define _COMMAND_EXTERN


#include <avr/io.h>
#include "command.h"
#include "ports.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "usart.h"
#include <stdint.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "adc_driver.h"
#include "accumulator.h"
#include "sensor.h"
#include "anaPos.h"
#include "servo.h"
#include "motor.h"
#include "rrt_receivetask.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "Pfadplanung.h"
#include "PSE541.h"
#include "ki.h"
#include "rrt_transmittingtask.h"
#include "rrt_transmittingtask.h"
#include "observation.h"



/**************************************************************************
***   FUNCTIONNAME:        cmd_SetServo                                 ***
***   FUNCTION:            setzt eine Dynamixl                          ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   nbr ... number of servo                      ***
***                        angle ... angle (0° .. 135°)					***
**************************************************************************/
void cmd_SetServo(uint8_t nbr, int16_t angle)
{
	/* set servo from µC1 */
	if (nbr < SE_MAX_uC1_SERVOS)
	{
		se_array[nbr].Phi = (float)angle;
	}
	/* set servo from µC2 */
	// 	else if (nbr < SE_MAX_uC2_SERVOS)
	// 	{
	// 		setServo_RRTLAN(&MCU2, nbr /*(nbr - SE_uC2_OFFSET)*/, angle);
	// 	}
	/* set servo from µC3 */
	//else if (nbr < SE_MAX_uC3_SERVOS)
	//{
	//setServo_RRTLAN(&MCU3, nbr /*(nbr - SE_uC3_OFFSET)*/, angle);
	//}
}


/**************************************************************************
***   FUNCTIONNAME:        cmd_SetMotorPos                              ***
***   FUNCTION:            set the position of a motor                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   nbr ... number of motor                      ***
***                        vel ... velocity [m/s]						***
***						   pos ... position [m]						    ***
**************************************************************************/
void cmd_SetMotorPos(uint8_t nbr, float vel, float pos)
{
	/* lift front left - µC1 */
	if (nbr == LIFT_FRONT_LEFT_NBR)
	{
		setMotion(&liftVL, vel, pos);
	}
	/* lift front right - µC1 */
	else if (nbr == LIFT_FRONT_RIGHT_NBR)
	{
		setMotion(&liftVR, vel, pos);
	}
	/* lift rear left - µC2 */
	else if (nbr == LIFT_REAR_NBR)
	{
		PositionCommand_RRTLAN(&MCU2, nbr, vel, pos);
	}
	///* lift rear right - µC3 */
	//else if (nbr == LIFT_REAR_RIGHT_NBR)
	//{
	//PositionCommand_RRTLAN(&MCU3, nbr, vel, pos);
	//}
}

/**************************************************************************
***   FUNCTIONNAME:        cmd_SetDigtialOut                            ***
***   FUNCTION:            set the position of a motor                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   nbr ... number of motor                      ***
***                        vel ... velocity [m/s]						***
***						   pos ... position [m]						    ***
**************************************************************************/
void cmd_SetDigitalOut(uint8_t nbr, uint8_t val)
{

	/* lift rear left - µC3 */
	if (nbr == DO_LED1_NBR || nbr == DO_LED2_NBR || nbr == DO_LED3_NBR || nbr == DO_POLWENDER_NBR )
	{
		setDigitalOut_RRTLAN(&MCU2, nbr, val);
	}
	
	///* lift rear right - µC3 */
	//else if (nbr == LIFT_REAR_RIGHT_NBR)
	//{
	//PositionCommand_RRTLAN(&MCU3, nbr, vel, pos);
	//}
}


/**************************************************************************
***   FUNCTIONNAME:        cmd_SetMotorVel                              ***
***   FUNCTION:            set the velocity of a motor                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   nbr ... number of motor                      ***
***                        vel ... velocity [m/s]						***
**************************************************************************/
void cmd_SetMotorVel(uint8_t nbr, float vel)
{
	/* lift front left - µC1 */
	if (nbr == LIFT_FRONT_LEFT_NBR)
	{
		setVelocity(&liftVL, vel);
	}
	/* lift front right - µC1 */
	else if (nbr == LIFT_FRONT_RIGHT_NBR)
	{
		setVelocity(&liftVR, vel);
	}
	///* lift rear left - µC2 */
	else if (nbr == LIFT_REAR_NBR)
	{
		VelocityCommand_RRTLAN(&MCU2, nbr, vel);
	}
	///* lift rear right - µC3 */
	//else if (nbr == LIFT_REAR_RIGHT_NBR)
	//{
	//VelocityCommand_RRTLAN(&MCU3,nbr , vel);
	//}
}


/**************************************************************************
***   FUNCTIONNAME:        cmd_CtrlVacuum                               ***
***   FUNCTION:            control vacuum (on/off)                      ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   nbr ... number of vacuum                     ***
***                        val ... on/off (1/0)							***
**************************************************************************/
void cmd_CtrlVacuum(uint8_t nbr, uint8_t val)
{
	///* vacuum-pump front left - µC1 */
	//if (nbr == VACUUM_FRONT_LEFT_NBR)
	//{
	//if (val == CMD_VACUUM_ON)
	//{
	//SET_VACUUM_PUMP_LEFT;
	//SET_MV_LEFT;
	//}
	//else
	//{
	//CLR_VACUUM_PUMP_LEFT;
	//CLR_MV_LEFT;
	//}
	//}
	///* vacuum-pump front right - µC1 */
	//else if (nbr == VACUUM_FRONT_RIGHT_NBR)
	//{
	//if (val == CMD_VACUUM_ON)
	//{
	//SET_VACUUM_PUMP_RIGHT;
	//SET_MV_RIGHT;
	//}
	//else
	//{
	//CLR_VACUUM_PUMP_RIGHT;
	//CLR_MV_RIGHT;
	//}
	//}
	//else if (nbr == VACUUM_REAR_LEFT_NBR)
	//{
	//if (val == CMD_VACUUM_ON)
	//{
	//setDigitalOut_RRTLAN(&MCU3, VACUUM_REAR_LEFT_NBR, 1);
	//}
	//else
	//{
	//setDigitalOut_RRTLAN(&MCU3, VACUUM_REAR_LEFT_NBR, 0);
	//}
	//}
	//else if (nbr == VACUUM_REAR_RIGHT_NBR)
	//{
	//if (val == CMD_VACUUM_ON)
	//{
	//setDigitalOut_RRTLAN(&MCU3, VACUUM_REAR_RIGHT_NBR, 1);
	//}
	//else
	//{
	//setDigitalOut_RRTLAN(&MCU3, VACUUM_REAR_RIGHT_NBR, 0);
	//}
	//}
	//else if (nbr == MV_REAR_LEFT_NBR)
	//{
	//if (val == CMD_VACUUM_ON)
	//{
	//setDigitalOut_RRTLAN(&MCU3, MV_REAR_LEFT_NBR, 1);
	//}
	//else
	//{
	//setDigitalOut_RRTLAN(&MCU3, MV_REAR_LEFT_NBR, 0);
	//}
	//}
	//else if (nbr == MV_REAR_RIGHT_NBR)
	//{
	//if (val == CMD_VACUUM_ON)
	//{
	//setDigitalOut_RRTLAN(&MCU3, MV_REAR_RIGHT_NBR, 1);
	//}
	//else
	//{
	//setDigitalOut_RRTLAN(&MCU3, MV_REAR_RIGHT_NBR, 0);
	//}
	//}
}


/**************************************************************************
***   FUNCTIONNAME:        cmd_Drive		                            ***
***   FUNCTION:            set a motion					                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vStart .. Anfangsgeschwindigkeit             ***
***                        vEnd ... Endgeschwindigkeit                  ***
***                        vMax ... Maximalgeschwindigkeit [mm/s, °/s]  ***
***                        phiSoll ... Winkel (bei Drehung und          ***
***                                    Kreisbogen)                      ***
***                        rSoll ... Radius (bei Kreisbogen)            ***
***                        xSoll ... X-Position                         ***
***                        ySoll ... Y-Position                         ***
***                        sSoll ... Weg                                ***
***                        type ... Bewegungsart                        ***
***                        gegnerErkennung ... Gegnererkennung ein/aus	***
***                        wP ... Pointer on way points (X, Y)			***
***						   nOP ... Number of points						***
***						   AmaxUs ... Acceleration UpSlope				***
***						   AmaxDs ... Acceleration DownSlope   			***
***																		***
***	  EXAMPLES															***
***		- drive to an absolute point (x,y) with a speed (v):			***
***       cmd_Drive(0,0,v,0,0,x,y,0,POS_ABS,ON,NULL,NULL)				***
***		- drive a relative way (s) with a speed (v):					***
***       cmd_Drive(0,0,v,0,0,0,0,s,POS_REL,ON,NULL,NULL)				***
***		- drive to an absolute angle (phi) with a speed (w):			***
***       cmd_Drive(0,0,w,phi,0,0,0,0,TURN_ABS,ON,NULL,NULL)			***
***		- drive a relative angle (phi) with a speed (w):				***
***       cmd_Drive(0,0,v,phi,0,0,0,s,TURN_REL,ON,NULL,NULL)			***
***		- drive an arc with radius (r) and an angle (phi)				***
***       with a speed (v):												***
***       cmd_Drive(0,0,v,phi,r,0,0,0,ARC,ON,NULL,NULL)					***
***		- drive a way allong the points (p1 ... pn -> wP) with a		***
***       speed (v) and with a clothoid between the straigt parts:		***
***       cmd_Drive(0,0,v,0,0,0,0,0,NULL,ON,&wP,n)						***
***		- interrupt a motion:											***
***       cmd_Drive(0,0,0,0,0,0,0,0,MOTION_INTERRUPT,0,NULL,NULL)		***
**************************************************************************/
void cmd_Drive(int16_t vStart,
int16_t vEnd,
int16_t vMax,
int16_t phiSoll,
uint16_t rSoll,
uint16_t xSoll,
uint16_t ySoll,
uint16_t sSoll,
uint8_t type,
uint8_t gE,
element_t *wP,
uint8_t nOP,
uint8_t AmaxUs,
uint8_t AmaxDs)
{
	gegnerErkennung = gE;
	
	/* start observation */
	observationStarted = OBSERVATION_STARTED;
	observationResult = OBSERVATION_RUNNING;
	
	/* set start-point */
	observationStartPos.Xpos = xPos;
	observationStartPos.Ypos = yPos;
	
	/* set goal-point */
	/* POS_ABS: xSoll/ySoll */
	if (type == POS_ABS)
	{
		observationGoalPos.Xpos = xSoll;
		observationGoalPos.Ypos = ySoll;
	}
	/* POS_REL: xPos + cos(phiPos)/xPos + sin(phiPos) */
	else if (type == POS_REL)
	{
		observationGoalPos.Xpos = xPos + (int16_t)((float)sSoll * cos((float)phiPos * M_PI / 1800.0));
		observationGoalPos.Ypos = yPos + (int16_t)((float)sSoll * sin((float)phiPos * M_PI / 1800.0));
	}
	/* any rotation or motion-interrupt: xPos/yPos */
	else if ((type == TURN_REL) || (type == TURN_ABS) || (type == MOTION_INTERRUPT))
	{
		observationGoalPos.Xpos = xPos;
		observationGoalPos.Ypos = yPos;
	}
	/* ARC: xPos + 2*rSoll*sin(PhiSoll)*cos(phiSoll+phiPos)/xPos + 2*rSoll*sin(PhiSoll)*sin(phiSoll+phiPos) */
	else if(type == ARC)
	{
		observationGoalPos.Xpos = xPos + (int16_t)((float)rSoll * 2.0 * sin(((float)(phiSoll) * M_PI / 1800.0)) * cos((float)(phiPos + phiSoll) * M_PI / 1800.0));
		observationGoalPos.Ypos = yPos + (int16_t)((float)rSoll * 2.0 * sin(((float)(phiSoll) * M_PI / 1800.0)) * sin((float)(phiPos + phiSoll) * M_PI / 1800.0));
	}
	setACCAntrieb_RRTLAN(AmaxUs,AmaxDs);
	if (nOP == 0)
	{
		setAntrieb_RRTLAN(vStart, vEnd,vMax, phiSoll, rSoll, xSoll, ySoll, sSoll, type, gE);
	}
	else
	{
		/* for more then one driving-points: last point in list */
		observationGoalPos.Xpos = wP[nOP-1].Xpos;
		observationGoalPos.Ypos = wP[nOP-1].Ypos;
		
		setAntriebClothoid_RRTLAN(vMax, wP, nOP);
	}
}


/**************************************************************************
***   FUNCTIONNAME:        cmd_DriveToPoint_ABS                         ***
***   FUNCTION:            set a motion to a point (absolute)           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vStart .. Anfangsgeschwindigkeit             ***
***                        vEnd ... Endgeschwindigkeit                  ***
***                        vMax ... Maximalgeschwindigkeit              ***
***                        xSoll ... X-Position                         ***
***                        ySoll ... Y-Position                         ***
**************************************************************************/
void cmd_DriveToPoint_ABS(int16_t vStart, int16_t vEnd, int16_t vMax, uint16_t xSoll, uint16_t ySoll, uint8_t gegnerErkennung)
{
	cmd_Drive(vStart, vEnd,vMax, 0, 0, xSoll, ySoll, 0, POS_ABS, gegnerErkennung, NULL, 0,STANDARD_ACC,PLANT_ACC);
}

/**************************************************************************
***   FUNCTIONNAME:        cmd_DriveToPos_REL	                        ***
***   FUNCTION:            set a motion					                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vStart .. Anfangsgeschwindigkeit             ***
***                        vEnd ... Endgeschwindigkeit                  ***
***                        vMax ... Maximalgeschwindigkeit              ***
***                        sSoll ... Weg                                ***
***                        type ... Bewegungsart                        ***
***                        gegnerErkennung ... Gegnererkennung ein/aus	***
**************************************************************************/
void cmd_DriveToPos_REL(int16_t vStart, int16_t vEnd, int16_t vMax, uint16_t sSoll, uint8_t gegnerErkennung)
{
	cmd_Drive(vStart, vEnd,vMax, 0, 0, 0, 0, sSoll, POS_REL, gegnerErkennung, NULL, 0,100,100);
}

/**************************************************************************
***   FUNCTIONNAME:        cmd_TurnAroundAngle_ABS                       ***
***   FUNCTION:            set a motion					                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vStart .. Anfangsgeschwindigkeit             ***
***                        vEnd ... Endgeschwindigkeit                  ***
***                        vMax ... Maximalgeschwindigkeit              ***
***                        phiSoll ... Winkel							***
***                        gegnerErkennung ... Gegnererkennung ein/aus	***
**************************************************************************/
void cmd_TurnAroundAngle_ABS(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint8_t gegnerErkennung)
{
	cmd_Drive(vStart, vEnd,vMax, phiSoll, 0, 0, 0, 0, TURN_ABS, gegnerErkennung, NULL, 0,100,100);
}

/**************************************************************************
***   FUNCTIONNAME:        cmd_TurnAroundAngle_REL                      ***
***   FUNCTION:            set a motion					                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vStart .. Anfangsgeschwindigkeit             ***
***                        vEnd ... Endgeschwindigkeit                  ***
***                        vMax ... Maximalgeschwindigkeit              ***
***                        phiSoll ... Winkel (bei Drehung und          ***
***                                    Kreisbogen)                      ***
**************************************************************************/
void cmd_TurnAroundAngle_REL(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint8_t gegnerErkennung)
{
	cmd_Drive(vStart, vEnd,vMax, phiSoll, 0, 0, 0, 0, TURN_REL, gegnerErkennung, NULL, 0,100,100);
}

/**************************************************************************
***   FUNCTIONNAME:        cmd_DriveArc		                            ***
***   FUNCTION:            set a motion					                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vStart .. Anfangsgeschwindigkeit             ***
***                        vEnd ... Endgeschwindigkeit                  ***
***                        vMax ... Maximalgeschwindigkeit              ***
***                        phiSoll ... Winkel (bei Drehung und          ***
***                                    Kreisbogen)                      ***
***                        rSoll ... Radius (bei Kreisbogen)            ***
***                        gegnerErkennung ... Gegnererkennung ein/aus	***
**************************************************************************/
void cmd_DriveArc(int16_t vStart, int16_t vEnd, int16_t vMax, int16_t phiSoll, uint16_t rSoll, uint8_t gegnerErkennung)
{
	cmd_Drive(vStart, vEnd,vMax, phiSoll, rSoll, 0, 0, 0, ARC, gegnerErkennung, NULL, 0,100,100);
}

/**************************************************************************
***   FUNCTIONNAME:        cmd_DrivePath	                            ***
***   FUNCTION:            set a motion					                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   vMax ... Maximalgeschwindigkeit              ***
***                        gegnerErkennung ... Gegnererkennung ein/aus	***
***                        wP ... Pointer on way points (X, Y)			***
***						   nOP ... Number of points						***
**************************************************************************/
void cmd_DrivePath(int16_t vMax,uint8_t gegnerErkennung, point_t *wP, uint8_t nOP)
{
	cmd_Drive(0, 0, vMax, 0, 0, 0, 0, 0, 0, gegnerErkennung, wP, nOP,100,100);
}

