/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Dynamixel Servo driver header file.
 *
 *      This file contains the function to deal with the Dynamixel servos
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


#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_


#ifndef _DYNAMIXEL_EXTERN
	#define _DYNAMIXEL_EXTERN extern
#endif


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */

/* number of servos */
#define AX_SERVO_NBR				5

/* response */
#define AX_RESPOND_WRITE(a)	((a == AX_RESPOND_TO_ALL) ? AX_RESPONSE : AX_NO_RESPONSE))
#define AX_RESPOND_READ(a)		(((a == AX_RESPOND_TO_ALL) || (a == AX_RESPOND_TO_READ)) ? AX_RESPONSE : AX_NO_RESPONSE))
#define AX_NO_RESPONSE			0
#define AX_RESPONSE				1

/* message received */
#define AX_MESSAGE_OK			255

/* concert angle-range from (0.0° ... +300.0°) into (-150.0° ... +150.0°) -> this range is needed !!! */
#define AX_CEONVERT_ANGLE(a)		(a - 150.0)


/* EEPROM area */
#define AX_MODEL_NUMBER_L          0
#define AX_MODEL_NUMBER_H          1
#define AX_VERSION                 2
#define AX_SERVO_ID                3
#define AX_BAUD_RATE               4
#define AX_RETURN_DELAY_TIME       5
#define AX_CW_ANGLE_LIMIT_L        6
#define AX_CW_ANGLE_LIMIT_H        7
#define AX_CCW_ANGLE_LIMIT_L       8
#define AX_CCW_ANGLE_LIMIT_H       9
#define AX_LIMIT_TEMPERATURE       11
#define AX_LOW_LIMIT_VOLTAGE       12
#define AX_HIGH_LIMIT_VOLTAGE      13
#define AX_MAX_TORQUE_L            14
#define AX_MAX_TORQUE_H            15
#define AX_RETURN_LEVEL            16
#define AX_ALARM_LED               17
#define AX_ALARM_SHUTDOWN          18
#define AX_DOWN_CALIBRATION_L      20
#define AX_DOWN_CALIBRATION_H      21
#define AX_UP_CALIBRATION_L        22
#define AX_UP_CALIBRATION_H        23
/* RAM area */
#define AX_TORQUE_ENABLE           24
#define AX_LED                     25
#define AX_CW_COMPLIANCE_MARGIN    26
#define AX_CCW_COMPLIANCE_MARGIN   27
#define AX_CW_COMPLIANCE_SLOPE     28
#define AX_CCW_COMPLIANCE_SLOPE    29
#define AX_GOAL_POSITION_L         30
#define AX_GOAL_POSITION_H         31
#define AX_GOAL_SPEED_L            32
#define AX_GOAL_SPEED_H            33
#define AX_TORQUE_LIMIT_L          34
#define AX_TORQUE_LIMIT_H          35
#define AX_PRESENT_POSITION_L      36
#define AX_PRESENT_POSITION_H      37
#define AX_PRESENT_SPEED_L         38
#define AX_PRESENT_SPEED_H         39
#define AX_PRESENT_LOAD_L          40
#define AX_PRESENT_LOAD_H          41
#define AX_PRESENT_VOLTAGE         42
#define AX_PRESENT_TEMPERATURE     43
#define AX_REGISTERED_INSTRUCTION  44
#define AX_MOVING                  46
#define AX_LOCK                    47
#define AX_PUNCH_L                 48
#define AX_PUNCH_H                 49

/* instructions */
#define AX_PING							0x01
#define AX_READ_DATA						0x02
#define AX_WRITE_DATA					0x03
#define AX_REG_WRITE						0x04
#define AX_ACTION							0x05
#define AX_RESET							0x06
#define AX_SYNC_WRITE					0x07

/* message start */
#define AX_MSG_START						0xFF

/* broadcast ID */
#define AX_BROADCAST_ID					0xFE
 
/* message */
#define AX_START_1					0
#define AX_START_2					1
#define AX_ID							2
#define AX_LENGTH						3
#define AX_INSTRUCTION				4
#define AX_ERROR						4
#define AX_FIRST_PARAMETER			5

/* error */
#define AX_MESSAGE_ERROR			0x80
#define AX_INSTRUCTION_ERROR		0x40
#define AX_OVERLOAD_ERROR			0x20
#define AX_CHECKSUM_ERROR			0x10
#define AX_RANGE_ERROR				0x08
#define AX_OVERHEATING_ERROR		0x04
#define AX_ANGLE_LIMIT_ERROR		0x02
#define AX_INPUT_VOLTAGE_ERROR	0x01
#define AX_NO_ERROR					0x00

/* respond */
#define AX_RESPOND_NEVER			0x00
#define AX_RESPOND_TO_READ			0x01
#define AX_RESPOND_TO_ALL			0x02

/* baudrate: baudrate = 2000000/(x+1) */
#define AX_1000_KBAUD				1
#define AX_500_KBAUD					3
#define AX_400_KBAUD					4
#define AX_250_KBAUD					7
#define AX_200_KBAUD					9
#define AX_115_2_KBAUD				16
#define AX_57_6_KBAUD				34
#define AX_19_2_KBAUD				103
#define AX_9_6_KBAUD					207

/* torque control */
#define AX_MODE_TORQUE_FREE_RUN		0
#define AX_MODE_TORQUE_ENABLE			1		

/* motion modes */
#define AX_JOINT_MODE				0
#define AX_WHEEL_MODE				1			


/* read/write-control */
#define AX_WRITE(servo)			(servo->port->OUTSET = (0x01 << servo->pin))
#define AX_READ(servo)			(servo->port->OUTCLR = (0x01 << servo->pin))


/* servo struct */
typedef struct
{
   tsUsart *usart;			/* USART */
	uint8_t ID;					/* servo ID: 0 ... 254 */
	PORT_t *port;				/* read/write PORT */
	uint8_t pin;				/* read/write PIN */
	uint8_t error;				/* actual error-state */
	uint8_t temperature;		/* actual servo temperature */
} dynamixel_t;


_DYNAMIXEL_EXTERN dynamixel_t ax_servo[5];
_DYNAMIXEL_EXTERN uint8_t AX_Buffer[40];


/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void AX_initDynamixel();
void AX_dynamixelTask();
uint16_t AX_transformAngle(float angle);
uint8_t AX_calcChecksum(uint8_t *packet, uint8_t n);
uint8_t AX_Ping(dynamixel_t *servo);
uint8_t AX_setId(dynamixel_t *servo, uint8_t idNew);
uint8_t AX_setBaudrate(dynamixel_t *servo, uint8_t baudrate);
uint8_t AX_setReturnDelayTime(dynamixel_t *servo, uint8_t delay);
uint8_t AX_setOperatingAngleLimit(dynamixel_t *servo, float minAngle, float maxAngle);
uint8_t AX_setOperatingMode(dynamixel_t *servo, uint8_t mode);
uint8_t AX_setTemperatureLimit(dynamixel_t *servo, uint8_t temperature);
uint8_t AX_setVoltageLimit(dynamixel_t *servo, float minVoltage, float maxVoltage);
uint8_t AX_setMaximumTorqueEEPROM(dynamixel_t *servo, uint8_t maxTorque);
uint8_t AX_setMaximumTorqueRAM(dynamixel_t *servo, uint8_t maxTorque);
uint8_t AX_setStatus(dynamixel_t *servo, uint8_t respond);
uint8_t AX_setAlarmLED(dynamixel_t *servo, uint8_t alarm);
uint8_t AX_setAlarmShutdown(dynamixel_t *servo, uint8_t shutdown);
uint8_t AX_setTorqueControl(dynamixel_t *servo, uint8_t control);
uint8_t AX_setLEDControl(dynamixel_t *servo, uint8_t control);
uint8_t AX_setComplianceMarginSlope(dynamixel_t *servo, uint8_t CW_complianceMargin, uint8_t CCW_complianceMargin, uint8_t CW_complianceSlope, uint8_t CCW_complianceSlope);
uint8_t AX_setPositionSpeed_direct(dynamixel_t *servo, float position, uint8_t speed);
uint8_t AX_setPositionSpeed_notDirect(dynamixel_t *servo, float position, uint8_t speed);
uint8_t AX_setSpeed_direct(dynamixel_t *servo, int8_t speed) ;
uint8_t AX_setSpeed_notDirect(dynamixel_t *servo, int8_t speed);
uint8_t AX_Action(dynamixel_t *servo);
uint8_t AX_Reset(dynamixel_t *servo) ;
uint8_t AX_setPunch(dynamixel_t *servo, uint8_t minPunch) ;
uint8_t AX_setReadCommand(dynamixel_t *servo, uint8_t reg, uint8_t length) ;
uint8_t AX_setWriteCommand(dynamixel_t *servo, uint8_t reg, uint8_t *value, uint8_t length);


#endif /* DYNAMIXEL_H_ */