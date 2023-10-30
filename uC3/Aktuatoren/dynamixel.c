/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Dynamixel Servo driver source file.
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


#define _DYNAMIXEL_EXTERN


#include <avr/io.h> 
#include "usart.h"
#include "dynamixel.h"
#include "multitask.h"
#include "define.h"
#include <util/delay.h>


uint8_t responseEnable;
uint8_t AX_Index, AX_ReadoutOldMsg, AX_respondState;


/* ************************************************************** */
/*! \brief Init Dynamixel servos.
 *
 *  Function initalize the Dynamixel servos
 *
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
void AX_initDynamixel()
{
	// ##########################################################################
	// ##########################################################################
	// ############                  A R M M O D E L                 ############
	// ##########################################################################
	// ##########################################################################  
	
	/* *********************************************************** */
	/* ***           SERVO 0 -> AX18									  *** */
	/* *********************************************************** */
	ax_servo[0].usart = &usartD0;
	ax_servo[0].ID = 1;
	ax_servo[0].port = &PORTD;
	ax_servo[0].pin = 4;
	ax_servo[0].minLimit = 0;
	ax_servo[0].maxLimit = 1023;
	ax_servo[0].maxAngle = M_PI * 150.0 / 180.0;
	ax_servo[0].type = AX_TYPE_18;

	/* *********************************************************** */
	/* ***         SERVO 1 -> AX18									  *** */
	/* *********************************************************** */
	ax_servo[1].usart = &usartD0;
	ax_servo[1].ID = 2;
	ax_servo[1].port = &PORTD;
	ax_servo[1].pin = 4;
	ax_servo[1].minLimit = 0;
	ax_servo[1].maxLimit = 1023;
	ax_servo[1].maxAngle = M_PI * 150.0 / 180.0;
	ax_servo[1].type = AX_TYPE_18;
	
	/* *********************************************************** */
	/* ***         SERVO 2 -> AX18									  *** */
	/* *********************************************************** */
	ax_servo[2].usart = &usartD0;
	ax_servo[2].ID = 3;
	ax_servo[2].port = &PORTD;
	ax_servo[2].pin = 4;
	ax_servo[2].minLimit = 0;
	ax_servo[2].maxLimit = 1023;
	ax_servo[2].maxAngle = M_PI * 150.0 / 180.0;
	ax_servo[2].type = AX_TYPE_18;

	/* *********************************************************** */
	/* ***           SERVO 3 -> AX18               				  *** */
	/* *********************************************************** */
	ax_servo[3].usart = &usartD0;
	ax_servo[3].ID = 4;
	ax_servo[3].port = &PORTD;
	ax_servo[3].pin = 4;
	ax_servo[3].minLimit = 0;
	ax_servo[3].maxLimit = 1023;
	ax_servo[3].maxAngle = M_PI * 150.0 / 180.0;
	ax_servo[3].type = AX_TYPE_18;

	responseEnable = AX_RESPOND_TO_ALL;//AX_RESPOND_NEVER;
	
	/* initialize the serial interface */
	usartd0_init(USART_D0_BUF_SIZE, USART_D0_BUF_SIZE);
}



/* ************************************************************** */
/*! \brief Dynamixel servos send data.
 *
 *  Function send data packet
 *
 *  \param pointer to servo struct.
 *  \param pointer to packet.
 *  \param data length.
 *  \param hasResponse ... (0 ... no response, 1 ... response).
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error  
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_receivePacket(dynamixel_t *servo)
{
	getChar_t data;
	uint8_t ret = 0;
	
	char text[100];
	
	/* ******************** */	
	/* readout old messages */
	/* ******************** */
	if (AX_ReadoutOldMsg)
	{
		do
		{
			data = getChar_uart(servo->usart);
			
			AX_ReadoutOldMsg = 0;
		} while (data.Status != USART_REC_NO_DATA);
	}
	
	/* read out message */
	data = getChar_uart(servo->usart);
	if(data.Status == USART_REC_OK)
	{
		switch (AX_respondState)
		{
			/* *************** */
			/* first MSG_START */
			/* *************** */
			case AX_START_1:
			{
				/* store data */
				AX_Buffer[AX_Index++] = data.Data;
				
  				if(data.Data == AX_MSG_START)
					AX_respondState = AX_START_2;
				else
					return(AX_MESSAGE_ERROR);
				break;
			}
			/* **************** */
			/* second MSG_START */
			/* **************** */
			case AX_START_2:
			{
				/* store data */
				AX_Buffer[AX_Index++] = data.Data;

				if(data.Data == AX_MSG_START)
					AX_respondState = AX_ID;
				else
					return(AX_MESSAGE_ERROR);
				break;
			}
			/* ** */
			/* ID */
			/* ** */
			case AX_ID:
			{
				/* store data */
				AX_Buffer[AX_Index++] = data.Data;

				if(data.Data == servo->ID)
					AX_respondState = AX_LENGTH;
				else
					return(AX_MESSAGE_ERROR);
				break;
			}
			/* ****** */
			/* length */
			/* ****** */
			case AX_LENGTH:
			{
				/* store data */
				AX_Buffer[AX_Index++] = data.Data;

				if(data.Data > 0)
				AX_respondState = AX_ERROR;
				else
				return(AX_MESSAGE_ERROR);
				break;
			}
			/* ***** */
			/* error */
			/* ***** */
			case AX_ERROR:
			{
				/* store data */
				AX_Buffer[AX_Index++] = data.Data;
				servo->error = data.Data;

				if(data.Data == 0)
					AX_respondState = AX_FIRST_PARAMETER;
				else
					return(data.Data);
					
				break;
			}
			/* ********* */
			/* parameter */
			/* ********* */
			default:
			{
				/* store data */
				AX_Buffer[AX_Index++] = data.Data;

				/* readout checksum */
				if((AX_Buffer[3] == (AX_Index - 4)) && (AX_Index >= 4))
				{
					/* message receive complete */
					if(AX_Buffer[AX_Index-1] == AX_calcChecksum(AX_Buffer, AX_Index))
						return(AX_MESSAGE_OK);
					/* checksum error */
					else
						return(AX_CHECKSUM_ERROR);
				}

				break;
			}
			
		}
	}

	return(ret);	
}




/* ************************************************************** */
/*! \brief Dynamixel servos send data.
 *
 *  Function send data packet
 *
 *  \param pointer to servo struct.
 *  \param pointer to packet.
 *  \param data length.
 *  \param hasResponse ... (0 ... no response, 1 ... response).
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error  
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_sendPacket(dynamixel_t *servo, uint8_t *packet, uint8_t length, uint8_t hasResponse)
{
	getChar_t data;
	uint8_t ret = 0;
	
	/* send data packet */
	writePacket_usart(servo->usart, packet, length);	
	
	/* waiting for response */ 
	/* set timeout */
	uint8_t timeout = hasResponse ? 100 : 0;

	/* set up receive message */
	AX_Index = 0;
	AX_ReadoutOldMsg = 1;
	AX_respondState = AX_START_1;
	
	while ((ret == 0) && (timeout > 0)) 
	{
		ret = AX_receivePacket(servo);
		timeout--;
		_delay_us(25);
	}
	
	/* timeout occurred */
	if ((timeout == 0) && (hasResponse == AX_RESPONSE))
	{
		ret = AX_MESSAGE_ERROR + 0x0F;
	}
	
	/* wait before sending the next message */
	_delay_us(100);
	
	return((ret == AX_MESSAGE_OK) ? 0 : ret);	
}





/* ************************************************************** */
/*! \brief Transform angle.
 *
 *  Function transform the angle from -2,618 rad ... +2,618 rad to 0 ... 1023
 *
 *  \param angle [-2,618 rad ... +2,618 rad].
 *
 *  \retval angle [0 ... 1023].
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint16_t AX_transformAngle(float angle, dynamixel_t* servo)
{
	int16_t ret_angle = (int16_t)((float)(servo->maxLimit/2) + (angle * ((float)(servo->maxLimit)/(2.0*servo->maxAngle))));

	/* limit ret_angle to 0 ... 1023 */
	ret_angle =((ret_angle > servo->maxLimit) ? servo->maxLimit : ret_angle); 
	ret_angle =((ret_angle < 0) ? 0 : ret_angle);
	
	return((uint16_t) ret_angle);
}


/* ************************************************************** */
/*! \brief Transform angle.
 *
 *  Function transform the angle from 0 ... 1023 to -2,618 rad ... +2,618 rad 
 *
 *  \param angle [0 ... 1023].
 *
 *  \retval angle [-2,618 rad ... +2,618 rad].
 *
 *  \version 28.06.2016
 */
/* ************************************************************** */
float AX_invers_transformAngle(uint16_t angle, dynamixel_t* servo)
{
	float div = ((servo->type == AX_TYPE_64) ? 651.9 : 195.6);
	float ret_angle = ((float)angle - (servo->maxLimit/2)) / div;

	/* limit ret_angle to -2,618 rad ... +2,618 rad */
	ret_angle =((ret_angle > servo->maxAngle) ? servo->maxAngle : ret_angle); 
	ret_angle =((ret_angle < -(servo->maxAngle)) ? -(servo->maxAngle) : ret_angle);
	
	return(ret_angle);
}


/* ************************************************************** */
/*! \brief Calculate checksum.
 *
 *  Function calculate the checksum
 *
 *  \param pointer to packet.
 *  \param length of the packet.
 *
 *  \retval calculated checksum.
 *
 *  \version 23.05.2013
 */
/* ************************************************************** */
uint8_t AX_calcChecksum(uint8_t *packet, uint8_t n)
{
	uint8_t chksum = 0;	
		
	for (uint8_t i = 2; i < (n-1); i++)
	{
		chksum += packet[i];
	}
	
	return(~chksum);
}



/* ************************************************************** */
/*! \brief Send set PING message.
 *
 *  Function send set PING message
 *
 *  \param pointer to servo struct.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_Ping(dynamixel_t *servo) 
{
	uint8_t len = 6;
	uint8_t packet[6];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_PING;
	packet[5] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);

	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set ID message.
 *
 *  Function send set ID message
 *
 *  \param pointer to servo struct.
 *  \param new ID.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 23.05.2013
 */
/* ************************************************************** */
uint8_t AX_setId(dynamixel_t *servo, uint8_t idNew) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_SERVO_ID;
	packet[6] = idNew;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Baudrate message.
 *
 *  Function send set Baudrate message
 *
 *  \param pointer to servo struct.
 *  \param new baudrate.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setBaudrate(dynamixel_t *servo, uint8_t baudrate) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_BAUD_RATE;
	packet[6] = baudrate;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Return Delay Time message.
 *
 *  Function send set Return Delay Time message
 *
 *  \param pointer to servo struct.
 *  \param delay ... time = 2µs * delay.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setReturnDelayTime(dynamixel_t *servo, uint8_t delay) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_RETURN_DELAY_TIME;
	packet[6] = delay;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Operation Angle Limit message.
 *
 *  Function send set Operation Angle Limit message
 *
 *  \param pointer to servo struct.
 *  \param minAngle [-150.0° ... +150.0°].
 *  \param maxAngle [-150.0° ... +150.0°].
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setOperatingAngleLimit(dynamixel_t *servo, float minAngle, float maxAngle) 
{
	uint8_t len = 11;
	uint8_t packet[11];
	uint16_t min = AX_transformAngle(minAngle, servo);
	uint16_t max = AX_transformAngle(maxAngle, servo);
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_CW_ANGLE_LIMIT_L;
	packet[6] = (uint8_t)(min & 0x00FF);
	packet[7] = (uint8_t)((min >> 8) & 0x00FF);
	packet[8] = (uint8_t)(max & 0x00FF);
	packet[9] = (uint8_t)((max >> 8) & 0x00FF);
	packet[10] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Send set Operation Mode message.
 *
 *  Function send set Operation Mode message
 *
 *  \param pointer to servo struct.
 *  \param mode ... 0 -> Joint Mode (possible to set angle and speed).
 *                  1 -> Wheel Mode (free run mode)
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 04.06.2013
 */
/* ************************************************************** */
uint8_t AX_setOperatingMode(dynamixel_t *servo, uint8_t mode) 
{
	uint8_t len = 11;
	uint8_t packet[11];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_CW_ANGLE_LIMIT_L;
	packet[6] = ((mode != 0) ? 0 : 1);
	packet[7] = 0;
	packet[8] = ((mode != 0) ? 0 : 255);
	packet[9] = ((mode != 0) ? 0 : 3);
	packet[10] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Highest Limit Temperature message.
 *
 *  Function send set Highest Limit Temperature message
 *
 *  \param pointer to servo struct.
 *  \param temperature [°C].
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setTemperatureLimit(dynamixel_t *servo, uint8_t temperature) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_LIMIT_TEMPERATURE;
	packet[6] = temperature;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Voltage Limit message.
 *
 *  Function send set Voltage Limit message
 *
 *  \param pointer to servo struct.
 *  \param minVoltage [9.0 V ... 12.0 V].
 *  \param maxVoltage [9.0 V ... 12.0 V].
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setVoltageLimit(dynamixel_t *servo, float minVoltage, float maxVoltage) 
{
	uint8_t len = 9;
	uint8_t packet[9];
	uint8_t min = (uint8_t)(minVoltage * 10.0);
	uint8_t max = (uint8_t)(maxVoltage * 10.0);
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_LOW_LIMIT_VOLTAGE;
	packet[6] = min;
	packet[7] = max;
	packet[8] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Maximum Torque message.
 *
 *  Function send set Maximum Torque message
 *
 *  \param pointer to servo struct.
 *  \param max torque [0 % ... 100 %].
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setMaximumTorqueEEPROM(dynamixel_t *servo, uint8_t maxTorque) 
{
	uint8_t len = 9;
	uint8_t packet[9];
	uint16_t torque;
	
	torque = (uint16_t)((float)(maxTorque) * 10.23);
	
	/* linit ret_angle to 0 ... 1023 */
	torque = ((torque > 1023) ? 1023 : torque);
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_MAX_TORQUE_L;
	packet[6] = (uint8_t)(torque & 0x00FF);
	packet[7] = (uint8_t)((torque >> 8) & 0x00FF);
	packet[8] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Maximum Torque message.
 *
 *  Function send set Maximum Torque message
 *
 *  \param pointer to servo struct.
 *  \param max torque [0 % ... 100 %].
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setMaximumTorqueRAM(dynamixel_t *servo, uint8_t maxTorque) 
{
	uint8_t len = 9;
	uint8_t packet[9];
	uint16_t torque;
	
	torque = (uint16_t)((float)(maxTorque) * 10.23);
	
	/* limit ret_angle to 0 ... 1023 */
	torque = ((torque > 1023) ? 1023 : torque);
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_TORQUE_LIMIT_L;
	packet[6] = (uint8_t)(torque & 0x00FF);
	packet[7] = (uint8_t)((torque >> 8) & 0x00FF);
	packet[8] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Set Status message.
 *
 *  Function enable/disable the status message
 *
 *  \param pointer to servo struct.
 *  \param 0 ... Do not respond to any instruction 
 *         1 ... Respond only to READ_DATA instruction
 *         2 ... Respond to all instructions
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setStatus(dynamixel_t *servo, uint8_t respond) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	uint8_t ret;
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_RETURN_LEVEL;
	packet[6] = respond;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	ret = (AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
	
	/* save respond status */
	responseEnable = respond;

	return(ret);
}



/* ************************************************************** */
/*! \brief Set Alarm LED message.
 *
 *  Function sets the Alarm LED message
 *
 *  \param pointer to servo struct.
 *  \param alarm ... LED operation at different errors
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setAlarmLED(dynamixel_t *servo, uint8_t alarm) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_ALARM_LED;
	packet[6] = alarm;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Set Alarm Shutdown message.
 *
 *  Function sets the Alarm Shutdown message
 *
 *  \param pointer to servo struct.
 *  \param shutdown ... shutdown the torque at different errors
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setAlarmShutdown(dynamixel_t *servo, uint8_t shutdown) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_ALARM_LED;
	packet[6] = shutdown;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Set Torque Control message.
 *
 *  Function sets the Torque Control message
 *
 *  \param pointer to servo struct.
 *  \param 0 ... Torque Free Run condition (zero torque)
 *         1 ... Torque Enable
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setTorqueControl(dynamixel_t *servo, uint8_t control) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_TORQUE_ENABLE;
	packet[6] = control;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Set LED Control message.
 *
 *  Function sets the LED Control message
 *
 *  \param pointer to servo struct.
 *  \param 0 ... LED turn off
 *         1 ... LED turn on
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setLEDControl(dynamixel_t *servo, uint8_t control) 
{
	uint8_t len = 8;
	uint8_t packet[8];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_LED;
	packet[6] = control;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Send set Compliance Margin and Slope message.
 *
 *  Function send set Compliance Margin and Slope message
 *
 *  \param pointer to servo struct.
 *  \param CW_complianceMargin ... margin = 0.29° * x (e.g. x = 2 -> margin = 0.58°).
 *  \param CCW_complianceMargin ... margin = 0.29° * x (e.g. x = 2 -> margin = 0.58°).
 *  \param CW_complianceSlope ... slope = 0.29° * x (e.g. x = 0x40 -> margin = 18.8°).
 *                                takes effect with discrete steps of 2^n -> 0x01, 0x02, 0x04, ... , 0x80
 *  \param CCW_complianceSlope ... slope = 0.29° * x (e.g. x = 0x40 -> margin = 18.8°).
 *                                 takes effect with discrete steps of 2^n -> 0x01, 0x02, 0x04, ... , 0x80
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setComplianceMarginSlope(dynamixel_t *servo, uint8_t CW_complianceMargin, uint8_t CCW_complianceMargin, uint8_t CW_complianceSlope, uint8_t CCW_complianceSlope) 
{
	uint8_t len = 11;
	uint8_t packet[11];
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_CW_COMPLIANCE_MARGIN;
	packet[6] = CW_complianceMargin;
	packet[7] = CCW_complianceMargin;
	packet[8] = CW_complianceSlope;
	packet[9] = CCW_complianceSlope;
	packet[10] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Send set Position and Speed message.
 *
 *  Function send set Position and Speed message direct, that mean the
 *  message is sent as WRITE_DATA message. So the movement is executed 
 *  immediately after the message 
 *
 *  \param pointer to servo struct.
 *  \param position [-2,618 rad ... +2,618 rad].
 *  \param speed [0 ... 11,86 rad/s]
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setPositionSpeed_direct(dynamixel_t *servo, float position, float speed) 
{
	uint8_t len = 11;
	uint8_t packet[11];
	uint16_t pos = AX_transformAngle(position, servo);
	uint16_t vel;
	
	vel = (uint16_t)(speed * 85.7);
	
	/* limit ret_angle to 0 ... 1023 */
	vel = ((vel > 1023) ? 1023 : vel);
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_GOAL_POSITION_L;
	packet[6] = (uint8_t)(pos & 0x00FF);
	packet[7] = (uint8_t)((pos >> 8) & 0x00FF);
	packet[8] = (uint8_t)(vel & 0x00FF);
	packet[9] = (uint8_t)((vel >> 8) & 0x00FF);
	packet[10] = AX_calcChecksum(packet, len);

	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}


/* ************************************************************** */
/*! \brief Send set Position and Speed message.
 *
 *  Function send set Position and Speed message not direct, that mean the
 *  message is sent as REG_WRITE message. So the movement is executed 
 *  after receiving the ACTION message 
 *
 *  \param pointer to servo struct.
 *  \param position [-2,618 rad ... +2,618 rad].
 *  \param speed [0 ... 11,86 rad/s]
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setPositionSpeed_notDirect(dynamixel_t *servo, float position, float speed) 
{
	uint8_t len = 11;
	uint8_t packet[11];
	uint16_t pos = AX_transformAngle(position, servo);
	uint16_t vel;
	
	vel = (uint16_t)(speed * 85.7);
	
	/* limit ret_angle to 0 ... 1023 */
	vel = ((vel > 1023) ? 1023 : vel);
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_REG_WRITE;
	packet[5] = AX_GOAL_POSITION_L;
	packet[6] = (uint8_t)(pos & 0x00FF);
	packet[7] = (uint8_t)((pos >> 8) & 0x00FF);
	packet[8] = (uint8_t)(vel & 0x00FF);
	packet[9] = (uint8_t)((vel >> 8) & 0x00FF);
	packet[10] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Send set Speed message.
 *
 *  Function send set Speed message direct, that mean the
 *  message is sent as WRITE_DATA message. So the movement is executed 
 *  immediately after the message 
 *
 *  \param pointer to servo struct.
 *  \param speed [0 ... 11.86 rad/s], 1024 ... 2047 -> CW.
 *			  speed [0 ... -11.86 rad/s], 0 ... 1023 -> CCW.			
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 05.06.2013
 */
/* ************************************************************** */
uint8_t AX_setSpeed_direct(dynamixel_t *servo, float speed) 
{
	uint8_t len = 9;
	uint8_t packet[9];
	uint16_t vel;
	
	if (speed < 0.0)
	{
		vel = (uint16_t)(-speed * 85.7);

		/* limit ret_angle to 0 ... 1023 */
		vel = ((vel > 1023) ? 1023 : vel);	
	} 
	else
	{
		vel = (uint16_t)(speed * 85.7);
		vel |= 0x0400;
		
		/* limit ret_angle to 0 ... 2047 */
		vel = ((vel > 2047) ? 2047 : vel);
	}
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_GOAL_SPEED_L;
	packet[6] = (uint8_t)(vel & 0x00FF);
	packet[7] = (uint8_t)((vel >> 8) & 0x00FF);
	packet[8] = AX_calcChecksum(packet, len);

	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Send set Speed message.
 *
 *  Function send set Speed message not direct, that mean the
 *  message is sent as REG_WRITE message. So the movement is executed 
 *  after receiving the ACTION message 
 *
 *  \param pointer to servo struct.
 *  \param speed [1% ... 100%], 0 ... maximum possible speed.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setSpeed_notDirect(dynamixel_t *servo, float speed) 
{
	uint8_t len = 9;
	uint8_t packet[9];
	uint16_t vel;
	
	if (speed < 0.0)
	{
		vel = (uint16_t)(-speed * 85.7);

		/* limit ret_angle to 0 ... 1023 */
		vel = ((vel > 1023) ? 1023 : vel);	
	}
	else
	{
		vel = (uint16_t)(speed * 85.7);
		vel |= 0x0400;
		
		/* limit ret_angle to 0 ... 2047 */
		vel = ((vel > 2047) ? 2047 : vel);
	}
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_REG_WRITE;
	packet[5] = AX_GOAL_POSITION_L;
	packet[6] = (uint8_t)(vel & 0x00FF);
	packet[7] = (uint8_t)((vel >> 8) & 0x00FF);
	packet[8] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Send Action message.
 *
 *  Function send the Action message, this effects all REG_WRITE message
 *  will executed afterwards.
 *
 *  \param pointer to servo struct.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_Action(dynamixel_t *servo) 
{
	uint8_t len = 6;
	uint8_t packet[6];

	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = AX_BROADCAST_ID;
	packet[3] = len - 4;
	packet[4] = AX_ACTION;
	packet[5] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_NO_RESPONSE));
}


/* ************************************************************** */
/*! \brief Send Reset message.
 *
 *  Function send the Reset message, this effects all values are 
 *  set to factory values.
 *
 *  \param pointer to servo struct.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_Reset(dynamixel_t *servo) 
{
	uint8_t len = 6;
	uint8_t packet[6];

	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = AX_BROADCAST_ID;
	packet[3] = len - 4;
	packet[4] = AX_RESET;
	packet[5] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_NO_RESPONSE));
}




/* ************************************************************** */
/*! \brief Send set Punch message.
 *
 *  Function send set Punch message (minimum power for the Dynamixel servo). 
 *
 *  \param pointer to servo struct.
 *  \param max torque [0 % ... 100 %].
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setPunch(dynamixel_t *servo, uint8_t minPunch) 
{
	uint8_t len = 9;
	uint8_t packet[9];
	uint16_t punch;
	
	punch = (uint16_t)((float)(minPunch) * 10.23);
	
	/* limit ret_angle to 0 ... 1023 */
	punch = ((punch > 1023) ? 1023 : punch);
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_TORQUE_LIMIT_L;
	packet[6] = (uint8_t)(punch & 0x00FF);
	packet[7] = (uint8_t)((punch >> 8) & 0x00FF);
	packet[8] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_WRITE(responseEnable));
}



/* ************************************************************** */
/*! \brief Send Read message.
 *
 *  Function send the Read message.
 *
 *  \param pointer to servo struct.
 *  \param reg ... starting address.
 *  \param length ... number of bytes, they should read out
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 24.05.2013
 */
/* ************************************************************** */
uint8_t AX_setReadCommand(dynamixel_t *servo, uint8_t reg, uint8_t length) 
{
	uint8_t len = 8;
	uint8_t packet[8];

	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = len - 4;
	packet[4] = AX_READ_DATA;
	packet[5] = reg;
	packet[6] = length;
	packet[7] = AX_calcChecksum(packet, len);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, len, AX_RESPOND_READ(AX_RESPOND_TO_READ));
}


/* ************************************************************** */
/*! \brief Send set Write message.
 *
 *  Function send set Writemessage
 *
 *  \param pointer to servo struct.
 *  \param reg ... register start-address to write to.
 *  \param value ... pointer to value(s) they have to been written.
 *  \param length ... number of bytes they have to been written.
 *
 *  \retval 0 ... transmission OK.
 *          !0 ... transmission has an error
 *
 *  \version 23.05.2013
 */
/* ************************************************************** */
uint8_t AX_setWriteCommand(dynamixel_t *servo, uint8_t reg, uint8_t *value, uint8_t length) 
{
	uint8_t packet[50];
	uint8_t index = 6;
	
	/* set up message */
	packet[0] = AX_MSG_START;
	packet[1] = AX_MSG_START;
	packet[2] = servo->ID;
	packet[3] = length + 3;
	packet[4] = AX_WRITE_DATA;
	packet[5] = reg;
	for (uint8_t i = 0; i < length; i++)
	{
		packet[index++] = value[i];
	}
	packet[index] = AX_calcChecksum(packet, length + 7);
	
	AX_WRITE(servo);
	return(AX_sendPacket(servo, packet, length + 7, AX_RESPOND_WRITE(responseEnable));
}