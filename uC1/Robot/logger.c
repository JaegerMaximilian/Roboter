/*
* logger.c
*
* Created: 27.10.2023 08:58:28
*  Author: chri1999
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "logger.h"
#include <avr/io.h>
#include "multitask.h"
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

/* ************************************************************** */
/*! \brief Send Debug Message to Logger
*
*  Function sends given message to Logger on either Robot (Channel) 1 or 2.
*
*/
/* ************************************************************** */
void SendDebugMessage(char *debugText, uint8_t robot){
	char message[LOGGER_MESSAGE_SIZE];
	sprintf(message, "#%dTD%s*", robot, debugText);
	writeString_usart(USART_EUROBOTLOGGER, message);
}

/* ************************************************************** */
/*! \brief Send Trace data to Logger
*
*  Function sends label with values to Logger to display a Chart
*
*/
/* ************************************************************** */
void SendTraceMessage(char *labels[], uint32_t values[], uint8_t length){
	char message[LOGGER_MESSAGE_SIZE];
	char value[10];
	sprintf(message, "#1TG");
	for (uint8_t i = 0; i < length; ++i) {
		sprintf(value, "%d", values[i]);

		strcat(message, labels[i]);
		strcat(message, ",");
		strcat(message, value);
		if (i+1 < length) {
			strcat(message, "|");
		}
	}
	strcat(message, "*");
	writeString_usart(USART_EUROBOTLOGGER, message);
}

/* ************************************************************** */
/*! \brief Sends Oponents to Logger.
*
*  Function Sends Oponent Position, detected by Lidar, to Logger
*
*/
/* ************************************************************** */
void SendLidarMessage(uint8_t uid[], uint16_t radius[], uint16_t angle[], uint8_t length){
	char message[LOGGER_MESSAGE_SIZE];
	char obstacleData[10];
	sprintf(message, "#1TL");
	for (uint8_t i = 0; i < length; ++i) {
		sprintf(obstacleData, "%02d%04d%03d", uid[i], radius[i], angle[i]);
		strcat(message, obstacleData);
		if (i+1 < length) {
			strcat(message, "|");
		}
	}
	strcat(message, "*");
	writeString_usart(USART_EUROBOTLOGGER, message);
}

/* ************************************************************** */
/*! \brief Sends Pathplaner Points to Logger
*
*  Function sends Points of Pathplaner to Logger
*
*/
/* ************************************************************** */
void SendPathPlanerMessage(uint16_t xCoords[],uint16_t yCoords[], uint8_t length){
	char message[LOGGER_MESSAGE_SIZE];
	char path[10];
	sprintf(message, "#1TW");
	for (uint8_t i = 0; i < length; ++i) {
		sprintf(path, "%04d%04d", xCoords[i], yCoords[i]);
		strcat(message, path);
		if (i+1 < length) {
			strcat(message, "|");
		}
	}
	strcat(message, "*");
	writeString_usart(USART_EUROBOTLOGGER, message);
}

/* ************************************************************** */
/*! \brief Sends Position of a Robot
*
*  Function sends uid and Position to the Logger, which is then displayed on the playground
*
*/
/* ************************************************************** */
void SendPlaygroundPositionMessage(uint8_t uid[], uint16_t xCoords[],uint16_t yCoords[], uint8_t length){
	char message[LOGGER_MESSAGE_SIZE];
	char posData[12];
	sprintf(message, "#1TT");
	for (uint8_t i = 0; i < length; ++i) {
		sprintf(posData, "%02d%04d%04d", uid[i], xCoords[i], yCoords[i]);
		strcat(message, posData);
		if (i+1 < length) {
			strcat(message, "|");
		}
	}
	strcat(message, "*");
	writeString_usart(USART_EUROBOTLOGGER, message);
}

/* ************************************************************** */
/*! \brief Send Priority and State of Tasks
*
*  Function sends label with values to Logger to display a Chart
*
*/
/* ************************************************************** */
void SendTaskInfo(uint8_t index[], uint8_t state[], uint8_t priority[])
{
	char message[LOGGER_MESSAGE_SIZE];
	char text[5];
	uint8_t length = sizeof(index);
	sprintf(message, "#1TA");
	if(sizeof(index) == sizeof(state) && sizeof(state) == sizeof(priority))
	{
		for (uint8_t i = 0; i < length; ++i)
		{
			sprintf(text, "%02d%01d%02d", index[i], state[i], priority[i]);
			strcat(message, text);
			if (i+1 < length) {
				strcat(message, "|");
			}
			strcat(message, "*");
			writeString_usart(USART_EUROBOTLOGGER, message);
		}
	}
}