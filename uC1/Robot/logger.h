/*
 * logger.h
 *
 * Created: 27.10.2023 08:56:34
 *  Author: chri1999
 */ 


#ifndef LOGGER_H_
#define LOGGER_H_
#include "usart.h"

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */



/* **************************** */
/* ***      prototypes      *** */
/* **************************** */

#define LOGGER_MESSAGE_SIZE 100
#define LOGGER_MEMBER_ROBOT1 1
#define LOGGER_MEMBER_ROBOT2 2
#define USART_EUROBOTLOGGER &WIFI_IF 


// Variables



// Functions
void SendDebugMessage(char *debugText, uint8_t robot);
void SendTraceMessage(char *labels[], uint32_t values[], uint8_t length);
void SendLidarMessage(uint8_t uid[], uint16_t radius[], uint16_t angle[], uint8_t length);
void SendPathPlanerMessage(uint16_t xCoords[],uint16_t yCoords[], uint8_t length);
void SendPlaygroundPositionMessage(uint8_t uid[], uint16_t xCoords[],uint16_t yCoords[], uint8_t length);


#endif /* LOGGER_H_ */