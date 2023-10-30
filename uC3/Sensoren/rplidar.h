/*
 * rplidar.h
 *
 * Created: 05.02.2019 07:33:15
 *  Author: P20087
 */ 


#ifndef RPLIDAR_H_
#define RPLIDAR_H_

#include <avr/io.h>
#include "usart.h"
#include "timer.h"

#ifndef _RPLIDAR_EXTERN
	#define _RPLIDAR_EXTERN extern
#endif


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* Requests */
/* **************************** */
/* 1. requests with no response */
/* **************************** */
/* Exit the current state and enter the idle state */
#define RPLIDAR_STOP			0x25
/* Reset(reboot) the RPLIDAR core */
#define RPLIDAR_RESET			0x40
/* ********************************** */
/* 2. requests with multiple response */
/* ********************************** */
/* Enter the scanning state */
#define RPLIDAR_SCAN			0x20
/* Enter the scanning state and working at the highest speed */
#define RPLIDAR_EXPRESS_SCAN	0x82
/* Enter the scanning state and force data output without checking rotation speed */
#define RPLIDAR_FORCE_SCAN		0x21
/* ******************************** */
/* 3. requests with single response */
/* ******************************** */
/* Send out the device info (e.g. serial number) */
#define RPLIDAR_GET_INFO		0x50
/* Send out the device health info */
#define RPLIDAR_GET_HEALTH		0x52
/* Send out single sampling time */
#define RPLIDAR_GET_SAMPLERATE	0x59

/* ******************** */
/* transmission control */
/* ******************** */
/* STX request packet */
#define RPLIDAR_STX1			0xA5
/* additional STX for response packet */
#define RPLIDAR_STX2			0x5A

/* *********************************** */
/* RPLIDAR data response packets value */
/* *********************************** */
/* Single Request - Single Response mode, RPLIDAR will send only one data
   response packet in the current session. */
#define RPLIDAR_SINGLE_REQUEST		(0x00 << 6)
/* Single Request - Multiple Response mode, RPLIDAR will continuously send
   out data response packets with the same format in the current session. */
#define RPLIDAR_MULTIPLE_REQUEST	(0x01 << 6)

/* ****** */
/* states */
/* ****** */
/* idle state -> ready to send a request */
#define RPLIDAR_IDLE			0

/* startup state -> check the health of the LIDAR  */
#define RPLIDAR_CHECK_HEALTH_STATE		0
/* start scan state -> start scanning  */
#define RPLIDAR_START_SCAN_STATE		1
/* reset state -> reset LIDAR */
#define RPLIDAR_RESET_STATE				2

/* ************* */
/* health status */
/* ************* */
#define RPLIDAR_HEALTH_GOOD		0
#define RPLIDAR_HEALTH_WARNING	1
#define RPLIDAR_HEALTH_ERROR	2


/*!< Info structure */
typedef struct  
{
	/*!< The model ID of the RPLIDAR being used */
	uint8_t model;	
	/*!< Firmware version number */
	uint16_t firmware;
	/*!< Hardware version number */
	uint8_t hardware;
	/*!< 128bit unique serial number */
	uint8_t serialnumber[16];
} rpLidar_Info_t;

/*!< Health structure */
typedef struct  
{
	/*!< RPLIDAR health state (0 ... Good, 1 ... Warning, 2 ... Error) */
	uint8_t status;	
	/*!< The related error code that caused a warning/error. */
	uint16_t error_code;
} rpLidar_Health_t;

/*!< Sample Rate structure */
typedef struct  
{
	/*!< In scan(SCAN) mode, the time used when RPLIDAR takes a single laser
		 ranging Unit: microsecond (µs) */
	uint16_t Tstandard;	
	/*!< In express scan(EXPRESS_SCAN) mode, the time used when RPLIDAR
		 takes a single laser ranging Unit: microsecond(µs) */
	uint16_t Texpress;
} rpLidar_SampleRate_t;

/*!< Scan structure */
typedef struct  
{
	/*!< Quality of the current measurement sample */
	uint8_t quality;	
	/*!< The measurement heading angle 	related to RPLIDAR’s heading. In degree unit, 
	     [0-360[ Stored using fix point number. */
	float angle;
	/*!< Measured object distance related to RPLIDAR’s rotation center. In millimeter (mm) 
	     unit. Represents using fix point. Set to 0 when the measurement is invalid. */
	float distance;
} rpLidar_Scan_t;


/*!< RPLIDAR structure */
typedef struct  
{
	/*!< mode of the communication */
	uint8_t mode;
	/*!< request type */
	uint8_t request;
	/*!< mode of operation */
	uint8_t state;
	/*!< index to read out data */
	uint8_t msg_index;
	/*!< receive buffer */
	uint8_t recBuf[100];
	/*!< pointer to the USART */
	tsUsart *usart;
	
	/*!< Field Definition of Device Info Data Response Packet */
	rpLidar_Info_t info;
	
	/*!< Field Definition of Device Health Status Data Response Packet */
	rpLidar_Health_t health;
	
	/*!< Field Definition of Sample Rate Data Response Packet */
	rpLidar_SampleRate_t sampleRate;
	
	/*!< Field Definition of a RPLIDAR Measurement Result Data Response Packet */
	rpLidar_Scan_t scan;
	uint8_t scan_active;
	
	/*!< pointer to timer for MOTOCTRL */	
	TC0_t *qTimer0;
	TC1_t *qTimer1;
	uint8_t timer;
	/*!< PWM channel for MOTOCTRL */
	uint8_t channel;		
	
} rplidar_t;

_RPLIDAR_EXTERN rplidar_t rpLidar;

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void rpLidar_Init();
uint8_t rpLidar_calcChecksum(uint8_t *msg, uint8_t n);
void rpLidar_setPWM(rplidar_t *lidar, uint8_t pwm);
void rpLidar_sendRequest(rplidar_t *lidar, uint8_t request);
uint8_t rpLidar_MsgHandlerTask();
uint8_t rpLidar_Task();


#endif /* RPLIDAR_H_ */