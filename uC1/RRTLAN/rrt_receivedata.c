/***************************************************************

************************************
** FH OBEROESTERREICH CAMPUS WELS **
************************************

Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      DEBUG.c
Version :  V 1.0
Date    :  28.02.2011
Author  :  MUCKENHUMER BERNHARD

Comments:

Last edit:
Programmchange:

*)....
*).....

Chip type           : ATXmega256a3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024

Copyright (c) 2008 by FH-Wels
All Rights Reserved.
****************************************************************/

#define RRTLAN_RECEIVEDATA_EXTERN

#include <avr/io.h>
#include "multitask.h"
#include "rrt_receivedata.h"
#include "rrt_serialconfig.h"
#include "rrt_applicationLayer.h"
#include "rrt_transportLayer.h"
#include "ports.h"
#include "global.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "usart.h"
#include "sensor.h"
#include "command.h"
#include "wifi.h"
#include "observation.h"
#include "logger.h"

/**************************************************************************
***   FUNKTIONNAME: InitDebug                                           ***
***   FUNKTION: initialisiert die Gegner/Hindernis-Erkennung            ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitReceiveData(void)
{
	Port_App_allocation(DEBUG_MSG_UC2_PORTNBR, DEBUG_MSG_UC2_TASKNBR);
	Port_App_allocation(SENSOR_UC2_PORTNBR, SENSOR_UC2_TASKNBR);
	Port_App_allocation(DEBUG_MSG_GEGNER_PORTNBR, DEBUG_MSG_GEGNER_TASKNBR);
	Port_App_allocation(POS_MSG_GEGNER_PORTNBR, ENEMY_POS_TASKNBR);
	Port_App_allocation(ANTRIEB_UC2_PORTNBR, ANTRIEB_UC2_TASKNBR);
	Port_App_allocation(POS_UC2_PORTNBR, POS_UC2_TASKNBR);
	Port_App_allocation(EINSTELLUNG_LCD_PORTNBR, EINSTELLUNG_LCD_TASKNBR);
	Port_App_allocation(DEBUG_MSG_LCD_PORTNBR, DEBUG_MSG_LCD_TASKNBR);
	// 	Port_App_allocation(ENEMYDATA_TO_PATHPLANER_PORTNBR,ENEMY_LIDAR_TASKNBR);
	// 	Port_App_allocation(ENEMYDATA_RAW_TO_WIFI_PORTNBR,ENEMY_TO_WIFI_TASKNBR);
	// 	if (RobotType_RAM == SLAVE_ROBOT)
	// 	{
	// 		Port_App_allocation(WIFI_MOSI_PORTNBR, WIFI_MOSI_TASKNBR);
	// 		SET_TASK_HANDLE(WIFI_MOSI_TASKNBR, rrtlan_WIFI_Mosi_Task);
	// 	}

	SET_TASK_HANDLE(DEBUG_MSG_UC2_TASKNBR, rrtlanDebugMsg_uC2Task);
	SET_TASK_HANDLE(SENSOR_UC2_TASKNBR, rrtlanSensor_uC2Task);
	SET_TASK_HANDLE(DEBUG_MSG_GEGNER_TASKNBR, rrtlanDebugMsg_GegnerTask);
	SET_TASK_HANDLE(ENEMY_POS_TASKNBR, rrtlanEnemyPos_GegnerTask);
	SET_TASK_HANDLE(ANTRIEB_UC2_TASKNBR, rrtlanAntrieb_uC2Task);
	SET_TASK_HANDLE(POS_UC2_TASKNBR, rrtlanPos_uC2Task);
	SET_TASK_HANDLE(EINSTELLUNG_LCD_TASKNBR, rrtlanEinstellung_LCDTask);
	SET_TASK_HANDLE(DEBUG_MSG_LCD_TASKNBR, rrtlanDebugMsg_LCDTask);
	SET_TASK_HANDLE(ENEMY_LIDAR_TASKNBR, rrtlanEnemyData_to_Pathplaner_Task);
	SET_TASK_HANDLE(ENEMY_TO_WIFI_TASKNBR, rrtlanEnemyDataRaw_to_WIFI_Task);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanDebugMsg_uC2Task                       ***
***   FUNCTION:            receives debug message -> uC2                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanDebugMsg_uC2Task(void)
{
	unsigned char nbr_of_bytes = 0;
	unsigned char receiveArray[200];

	if(Received_AppData_Available(&MCU2, DEBUG_MSG_UC2_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU2, DEBUG_MSG_UC2_PORTNBR, receiveArray);
		
		//writeString_usart(&usartF0, receiveArray);
		writeString_usart(&WIFI_IF, receiveArray);
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanSensor_uC2Task                         ***
***   FUNCTION:            empfängt Daten von uC2 -> Sensordaten        ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanSensor_uC2Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[20];

	if(Received_AppData_Available(&MCU2, SENSOR_UC2_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU2, SENSOR_UC2_PORTNBR, receiveArray);
		
		/* reflex sensors for north/south detection */
		posLiftRear = ((float)(receiveArray[0])) / 1000.0;
	}
	
	return(DISABLE);
}



/**************************************************************************
***   FUNCTIONNAME:        rrtlanSensor_uC3Task                         ***
***   FUNCTION:            empfängt Daten von uC3 -> Sensordaten        ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanEnemyPos_GegnerTask(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[15];
	char text1[100];
	convData_t x[5], y[5];
	uint8_t j = 1;
	
	

	if(Received_AppData_Available(&GEGNER, POS_MSG_GEGNER_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&GEGNER, POS_MSG_GEGNER_PORTNBR, receiveArray);
		
		for (uint8_t i = 0; i <= 5; i++)
		{
			x[i].uint8[0] = 0;
			x[i].uint8[1] = 0;
			
			y[i].uint8[0] = 0;
			y[i].uint8[1] = 0;
			
			enemyRobot[i].Xpos = 10000;
			enemyRobot[i].Ypos = 10000;
	
		}
		
		
		for (uint8_t i = 0; i < receiveArray[0]; i++)
		{
			
			x[i].uint8[0] = receiveArray[j++];
			x[i].uint8[1] = receiveArray[j++];
			
			y[i].uint8[0] = receiveArray[j++];
			y[i].uint8[1] = receiveArray[j++];
			
			enemyRobot[i].Xpos = x[i].uint16[0];
			enemyRobot[i].Ypos = y[i].uint16[0];
		}


		sprintf(text1, "Gegner %d, (%d/%d), (%d/%d), (%d/%d)", receiveArray[0], enemyRobot[0].Xpos, enemyRobot[0].Ypos, enemyRobot[1].Xpos, enemyRobot[1].Ypos, enemyRobot[2].Xpos, enemyRobot[2].Ypos);
		SendDebugMessage(text1, 2);

//		sprintf(text1, "# Gegner1 ;x; %d; y;  %d  \r\n *", x[1].uint16[0], y[1].uint16[0]);
		//writeString_usart(&usartC0, text1);
		//
		//sprintf(text1, "# Gegner2 ;x; %d; y;  %d  \r\n *", x[2].uint16[0], y[2].uint16[0]);
		//writeString_usart(&usartC0, text1);
		//
		//sprintf(text1, "# Gegner3 ;x; %d; y;  %d  \r\n *", x[3].uint16[0], y[3].uint16[0]);
		//writeString_usart(&usartC0, text1);
		
		
		// 		d.uint8[0] = receiveArray[0];
		// 		d.uint8[1] = receiveArray[1];
		// 		rearpressureleft = d.uint16[0];
		//
		// 		d.uint8[0] = receiveArray[2];
		// 		d.uint8[1] = receiveArray[3];
		// 		rearpressureright = d.uint16[0];
		//
		// 		d.uint8[0] = receiveArray[4];
		// 		d.uint8[1] = receiveArray[5];
		// 		d.uint8[2] = receiveArray[6];
		// 		d.uint8[3] = receiveArray[7];
		// 		liftposrearleft = d.f;
		//
		// 		d.uint8[0] = receiveArray[8];
		// 		d.uint8[1] = receiveArray[9];
		// 		d.uint8[2] = receiveArray[10];
		// 		d.uint8[3] = receiveArray[11];
		// 		liftposrearright = d.f;
		//
		// 		d.uint8[0] = receiveArray[12];
		// 		d.uint8[1] = receiveArray[13];
		// 		galh = d.uint16[0];
		//
		// 		d.uint8[0] = receiveArray[14];
		// 		d.uint8[1] = receiveArray[15];
		// 		galv = d.uint16[0];
		
		//sprintf(text1, "%d; %d; %d; %d;\n", usRightRear, usLeftRear, usLeftFront, usRightFront);
		//writeString_usart(&usartF0, text1);
		//	TOGGLE_PIN(LED_PORT, LED1);
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanDebugMsg_uC3Task                       ***
***   FUNCTION:            receives debug message -> uC3                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanDebugMsg_GegnerTask(void)
{
	unsigned char nbr_of_bytes = 0;
	unsigned char receiveArray[200];

	if(Received_AppData_Available(&GEGNER, DEBUG_MSG_GEGNER_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&GEGNER, DEBUG_MSG_GEGNER_PORTNBR, receiveArray);
		
		writeString_usart(&WIFI_IF, receiveArray);
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanAntrieb_uC2Task                        ***
***   FUNCTION:            empfängt Daten von uC2 -> Antriebstatus      ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanAntrieb_uC2Task(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[3];

	if(Received_AppData_Available(&MCU2, ANTRIEB_UC2_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU2, ANTRIEB_UC2_PORTNBR, receiveArray);
		
		statusAntrieb = receiveArray[0];
	}

	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanPos_uC2Task                            ***
***   FUNCTION:            empfängt Daten von uC2 -> Positionsdaten     ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanPos_uC2Task(void)
{
	int16_t x, y, phi, v;
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[10];

	if(Received_AppData_Available(&MCU2, POS_UC2_PORTNBR, &nbr_of_bytes))
	{
		Receive_Application_Data(&MCU2, POS_UC2_PORTNBR, receiveArray);
		
		x = (int16_t)(((uint16_t)(receiveArray[0]) << 8) + (uint16_t)(receiveArray[1]));
		y = (int16_t)(((uint16_t)(receiveArray[2]) << 8) + (uint16_t)(receiveArray[3]));
		phi = (int16_t)(((uint16_t)(receiveArray[4]) << 8) + (uint16_t)(receiveArray[5]));
		v = (int16_t)(((uint16_t)(receiveArray[6]) << 8) + (uint16_t)(receiveArray[7]));
		
		xPos = x / 10;
		yPos = y / 10;
		phiPos = phi;
		vIst = v;
		
		sendPos2EnemyDetection_RRLAN();
	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanDebugMsg_LCDTask                       ***
***   FUNCTION:            receives debug message -> LCD                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanDebugMsg_LCDTask(void)
{
	unsigned char nbr_of_bytes = 0;
	unsigned char receiveArray[200];

// 	if(Received_AppData_Available(&LCD, DEBUG_MSG_LCD_PORTNBR, &nbr_of_bytes))
// 	{
// 		Receive_Application_Data(&LCD, DEBUG_MSG_LCD_PORTNBR, receiveArray);
// 		
// 		writeString_usart(&usartF0, receiveArray);
// 	}
	
	return(DISABLE);
}


/**************************************************************************
***   FUNCTIONNAME:        rrtlanEinstellung_LCDTask                    ***
***   FUNCTION:            empfängt Daten von LCD					    ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlanEinstellung_LCDTask(void)
{
	uint8_t nbr_of_bytes = 0;
	uint8_t receiveArray[6];

// 	if(Received_AppData_Available(&LCD, EINSTELLUNG_LCD_PORTNBR, &nbr_of_bytes))
// 	{
// 		Receive_Application_Data(&LCD, EINSTELLUNG_LCD_PORTNBR, receiveArray);
// 		
// 		SpielFarbe = receiveArray[0];
// 		Strategie = receiveArray[1];
// 	}
	
	return(DISABLE);
}
/**************************************************************************
***   FUNCTIONNAME:        rrtlan_WIFI_Mosi_Task                       ***
***   FUNCTION:            receives debug message -> uC2                ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char rrtlan_WIFI_Mosi_Task(void)
{
	unsigned char nbr_of_bytes = 0;
	unsigned char receiveArray[200];

	// 	if(Received_AppData_Available(&WIFI, WIFI_MOSI_PORTNBR, &nbr_of_bytes))
	// 	{
	// 		Receive_Application_Data(&WIFI, WIFI_MOSI_PORTNBR, receiveArray);
	//
	// 		EndPositionSlave = receiveArray[0];
	// 	}
	
	return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanEnemyData_Task						***
***   FUNCTION:            empfängt Daten von uC3 -> für Gegnererkennung ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t rrtlanEnemyData_to_Pathplaner_Task()
{
	//uint8_t nbr_of_bytes = 0;
	//uint8_t receiveArray[50];
	//uint8_t count;
	//char msg[100];
	//
	//
	//if(Received_AppData_Available(&MCU3, ENEMYDATA_TO_PATHPLANER_PORTNBR, &nbr_of_bytes))
	//{
	//Receive_Application_Data(&MCU3, ENEMYDATA_TO_PATHPLANER_PORTNBR, receiveArray);
	//
	//count = receiveArray[0];
	//
	//for (uint8_t i = 0 ; i < 3; i++)
	//{
	//enemyRobot[i].Xpos= ((uint16_t)receiveArray[1+i*4]) * 256 + ((uint16_t)receiveArray[2+i*4]);
	//enemyRobot[i].Ypos = ((uint16_t)receiveArray[3+i*4]) * 256 + ((uint16_t)receiveArray[4+i*4]);
	//
	//// Robter darf sich nicht selbst als Hindernis hinzufügen
	//if (abs(enemyRobot[i].Xpos - xPos) < 250 && abs(enemyRobot[i].Ypos - yPos) < 250
	//&& enemyRobot[i].Xpos > 100 && enemyRobot[i].Xpos < 2900 && enemyRobot[i].Ypos > 100 && enemyRobot[i].Ypos < 1900)
	//{
	//enemyRobot[i].Xpos = 0;
	//enemyRobot[i].Ypos = 0;
	//}
	//
	//}
	//
	//// 		sprintf(msg, "GEGNER -> %d:%d :: %d:%d :: %d:%d - (%d/%d/%d)\n",
	//// 		enemyRobot[0].Xpos,enemyRobot[0].Ypos,
	//// 		enemyRobot[1].Xpos,enemyRobot[1].Ypos,
	//// 		enemyRobot[2].Xpos,enemyRobot[2].Ypos,
	//// 		xPos,yPos,phiPos/10);
	//// 		writeString_usart(&WIFI_IF, msg);
	//
	//
	//
	//}
	//return(DISABLE);
}

/**************************************************************************
***   FUNCTIONNAME:        rrtlanEnemyDataRaw_to_WIFI_Task				***
***   FUNCTION:            empfängt Daten von uC3 -> für Gegnererkennung ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t rrtlanEnemyDataRaw_to_WIFI_Task()
{
	//uint8_t nbr_of_bytes = 0;
	//uint8_t receiveArray[50];
	//volatile EnemyDataRaw_t enemyRaw;
	//
	//if(Received_AppData_Available(&MCU3, ENEMYDATA_RAW_TO_WIFI_PORTNBR, &nbr_of_bytes))
	//{
	//Receive_Application_Data(&MCU3, ENEMYDATA_RAW_TO_WIFI_PORTNBR, receiveArray);
	//
	//
	//enemyRaw.count = receiveArray[0];
	//
	//for (uint8_t i = 0 ; i < 3; i++)
	//{
	//enemyRaw.x[i] = ((uint16_t)receiveArray[1+i*6]) * 256 + ((uint16_t)receiveArray[2+i*6]);
	//enemyRaw.y[i] = ((uint16_t)receiveArray[3+i*6]) * 256 + ((uint16_t)receiveArray[4+i*6]);
	//
	//enemyRaw.var[i] = ((uint16_t)receiveArray[5+i*6]) * 256 + ((uint16_t)receiveArray[6+i*6]);
	//}
	//}
	//
	//
	//// 		// Sendet die Rohdaten des Lidar(Daten vom uC3) an den anderen Roboter
	//char msg[200];
	//sprintf(msg, "#e%04d%04d%04d%04d%04d%04d%04d%04d%04d*",
	//enemyRaw.x[0],enemyRaw.y[0], enemyRaw.var[0],
	//enemyRaw.x[1],enemyRaw.y[1], enemyRaw.var[1],
	//enemyRaw.x[2],enemyRaw.y[2], enemyRaw.var[2]);
	//
	//
	//
	//writeString_usart(&WIFI_IF, msg);
	//
	//return(DISABLE);
}





