/***************************************************************

************************************
** FH OBEROESTERREICH CAMPUS WELS **
************************************

Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      START.c
Version :  V 1.0
Date    :  01.03.2011
Author  :  ZAUNER MICHAEL

Comments:

Last edit:
Programmchange:

*)....
*).....

Chip type           : Xmega256A3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024

Copyright (c) 2011 by FH-Wels
All Rights Reserved.
****************************************************************/

#define _START_EXTERN


#include <avr/io.h>
#include "multitask.h"
#include "start.h"
#include <ports.h>
#include <util/delay.h>
#include "define.h"
#include "rrt_usart_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "ki.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "usart.h"
#include <avr/eeprom.h>
#include "rrt_serialconfig.h"
#include "spielZeit.h"
#include "command.h"
#include "nextion.h"


/**************************************************************************
***   FUNKTIONNAME: InitStart                                           ***
***   FUNKTION: initialisiert die Überwachung der Startschnur           ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitStart(void)
{
	// zyklischer Task - Zykluszeit: 500 ms
	SET_CYCLE(START_TASKNBR, 50);
	SET_TASK(START_TASKNBR, CYCLE);
	SET_TASK_HANDLE(START_TASKNBR, StartTask);
}

/**************************************************************************
***   FUNCTIONNAME:        StartTask                                    ***
***   FUNCTION:            Überwachung der Startschnur                  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char StartTask(void)
{
	SET_CYCLE(START_TASKNBR, 50);
	SET_TASK(START_TASKNBR, CYCLE);
	
	
	// Startschnur überwachen
	if(START_SCHNUR)
	{
		// Nullposition für die Spielfarbe setzen
		
		if(SpielFarbe == BLUE_L1)			// Spielfeld BLUE L1
		{
			setPosition_RRTLAN(X0_POS_BLUE_L1_ZONE, Y0_POS_BLUE_L1_ZONE, PHI0_BLUE_L1_ZONE);
		}
		else if (SpielFarbe == Yellow_L2) // Spielfeld Yellow L2
		{
			setPosition_RRTLAN(X0_POS_Yellow_L2_ZONE, Y0_POS_Yellow_L2_ZONE, PHI0_Yellow_L2_ZONE);
		}
		else if (SpielFarbe == BLUE_L3)// Spielfeld BLUE L3
		{
			setPosition_RRTLAN(X0_POS_BLUE_L3_ZONE, Y0_POS_BLUE_L3_ZONE, PHI0_BLUE_L3_ZONE);
		}
		else if (SpielFarbe == Yellow_R1)// Spielfeld Yellow R1
		{
			setPosition_RRTLAN(X0_POS_Yellow_R1_ZONE, Y0_POS_Yellow_R1_ZONE, PHI0_Yellow_R1_ZONE);
		}
		else if (SpielFarbe == BLUE_R2)// Spielfeld BLUE R2
		{
			setPosition_RRTLAN(X0_POS_BLUE_R2_ZONE, Y0_POS_BLUE_R2_ZONE, PHI0_BLUE_R2_ZONE);
		}
		else if (SpielFarbe == Yellow_R3)// Spielfeld Yellow R3
		{
			setPosition_RRTLAN(X0_POS_Yellow_R3_ZONE, Y0_POS_Yellow_R3_ZONE, PHI0_Yellow_R3_ZONE);
		}
		
		InitKI();
		
		return(DISABLE);
	}
	// Start play time
	spielZeit_init();


	return(CYCLE);
}
