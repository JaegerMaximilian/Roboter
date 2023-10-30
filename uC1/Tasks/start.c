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
		if(SpielFarbe == GREEN_1)			// Spielfeld Grün1
		{
			setPosition_RRTLAN(X0_POS_GREEN1_ZONE, Y0_POS_GREEN1_ZONE, PHI0_GREEN1_ZONE);
		}
		else if (SpielFarbe == GREEN_2)
		{
			setPosition_RRTLAN(X0_POS_GREEN2_ZONE, Y0_POS_GREEN2_ZONE, PHI0_GREEN2_ZONE);
		}
		else if (SpielFarbe == GREEN_3)
		{
			setPosition_RRTLAN(X0_POS_GREEN3_ZONE, Y0_POS_GREEN3_ZONE, PHI0_GREEN3_ZONE);
		}
		else if (SpielFarbe == GREEN_4)
		{
			setPosition_RRTLAN(X0_POS_GREEN4_ZONE, Y0_POS_GREEN4_ZONE, PHI0_GREEN4_ZONE);
		}
		else if (SpielFarbe == GREEN_5)
		{
			setPosition_RRTLAN(X0_POS_GREEN5_ZONE, Y0_POS_GREEN5_ZONE, PHI0_GREEN5_ZONE);
		}
		else if (SpielFarbe == BLUE_1)
		{
			setPosition_RRTLAN(X0_POS_BLUE1_ZONE, Y0_POS_BLUE1_ZONE, PHI0_BLUE1_ZONE);
		}
		else if (SpielFarbe == BLUE_2)
		{
			setPosition_RRTLAN(X0_POS_BLUE2_ZONE, Y0_POS_BLUE2_ZONE, PHI0_BLUE2_ZONE);
		}
		else if (SpielFarbe == BLUE_3)
		{
			setPosition_RRTLAN(X0_POS_BLUE3_ZONE, Y0_POS_BLUE3_ZONE, PHI0_BLUE3_ZONE);
		}
		else if (SpielFarbe == BLUE_4)
		{
			setPosition_RRTLAN(X0_POS_BLUE4_ZONE, Y0_POS_BLUE4_ZONE, PHI0_BLUE4_ZONE);
		}
		else if (SpielFarbe == BLUE_5)
		{
			setPosition_RRTLAN(X0_POS_BLUE5_ZONE, Y0_POS_BLUE5_ZONE, PHI0_BLUE5_ZONE);
		}
		
		InitKI();
		
		
		return(DISABLE);
	}
	// Start play time
	spielZeit_init();


	return(CYCLE);
}
