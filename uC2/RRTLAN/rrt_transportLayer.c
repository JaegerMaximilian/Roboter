/***************************************************************

************************************
** FH OBEROESTERREICH CAMPUS WELS **
************************************

Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      TRANSPORTLAYER.c
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

#define RRTLAN_TRANSPORTLAYER_EXTERN

#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "multitask.h"
#include "ports.h"
#include "define.h"
#include "global.h"
#include "rrt_usart_driver.h"
#include "rrt_serialconfig.h"
#include "rrt_transportLayer.h"
#include "rrt_dataLinkLayer.h"

port_app_alloc_t port_alloc[NUM_MESSAGE_STATUS];
uint8_t port_alloc_index;

/**************************************************************************
***   FUNCTIONNAME:        InitPortApp                                  ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void InitPortApp()
{
	uint8_t i;
	
	for(i = 0; i < NUM_MESSAGE_STATUS; i++)
	{
		port_alloc[i].portnbr = 0;
		port_alloc[i].appnbr = 0;
	}
	
	port_alloc_index = 0;
}

/**************************************************************************
***   FUNCTIONNAME:        Port_App_allocation                          ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Port_App_allocation(uint8_t portnbr, uint8_t appnbr)
{
	if(port_alloc_index < NUM_MESSAGE_STATUS)
	{
		port_alloc[port_alloc_index].portnbr = portnbr;
		port_alloc[port_alloc_index].appnbr = appnbr;
		port_alloc_index++;
	}
}

/**************************************************************************
***   FUNCTIONNAME:        Send_Transport_Data                          ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Send_Transport_Data(USART_data_t* usart, uint8_t dest_portnbr, uint8_t* memptr)
{
	//Zielportnummer eintragen
	memptr--;
	*memptr = dest_portnbr;
	
	//Message dem DataLinkLayer übergeben
	Send_DataLink_Data(usart, memptr);
}

/**************************************************************************
***   FUNCTIONNAME:        Receive_tranport_Data                        ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t Receive_transport_Data(USART_data_t* usart)
{
	uint8_t i,j;
	
	//für jeden Index im receive_status Array dieser Schnittstelle...
	for(i = 0; i < NUM_MESSAGE_STATUS; i++)
	{
		//wenn die empfangene Portnummer einem Task zugewiesen ist,...
		if(port_alloc[i].portnbr == usart->memtemp[2])
		{
			//Den Empfangsdatenpointer speichern, wenn dies möglich ist und den Zieltask aktivieren
			for(j = 0; j < NUM_MESSAGE_STATUS; j++)
			{
				if(usart->receive_status[j].freeptr == NULL)
				{
					//Task enable
					SET_TASK(port_alloc[i].appnbr, ENABLE);
					SET_CALLER(port_alloc[i].appnbr, SERIAL_RECEIVE_TASKNBR);
					
					//Daten und Portnummer speichern
					usart->receive_status[j].memptr = usart->memptr;
					usart->receive_status[j].freeptr = usart->freeptr;
					usart->freeptr = NULL;
					usart->receive_status[j].portnbr = usart->memtemp[2];
					
					return(TRUE);
				}
			}
		}
	}
	
	
	cli();
	if(usart->freeptr != NULL)
	{
		free(usart->freeptr);
	}

	sei();
	
	usart->freeptr = NULL;
	
	return(FALSE);
}

