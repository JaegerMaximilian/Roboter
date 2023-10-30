/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      DATALINKLAYER.c
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


// ***************************************************************************************************************
// ***************************************************************************************************************
// *****  ÄNDERUNG 20.12.2012 - Michael Zauner - MESSAGE_TYPE_DATA hart setzen -> verhindert Fehler des      *****
// *****                                          Headers wenn der Speicher nicht auf 0 initialisiert ist!!  *****
// ***************************************************************************************************************
// ***************************************************************************************************************


#define RRTLAN_DATALINKLAYER_EXTERN

#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include "rrt_serialconfig.h"
#include "rrt_usart_driver.h"               
#include "rrt_dataLinkLayer.h"
#include "rrt_applicationLayer.h"
#include "rrt_transportLayer.h"
#include "define.h"

crc  crcTable[256];

/**************************************************************************
***   FUNCTIONNAME:        InitTransmissionStatus                       ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void InitTransmissionStatus(USART_data_t* usart)
{
   uint8_t i;
   
   for(i = 0; i < NUM_MESSAGE_STATUS; i++)
   {  
      //Transferstatus initialisieren
      usart->message_status[i].memptr = NULL;
      usart->message_status[i].freeptr = NULL;
      usart->message_status[i].allocated = FALSE;
      usart->message_status[i].nbr_timeouts = FALSE; 
      usart->message_status[i].trial_counter = FALSE; 
      
      //Empfangsstatus initialisieren
      usart->receive_status[i].memptr = NULL;
      usart->receive_status[i].freeptr = NULL;
      usart->receive_status[i].portnbr = 0;
      usart->receive_status[i].timeout_counter = 0;
   }
}

/**************************************************************************
***   FUNCTIONNAME:        Send_DataLink_Data                           ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Send_DataLink_Data(USART_data_t* usart, uint8_t* memptr)
{
   uint8_t temp_nbr_of_bytes;
   uint16_t checksum;
   uint8_t i; 
   
   //SetOutput(PORTF,4);
   
     //Number of Bytes auslesen  
     memptr++;
     temp_nbr_of_bytes = *memptr;
     
     //Unique ID eintragen
     memptr -= 2;
     *memptr = usart->unique_ID; 
     
     //Message_type + trial_counter eintragen  
     usart->message_status[usart->unique_ID].trial_counter = 1;
     memptr--; 
  
  
// *************************************************************************************************************** 
// ***************************************************************************************************************
// *****  ÄNDERUNG 20.12.2012 - Michael Zauner - MESSAGE_TYPE_DATA hart setzen -> verhindert Fehler des      *****
// *****                                          Headers wenn der Speicher nicht auf 0 initialisiert ist!!  *****
// ***************************************************************************************************************
// ***************************************************************************************************************
  //   *memptr |= _MESSAGE_TYPE_DATA;
     *memptr = _MESSAGE_TYPE_DATA;
     *memptr |= usart->message_status[usart->unique_ID].trial_counter;
     
     //Checksum bilden und eintragen  
     checksum = crc_calc(memptr,(unsigned int)temp_nbr_of_bytes + FIRST_BLOCK); 
     memptr += (unsigned int)temp_nbr_of_bytes + FIRST_BLOCK; 
     *memptr = (uint8_t)(checksum >> 8);
     memptr++;
     *memptr = (uint8_t)(checksum);
     memptr -= (unsigned int)temp_nbr_of_bytes + FIRST_BLOCK + 1;
     
     //STX einfügen (ESC + 1)
     memptr -= 2;
     *memptr = ESC;
     usart->message_status[usart->unique_ID].freeptr = memptr;
     memptr++;
     *memptr = ESC + 1;
   
     //ETX einfügen (ESC + 2)
     memptr += (uint8_t)((unsigned int)temp_nbr_of_bytes + 3 + FIRST_BLOCK); 
     *memptr = ESC;
     memptr++;
     *memptr = ESC + 2; 
     
     //Pointer auf erstes zu sendendes Byte stellen
     memptr -= (uint8_t)((unsigned int)temp_nbr_of_bytes + 3 + 2 + FIRST_BLOCK);

     //Sendestatus setzen
     usart->message_status[usart->unique_ID].memptr = memptr;  
     usart->message_status[usart->unique_ID].allocated = TRUE;
     usart->message_status[usart->unique_ID].nbr_timeouts = 0;  
     
     //Daten in Sendefifo schreiben (werden dann im Interrupt aus dem FIFO gesendet)
     i = 0;
     while (i < (uint8_t)((unsigned int)temp_nbr_of_bytes + NBR_HEADER_BYTES))
     {
		uint8_t byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(usart, memptr);  

		if(byteToBuffer)
        {
		  memptr++;
          i++;
		}
     }
     
   //ResetOutput(PORTF,4);
}

/**************************************************************************
***   FUNCTIONNAME:        Receive_DataLink_Data                        ***
***   FUNCTION:            Statemachine für den Empfang                 ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Receive_DataLink_Data(USART_data_t* usart)
{
 uint8_t i;
 uint16_t checksum; 
  
  switch(usart->receivestate)
   {
     ///////////////////////////////////////////////////////////////////////////////////////////// 
     case _FIND_STX_I: //Startzeichen suchen (ESC) 
     ///////////////////////////////////////////////////////////////////////////////////////////// 
       
       //printf("_FIND_STX_I");
       //Erstes Startzeichen suchen
       while((USART_RXBufferData_Available(usart)) && (!usart->stx_received)) 
        { 
		  if(USART_RXBuffer_GetByte(usart) == ESC)
			{ 
              usart->receivestate = _FIND_STX_II;
              usart->stx_received = TRUE;
            }  
		}
        
       if(!usart->stx_received)
        {
          break;
        }
       else
        {
          usart->stx_received = FALSE;
        }
     
     ////////////////////////////////////////////////////////////////////////////////////////////      
     case _FIND_STX_II: //Startzeichen suchen (ESC + 1)
     ////////////////////////////////////////////////////////////////////////////////////////////
     
       //printf("_FIND_STX_II");
       //Zweites Startzeichen suchen
       if(USART_RXBufferData_Available(usart)) 
        {
         if(USART_RXBuffer_GetByte(usart) == (ESC+1))
		  {
            usart->receivestate = _WAIT_FOR_FIRST_BLOCK;  
          }
         else
          { 
            //printf("\rS");
            usart->receivestate = _FIND_STX_I;
            break;
          }       
        }
       else 
        {
          break;
        } 
     
     ////////////////////////////////////////////////////////////////////////////////////////////
     case _WAIT_FOR_FIRST_BLOCK: //Ersten Block der Message einlesen (MT, ID, DP, NoB)
     ////////////////////////////////////////////////////////////////////////////////////////////
       
       //printf("_WAIT_FOR_FIRST_BLOCK");
       //Ersten Block der Message einlesen
       while((USART_RXBufferData_Available(usart)) && (usart->first_block_counter < FIRST_BLOCK))
        {
          usart->memtemp[usart->first_block_counter] = USART_RXBuffer_GetByte(usart);
          usart->first_block_counter++; 
        }
       
       if(usart->first_block_counter < FIRST_BLOCK)
        {
          break;       
        }
       else
        { 
          usart->receivestate = _MEMORY_ALLOC;
          usart->first_block_counter = 0;
        } 
     
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
     case _MEMORY_ALLOC: //Benötigten Speicher für die empfangene Message allokieren
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
       //printf("_MEMORY_ALLOC");  
       //Speicher allokieren
       cli();
       usart->memptr = (uint8_t*)(malloc((uint8_t)((unsigned int)(usart->memtemp[FIRST_BLOCK - 1]) + FIRST_BLOCK + 2))); 
       usart->freeptr = usart->memptr;
       sei();
       
       if(usart->memptr != NULL)
        {   
          usart->receivestate = _RECEIVE_DATA;
          
          for(i = 0; i < FIRST_BLOCK; i++)
           {
            *usart->memptr = usart->memtemp[i];
             usart->memptr++;
           } 
        }
       else
        { 
          printf("MEMFULL");
          usart->receivestate = _FIND_STX_I;
          break;
        }
     
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     case _RECEIVE_DATA: //Nutzdaten und Checksum einlesen
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
       //printf("_RECEIVE_DATA");
       //Nutzdaten und Checksum empfangen
       while((USART_RXBufferData_Available(usart)) && (usart->user_data_counter < (usart->memtemp[FIRST_BLOCK - 1] + 2))) 
        { 
          *usart->memptr = USART_RXBuffer_GetByte(usart); 
          usart->memptr++;
		  usart->user_data_counter++; 
		}
        
       if(usart->user_data_counter < usart->memtemp[FIRST_BLOCK - 1] + 2)
        {
          break;
        }
       else
        { 
          usart->receivestate = _FIND_ETX_I;
          usart->user_data_counter = 0;
        }  
     
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////          
     case _FIND_ETX_I: //Stopzeichen suchen (ESC)
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
       //printf("_FIND_ETX_I");  
       //Erstes Stopzeichen suchen
       if(USART_RXBufferData_Available(usart)) 
        {
		  if(USART_RXBuffer_GetByte(usart) == ESC)
			{
              usart->receivestate = _FIND_ETX_II;
            }
          else
            { 
              //printf("\rE");
              cli();
              free(usart->freeptr);
              sei();
              
              usart->receivestate = _FIND_STX_I;
              break;
            }  
		}
       else
        {
          break;
        }
     
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
     case _FIND_ETX_II: //Stopzeichen (ESC + 2)
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
       //printf("_FIND_ETX_II");
       //zeites Stopzeichen suchen
       if(USART_RXBufferData_Available(usart)) 
        {
         if(USART_RXBuffer_GetByte(usart) == (ESC+2))
		  {
            usart->receivestate = _CHECK_SUM;  
          }
         else
          {
            //printf("\rEI"); 
            cli();
            free(usart->freeptr); 
            sei();
            
            usart->receivestate = _FIND_STX_I;  
            break;
          }       
        }
       else 
        {
          break;
        } 
     
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
     case _CHECK_SUM: //Checksum berechnen und auf Gültigkeit prüfen
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
       //Pointer auf Eintrag MESSAGE_TYPE stellen und die Checksum berechnen
       usart->memptr -= (uint8_t)((unsigned int)(usart->memtemp[FIRST_BLOCK - 1]) + FIRST_BLOCK + 2);        
       checksum = crc_calc(usart->memptr,(unsigned int)(usart->memtemp[FIRST_BLOCK - 1]) + (FIRST_BLOCK) + 2); 
        
       if(checksum == 0)
        {  
          //Wenn die Checksum = 0 ist, dann den Zustand _MESSAGE_TYPE setzten, um die Message auszuwerten.
          //Message gültig
          usart->receivestate = _MESSAGE_TYPE;
        }
       else
        {  
          //printf("\rCS");
          //Wenn die Checksum != 0 ist, dann den Speicher freigeben und den Zustand _FIND_STX_I setzen.
          //Message ungültig 
          cli();
          free(usart->freeptr);
          sei();
          
          usart->receivestate = _FIND_STX_I;
          break;
        }
     
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
     case _MESSAGE_TYPE: //Je nach Art der Message (DATEN oder ACK) reagieren
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                               
        //printf("_MESSAGE_TYPE"); 
        //Pointer auf "Number of Bytes" stellen
        usart->memptr += 3; 
        
        //Messagetype feststellen (Daten oder ACK)                  
        if((usart->memtemp[0] & 0xf8) == _MESSAGE_TYPE_DATA)
         {  
            //ACK Message senden, als response auf gültig empfangene Daten
            //SetOutput(PORTF,5);
            Send_DataLink_ACK(usart);   
            
            //Message an die darüberliegende Ebene übergeben
            Receive_transport_Data(usart);    
         }
        else if((usart->memtemp[0] & 0xf8) == _MESSAGE_TYPE_ACK)
         {  
            //Wenn eine ACK Message empfangen wurde, dann den Sendespeicher freigegeben 
            Check_transmission_status(usart);
         }
        else
         {  
            //Wenn der Messagetype unbekannt ist, dann den allokierten Speicher freigeben
            //Die Senderseite wird wegen des Nichterhaltens des ACK die Message noch einmal senden 
            //printf("\rF");
            cli();
            free(usart->freeptr); 
            sei();
         }
        
        //Statemachine wieder auf "Startzeichen suchen" setzen 
        usart->receivestate = _FIND_STX_I;
  }
}

/**************************************************************************
***   FUNCTIONNAME:        Check_transmission_status                    ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Check_transmission_status(USART_data_t* usart)
{
   //Wenn die ACK Message die richtige trial_counter Anzahl enthält
   if((usart->memtemp[0] & 0x07) == (usart->message_status[usart->memtemp[1]].trial_counter))
    {  
       //und der jeweilige Index belegt ist, dann kann der Speicher freigegeben werden
       if(usart->message_status[usart->memtemp[1]].allocated)
        {  
           //den allokierten Speicher des Sendespeichers freigeben
           cli();
           free(usart->message_status[usart->memtemp[1]].freeptr);
           sei();
           
           usart->message_status[usart->memtemp[1]].allocated = FALSE;
           usart->message_status[usart->memtemp[1]].nbr_timeouts = 0;  
           usart->message_status[usart->memtemp[1]].freeptr = NULL;
        }
    }
    
    //den allokierten Speicher für die ACK Message freigeben
    cli();
    free(usart->freeptr);
    sei();
}

/**************************************************************************
***   FUNCTIONNAME:        Send_DataLink_ACK                            ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void Send_DataLink_ACK(USART_data_t* usart)
{
     uint8_t ACK_array[10]; 
     uint16_t checksum;
     uint8_t i; 
     
     //ACK Message zusammenstellen
     ACK_array[0] = ESC;
     ACK_array[1] = ESC + 1;
     ACK_array[2] = (_MESSAGE_TYPE_ACK | (usart->memtemp[0] & 0x07));
     ACK_array[3] = usart->memtemp[1];
     ACK_array[4] = usart->memtemp[2];
     ACK_array[5] = 0;
     ACK_array[6] = 0;
     ACK_array[7] = 0;
     ACK_array[8] = ESC;
     ACK_array[9] = ESC + 2;
     
     //Checksum bilden und eintragen
     checksum = crc_calc(&ACK_array[2], FIRST_BLOCK); 
     ACK_array[6] = (uint8_t)(checksum >> 8);
     ACK_array[7] = (uint8_t)(checksum);
   
     //Daten in Sendefifo schreiben     
     i = 0;
     while (i < NBR_HEADER_BYTES)
     {
		uint8_t byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(usart, &ACK_array[i]);  
        
		if(byteToBuffer)
        { 
          i++;
		}
     }
     
     //ResetOutput(PORTF,5);
}

/**************************************************************************
***   FUNCTIONNAME:        BuildCRC16                                   ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void BuildCRC16(uint8_t buff_length, uint8_t* memptr)                                        
{   
    uint8_t index;                                                        
    uint8_t counter;                                                    
    uint8_t temp;                                                           
	uint8_t bit_setzen;
    uint8_t crc_high;
    uint8_t crc_low;	
                           
	crc_high = 0;							
	crc_low = 0;										
		
	for (index=0; index <buff_length; index++)			 	        
	{	
        temp = *memptr;						            			
		counter = 8;											  
        
		while (counter)												
		{ 
            --counter;
        												
			if(crc_low & 0x01)										
			{	
              bit_setzen = 1;										
			}													
			else													
			{ 
              bit_setzen = 0;										
			}														
																	
            crc_high >>= 1;
            crc_low >>= 1;											
			crc_high &= 0x7f;									    
			
			if (bit_setzen)											
			{	
               crc_high |= 0x80;									
			}															
			if (temp & 0x01)										
			{ 
               crc_high ^= 0x80;									
			}
            														
			temp >>= 1;												
            
			if (crc_high & 0x80)	
			{	
                crc_high ^= 0x04;					
				crc_low ^= 0x08;						
			}											
		}
        															
		memptr++;											      
	}			
    
    *memptr = crc_high;
     memptr++;
    *memptr = crc_low;	
}

/**************************************************************************
***   FUNCTIONNAME:        crcInit                                      ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
void crcInit(void)
{
    crc  remainder;
    int dividend;
    uint8_t _bit;

    /*
     * Compute the remainder of each possible dividend.
     */
    for (dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (_bit = 8; _bit > 0; --_bit)
        {
            /*
             * Try to divide the current data bit.
             */			
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;
    }

}  

/**************************************************************************
***   FUNCTIONNAME:        crc_calc                                     ***
***   FUNCTION:                                                         ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
crc crc_calc(uint8_t* memptr, unsigned int nBytes)
{
    uint8_t data;
    crc remainder = 0;
    int _byte;

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (_byte = 0; _byte < nBytes; ++_byte)
    {
        data = *memptr ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
        memptr++;
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder);

}   /* crcFast() */

																	


