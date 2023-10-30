/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      SERIALCONFIG.H
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

//////////////////////////////////////////////////////////////////
//****************************************************************
//ACHTUNG: Bei Verwendung des seriellen Protokolls in Verbindung mit
//         CodeVision muss eine Heap size in der Projekkonfiguration
//         freigegeben werden. Bei AVRStudio ist dies nicht notwendig
//****************************************************************
////////////////////////////////////////////////////////////////// 

/* Die konfiguration der Schnittstellen erfolgt im file serialconfig.h. Die jeweiligen Defines(siehe weiter unten)
   müssen eingefügt werden. Die Funktion InitUSARTS() muss ausgeführt werden, um alle definierten Schnittstellen
   zu initialisieren. Die Zuweisung der Portnummern zu den Tasks erfolgt über die Funtkion
   port_App_allocation(portnbr, tasknbr). Dies ist erforderlich, um durch die Zielportnummer den zugewiesenen Task
   zu aktivieren.
   
   Werden gültige Daten empfangen, die eine nicht zugewiesene Zielportnummer enthalten, wird der 
   für diese Message allokierte Speicher nicht mehr freigegeben. Das führt zu einem vollen heap im Speicher und eine
   weitere Kommunikation ist nicht mehr möglich!!! 
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Data Message
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -----------------------------------------------------------------------------
// | ESC | ESC+1 | MT | ID | DP | NoB | DATA | CRC_high | CRC_low | ESC | ESC+2 |
// -----------------------------------------------------------------------------
// ESC = 27
// MT = Messagetype (DATA, ACK)
// ID = unique Messagenumber
// DP = destination port of receivertask
// NoB = Nomber of bytes of the payload
// DATA = NoB * Data_bytes (payload)
// CRC_high = high byte of CRC16
// CRC_low = low byte of CRC16

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ACK Message
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------
//|ESC | ESC+1 | MT | ID | DP | NoB | CRC_high | CRC_low | ESC | ESC+2 |
//---------------------------------------------------------------------
// NoB = 0

 
#ifndef RRTLAN_SERIALCONFIG_H
#define RRTLAN_SERIALCONFIG_H

#include <avr/io.h>
#include "rrt_usart_driver.h"
#include <stdio.h>


#ifndef RRTLAN_SERIALCONFIG_EXTERN
   #define RRTLAN_SERIALCONFIG_EXTERN extern
#endif

//#define _SERIAL_INTERFACE_PC_C0
//#define BAUDRATE_PC_C0 57600
//#define _SERIAL_INTERFACE_PC_D0
//#define BAUDRATE_PC_D0
//#define _SERIAL_INTERFACE_PC_E0
//#define BAUDRATE_PC_E0 57600
//////////////////////////////////////////////////////////////////////////////////////////
//Schnittstellen auf die das Protokoll angewendet werden soll
//////////////////////////////////////////////////////////////////////////////////////////
//#define _SERIAL_INTERFACE_C0
//#define BAUDRATEC0 9600
//#define _SERIAL_INTERFACE_C1
//#define BAUDRATEC1 9600
//#define _SERIAL_INTERFACE_D0
//#define BAUDRATED0 230400
//#define _SERIAL_INTERFACE_D1
//#define BAUDRATED1 230400
//#define _SERIAL_INTERFACE_E0
//#define BAUDRATEE0 9600
//#define _SERAIL_INTERFACE_E1
//#define BAUDRATEE1 9600

///////////////////////////////////////////////////////////////////////////////////////////
//Main MCU 1 definitions
///////////////////////////////////////////////////////////////////////////////////////////
//#define _SERIAL_INTERFACE_D1
//#define BAUDRATED1 230400
//#define _SERIAL_INTERFACE_E1
//#define BAUDRATEE1 230400

///////////////////////////////////////////////////////////////////////////////////////////
//PC, Camera definitions
///////////////////////////////////////////////////////////////////////////////////////////
//#define _SERIAL_INTERFACE_PC_D0
//#define BAUDRATE_PC_D0 57600
//#define _SERIAL_INTERFACE_PC_E0
//#define BAUDRATE_PC_E0 57600

///////////////////////////////////////////////////////////////////////////////////////////
//MCU 2 definitions
///////////////////////////////////////////////////////////////////////////////////////////
 #define _SERIAL_INTERFACE_E0		// LCD -> µC1
 #define BAUDRATEE0 230400
// #define _SERIAL_INTERFACE_E1		// µC2 -> µC3
// #define BAUDRATEE1 230400



///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//defines 
///////////////////////////////////////////////////////////////////////////////////////////
#define F_MCU                 32000000L   //Taktfrequenz
#define ESC                   27          //ESC Zeichen
#define PC_INTERFACE          1   
#define PROTOKOLL_INTERFACE   0
#define NBR_TRIALS            3           //Anzahl der Versuche für das Senden 
                                          //einer Nachricht bei Timeout
#define NUM_MESSAGES_PER_INTERFACE 3      //Anzahl der Messages, die pro Interface und Durchgang 
                                          //ausgelesen werden
#define POLYNOMIAL  0xD8                  // 11011 followed by 0's; CRC Polynom

//////////////////////////////////////////////////////////////////////////////////////////
//Strukturen für PC bzw. Kamera Interfaces anlegen
//////////////////////////////////////////////////////////////////////////////////////////
#ifdef _SERIAL_INTERFACE_PC_C0
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_PC_C0; 
#endif

#ifdef _SERIAL_INTERFACE_PC_D0
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_PC_D0; 
#endif

#ifdef _SERIAL_INTERFACE_PC_E0
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_PC_E0; 
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//Strukturen für Protokoll Interfaces anlegen
//////////////////////////////////////////////////////////////////////////////////////////
#ifdef _SERIAL_INTERFACE_C0
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_C0;
#endif

#ifdef _SERIAL_INTERFACE_C1
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_C1;
#endif

#ifdef _SERIAL_INTERFACE_D0
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_D0;
#endif

#ifdef _SERIAL_INTERFACE_D1
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_D1;
#endif

#ifdef _SERIAL_INTERFACE_E0
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_E0;
#endif

#ifdef _SERIAL_INTERFACE_E1
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_E1;
#endif

#ifdef _SERIAL_INTERFACE_F0
  RRTLAN_SERIALCONFIG_EXTERN USART_data_t USART_data_F0;
#endif


///////////////////////////////////////////////////////////////////////////////////////////
//Microcontroller Synonyme
///////////////////////////////////////////////////////////////////////////////////////////
#define MCU1   USART_data_E0

///////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////////////
//Prototypen
///////////////////////////////////////////////////////////////////////////////////////////////////////
void InitUSART(USART_data_t* usart_data, USART_t* usart, unsigned long baudrate, uint8_t pc_interface);
void InitUSARTS();
///////////////////////////////////////////////////////////////////////////////////////////////////////


#endif

