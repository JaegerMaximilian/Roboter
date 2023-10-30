/***************************************************************

************************************
** FH OBEROESTERREICH CAMPUS WELS **
************************************

Project :  Eurobot
Modul   :  Hauptplatine
File    :  uC2.c
Version :  V 1.0
Date    :  14.02.2013
Author  :  Michael Zauner

Comments:

Last edit:
Programmchange:

*)....
*).....

Chip type           : Xmega256
Program type        : Application
Clock frequency     : 32,000000 MHz
External SRAM size  : 0
Data Stack size     : 1024

Copyright (c) 2012 by FH-Wels
All Rights Reserved.
****************************************************************/

#define _GLOBAL_EXTERN

#include "global.h"
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "ports.h"
#include "systemclock.h"
#include "timer.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include "multitask.h"
#include "adc_driver.h"
#include <avr/eeprom.h>
#include "debug.h"
#include "define.h"
#include "rrt_receivetask.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "timer.h"
#include "genTrajAntrieb.h"
#include "servo.h"
#include "antrieb.h"
#include "watchdog.h"
#include "accumulator.h"
#include "sensor.h"
#include "liftMotor.h"
#include "anaPos.h"
#include "cupLift.h"

// define Robot type => save in EEPROM
// (Position Süd = Master / Position Nord = Slave)
uint8_t RobotType_EEPROM[1] EEMEM = {MASTER_ROBOT};


/***************************************************************
Functionname : initDevice
Comment      : initialisiert den µC
Transmit     : NO
Receive      : NO

Version      : V 1.0
Date         : 13.9.2011
Author       : ZAUNER MICHAEL
***************************************************************/
void initDevice ()
{
	uint8_t n;
	
	// *********************************
	// Disable all Interrupts
	// *********************************
	cli();
	
	// *********************************
	// Low level interrupt: On
	// Round-robin scheduling for low level interrupt: Off
	// Medium level interrupt: On
	// High level interrupt: On
	// The interrupt vectors will be placed at the start of the Application FLASH section
	// *********************************
	n=(PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
	PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	CCP=CCP_IOREG_gc;
	PMIC.CTRL=n;
	// Set the default priority for round-robin scheduling
	PMIC.INTPRI=0x00;


	// *************************************
	// System clocks initialization
	// *************************************
	system_clocks_init();

	// *************************************
	// Watchdog timer initialization
	// *************************************
	watchdog_init();

	// *************************************
	// Ports initialization
	// *************************************
	ports_init();

	// *************************************
	// Usart initialisation
	// *************************************
	InitUSARTS();

	// *********************************
	// Servos initialisieren
	// *********************************
	//tcc0_init();
	
	// *********************************
	// initialize accumulator and temperature measurement
	// *********************************
	Accumulator_Init();
	
	// Robot type
	RobotType_RAM = eeprom_read_byte(RobotType_EEPROM);

	// *********************************
	// initialize multitasking-system
	// *********************************
	InitMultitasking();

	// *********************************
	// initialize tasks
	// *********************************
	InitDebug();
	InitAntrieb();
	//InitServo();
	InitSensor();
	//anaPos_init();
	InitCupLift();
	//liftMotor_init(0, 0);
	
	// *********************************
	// initialize the RRTLAN-system-tasks
	// *********************************
	InitReceivetask();
	InitTimeoutManager();
	InitTransmit();
	InitReceiveData();
	
	Schleppfehler = SCHLEPP_FEHLER_TRANS_;
	
	
	// *********************************
	// enable all interrupts
	// *********************************
	sei();
}

/***************************************************************
Functionname : main
Comment      : Hauptprogramm
Transmit     : NO
Receive      : NO

Version      : V 1.0
Date         : 29.03.2012
Author       : ZAUNER MICHAEL
***************************************************************/
int main(void)
{
	// *********************************
	// µC initialisieren
	// *********************************
	initDevice();
	//
			//_delay_ms(6000);

	//#####################################################################
	//Fahrbewegung: 1.4m gerade aus
	//#####################################################################
	//paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVmax = 0.4;
	//paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fSsoll = 2.0;
	//paramInputAntrieb[indexInputInAntrieb].ucType = _POS_REL;
	//
	//indexInputInAntrieb++;
	//indexInputAntrieb++;

	// #####################################################################
    // Fahrbewegung: 5 Drehungen um die eigene Achse (Linksdrehung)
     //#####################################################################
          	    //paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
          	    //paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
          	    //paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
          	    //paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
          	    //paramInputAntrieb[indexInputInAntrieb].fVmax = -90.0;
          	    //paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 1800.0;
          	    //paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
          	    //paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
          	    //paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
          	    //paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.00;
          	    //paramInputAntrieb[indexInputInAntrieb].ucType = _TURN_REL;
          	    //
          	    //indexInputInAntrieb++;
          	    //indexInputAntrieb++;
        	    

	 //#####################################################################
	// Andere Testfälle
	// #####################################################################
	
	//paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVmax = 0.75;
	//paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.975;
	//paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//indexInputInAntrieb++;
	//indexInputAntrieb++;
	//
	//
	//paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVmax = 1.0;
	//paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.975;
	//paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.875;
	//paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//indexInputInAntrieb++;
	//indexInputAntrieb++;
	//
	//
	//paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVmax = 0.5;
	//paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 180.0;
	//paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.28;
	//paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].ucType = _KREISBOGEN;
	//
	//indexInputInAntrieb++;
	//indexInputAntrieb++;
	//
	//
	//paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVmax = 0.14;
	//paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 90.0;
	//paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.610;
	//paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].ucType = _KREISBOGEN;
	//
	//indexInputInAntrieb++;
	//indexInputAntrieb++;
	//
	//
	//paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fVstart = -0.2;
	//paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVmax = -0.75;
	//paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//indexInputInAntrieb++;
	//indexInputAntrieb++;
	//
	//paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//paramInputAntrieb[indexInputInAntrieb].fVstart = -0.2;
	//paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fVmax = 90;
	//paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//paramInputAntrieb[indexInputInAntrieb].ucType = _TURN_ABS;
	//
	//indexInputInAntrieb++;
	//indexInputAntrieb++;
//

	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.15;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = 0.3;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.1;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_REL;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.15;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.15;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = 0.5;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 180.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.28;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _KREISBOGEN;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.15;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = 0.25;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.1;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_REL;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = -0.4;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.45;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 1.25;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = 0.6;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.2;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.97;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = -0.75;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.2;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 1.53;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = -0.6;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.2;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 1.81;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = 0.75;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 0.2;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 1.53;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = 1.5;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 1.675;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 1.53;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = -0.5;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 1.325;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 1.825;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;


	//   paramInputAntrieb[indexInputInAntrieb].fTus = T_UP_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fTds = T_DOWN_AUTO_;
	//   paramInputAntrieb[indexInputInAntrieb].fVstart = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVend = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fVmax = 1.9;
	//   paramInputAntrieb[indexInputInAntrieb].fPhiSoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fRsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].fXsoll = 2.025;
	//   paramInputAntrieb[indexInputInAntrieb].fYsoll = 0.875;
	//   paramInputAntrieb[indexInputInAntrieb].fSsoll = 0.0;
	//   paramInputAntrieb[indexInputInAntrieb].ucType = _POS_ABS;
	//
	//   indexInputInAntrieb++;
	//   indexInputAntrieb++;

	
	// *********************************
	// MAIN-LOOP
	// *********************************
	while(1)
	{
		// *********************************
		// Multitaskingssystem ausführen
		// *********************************
		MultitaskingSystem();
	}
}
