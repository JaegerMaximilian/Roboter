/***************************************************************

************************************
** FH OBEROESTERREICH CAMPUS WELS **
************************************

Project :  Black Scorpion
Modul   :  Hauptplatine
File    :  uC1.c
Version :  V 1.0
Date    :  12.12.2012
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

//#define F_CPU 32000000UL

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
#include "usart.h"
#include "define.h"
#include "rrt_receivetask.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "rrt_timeoutmanager.h"
#include "rrt_serialconfig.h"
#include "servo.h"
#include "start.h"
#include "accumulator.h"
#include "ki.h"
#include "liftMotor.h"
#include "PSE541.h"
#include "sensor.h"
#include "anaPos.h"
#include "parser.h"
#include "Arm.h"
#include "command.h"
#include "wifi.h"
#include "enemyDetection.h"
#include "usDataFusion.h"
#include "UNDK20.h"
#include "U300D50.h"
#include "gripper.h"
#include "observation.h"
#include "nextion.h"
#include "mergeEnemyPos.h"

// define Robot type => save in EEPROM
// (Position Süd = Master / Position Nord = Slave)
uint8_t RobotType_EEPROM[1] EEMEM = {SLAVE_ROBOT};


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

	// *********************************
	// initialize system-clock
	// *********************************
	system_clocks_init();
	
	// *********************************
	// initialize ports
	// *********************************
	ports_init();
	
	// *********************************
	// initialize USART C0 (WIFI-interface)
	// *********************************
	usartc0_init(USART_C0_BUF_SIZE, USART_C0_BUF_SIZE);

	// *********************************
	// initialize the RRTLAN interfaces
	// *********************************
	InitUSARTS();

	// *********************************
	// EEPROM auslesen
	// *********************************
	// Robot type
	RobotType_RAM = eeprom_read_byte(RobotType_EEPROM);
	
	// *********************************
	// initialize accumulator measurement
	// *********************************
	Accumulator_Init();
	
	// *********************************
	// initialize ADCA
	// *********************************
	adca_init();
	adcb_init();
	
	// *********************************
	// initialize multitasking-system
	// *********************************
	InitMultitasking();

	// *********************************
	// initialize tasks
	// *********************************
	InitDebug();
	InitStart();
	InitMergeEnemyPos();
	
	// *********************************
	// initialize the RRTLAN-system-tasks
	// *********************************
	InitReceivetask();
	InitTimeoutManager();
	InitTransmit();
	InitReceiveData();
	
	// *********************************
	// initialize lift motors
	// *********************************
	//liftMotor_init(0,0,LIFT_FRONT_RIGHT_NBR);
	//liftMotor_init(0,0,LIFT_FRONT_LEFT_NBR);
	
	// *********************************
	// Servos initialisieren
	// *********************************
	tcd0_init();
	tcd1_init();
	tce1_init();
	
	InitServo();
	
	
	// *********************************
	// initialize presure sensor PE541 -> front left, front right
	// *********************************
	// 	PSE541_init();
	UNDK20_init();
	U300D50_init();
	
	/* start sensor task */
	InitSensor();
	
	/* start parser for debug-interface */
	InitParser();
	
	/* start observation task */
	InitObservation();
	
	/* start LCD-communication */
	InitNextion();
	
	/* start enemydetection with US and Lidar */
	//InitEnemyDetection();

	/* start cup-stacker */
	//	InitArms();
	
	/* sensor fusion */
	//usDataFusion_Init();
	
	Points = 0;


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
