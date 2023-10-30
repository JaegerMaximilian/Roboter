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
#include "servo.h"
#include "watchdog.h"
#include "accumulator.h"
#include "sensor.h"
#include "usart.h"
#include "Transform.h"
#include "PSE541.h"
#include "liftMotor.h"
#include "dynamixel.h"
#include "Arm.h"


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
	// ADC initialization
	// *************************************
	adca_init();

	// *************************************
	// Usart initialisation
	// *************************************
	InitUSARTS();
	usartd0_init(USART_D0_BUF_SIZE, USART_D0_BUF_SIZE);

	// *********************************
	// Servos/Lidar initialisieren
	// *********************************
 	//tce0_init();
 	//tcf0_init();
   
	// *********************************
	// initialize accumulator and temperature measurement 
	// *********************************
	Accumulator_Init();
	
	/* ************************* */
	/* initialize UNDK20 sensors */
	/* ************************* */
	//UNDK20_init();
	
	// *********************************
	// initialize multitasking-system
	// *********************************
	InitMultitasking();

	// *********************************
	// initialize tasks
	// *********************************
	InitDebug();
	//InitServo();
	InitSensor();
	PSE541_init();
	//rpLidar_Init();
	//evaluation_Init();
	
// 	InitDataPoint();
// 	InitSortedDatapoints();
	
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
// 	liftMotor_init(0,0,LIFT_REAR_RIGHT_NBR);
// 	liftMotor_init(0,0,LIFT_REAR_LEFT_NBR);
	InitArms();
 	AX_initDynamixel();
 	AX_READ(ax_servo);
	
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
