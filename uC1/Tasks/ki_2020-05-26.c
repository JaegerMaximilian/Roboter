/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      KI.c
Version :  V 1.0
Date    :  23.03.2011
Author  :  ZAUNER MICHAEL

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : Xmega256A
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024                

               Copyright (c) 2011 by FH-Wels                          
                   All Rights Reserved.
****************************************************************/

#define _KI_EXTERN

#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>
#include "multitask.h"
#include "ki.h"
#include "ports.h"
#include "define.h" 
#include "rrt_usart_driver.h"
#include "global.h"
#include "adc_driver.h"
#include "rrt_receivedata.h"
#include "rrt_transmittingtask.h"
#include "servo.h"
#include "usart.h"
#include "rrt_serialconfig.h"
#include "Pfadplanung.h"
#include "poi.h"




char text[200];

// Aufbau des Punkte-Arrays
// erster Wert -> FARBE (gelb/rot)
// zweiter Wert -> PUNKT (1. Pos, 2. Pos, ....)
// dritter Wert -> X[mmm]/Y[mm]/Speed[mm/s]  
#define X_POS     0
#define Y_POS	   1
#define V_POS		2
#define SPEED		2
#define PHI_POS	2

// Zum Debuggen einkommentieren
//#define __DEBUG__
//#define __MIST__
//#define __NET__


#define RAND_MIN 150
#define RAND_MAX_X 2850
#define RAND_MAX_Y 1850


// Table-Defines
#define MIND_ABSTAND_WALL     150
#define TABLE_SIZE_X				3000
#define TABLE_SIZE_Y				2000

/* the last 5 sec no new task will be chosen */
#define KI_DISABLE_TIME			5


// Timeout zum Abfangen eines Deadlocks in der Fahrtenüberwachung (Case: 50000)
static int16_t timeOut = 0;

uint8_t stopEngin = 0;
uint8_t motionIR = 0;


// Grenz DRUCK
#define VAKUUM_LIMIT 500


uint8_t gegnerErkennung = OFF;
uint8_t i = 0;


// Für die Wegberechung damit nur Punkte hintereinander geschickt werden
int16_t driveToPoints[MAX_WAY_POINTS][3];
int8_t driveToPointsIndex = 0;
int8_t driveToPointsIndexOut = 0;
uint8_t withClothoid = OFF;
                    
/**************************************************************************
***   FUNKTIONNAME: InitKI                                              ***
***   FUNKTION: initialisiert die KI                                    ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitKI(void)
{
	// zyklischer Task - Zykluszeit: 10 ms
	SET_TASK(KI_TASKNBR, CYCLE);
	SET_CYCLE(KI_TASKNBR, 10);
	SET_TASK_HANDLE(KI_TASKNBR, KiTask);
   
	// zyklischer Task - Zykluszeit: 100 ms
	SET_TASK(KI_WATCH_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_TASKNBR, 100);
	SET_TASK_HANDLE(KI_WATCH_TASKNBR, KiWatchTask);

	// KiWatchRobotPositionTask initialisieren
	SET_TASK(KI_WATCH_ROBOT_POS_TASKNBR, CYCLE);
	SET_CYCLE(KI_WATCH_ROBOT_POS_TASKNBR, 100);
	SET_TASK_HANDLE(KI_WATCH_ROBOT_POS_TASKNBR, KiWatchRobotPositionTask);

   
	KI_State = 0; 
	
  
	// Enemy detection
	gegnerErkennung = OFF;
	
   // ********************************************************************
   // ********************************************************************
   // ******
   // ******   A U F G A B E N   I N I T I A L I S I E R E N
   // ******
   // ********************************************************************
   // ********************************************************************
   	for (uint8_t i = 0; i < MAX_KI_TASKS; i++)
   	{
	   	for (uint8_t j = 0; j < 4; j++)
		   {
			   KI_Task[i].linkTask[j] = -1;
		   }
   	}

   
   
	// Task 0000 - Default-Task: !!! Do not use !!!
	KI_Task[0].Status = OPEN;          
	KI_Task[0].Start = 0;
	KI_Task[0].maxPoints = 0;
	   
 	// Task 1000 - Becher 11 -> Zone NORD
 	KI_Task[1].Status = OPEN;
 	KI_Task[1].Start = 1000;
 	KI_Task[1].maxPoints = 3;
	KI_Task[1].startPosition[GELB].Xpos = 686;
	KI_Task[1].startPosition[GELB].Ypos = 2770;
	KI_Task[1].startPosition[BLAU].Xpos = 686;
	KI_Task[1].startPosition[BLAU].Ypos = 230;
	KI_Task[1].taskArea[0].Xpos = 2550;
	KI_Task[1].taskArea[0].Ypos = 250; 														// New distributor for handling the fruit baskets
	KI_Task[1].taskArea[1].Xpos = 2850;
	KI_Task[1].taskArea[1].Ypos = 550;
	KI_Task[1].linkTask[0] = 2000; 
															// New distributor for handling the fruit baskets
 	// Task 2000 - Drive out of the starting area
 	KI_Task[2].Status = OPEN;
 	KI_Task[2].Start = 2000;
 	KI_Task[2].maxPoints = 0;														// -
 	// Task 3000 - 
 	KI_Task[3].Status = OPEN;
 	KI_Task[3].Start = 3000;
 	KI_Task[3].maxPoints = 2;														// -
 	// Task 4000 - Fire 4 (yellow side)
 	KI_Task[4].Status = OPEN;
 	KI_Task[4].Start = 4000;
 	KI_Task[4].maxPoints = 2;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 5000 - Fire 5 (red side)
 	KI_Task[5].Status = OPEN;
 	KI_Task[5].Start = 5000;
 	KI_Task[5].maxPoints = 2;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 6000 - Fire 6 (yellow side)
 	KI_Task[6].Status = OPEN;
 	KI_Task[6].Start = 6000;
 	KI_Task[6].maxPoints = 2;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 7000 - Fire 7 (red side)
 	KI_Task[7].Status = OPEN;
 	KI_Task[7].Start = 7000;
 	KI_Task[7].maxPoints = 2;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 8000 - Fire 8 (yellow side)
 	KI_Task[8].Status = OPEN;
 	KI_Task[8].Start = 8000;
 	KI_Task[8].maxPoints = 2;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 9000 - Fire 9 (red side)
 	KI_Task[9].Status = OPEN;
 	KI_Task[9].Start = 9000;
 	KI_Task[9].maxPoints = 2;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 10000 - Drop Fire
 	KI_Task[10].Status = OPEN;
 	KI_Task[10].Start = 10000;
 	KI_Task[10].maxPoints = 2;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 11000 - Grab Fire Stack (yellow side)
 	KI_Task[11].Status = OPEN;
 	KI_Task[11].Start = 11000;
 	KI_Task[11].maxPoints = 6;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 12000 - Grab Fire Stack (red side)
 	KI_Task[12].Status = OPEN;
 	KI_Task[12].Start = 12000;
 	KI_Task[12].maxPoints = 6;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 13000 - Collect fruits on tree (red sideways)
 	KI_Task[13].Status = OPEN;
 	KI_Task[13].Start = 13000;
 	KI_Task[13].maxPoints = 4;														// 1 point (fruit) and -2 points (toxic fruit)
 	// Task 14000 - Collect fruits on tree (yellow sideways)
 	KI_Task[14].Status = OPEN;
 	KI_Task[14].Start = 14000;
 	KI_Task[14].maxPoints = 4;														// 1 point (fruit) and -2 points (toxic fruit)
 	// Task 15000 - Collect fruits on tree (red front)
 	KI_Task[15].Status = OPEN;
 	KI_Task[15].Start = 15000;
 	KI_Task[15].maxPoints = 4;														// 1 point (fruit) and -2 points (toxic fruit)
 	// Task 16000 - Collect fruits on tree (yellow front)
 	KI_Task[16].Status = OPEN;
 	KI_Task[16].Start = 16000;
 	KI_Task[16].maxPoints = 4;														// 1 point (fruit) and -2 points (toxic fruit)
 	// Task 17000 - Tree distributor
 	KI_Task[17].Status = OPEN;
 	KI_Task[17].Start = 17000;
 	KI_Task[17].maxPoints = 0;														// Distributor for handling the trees
 	// Task 18000 - Tree Active distributor
 	KI_Task[18].Status = OPEN;
 	KI_Task[18].Start = 18000;
 	KI_Task[18].maxPoints = 0;														// Distributor for handling the active trees
 	// Task 19000 - Empty the fruits (red side)
 	KI_Task[19].Status = ((spielfarbeRAM == BLAU) ? OPEN : DONE); 
 	KI_Task[19].Start = 19000;
 	KI_Task[19].maxPoints = 0;														// Points depends on the collected fruits
 	// Task 20000 - Empty the fruits (yellow side)
 	KI_Task[20].Status = ((spielfarbeRAM == GELB) ? OPEN : DONE); 
 	KI_Task[20].Start = 20000;
 	KI_Task[20].maxPoints = 0;														// Points depends on the collected fruits
 	// Task 21000 - Fruit basket distributor
 	KI_Task[21].Status = OPEN;
 	KI_Task[21].Start = 21000;
 	KI_Task[21].maxPoints = 0;														// Distributor for handling the fruit baskets
 	// Task 22000 - Only fire the net (for SPEZIAL_MODUS)
 	KI_Task[22].Status = OPEN;
 	KI_Task[22].Start = 22000;
 	KI_Task[22].maxPoints = 6;														// Only fire the net (for SPEZIAL_MODUS)
 	// Task 23000 - Funny action
 	KI_Task[23].Status = OPEN;
 	KI_Task[23].Start = 23000;
 	KI_Task[23].maxPoints = 6;														// 6 points (net thrown on a Mammoth)
 	// Task 24000 - Funny action distributor
 	KI_Task[24].Status = OPEN;
 	KI_Task[24].Start = 24000;
 	KI_Task[24].maxPoints = 0;														// Distributor for handling the funny action
 	// Task 25000 - Fire place distributor
 	KI_Task[25].Status = OPEN;
 	KI_Task[25].Start = 25000;
 	KI_Task[25].maxPoints = 0;														// Distributor for handling the fire places
 	// Task 26000 - Fire search (yellow side - right top)
 	KI_Task[26].Status = OPEN;
 	KI_Task[26].Start = 26000;
 	KI_Task[26].maxPoints = 0;														// 1 point (outside a fire place) and 2 points (inside a fire place) 
 	// Task 27000 - Fire search (red side - left top)
 	KI_Task[27].Status = OPEN;
 	KI_Task[27].Start = 27000;
 	KI_Task[27].maxPoints = 0;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 28000 - Fire search (yellow side - right bottom)
 	KI_Task[28].Status = OPEN;
 	KI_Task[28].Start = 28000;
 	KI_Task[28].maxPoints = 0;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	// Task 29000 - Fire search (red side - left bottom)
 	KI_Task[29].Status = OPEN;
 	KI_Task[29].Start = 29000;
 	KI_Task[29].maxPoints = 0;														// 1 point (outside a fire place) and 2 points (inside a fire place)
 	
 	// Task 40000 - Enemy detection round
 	KI_Task[40].Status = OPEN;
 	KI_Task[40].Start = 40000;
 	KI_Task[40].maxPoints = 0;
		
   // *******************************************
   // Initialize the strategies
   // *******************************************
   for(i = 0; i < MAX_KI_TASKS; i++)
   {
      KI_Task[i].Priority = 0;   
   }

	// *******************************************
	// Strategy: PASSIVER_MODUS & PASSIVER_GLITZER_MODUS   
	// *******************************************
//    if((strategieRAM == PASSIVER_MODUS) || (strategieRAM == PASSIVER_GLITZER_MODUS))
//    {
// 		if(strategieRAM == PASSIVER_GLITZER_MODUS)	ist_gegner_sensor = ON;
// 		else														ist_gegner_sensor = OFF;
// 
//       // ****************************************
//       // G E L B
//       // ****************************************
//       if(spielfarbeRAM == GELB)
//       {
// 			KI_Task[16].Priority = 150;		// Task 16000 - Collect fruits on tree (yellow front)
// 			KI_Task[14].Priority = 149;		// Task 14000 - Collect fruits on tree (yellow sideways)
// 			KI_Task[11].Priority = 148;		// Task 11000 - Grab Fire Stack (yellow side)
// 			KI_Task[1].Priority =  147;		// Task 1000 - Empty the fruits new (all sides)
// 						
// 			//KI_Task[13].Priority = 0;		// Task 13000 - Collect fruits on tree (red sideways)	
// 			//KI_Task[15].Priority = 0;		// Task 15000 - Collect fruits on tree (red front)
// 			//KI_Task[12].Priority = 0;		// Task 12000 - Grab Fire Stack (red side)
//       }
//       // ****************************************
//       // R O T
//       // ****************************************
//       else
//       {
// 			KI_Task[15].Priority = 150;		// Task 15000 - Collect fruits on tree (red front)	
// 			KI_Task[13].Priority = 149;		// Task 13000 - Collect fruits on tree (red sideways)
// 			KI_Task[12].Priority = 148;		// Task 12000 - Grab Fire Stack (red side)	
// 			KI_Task[1].Priority =  147;		// Task 1000 - Empty the fruits new (all sides)
// 			
// 			//KI_Task[14].Priority = 0;		// Task 14000 - Collect fruits on tree (yellow sideways)
// 			//KI_Task[16].Priority = 0;		// Task 16000 - Collect fruits on tree (yellow front)
// 			//KI_Task[11].Priority = 0;		// Task 11000 - Grab Fire Stack (yellow side)
//       }
//    }
// 
// 	// *******************************************
// 	// Strategy: AKTIVER_MODUS & AKTIVER_GLITZER_MODUS
// 	// *******************************************  
//    else if((strategieRAM == AKTIVER_MODUS) || (strategieRAM == AKTIVER_GLITZER_MODUS))
//    {
// 		if(strategieRAM == AKTIVER_GLITZER_MODUS)		ist_gegner_sensor = ON;
// 		else														ist_gegner_sensor = OFF;
// 				
//       // ****************************************
//       // G E L B
//       // ****************************************
//       if(spielfarbeRAM == GELB)
//       {
// 			KI_Task[16].Priority = 150;		// Task 16000 - Collect fruits on tree (yellow front)
// 			KI_Task[14].Priority = 149;		// Task 14000 - Collect fruits on tree (yellow sideways)
// 			KI_Task[11].Priority = 148;		// Task 11000 - Grab Fire Stack (yellow side)
// 			KI_Task[1].Priority =  147;		// Task 1000 - Empty the fruits new (all sides)
// 			
// 			// Alte Schweiz
// 			//KI_Task[2].Priority = 150;	// Task 2000 - Only fire the net (for AKTIVER_MODUS)
// 			//KI_Task[3].Priority = 149;
//       }
//       // ****************************************
//       // R O T
//       // ****************************************
//       else
//       {
// 			KI_Task[15].Priority = 150;		// Task 15000 - Collect fruits on tree (red front)
// 			KI_Task[13].Priority = 149;		// Task 13000 - Collect fruits on tree (red sideways)
// 			KI_Task[12].Priority = 148;		// Task 12000 - Grab Fire Stack (red side)
// 			KI_Task[1].Priority =  147;		// Task 1000 - Empty the fruits new (all sides)
// 			
// 			// Alte Schweiz
// 			//KI_Task[2].Priority = 150;	// Task 2000 - Only fire the net (for AKTIVER_MODUS)
// 			//KI_Task[3].Priority = 149;
//       }
//    }
// 	
// 	// *******************************************
// 	// Strategy: SPEZIAL_MODUS
// 	// *******************************************
// 	else if((strategieRAM == SPEZIAL_MODUS) || (strategieRAM == SPEZIAL_GLITZER_MODUS))
// 	{
// 		if(strategieRAM == SPEZIAL_GLITZER_MODUS)		ist_gegner_sensor = ON;
// 		else														ist_gegner_sensor = OFF;
// 		
//       // ****************************************
//       // G E L B
//       // ****************************************
//       if(spielfarbeRAM == GELB)
//       {
// 			KI_Task[22].Priority = 150;	// Task 22000 - Only fire the net (for SPEZIAL_MODUS)
//       }
//       // ****************************************
//       // R O T
//       // ****************************************
//       else
//       {
// 			KI_Task[22].Priority = 150;	// Task 22000 - Only fire the net (for SPEZIAL_MODUS)
//       }
// 	}
//   
// 	// *******************************************
// 	// Strategy: GEGNER_BEIDE
// 	// *******************************************
// 	else if((strategieRAM == GEGNER_ERKENNUNG_BEIDE))
// 	{
// 		// !!! Haupt- und Nebenroboter starten gemeinsam !!!
// 		
// 		// -
// 	}
// 	
// 	// *******************************************
// 	// Strategy: GEGNER_KLEIN
// 	// *******************************************
// 	else if((strategieRAM == GEGNER_ERKENNUNG_SMALL))
// 	{
// 		// !!! Nebenroboter startet alleine !!!
// 		
// 		// -
// 	}
// 
// 	// *******************************************
// 	// Strategy: GEGNER_GROSS
// 	// *******************************************
// 	else if((strategieRAM == GEGNER_ERKENNUNG_BIG))
// 	{
// 		// !!! Hauptroboter startet alleine !!!
// 		
// 		KI_Task[40].Priority = 150;
// 	}
// 	
// 	// *******************************************
// 	// Strategy: TECH_ABNAHME
// 	// *******************************************
// 	else if(strategieRAM == TECH_ABNAHME)
// 	{
// 		// ****************************************
// 		// G E L B
// 		// ****************************************
// 		if(spielfarbeRAM == GELB)
// 		{
// 			
// 		}
// 		// ****************************************
// 		// R O T
// 		// ****************************************
// 		else
// 		{
// 			
// 		}
// 	}
}

/**************************************************************************
***   FUNKTIONNAME: SetNextStep                                         ***
***   FUNKTION: Setzt die nächsten Schritte                             ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: Current ... aktueller State                   ***
***                       Next ... nächster State -> bei OK             ***
***                       Error ... nächster State -> bei Error         ***
**************************************************************************/
void SetNextStep(unsigned int Current, unsigned int Next, unsigned int Error)
{
   KI_State = 50000;
   KI_StateNext = Next;
   KI_StateOld = Current;
   KI_StateError = Error;
}



#define UEBERWACHUNGS_DIS_FRONT_SPECIAL 550
#define UEBERWACHUNGS_DIS_SEITE_SPECIAL 400
#define ROBOTER 140



/**************************************************************************
***   FUNKTIONNAME: AreaIsFree                                          ***
***   FUNKTION: Gibt TRUE zurück wenn gGebiet frei ist                  ***
**************************************************************************/
uint8_t AreaIsFree(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	
	if ((smallRobot.Xpos > x0) && (smallRobot.Xpos < x1) && (smallRobot.Ypos > y0) && (smallRobot.Ypos < y1))
	{
		return(FALSE);
	}
	else if ((enemyRobot_1.Xpos > x0) && (enemyRobot_1.Xpos < x1) && (enemyRobot_1.Ypos > y0) && (enemyRobot_1.Ypos < y1))
	{
		return(FALSE);
	}
	else if ((enemyRobot_2.Xpos > x0) && (enemyRobot_2.Xpos < x1) && (enemyRobot_2.Ypos > y0) && (enemyRobot_2.Ypos < y1))
	{
		return(FALSE);
	}
	
	return(TRUE);
}




/**************************************************************************
***   FUNCTIONNAME:        KiTask                                       ***
***   FUNCTION:            KI                                           ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiTask(void)
{   
   // Cycle per default auf 50 ms setzen
   SET_CYCLE(KI_TASKNBR, 50);
	
   // ********************************************************************
   // ********************************************************************
   // ******																				  
   // ******   S P I E L Z E I T U E B E R W A C H U N G						  
   // ******																				  
   // ********************************************************************
   // ********************************************************************
   // ***********************
   // in der letzten Sekunde 
	//		-> Roboter gegebenfalls stoppen
   // ***********************
    if(spielZeit < 1)
    {      
 		// ******************************************************
 		// Hier alle notwendigen Systeme deaktivieren
 		// ******************************************************  
 		
 		// ******************************************************
 		// wenn eine Bewegung ausgeführt wird
 		//		-> Roboter stoppen und auf Leerlaufstate springen
 		// ******************************************************
 		if((statusAntrieb == 0) && (stopEngin == 0))
 		{
 			setAntrieb_RRTLAN(0, 0, 0, 0, 0, 0, 0, 0, MOTION_INTERRUPT, GEGNER_OFF); 
 			SetNextStep(KI_State, 24008, 24008);
 			
 			stopEngin = 1;
 		}
 		// -> ansonsten nur auf Leerlaufstate springen
 		else
 		{

 		}                
    }
	

   // ********************************************************************
   // ********************************************************************
   // ******                                                        
   // ******   K I  -  S T A T E M A C H I N E             
   // ******                                                        
   // ********************************************************************
   // ********************************************************************
   switch(KI_State)
   {
      // ********************************************************************
      // ********************************************************************
      // ******                                                        
      // ******   K I  -  V E R T E I L E R                 
      // ******                                                        
      // ********************************************************************
      // ********************************************************************
      // In den 1. State der KI springen
      case 0:
      {			
         KI_State = 10;
         return(ENABLE);
      
         break;
      } 
		
		// *****************************************
		// in den KI-Verteiler springen
		// *****************************************
		case 10:
		{
			// Aus der Startbox fahren
//			setAntrieb(0, 0, SPEED_STARTBOX, 0, 0, 0, 0, 400, POS_REL, GEGNER_OFF);
			         
			// KI-Verteiler starten
			SetNextStep(KI_State, 20, 20);
			
			break;
		}
      // *****************************************
      // KI Verteiler
      // *****************************************  
      case 20:
      {
         uint8_t i, Aufgabe_gefunden = 0; 
       	              
         // ************************************************************
         // Aufgabe mit der höchsten Priorität fürs Punkten suchen
         // ************************************************************
         if(spielZeit >= KI_DISABLE_TIME)
         {
            KI_maxPriority = 0; 

            // *********************************************************
            // Die Aufgabe mit der höchsten Priorität suchen
            // *********************************************************
			for(i = 1; i < MAX_KI_TASKS; i++)
			{
				if((KI_Task[i].Priority >= KI_Task[KI_maxPriority].Priority) && (KI_Task[i].Priority > 0))
				{
					// ***************************************************
					// Überprüfen, ob die Aufgabe noch nicht erledigt ist
					// ***************************************************
						
					if(KI_Task[i].Status == OPEN)
					{
						KI_maxPriority = i;
						Aufgabe_gefunden = 1;
					}
				}
			} 
            // Wenn eine Aufgabe gefunden ist, wird auf diese Aufgabe gesprungen
            if(Aufgabe_gefunden == 1)
            {
               KI_State = KI_Task[KI_maxPriority].Start;
               return(ENABLE);
            }
         }
         
         break;
      }
      
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******	Task 1000 - Empty the fruits new (all sides)
		// ******
		// ********************************************************************
		// ********************************************************************
		case 1000:
		{
// 			// Gebiet vor Box ist frei -> direkt zur Box fahren
// 			if (AreaIsFree(KI_Task[1].taskArea[0].Xpos, KI_Task[1].taskArea[0].Ypos, KI_Task[1].taskArea[1].Xpos, KI_Task[1].taskArea[1].Ypos) == TRUE)
// 			{
// 				if(DriveTo(500, KI_Task[1].startPosition[SpielFarbe], ON, ON)) SetNextStep(KI_State, 1010, 20);
// 				else														   KI_State = 20;
// 			}
// 			// ansonsten Box seitlich anfahren
// 			else
// 			{
// 				KI_State = 20;
// 			}
// 

		// grüner Becherlift in Position fahren -> 1010
			
			break;
		}
		case 1010:
		{
// 			// Gebiet vor Box ist frei -> direkt zur Box fahren
// 			if (AreaIsFree(KI_Task[1].taskArea[0].Xpos, KI_Task[1].taskArea[0].Ypos, KI_Task[1].taskArea[1].Xpos, KI_Task[1].taskArea[1].Ypos) == TRUE)
// 			{
// 				if(DriveTo(500, KI_Task[1].startPosition[SpielFarbe], ON, ON)) SetNextStep(KI_State, 1010, 1005);
// 				else														   KI_State = 20;
// 			}
// 			// ansonsten Box seitlich anfahren
// 			else
// 			{
// 				KI_State = 20;
// 			}
// 
			// Fahre auf Init-Position -> 1020
			
			break;
		}
		
		case 1020:
		{
			
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 1100 - Cup 11 Green 
		// ******
		// ********************************************************************
		// ********************************************************************
		case 1100:
		{
			int x = Pois[0][1];
			int y = Pois[0 , 2];
			// Welcher Punkt ist am besten anzufahren		
			// Dieser Punkt ist vom Start aus der Punkt 2000
			break;
		}

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 1200 - Cup 12
		// ******
		// ********************************************************************
		// ********************************************************************
		case 1200:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 1300 - Cup 13
		// ******
		// ********************************************************************
		// ********************************************************************
		case 1300:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 1400 - Cup 14
		// ******
		// ********************************************************************
		// ********************************************************************
		case 1400:
		{
			
			break;
		}
      
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 1500 - Cup 15 - 19
		// ******
		// ********************************************************************
		// ********************************************************************
		case 1500:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 2100 - Cup 21
		// ******
		// ********************************************************************
		// ********************************************************************
		case 2100:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 2200 - Cup 22
		// ******
		// ********************************************************************
		// ********************************************************************
		case 2200:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 2300 - Cup 23
		// ******
		// ********************************************************************
		// ********************************************************************
		case 2300:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 2400 - Cup 24
		// ******
		// ********************************************************************
		// ********************************************************************
		case 2400:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 4100 - Cup 41 - 44
		// ******
		// ********************************************************************
		// ********************************************************************
		case 4100:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 5100 - Cup 51
		// ******
		// ********************************************************************
		// ********************************************************************
		case 5100:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 5200 - Cup 52
		// ******
		// ********************************************************************
		// ********************************************************************
		case 5200:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 5300 - Cup 53
		// ******
		// ********************************************************************
		// ********************************************************************
		case 5300:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 5400 - Cup 54
		// ******
		// ********************************************************************
		// ********************************************************************
		case 5400:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 7100 - Cup 71 - 75
		// ******
		// ********************************************************************
		// ********************************************************************
		case 7100:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 8100 - Cup 81 - 85
		// ******
		// ********************************************************************
		// ********************************************************************
		case 8100:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 9100 - Fahne 1
		// ******
		// ********************************************************************
		// ********************************************************************
		case 9100:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 9200 - Fahne 2
		// ******
		// ********************************************************************
		// ********************************************************************
		case 9200:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 9300 - Lighthouse
		// ******
		// ********************************************************************
		// ********************************************************************
		case 9300:
		{
			
			break;
		}

		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 10010 - Empty Silo Right
		// ******
		// ********************************************************************
		// ********************************************************************
		case 10010:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 10020 - Empty Silo Left
		// ******
		// ********************************************************************
		// ********************************************************************
		case 10020:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 10030 Empty Silo Grabber
		// ******
		// ********************************************************************
		// ********************************************************************
		case 10030:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 11000 - Empty Green Line
		// ******
		// ********************************************************************
		// ********************************************************************
		case 11000:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 12000 - Empty Red Line
		// ******
		// ********************************************************************
		// ********************************************************************
		case 12000:
		{
			
			break;
		}
		
		// ********************************************************************
		// ********************************************************************
		// ******
		// ******   Task 13000 - Coming Home
		// ******
		// ********************************************************************
		// ********************************************************************
		case 13000:
		{
			
			break;
		}
	}
	
	
   return(CYCLE);
}

/**************************************************************************
***   FUNCTIONNAME:        KiWatchTask                                  ***
***   FUNCTION:            überwacht ob Aufgaben bereits erledigt sind  ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiWatchTask(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_CYCLE(KI_WATCH_TASKNBR, 100);
	
	if (spielZeit > KI_DISABLE_TIME)
	{
	}
	
	return(CYCLE);
}

/**************************************************************************
***   FUNCTIONNAME:        KiWatchRobotPositionTask                     ***
***   FUNCTION:            Sendet die Daten an den Nebenroboter			***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
uint8_t KiWatchRobotPositionTask(void)
{
	// zyklischer Task - Zykluszeit: 100 ms
	SET_CYCLE(KI_WATCH_ROBOT_POS_TASKNBR, 100);
	
	// ********************************************************
	// Status2SmallRobot
	// ********************************************************
	// 0000 | 0001 ... Startschnur-Status
	// 0000 | 0010 ... Spielfeld-Farbe
	// 0000 | 0100 ... xxx
	// 0000 | 1000 ... xxx
	// 0001 | 0000 ... xxx
	// 0010 | 0000 ... xxx
	// 0100 | 0000 ... xxx
	// 1000 | 0000 ... xxx
	// ********************************************************
	
	// Startschnur-Status
	if(START_SCHNUR)
		Status2SmallRobot |= 0x01;
	else
		Status2SmallRobot &= ~0x01;
	
	// Spielfeld-Farbe
	if(SpielFarbe == GELB)
		Status2SmallRobot |= 0x02;
	else
		Status2SmallRobot &= ~0x02;
	
	
	// Send message to small robot
//	Send2SmallRobot();
	
	return(CYCLE);
}

// ****************************************************
// Dot2D, Norm2D, AngleToXAxis2D from path_math.c (µC2)
// ****************************************************
float Dot2D(float* vectorA, float* vectorB)
{
	return (vectorA[X_KOR] * vectorB[X_KOR] + vectorA[Y_KOR] * vectorB[Y_KOR]);
}

float Norm2D(float* vector)
{
	return ((float)(sqrt(pow(vector[X_KOR], 2.0) + pow(vector[Y_KOR], 2.0))));
}

float AngleToXAxis2D(float* vector)
{
	float phi;
	float e_x[2] = {1, 0};
	float delta_x, delta_y;
	
	// Calculating phi_max -> Angle between the X-axis and the vector: Initial point <-> Destination point
	phi = acos(Dot2D(e_x, vector) / (Norm2D(e_x) * Norm2D(vector)));
	
	delta_y = vector[Y_KOR] - e_x[Y_KOR];
	delta_x = vector[X_KOR] - e_x[X_KOR];
	
	// If necessary, correction of the angle
	if((delta_y < 0.0) && (delta_x < 0.0))
		phi = 2 * M_PI - phi;
	if((delta_y < 0.0) && (delta_x > 0.0))
		phi = 2 * M_PI - phi;

	return (phi);
}

// ****************************************************
// GetPx, GetPy for shooting the mammoth
// ****************************************************
int16_t GetPx(int16_t Mx, int16_t My)
{
	int16_t Px = 0;
	float R = 11025.0;		// Rx = 10,5 cm
	
	float Mx_x = (float)(Mx - xPos);
	float My_y = (float)(My - yPos);
	float Mx_x2 = pow(Mx_x, 2.0);
	float My_y2 = pow(My_y, 2.0);
	
	float result1 = R * Mx_x;
	float result2 = (float)xPos * (Mx_x2 + My_y2);
	float result3 = sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
	float result4 = Mx_x2 + My_y2;
	
	Px = (int16_t)((result1 + result2 + result3) / result4); 
	
	return(Px);
}

int16_t GetPy(int16_t Mx, int16_t My)
{
	int16_t Py = 0;
	float R = 11025.0;		// Rx = 10,5 cm
	
	float Mx_x = (float)(Mx - xPos);
	float My_y = (float)(My - yPos);
	float Mx_x2 = pow(Mx_x, 2.0);
	float My_y2 = pow(My_y, 2.0);
	
	float result1 = R * My_y2;
	float result2 = Mx * sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
	float result3 = (float)xPos * sqrt(R * (-R + Mx_x2 + My_y2) * My_y2);
	float result4 = (Mx_x2 + My_y2) * My_y * (float)yPos;
	float result5 = (Mx_x2 + My_y2) * My_y;
	
	Py = (int16_t)((result1 - result2 + result3 + result4) / result5);
	
	return(Py);
}