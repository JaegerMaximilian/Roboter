/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      KI.H
Version :  V 1.0
Date    :  23.03.2011
Author  :  MICHAEL ZAUNER

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

#ifndef _KI_H
#define _KI_H

#include <stdint.h>
#include "global.h"
#include "Pfadplanung.h"

#ifndef _KI_EXTERN
#define _KI_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
// Status der Tasks
#define DONE				0     // Die eigene Aufgabe wurde bereits erledigt
#define OPEN				1     // Die eigene/fremde Aufgabe ist noch offen
#define DID					2     // Die fremde Aufgaben wurde bereits erledigt
#define IS_DOING			3     // Der fremde Roboter erledigt diese Aufgabe im Augenblick
#define PENDING				4	  // Aufgabe die noch nicht vorhanden ist
#define LOCKED				5	  // Task locked if it is not configured or enemy area


//State of Game
#define GetPlants			1
#define ParkPlants			10
#define SolarPanels			30
#define StealPlants			40
#define driveHome			60

#define MAX_KI_TASKS		65		// Maximale Aufgaben (Tasks)

#define RRT_ROBOT			0
#define ENEMY_ROBOT			1

#define MAX_WAY_POINTS		15

// Settings für Roboter Verfahren
#define STANDARD_VELOCITY			500		//[m/s]
#define START_VELOCITY				310		//[m/s]
#define MIN_VELOCITY				170		//[m/s]
#define ENEMY_VELOCITY				200		//[m/s]
#define MAX_VELOCITY				800		//[m/s]

#define  STANDARD_ACC				120
#define  PLANT_ACC					60

// Struktur der Roboter (Position)
typedef point_t robot_t;

// Struktur der Elemente (Position)
typedef point_t element_t;

// Struktur der zu erledigenden Aufgaben (Tasks)
typedef struct
{
   uint8_t Status;				// Zeigt an, ob die Aufgabe bereits erledigt (DONE) oder nicht erledigt (OPEN) ist
   uint16_t Start;				// KI-State an der die Aufgabe startet
   uint8_t taskTime;			// Wie lange braucht man für die Aufgabe braucht
   uint8_t Priority;			// Priorität der Aufgabe 
   robot_t startPosition;		// Einstiegspunkt in die Aufgabe [0]
   robot_t taskArea[2];			// gibt den Bereich der Aufgabe an ([0] ... links oben; [1] ... rechts unten)
   int16_t timeToTask;			// Wie lange braucht man bis man beim Anfahrtspunkt der Aufgabe ist
} task_t;



_KI_EXTERN task_t KI_Task[MAX_KI_TASKS];												// Aufgabenliste
_KI_EXTERN int16_t speedSign, enemySign;												// Geschwindigkeitsumkehr
_KI_EXTERN uint16_t KI_State,OldKI_State, KI_StateNext, KI_StateOld, KI_StateError, KI_StateOldPath, KI_StateNextPath;		// Zustände der Statemachine
_KI_EXTERN uint8_t KI_maxPriority;														// Maximal ausgewählte Priorität
_KI_EXTERN uint8_t Status2SmallRobot;													// Statusmeldung für Nebenroboter


_KI_EXTERN point_t targetposition;
_KI_EXTERN uint16_t DecisionPathplanner;
_KI_EXTERN uint8_t HomePositionReached;  //0-> home position not reached; 1-> home postion reached


/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void InitKI(void);
void SetNextStepKI(unsigned int Current, unsigned int Next, unsigned int Error);
uint8_t KiTask(void);
uint8_t KiWatchRobotPositionTask(void);

#endif


