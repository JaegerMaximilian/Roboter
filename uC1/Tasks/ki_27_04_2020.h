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

#define MAX_KI_TASKS		50		// Maximale Aufgaben (Tasks)

#define RRT_ROBOT			0
#define ENEMY_ROBOT			1

#define MAX_WAY_POINTS		15

// Struktur der Roboter (Position)
typedef point_t robot_t;

// Struktur der Elemente (Position)
typedef point_t element_t;

// Struktur der zu erledigenden Aufgaben (Tasks)
typedef struct
{
   uint8_t Status;				// Zeigt an, ob die Aufgabe bereits erledigt (DONE) oder nicht erledigt (OPEN) ist
   uint16_t Start;				// KI-State an der die Aufgabe startet
   uint8_t maxPoints;			// Punkte, die die Aufgabe maximal einbringt  
   uint8_t Priority;			// Priorität der Aufgabe 
   robot_t startPosition[2];	// Einstiegspunkt in die Aufgabe [0] ... Position für gelb; [1] ... Position für blau
   robot_t taskArea[2];			// gibt den Bereich der Aufgabe an ([0] ... links oben; [1] ... rechts unten)
   int16_t linkTask[4];
} task_t;





// Struktur für die Wegermittlung mit Dijkstra-Algorithmus
typedef struct 
{
	char Besucht;					// Besuchter Knoten
	int Nach;						// Nachfolgender Knoten
	int Kosten;						// Kosten des Knoten
} TabelleDijkstra_t;

_KI_EXTERN task_t KI_Task[MAX_KI_TASKS];												// Aufgabenliste
_KI_EXTERN robot_t smallRobot, enemyRobot_1, enemyRobot_2;						// Roboter
_KI_EXTERN int16_t speedSign, enemySign;												// Geschwindigkeitsumkehr
_KI_EXTERN uint16_t KI_State, KI_StateNext, KI_StateOld, KI_StateError;		// Zustände der Statemachine
_KI_EXTERN uint8_t KI_maxPriority;														// Maximal ausgewählte Priorität
_KI_EXTERN uint8_t Status2SmallRobot;													// Statusmeldung für Nebenroboter



/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void InitKI(void);
void SetNextStep(unsigned int Current, unsigned int Next, unsigned int Error);
uint8_t KiTask(void);
uint8_t KiWatchTask(void);
uint8_t KiWatchRobotPositionTask(void);

// ****************************************************
// Dot2D, Norm2D, AngleToXAxis2D from path_math.c (µC2)
// ****************************************************
float Dot2D(float* vectorA, float* vectorB);
float Norm2D(float* vector);
float AngleToXAxis2D(float* vector);

// ****************************************************
// GetPx, GetPy for shooting the mammoth
// ****************************************************
int16_t GetPx(int16_t Mx, int16_t My);
int16_t GetPy(int16_t Mx, int16_t My);

#endif


