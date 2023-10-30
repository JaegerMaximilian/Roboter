/*
* GUI.c
*
* Created: 10.05.2019 11:43:27
*  Author: Schnadi
*/

/* ************************************************************** */
/*! \brief GUI-task.
*
*  Das nachfolgende Programm dient zur Steuerun des Beacons. Das Programm ist mitells Satemachine realisiert worden.
*	Diese Statemchine und eine weiterführende erklärung zu diessem Program bzw. der GUI und deren Ablauf ist in der Bachelorarbeit
*	" Konzept und Umsetzung eines bedienerfreundlichen Visualisierungssystems zur Lokalisierung von Robotern " von
*	 Werner Schnadenauer, zu finden.
*	Die benötigten Printf Befehle entnehmen Sie bitte dem  Datenblatt der Firma Electronic Assembly.
*
*
* \author
*      Werner Schnadenauer
*      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
*      Support email: werner.schnadenauer@students.fh-wels.at or roboracing@fh-wels.at
*
* $Revision: 1 $
* $Date: 2019-05-28  $  \n
*
* Copyright (c) 2019, RRT (University of Applied Sciences Upper Austria) All rights reserved.
*
*/
/* ************************************************************** */

#define _GUI_EXTERN_

#include <avr/io.h>
#include "GUI.h"
#include "GUIconfig.h"
#include "define.h"
#include "multitask.h"
#include "LEDcontrol.h"
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>


/* ************************************************************** */
/*! \brief Initialize GUI-task.
*
*  Function initialize the debug-task
*
*  \version 28.05.2019
*
*/
/* ************************************************************** */
void Init_GUI(void)
{
	/* cyclic task - cycle time: 500 ms */
	SET_CYCLE(GUI_TASKNBR, 500);
	SET_TASK(GUI_TASKNBR, CYCLE);
	SET_TASK_HANDLE(GUI_TASKNBR, GUI_Task);
}

/* *****************Globale Variablen**************************** */

uint8_t data[75]; //Variable für die Befehle des Displays

/* ************************************************************** */

uint8_t GUI_Task(void)
{
	/* *******************Variablen********************************** */
	//diese Variablen werden in den jeweiligen States benötigt
	static uint8_t state = GUI_COMMON_SETTINGS, FarbAuswahl=0,FarbAuswahl_Next=0, StrategieAuswahl=0, PunkteAlt = 0, caseHauptseite = 0;
	char textStrategie[50];
	
	SET_CYCLE(GUI_TASKNBR, 100); //delay für den durchlauf
	
	/* **************************************Beginn des Programmes************************************************* */
	
	for (uint8_t i = 0; i < 75; i++)
	{
		data[i] = 0;
	}
	
	/* Dieser Switch-Case realisiert die Statemachine für die GUI des LCD */
	switch (state)
	{
		/* ****************************************** */
		/* **** Allgemeine Einstellungen treffen **** */
		/* ****************************************** */
		case GUI_COMMON_SETTINGS:
		{
			/* auf nächsten State schalten -> Einstellung der Schrrift */
			state = GUI_FONT_SETTINGS;
			
			/* Befehl erstellen -> */
			/* #DL ... Display löschen */
			/* #AS ... Summer (0 .. aus; 1 .. an) */
			/* #YH ... Display-Helligkeit [%] */
			/* #FD ... Display-Farbe einstellen (vf, hf) */
			/* #TC ... Cursor (0 .. aus; 1 .. ein) */
			/* #TA ... Terminal ausschalten */
			sprintf(data,"#AS,0,#YH,100,#FD,%d,%d,#TC,0,#TA,#DL",SCHWARZ,WEISS);
			/* String an Display schicken */
			EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
			/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
			return(DISABLE);
		}

		/* ********************************************** */
		/* **** Einstellungen zur Schriftart treffen **** */
		/* ********************************************** */
		case GUI_FONT_SETTINGS:
		{
			/* auf nächsten State schalten -> Einstellung der Schrrift */
			state = GUI_BARGRAPH_SETTINGS;

			/* Befehl erstellen -> */
			/* #FZ ... Farbe der Zeichenkette einstellen (vf, hf) */
			/* #ZF ... Schriftart einstellen (Font-Nummer -> siehe Datenblatt S. 20f) */
			/* #AF ... Schriftart für Touch-Button (Font-Nummer -> siehe Datenblatt S. 20f) */
			/* #AE ... Touch-Rahmen auswählen (nr. -> siehe Datenblatt S. 23; Rahmenwinkel (0 .. 0°; 1 .. 90°; 2 .. 180°; 3 .. 270°)) */
			/* #FA ... Schriftfarbe für Touch-Button (normal;selektiert) */
			sprintf(data,"#FZ,%d,%d,#ZF,%d,#AF,4,#AE,18,0,#FA,%d,%d,",SCHWARZ,TRANSPARENT,SWISS30,SCHWARZ,ROT);
			/* String an Display schicken */
			EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
			/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
			return(DISABLE);
		}

		/* ********************************************** */
		/* **** Einstellungen zur Schriftart treffen **** */
		/* ********************************************** */
		case GUI_BARGRAPH_SETTINGS:
		{
			/* auf nächsten State schalten -> Einstellung der Schrrift */
			state = GUI_START;
			
			/* Befehl erstellen -> */
			/* #BM ... Muster der Bargraphs (Bargraph-Nummer -> siehe Datenblatt S. 23) */
			/* #FB ... Farbe Bargraph (vf,hf,rf) */
			/* #BE ... Rahmentyp des Bargraph (Bargraph-Nummer -> siehe Datenblatt S. 23) */
			sprintf(data,"#BM,0,#FB,%d,%d,%d,#BE,114,",ROT,SCHWARZ,HELLGRAU);
			/* String an Display schicken */
			EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
			/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
			return(DISABLE);
		}

		/* ************************************ */
		/* **** EUROBOT Startbild anzeigen **** */
		/* ************************************ */
		case GUI_START:
		{
			/* auf nächsten State schalten -> 2 sec warten */
			state=GUI_WAIT;
			/* Befehl erstellen -> EUROBOT Bild an der Stelle 0/0 (links-oben Bündig) ausgeben */
			/* #UI ... Bild ausgeben (x,y,Bildnummer) */
			sprintf(data,"#UI,%d,%d,%d,",0,0,BILD_EUROBOT);
			/* String an Display schicken */
			EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
			/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
			return(DISABLE);
		}
		/* 2 sec warten -> Bild soll 2 sec angezeigt werden */
		case GUI_WAIT:
		{
			SET_CYCLE(GUI_TASKNBR, 2000);
			state=GUI_COLOUR;
			return(CYCLE);
		}
		
		/* ******************************************************** */
		/* **** Touch-Tasten für Farbwahl (gelb/blau) ausgeben **** */
		/* ******************************************************** */
		case GUI_COLOUR:
		{
			/* dieser switch-case dient zur Anzeiger der 2 Buttons zur Auswahl des Aufstellungsortes */
			switch(FarbAuswahl)
			{
				/* Anzeige löschen */
				case 0:
				{
					FarbAuswahl=1;
					/* Befehl erstellen -> */
					/* #AF ... Font für Touch-Buttons einstellen */
					/* #FA ... Beschriftungsfarbe für Touch-Buttons einstellen (nf,sf) */
					/* #FD ... Display-Farbe einstellen (vf, hf) */
					/* #DL ... Diplay löschen */
					sprintf(data,"#AF,%d,#FA,%d,%d,#FD,%d,%d,#DL",GENEVA10,SCHWARZ,ROT,SCHWARZ,WEISS);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Text "Spielfarbe:" ausgeben  */
				case 1:
				{
					FarbAuswahl=2;
					/* Befehl erstellen -> */
					/* #ZF ... Font für Display einstellen - Font mit der Nr. 6 auswählen (SWISS30B) */
					/* #FZ ... Beschriftungsfarbe für Display einstellen (vf,hf) - Schriftfarbe auf SCHWARZ - Hintergrund auf TRANSPARENT */
					/* #ZC ... Text an Position (x,y) ausgeben - Text an Stelle 160/20 setzen, Text: "Spielfarbe:" */
					sprintf(data,"#ZF,%d,#FZ,%d,%d,#ZC,%d,%d,Spielfarbe:",SWISS30,SCHWARZ,TRANSPARENT,160,5);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Farbe für Touch-Button einstellen -  Grün_1 */
				case 2:
				{
					FarbAuswahl=3;
					/* Befehl erstellen -> */
					/* #FE ... Touch-Button (Rahmen) einstellen (n1,n2,n3,s1,s2,s3) */
					/* n .. normal; s .. selektiert - 1 .. Rahmen aussen; 2 .. Rahmen Innen; 3 ..Füllung  */
					sprintf(data,"#FE,%d,%d,%d,%d,%d,%d,",SCHWARZ,HELLGRAU,SPIELFARBE_2,HELLGRAU,SCHWARZ,SPIELFARBE_2);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				/* Touch-Button für Spielfarbe  Grün_1 ausgeben */
				case 3:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=7;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",20,40,60,80,0,TOUCH_GRUEN_1);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				/* Farbe für Touch-Button einstellen - Blau_1 */
				case 4:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=5;
					/* Befehl erstellen -> */
					/* #FE ... Touch-Button (Rahmen) einstellen (n1,n2,n3,s1,s2,s3) */
					/* n .. normal; s .. selektiert - 1 .. Rahmen aussen; 2 .. Rahmen Innen; 3 ..Füllung  */
					sprintf(data,"#FE,%d,%d,%d,%d,%d,%d,",SCHWARZ,HELLGRAU,SPIELFARBE_1,HELLGRAU,SCHWARZ,SPIELFARBE_1);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				/* Touch-Button für Spielfarbe Blau_1 ausgeben */
				case 5:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=9;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",100,40,140,80,0,TOUCH_BLAU_1);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Touch-Button für Spielfarbe Grün_2 ausgeben */
				case 7:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=11;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",180,40,220,80,0,TOUCH_GRUEN_2);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				
				/* Touch-Button für Spielfarbe Blau_2 ausgeben */
				case 9:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=13;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",260,40,300,80,0,TOUCH_BLAU_2);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Touch-Button für Spielfarbe Grün_3 ausgeben */
				case 11:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=15;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",260,90,300,130,0,TOUCH_GRUEN_3);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Touch-Button für Spielfarbe Blau_3 ausgeben */
				case 13:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=17;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",260,140,300,180,0,TOUCH_BLAU_3);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Touch-Button für Spielfarbe Grün_4 ausgeben */
				case 15:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=19;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",260,190,300,230,0,TOUCH_GRUEN_4);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Touch-Button für Spielfarbe Blau_4 ausgeben */
				case 17:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=21;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",180,190,220,230,0,TOUCH_BLAU_4);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Touch-Button für Spielfarbe Grün_5 ausgeben */
				case 19:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=4;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",100,190,140,230,0,TOUCH_GRUEN_5);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Touch-Button für Spielfarbe Blau_5 ausgeben */
				case 21:
				{
					FarbAuswahl=23;
					FarbAuswahl_Next=22;
					/* Befehl erstellen -> */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",20,190,60,230,0,TOUCH_BLAU_5);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				
				/* Weiter mit: Spielfarbenauswahl abwarten */
				case 22:
				{
					FarbAuswahl=0;
					SpielFarbe=0;
					FarbAuswahl_Next=0;
					state=GUI_COLOUR_WAIT;
					break;
				}
				case 23:
				{
					FarbAuswahl=FarbAuswahl_Next;
					SET_CYCLE(GUI_TASKNBR,50);
					break;
				}
			}
			break;
		}
		/* ********************************************************* */
		/* **** Touch-Tasten für Farbwahl (gelb/blau) auswerten **** */
		/* ********************************************************* */
		case GUI_COLOUR_WAIT:
		{
			/* Spielfarbe GELB wurde ausgewählt */
			if((lastTouchEvent == TOUCH_GRUEN_1) || (lastTouchEvent == TOUCH_GRUEN_2) || (lastTouchEvent == TOUCH_GRUEN_3) || (lastTouchEvent == TOUCH_GRUEN_4) || (lastTouchEvent == TOUCH_GRUEN_5))
			{
				/* weiter zu STRATEGIE-Auswahl */
				state = GUI_STRATAGY;
				/* Spielfarbe auf GELB setzen */
				SpielFarbe = lastTouchEvent;
				/* Befehl erstellen -> */
				/* #FE ... Touch-Button (Rahmen) einstellen (n1,n2,n3,s1,s2,s3) - Touch-Button auf GRAU setzen */
				/* #FD ... Display-Farbe einstellen (vf, hf) - Hintergrundfarbe auf GELB setzen */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#FE,%d,%d,%d,%d,%d,%d,#FD,%d,%d,#AL,0,1,#DL",SCHWARZ,DUNKELGRAU,HELLGRAU,DUNKELGRAU,SCHWARZ,HELLGRAU,SCHWARZ,SPIELFARBE_2);
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			/* Spielfarbe BLAU wurde ausgewählt */
			else if ((lastTouchEvent == TOUCH_BLAU_1)||(lastTouchEvent == TOUCH_BLAU_2)||(lastTouchEvent == TOUCH_BLAU_3)||(lastTouchEvent == TOUCH_BLAU_4) ||(lastTouchEvent == TOUCH_BLAU_5))
			{
				/* weiter zu STRATEGIE-Auswahl */
				state = GUI_STRATAGY;
				/* Spielfarbe auf BLAU setzen */
				SpielFarbe = lastTouchEvent;
				/* Befehl erstellen -> */
				/* #FE ... Touch-Button (Rahmen) einstellen (n1,n2,n3,s1,s2,s3) - Touch-Button auf GRAU setzen */
				/* #FD ... Display-Farbe einstellen (vf, hf) - Hintergrundfarbe auf BLAU setzen */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#FE,%d,%d,%d,%d,%d,%d,#FD,%d,%d,#AL,0,1,#DL",SCHWARZ,DUNKELGRAU,HELLGRAU,DUNKELGRAU,SCHWARZ,HELLGRAU, WEISS, SPIELFARBE_1);
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}

			break;
		}

		/* ***************************************************** */
		/* **** Touch-Tasten für Strategie-Auswahl ausgeben **** */
		/* ***************************************************** */
		case GUI_STRATAGY:
		{
			/* dieser switch-case dient zur Anzeiger der Buttons zur Auswahl der Strategie */
			switch(StrategieAuswahl)
			{
				/* Text "Strategie:" ausgeben */
				case 0:
				{
					StrategieAuswahl=1;
					/* Befehl erstellen -> */
					/* #FZ ... Beschriftungsfarbe für Display einstellen (vf,hf) - Schriftfarbe auf SCHWARZ/WEISS - Hintergrund auf TRANSPARENT */
					/* #ZC ... Text an Position (x,y) ausgeben - Text an Stelle 160/20 setzen, Text: "Strategie:" */
					sprintf(data,"#FZ,%d,%d,#ZC,%d,%d,Strategie:",SET_TEXT_COLOUR(SpielFarbe),TRANSPARENT,160,20);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Touch-Button für Strategie "PASSIVE" ausgeben */
				case 1:
				{
					StrategieAuswahl=2;
					/* Bezeichnung der ersten Strategie setzen */
					sprintf(textStrategie,TEXT_STRATEGIE_1);
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",10,60,155,95,0,TOUCH_STRATEGY_PASSIVE);
					/* Strategiename an Meldung anhängen */
					strcat(data, textStrategie);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Touch-Button für Strategie "ACTIVE" ausgeben */
				case 2:
				{
					StrategieAuswahl=3;
					/* Bezeichnung der ersten Strategie setzen */
					sprintf(textStrategie,TEXT_STRATEGIE_2);
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",10,105,155,140,0,TOUCH_STRATEGY_ACTIVE);
					/* Strategiename an Meldung anhängen */
					strcat(data, textStrategie);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Touch-Button für Strategie "SPECIAL 1" ausgeben */
				case 3:
				{
					StrategieAuswahl=4;
					/* Bezeichnung der ersten Strategie setzen */
					sprintf(textStrategie,TEXT_STRATEGIE_3);
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",10,150,155,185,0,TOUCH_STRATEGY_SPECIAL1);
					/* Strategiename an Meldung anhängen */
					strcat(data, textStrategie);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Touch-Button für Strategie "SPECIAL 2" ausgeben */
				case 4:
				{
					StrategieAuswahl=5;
					/* Bezeichnung der ersten Strategie setzen */
					sprintf(textStrategie,TEXT_STRATEGIE_4);
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,",10,195,155,230,0,TOUCH_STRATEGY_SPECIAL2);
					/* Strategiename an Meldung anhängen */
					strcat(data, textStrategie);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Touch-Button für Strategie "HOMOLOGATION" ausgeben */
				case 5:
				{
					StrategieAuswahl=6;
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,HOMOLOGATION",165,60,310,95,0,TOUCH_STRATEGY_HOMOLOGATION);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Touch-Button für Strategie "ENEMY" ausgeben */
				case 6:
				{
					StrategieAuswahl=7;
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,ENEMY",165,105,310,140,0,TOUCH_STRATEGY_ENEMY);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Touch-Button für Strategie "Perimeter" ausgeben */
				case 7:
				{
					StrategieAuswahl=8;
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AT,%d,%d,%d,%d,%d,%d,PERIMETER",165,150,310,185,0,TOUCH_STRATEGY_PERIMETER);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Weiter mit: Strategie abwarten */
				case 8:
				{
					StrategieAuswahl = 0;
					Strategie = 0;
					state = GUI_STRATAGY_WAIT;
				}
			}
			break;
		}
		/* ********************************************** */
		/* **** Touch-Tasten für Strategie auswerten **** */
		/* ********************************************** */
		case GUI_STRATAGY_WAIT:
		{
			/* Strategie PASSIV wurde ausgewählt */
			if(lastTouchEvent == TOUCH_STRATEGY_PASSIVE)
			{
				/* weiter zur HAUPTSEITE */
				state = GUI_HAUPTSEITE;
				/* Strategie PASSIV setzen */
				Strategie = TOUCH_STRATEGY_PASSIVE;
				/* Befehl erstellen -> */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#AL,0,1,#DL");
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			/* Strategie ACTIVE wurde ausgewählt */
			else if(lastTouchEvent == TOUCH_STRATEGY_ACTIVE)
			{
				/* weiter zur HAUPTSEITE */
				state = GUI_HAUPTSEITE;
				/* Strategie ACTIVE setzen */
				Strategie = TOUCH_STRATEGY_ACTIVE;
				/* Befehl erstellen -> */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#AL,0,1,#DL");
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			/* Strategie SPEZIAL 1 wurde ausgewählt */
			else if(lastTouchEvent == TOUCH_STRATEGY_SPECIAL1)
			{
				/* weiter zur HAUPTSEITE */
				state = GUI_HAUPTSEITE;
				/* Strategie SPEZIAL 1 setzen */
				Strategie = TOUCH_STRATEGY_SPECIAL1;
				/* Befehl erstellen -> */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#AL,0,1,#DL");
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			/* Strategie SPEZIAL 2 wurde ausgewählt */
			else if(lastTouchEvent == TOUCH_STRATEGY_SPECIAL2)
			{
				/* weiter zur HAUPTSEITE */
				state = GUI_HAUPTSEITE;
				/* Strategie SPEZIAL 2 setzen */
				Strategie = TOUCH_STRATEGY_SPECIAL2;
				/* Befehl erstellen -> */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#AL,0,1,#DL");
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			/* Strategie HOMOLOGATION wurde ausgewählt */
			else if(lastTouchEvent == TOUCH_STRATEGY_HOMOLOGATION)
			{
				/* weiter zur HAUPTSEITE */
				state = GUI_HAUPTSEITE;
				/* Strategie HOMOLAGATION setzen */
				Strategie = TOUCH_STRATEGY_HOMOLOGATION;
				/* Befehl erstellen -> */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#AL,0,1,#DL");
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			/* Strategie ENEMY wurde ausgewählt */
			else if(lastTouchEvent == TOUCH_STRATEGY_ENEMY)
			{
				/* weiter zur HAUPTSEITE */
				state = GUI_HAUPTSEITE;
				/* Strategie ENEMY setzen */
				Strategie = TOUCH_STRATEGY_ENEMY;
				/* Befehl erstellen -> */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#AL,0,1,#DL");
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			/* Strategie ENEMY wurde ausgewählt */
			else if(lastTouchEvent == TOUCH_STRATEGY_PERIMETER)
			{
				/* weiter zur HAUPTSEITE */
				state = GUI_HAUPTSEITE;
				/* Strategie ENEMY setzen */
				Strategie = TOUCH_STRATEGY_PERIMETER;
				/* Befehl erstellen -> */
				/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
				/* #DL ... Diplay löschen */
				sprintf(data,"#AL,0,1,#DL");
				/* String an Display schicken */
				EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
				/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
				return(DISABLE);
			}
			break;
		}

		/* *********************************************************** */
		/* Anzeige der Hauptseite mit Menü, Speilezeit und Punktestand */
		/* *********************************************************** */
		case GUI_HAUPTSEITE:
		{
			/* dieser switch-case dient zur Anzeiger des Hauptfensters */
			switch (caseHauptseite)
			{
				/* Textfarbe setzen */
				case 0:
				{
					caseHauptseite = 1;
					/* Befehl erstellen -> */
					/* #FZ ... Beschriftungsfarbe für Display einstellen (vf,hf) - Schriftfarbe auf SCHWARZ/WEISS - Hintergrund auf TRANSPARENT */
					/* #DL ... Diplay löschen */
					sprintf(data,"#FZ,%d,%d,#DL",SET_TEXT_COLOUR(SpielFarbe),TRANSPARENT);
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* ausgewählte Strategie ausgeben */
				case 1:
				{
					caseHauptseite = 2;
					/* Befehl erstellen -> */
					/* #ZC ... Text an Position (x,y) ausgeben - Text an Stelle 160/20 setzen, Text: <Strategie> */
					if (Strategie == TOUCH_STRATEGY_PASSIVE) {sprintf(data,"#ZC,%d,%d,",180,20); sprintf(textStrategie,TEXT_STRATEGIE_1); strcat(data, textStrategie);}
					else if (Strategie == TOUCH_STRATEGY_ACTIVE) {sprintf(data,"#ZC,%d,%d,",180,20); sprintf(textStrategie,TEXT_STRATEGIE_2); strcat(data, textStrategie);}
					else if (Strategie == TOUCH_STRATEGY_SPECIAL1) {sprintf(data,"#ZC,%d,%d,",180,20); sprintf(textStrategie,TEXT_STRATEGIE_3); strcat(data, textStrategie);}
					else if (Strategie == TOUCH_STRATEGY_SPECIAL2) {sprintf(data,"#ZC,%d,%d,",180,20); sprintf(textStrategie,TEXT_STRATEGIE_4); strcat(data, textStrategie);}
					else if (Strategie == TOUCH_STRATEGY_HOMOLOGATION) sprintf(data,"#ZC,%d,%d,HOMOLOGATION",180,20);
					else if (Strategie == TOUCH_STRATEGY_ENEMY) sprintf(data,"#ZC,%d,%d,ENEMY",180,20);
					else if (Strategie == TOUCH_STRATEGY_PERIMETER) sprintf(data,"#ZC,%d,%d,PERIMETER",180,20);
					else sprintf(data,"#ZC,%d,%d,*******",180,20);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				/* Exit-Button ausgeben */
				case 2:
				{
					caseHauptseite = 3;
					/* Befehl erstellen -> */
					/* #AF ... Font für Touch-Buttons einstellen */
					/* #FA ... Beschriftungsfarbe für Touch-Buttons einstellen (nf,sf) */
					/* #FE ... Touch-Button (Rahmen) einstellen (n1,n2,n3,s1,s2,s3) - Touch-Button auf GRAU setzen */
					/* #AT ... Touch-Button ausgbeben (xx1,yy1,xx2,yy2,dowCod,upCod,Text) */
					sprintf(data,"#AF,%d,#FA,%d,%d,#FE,%d,%d,%d,%d,%d,%d,#AT,%d,%d,%d,%d,%d,%d,x",SWISS30,WEISS,SCHWARZ,SCHWARZ,DUNKELGRAU,ROT,DUNKELGRAU,SCHWARZ,ROT, 10,10,52,50,0,TOUCH_AUSWAHL);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				/* Bargraph für Spielzeit ausgeben ausgeben */
				case 3:
				{
					caseHauptseite = 4;
					/* Befehl erstellen -> */
					/* #BR ... Bargraph einstellen (n1,xx1,yy1,xx2,yy2,aw,ew,typ) - (Bargraph -> siehe Datenblatt S. 23) */
					/* #BA ... Bargraph (mit der Nummer n1) aktualisieren */
					sprintf(data,"#BR,%d,%d,%d,%d,%d,%d,%d,%d,#BA,%d,%d,",10,20,200,300,220,0,100,5,10,100);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
				}
				/* Text umstellen und Punkte ausgeben */
				case 4:
				{
					caseHauptseite = 5;
					/* Befehl erstellen -> */
					/* #FZ ... Beschriftungsfarbe für Display einstellen (vf,hf) - Schriftfarbe auf SCHWARZ/WEISS - Hintergrund auf TRANSPARENT */
					/* #ZF ... Font für Display einstellen - Font mit der Nr. 8 auswählen (BIGZIFF100) */
					/* #ZC ... Text an Position (x,y) ausgeben - Text an Stelle 160/80 setzen, Text: <Punkte> */
					sprintf(data,"#FZ,%d,%d,#ZF,%d,#ZC,%d,%d,%03d",SET_TEXT_COLOUR(SpielFarbe),TRANSPARENT,BIGZIFF100,160,80,Punkte);
					/* String mit einer Null abschließen */
					data[strlen(data)] = 0;
					/* String ans Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data)+1);
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return (DISABLE);
				}
				
				/* kurze Wartezeit um das Bild nicht andauernt aufzubauen */
				case 5:
				{
					caseHauptseite = 6;
					SET_CYCLE(GUI_TASKNBR, 100);
					return (CYCLE);
				}
				/* Touch bereich löschen sowie den Bereich mit der Zeit. Anschließendes verlassen der Hauptseite bzw. erneuter Aufbau. */
				case 6:
				{
					caseHauptseite = 7;
					/* Befehl erstellen -> */
					/* #BA ... Bargraph (mit der Nummer n1) aktualisieren */
					sprintf(data,"#BA,%d,%d,",10,SpielZeit);
					/* String an Display schicken */
					EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
					/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
					return(DISABLE);
					
				}

				/* Touch bereich löschen sowie den Bereich mit der Zeit. Anschließendes verlassen der Hauptseite bzw. erneuter Aufbau. */
				case 7:
				{
					/* zurück zur FARB- und STRATEGIE-Auswahl */
					if (lastTouchEvent == TOUCH_AUSWAHL)
					{
						caseHauptseite = 0;
						state = GUI_COLOUR;
						/* Befehl erstellen -> */
						/* #AL ... löscht Touch-Bereich (Cod,n1) - Cod = 0 .. alle Touch-Return-Codes, n1 = 1 .. Touch-Bereiche löschen */
						/* #DL ... Diplay löschen */
						sprintf(data,"#AL,0,1,#DL");
						/* String an Display schicken */
						EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
						/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
						return(DISABLE);
					}
					else
					{
						caseHauptseite = 4;
						/* wenn sich die Punkte ändern -> Bereich löschen */
						if (Punkte != PunkteAlt)
						{
							/* Punktestand übernehmen */
							PunkteAlt = Punkte;
							/* Befehl erstellen -> */
							/* #RL ... Bereich löschen (xx1,yy1,xx2,yy2) */
							sprintf(data,"#RL,%d,%d,%d,%d,",0,80,320,180);
							/* String ans Display schicken */
							EA_eDIPTFT_SendMessageDC1(&EA_eDIPTFT_Display, GUI_TASKNBR, data, strlen(data));
							/* State-Machine disabeln -> warten bis OK von Display empfangen wird */
							return(DISABLE);
						}
						return (ENABLE);
					}
				}
			}
			break;
		}
	}
	return(CYCLE);
}

