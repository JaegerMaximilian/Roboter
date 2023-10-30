/*
 * GUI.h
 *
 * Created: 10.05.2019 11:43:44
 *  Author: schnadi
 */ 


#ifndef GUI_H_
#define GUI_H_

#include "EA_eDIPTFT.h"

#ifndef _GUI_EXTERN_
		#define _GUI_EXTERN_ extern
#endif


/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
/* ***************** */
/* States in der GUI */
/* ***************** */
#define GUI_COMMON_SETTINGS			0
#define GUI_FONT_SETTINGS			1
#define GUI_BARGRAPH_SETTINGS		2

#define GUI_START					110
#define GUI_WAIT					111

#define GUI_COLOUR					120
#define GUI_COLOUR_WAIT				121

#define GUI_STRATAGY				130
#define GUI_STRATAGY_WAIT			131

#define GUI_HAUPTSEITE				140


/* ************ */
/* TOUCH-Tasten */
/* ************ */
#define TOUCH_EINSTELLUNGEN		7
#define TOUCH_AUSWAHL			8

#define AUS		0
#define EIN		1

#define TOUCH_GRUEN_1					21
#define TOUCH_GRUEN_2					22
#define TOUCH_GRUEN_3					23
#define TOUCH_GRUEN_4					24
#define TOUCH_GRUEN_5					25

#define TOUCH_BLAU_1					31
#define TOUCH_BLAU_2					32
#define TOUCH_BLAU_3					33
#define TOUCH_BLAU_4					34
#define TOUCH_BLAU_5					35

#define TOUCH_STRATEGY_PASSIVE		50
#define TOUCH_STRATEGY_ACTIVE		51
#define TOUCH_STRATEGY_SPECIAL1		52
#define TOUCH_STRATEGY_SPECIAL2		53
#define TOUCH_STRATEGY_ENEMY		54
#define TOUCH_STRATEGY_HOMOLOGATION	55
#define TOUCH_STRATEGY_PERIMETER	56

/* ****** */
/* Bilder */
/* ****** */
#define BILD_EUROBOT					1
#define BILD_EUROBOT_OBEN				2
#define BILD_HINTERGRUND				3
#define BILD_MENU_BUTTON				4
#define BILD_MENU_OFFEN					5
#define BILD_KOORDINATENDARSTELLUNG		6
#define BILD_GRAFISCHEDARSTELLUNG		7
#define BILD_BEIDEDARSTELLUNGEN			8
#define BILD_EINSTELLUNGEN				9
#define BILD_EINSTELLUNGEN_HELLIGKEIT	10
#define BILD_MENU_RAHMEN				11
#define BILD_EINSTELLUNGEN_RAHMEN		12
#define BILD_EINSTELLUNGEN_UHRZEIT		13
#define BILD_ROBOTER1					14
#define BILD_ROBOTER2					15
#define BILD_GEGNER1					16
#define BILD_GEGNER2					17
#define BILD_BEACON_TEAM_G				18
#define BILD_BEACON_TEAM_V				19
#define BILD_EUROBOT_HINTERGRUND		20
#define BILD_LIDAR_EINSTELLUNGEN		21
#define BILD_BEDIENUNGSHILFE			22
#define BILD_UEBER_BEACON				23

/* ************ */
/* Schriftarten */
/* ************ */
#define MONOSPACE_4X6		1
#define MONOSPACE_6X8		2
#define MONOSPACE_7X12		3
#define GENEVA10			4
#define CHICAGO14			5
#define SWISS30				6
#define BIGZIFF50			7
#define BIGZIFF100			8


/* ****** */
/* Farben */
/* ****** */
#define TRANSPARENT		0
#define SCHWARZ			1
#define BLAU			2
#define ROT				3
#define GRUEN			4
#define MAGENTA			5
#define HIMMELBLAU		6
#define GELB			7
#define WEISS			8
#define DUNKELGRAU		9
#define ORANGE			10
#define VIOLETT			11
#define ROSA			12
#define TUERKIS			13
#define HELLGRUEN		14
#define HELLBLAU		15
#define HELLGRAU		16


/* ********************** */
/* globale Variablen für: */
/* ********************** */
/* Spielfarbe, Strategie ... werden zum Hauptprozessor geschickt */
/* aktuelle Speilzeit, aktulle Punkte ... werden vom Hauptprozessor enpfangen */
_GUI_EXTERN_ uint8_t SpielFarbe, Strategie, SpielZeit, Punkte;


/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void Init_GUI(void);
uint8_t GUI_Task(void);


#endif /* GUI_H_ */