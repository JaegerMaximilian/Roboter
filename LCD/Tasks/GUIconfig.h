/*
 * GUIconfig.h
 *
 * Created: 20.03.2020 08:46:06
 *  Author: P20087
 */ 


#ifndef GUICONFIG_H_
#define GUICONFIG_H_

#include "GUI.h"

/* ***************** */
/* Spielfarbe setzen */
/* ***************** */
#define SPIELFARBE_1		BLAU
#define SPIELFARBE_2		GRUEN

/* ******************* */
/* Schriftfarbe setzen */
/* ******************* */
#define SET_TEXT_COLOUR(a) (((a == TOUCH_GRUEN_1) || (a == TOUCH_GRUEN_2) || (a == TOUCH_GRUEN_3) || (a == TOUCH_GRUEN_4) || (a == TOUCH_GRUEN_5)) ? SCHWARZ : WEISS)

/* *************************** */
/* Namen der Strategien setzen */
/* *************************** */
#define TEXT_STRATEGIE_1		"PASSIVE"
#define TEXT_STRATEGIE_2		"ACTIVE"
#define TEXT_STRATEGIE_3		"SPECIAL 1"
#define TEXT_STRATEGIE_4		"SPECIAL 2"


#endif /* GUICONFIG_H_ */