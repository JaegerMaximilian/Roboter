/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      GENTRAJ.h
Version :  V 1.0
Date    :  03.03.2010
Author  :  MICHAEL ZAUNER

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : Xmega256A3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1500                

               Copyright (c) 2011 by FH-Wels                          
                   All Rights Reserved.
****************************************************************/

#ifndef _GENTRAJ_ANTRIEB_H
#define _GENTRAJ_ANTRIEB_H



#ifndef _GENTRAJ_ANTRIEB_EXTERN
   #define _GENTRAJ_ANTRIEB_EXTERN extern
#endif


/**************************************************************************
***                      Variablen-Definition                           ***
**************************************************************************/
#define _MOTION_READY_       0
#define _UP_SLOPE_           1
#define _DOWN_SLOPE_         2
#define _MAX_SPEED_          3      
#define _MOTION_INTERRUPT_   5
#define _NEW_MOTION_         4
#define _CLOTHOID_1_			  6
#define _CLOTHOID_2_			  7

#define _POS_REL           0
#define _POS_ABS           1
#define _TURN_REL          2
#define _TURN_ABS          3
#define _KREISBOGEN        4 
#define _CLOTHOID			5

// Zeit in der die Bewegung abgebrochen werden soll (in sec)
#define T_INTERRUPT     0.5

// maximales Verhältnis zwischen Geschwindigkeit und Weg 
#define SPEED_LIMIT_FACTOR_    2.5 //3.0	// 1.8 // 1.2
// Faktor zwischen Winkelgeschwindigkeit und Winkel
#define K_WINKEL_             1.0 //1.5 //0.7 // 2.0 // 1.2

// Rampenlimit (gibt an wie groß - in Prozent - die Rampe maximal werden darf)
#define RAMP_LIMIT_			 0.9

// min. Upslopezeit (in sec)
#define T_UPSLOPE_MIN_      0.001
// min. Downslopezeit (in sec)
#define T_DOWNSLOPE_MIN_    0.001
// min. max. Zeit (in sec.)
#define T_MAX_MIN_          0.001

// Abtastzeit
#define T_ABTAST_				0.006

// Wert für automatischen Up- bzw. Downslope
#define T_UP_AUTO_          10000.0
#define T_DOWN_AUTO_        10000.0
#define T_UD_AUTO_GRENZE_   9999.0
// Wert für Up- bzw. Downslope ausgeschaltet
#define T_UP_AUS_           0.0
#define T_DOWN_AUS_         0.0
#define T_UD_AUS_GRENZE_    0.0001

// Schleppfehler (translatorisch 20 cm)
#define SCHLEPP_FEHLER_TRANS_     0.2 //0.4 original - 0.05 für Hexagons
// Schleppfehler (rotatorisch 60°) 
#define SCHLEPP_FEHLER_ROT_       M_PI/3

// minimale Geschwindigkeit
#define V_MIN           0.05
// minimale Winkelgeschwindigkeit
#define W_MIN				(M_PI*10.0)/180.0
#define W_MAX			 	(M_PI*135.0)/180.0  //(M_PI*250.0)/180.0		//(M_PI*135.0)/180.0

// max. Motion (Calc)
#define MAX_CALC_ANTRIEB     50

// max. Motion (Eingabe)
#define MAX_IN_ANTRIEB       50

// Umwandlerfunktion DEG -> RAD und RAD -> DEG
#define DEG2RAD(a) ((float)(a) * M_PI / 180.0)
#define RAD2DEG(a) ((float)(a) * 180.0 / M_PI)

// *******************************************
// Eingangsparameter (vom Hauptprozessor)  
// *******************************************
typedef struct
{
   float fTus;
   float fTds;
   float fVstart;
   float fVend;
   float fVmax;
   float fPhiSoll;
   float fRsoll;
   float fXsoll;
   float fYsoll;
   float fSsoll;
   unsigned char ucType;
   unsigned char gegnerErkennung;
	uint8_t destinationPointsCount; 
}t_paramInputAntrieb;

// *******************************************
// Ausgangsparameter (zum Regler) 
// *******************************************
typedef struct
{
   float fX;
   float fY;
   float fSn;
   float fSn_1;
   float fVx;
   float fVy;
   float fVn;
	float fVn_1;
   float fPhi;
   float fPhin;
   float fPhin_1;
   float fW;
   float fWn;
}t_paramOutputAntrieb;

// *******************************************
// Werte für die Berechnung  
// *******************************************
typedef struct
{
   float fTus;
   float fTds;
   float fTmax;
   float fVstart;
   float fVend;
   float fVmax;
   float fWstart;
   float fWend;
   float fWmax;
   float fSold;
   float fSI;
   float fSII;
   float fS0;
   float fVold;
   float fPhin;
   float fPhiOld;
   float fPhiI;
   float fPhiII;
   float fPhi0;
   float fWold;
	uint8_t clothoid;
	float A;
	float T;
	float s_star;
	float alpha;	
	float K_W;
   unsigned char gegnerErkennung;
}t_paramCalcAntrieb;

// *******************************************
// allgemeine Parameter  
// *******************************************
typedef struct
{
   float fTinterrupt;
   float fT; 
   float fAmaxUs; 
   float fAmaxDs;
   float fAlphaMaxUs;
   float fAlphaMaxDs;
   unsigned char ucState;
   unsigned char ucInitMotionInterrupt;
}t_paramComAntrieb;

typedef struct  
{
	float x;
	float y;
}t_trajPoints;


// allgemeine Parameter für die Trajektorienberechnung
_GENTRAJ_ANTRIEB_EXTERN t_paramComAntrieb paramComAntrieb;
// Rechenparameter für die Trajektorienberechnung
_GENTRAJ_ANTRIEB_EXTERN t_paramCalcAntrieb paramCalcAntrieb[MAX_CALC_ANTRIEB];
// Ausgabewerte für den Regler
_GENTRAJ_ANTRIEB_EXTERN t_paramOutputAntrieb paramOutputAntrieb;
// Eingabewerte vom Hauptprozessor
_GENTRAJ_ANTRIEB_EXTERN t_paramInputAntrieb paramInputAntrieb[MAX_IN_ANTRIEB];

_GENTRAJ_ANTRIEB_EXTERN t_trajPoints trayPoints[15];

// Index auf die Rechenparameter
_GENTRAJ_ANTRIEB_EXTERN signed char indexCalcInAntrieb;
_GENTRAJ_ANTRIEB_EXTERN signed char indexCalcOutAntrieb;
_GENTRAJ_ANTRIEB_EXTERN signed char indexCalcAntrieb;

// Index auf die Rechenparameter
_GENTRAJ_ANTRIEB_EXTERN signed char indexInputInAntrieb;
_GENTRAJ_ANTRIEB_EXTERN signed char indexInputOutAntrieb;
_GENTRAJ_ANTRIEB_EXTERN signed char indexInputAntrieb;

_GENTRAJ_ANTRIEB_EXTERN uint8_t motionIR_Received;

/**************************************************************************
***                      Prototypen-Definition                           ***
**************************************************************************/
void genTrajAntrieb(void);
float getAngle(void);
                    


#endif