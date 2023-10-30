/***************************************************************

************************************
** FH OBEROESTERREICH CAMPUS WELS **
************************************

Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      ANTRIEB.c
Version :  V 1.0
Date    :  07.03.2011
Author  :  ZAUNER MICHAEL

Comments:

Last edit:
Programmchange:

*)....
*).....

Chip type           : XMega256A3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024

Copyright (c) 2011 by FH-Wels
All Rights Reserved.
****************************************************************/

#define _ANTRIEB_EXTERN


#include <avr/io.h>
#include "multitask.h"
#include "ports.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "timer.h"
#include "antrieb.h"
#include "qdec_driver.h"
#include "WMRctrlVelPosOdo0.h"         /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "genTrajAntrieb.h"
#include "timer.h"
#include "sensor.h"
#include "rrt_receivedata.h"
#include "accumulator.h"
#include "rrt_transmittingtask.h"
#include "global.h"

//#define _DEBUG_INCS_
//#define  _DEBUG_GEN_TRTAJ_


#define _LEFT_   0
#define _RIGHT_  1
#define _SPEED_  0
#define _OMEGA_  1

// Motoren freigeben/bremsen
// Motor1 -> LINKS
#define MOTOR_1_ENABLE     {PORTA.OUT |= 0x30;}
#define MOTOR_1_BRAKE      {PORTA.OUT &= ~0x30;}
// Motor2 -> RECHTS
#define MOTOR_2_ENABLE     {PORTA.OUT |= 0xC0;}
#define MOTOR_2_BRAKE      {PORTA.OUT &= ~0xC0;}


#define CHECK_ROBOT_TYPE(b, c) ((RobotType_RAM == MASTER_ROBOT) ? (b) : (c))

unsigned char ucStop;
signed long slIncrementMotor1;
signed long slIncrementMotor2;
signed long slIncrementMotorRight_Messrad;
signed long slIncrementMotorLeft_Messrad;

#define GET_LEFT_INCS   slIncrementMotor2
#define GET_RIGHT_INCS  slIncrementMotor1

static BlockIO_WMRctrlVelPosOdo0 WMRctrlVelPosOdo0_B;				/* Observable signals */
static D_Work_WMRctrlVelPosOdo0 WMRctrlVelPosOdo0_DWork;			/* Observable states */
static ExternalInputs_WMRctrlVelPosOdo WMRctrlVelPosOdo0_U;		/* External inputs */
static ExternalOutputs_WMRctrlVelPosOd WMRctrlVelPosOdo0_Y;		/* External outputs */

/**************************************************************************
***   Funktionsname:     Init_WMRctrlVelPosOdo0                         ***
***   Funktion:          initialisiert Regler und Odometrie             ***
***   Übergabe-Para.:    Keine                                          ***
***   Rückgabe-Para.:    Keine                                          ***
***   Erstellt:          Zauner Michael (30-06-2010)                    ***
***   Änderungen:                                                       ***
**************************************************************************/
void Init_WMRctrlVelPosOdo0(void)
{
	PCtrlOdo.PEncWhl_maxCntr = 2147483647;
	PCtrlOdo.PCtrl_DynKmDRa = 2.89189189E-002;	// DCX26 (Km ... 21.4 mNm/A, Ra ... 0.74 Ohm) => Km/Ra
	PCtrlOdo.PCtrl_DynN = 26.0;              				// Übersetzung des Getriebes => Ablesen vom Getriebe des Motors
	PCtrlOdo.PCtrl_DynR = 0.0405; //0.0405							// Radius vom Antriebsreifen (in m)
	PCtrlOdo.PCtrl_DynL = 0.1808;								// Abstand zwischen den Antriebsreifen (in m)
	PCtrlOdo.PCtrl_PosKAntiWind = 0.0;
	
	// erster Wert = Master
	PCtrlOdo.PCtrl_PosKI = CHECK_ROBOT_TYPE(1.0, 1.0); //1.0
	PCtrlOdo.PCtrl_PosKp = CHECK_ROBOT_TYPE(20.0, 20.0);	//20		// Positionsregler (Wenn bei Kreisdrehung Positionsverlust eintritt)
	
	PCtrlOdo.PCtrl_PosLr = 0.5;
	PCtrlOdo.PCtrl_PosUsat[0] = 1.0E+005;
	PCtrlOdo.PCtrl_PosUsat[1] = 1.0E+005;
	PCtrlOdo.PCtrl_Ts1 = 0.006;
	PCtrlOdo.PCtrl_VelKAntiWind = 2.0;
	
	PCtrlOdo.PCtrl_VelKI[0] = CHECK_ROBOT_TYPE(15.0,15.0); //15		// I-Anteil translatorisch
	PCtrlOdo.PCtrl_VelKI[1] = CHECK_ROBOT_TYPE(8.0,8.0); //8		// I-Anteil rotatorisch (Drehung)
	
	// erster Wert = Master
	PCtrlOdo.PCtrl_VelKp[0] = CHECK_ROBOT_TYPE(3.0, 3.0); //3.0    	// P-Anteil translatorisch
	PCtrlOdo.PCtrl_VelKp[1] = CHECK_ROBOT_TYPE(3.0, 3.0); //3.0   	// P-Anteil rotatorisch (Drehung)
	
	PCtrlOdo.PCtrl_VelUsat = 22.0;
	PCtrlOdo.PCtrl_VelnormPlantGains[0] = 1.643808E+002;
	PCtrlOdo.PCtrl_VelnormPlantGains[1] = 2.3642;
	PCtrlOdo.PEncWhl_IperRev = 20000.0;
	// Wenn bei Kreisdrehung Drehungsverlust eintritt (Sollwert bei 3 Umdrehungen: 360° * 3 = 1080)
	// => Roboter z.B. 15° zuweit dreht  => 1095 => 1080/1095 = 0.986301369 => Multiplikation
	// => Roboter z.B. 15° zuwenig dreht => 1065 => 1080/1065 = 1.014084507 => Multiplikation
	
	// erster Wert = Master
	PCtrlOdo.PEncWhl_l =  CHECK_ROBOT_TYPE(0.108*(1800.0/1798.0)*(1800.0/1820.0) ,0.1136*(1800.0/1810.0));	// Abstand zwischen den Messrädern
	
	// Wenn der Roboter zuweit rechts ist => linkes Messrad kleiner oder rechtes Messrad größer
	// Wenn der Roboter zuweit links ist  => linkes Messrad größer  oder rechtes Messrad kleiner
	// GRUNDLEGEND GILT: Durchmesser der Messräder größer => zurückgelegte Strecke kleiner
	
	// erster Wert = Master
	PCtrlOdo.PEncWhl_rl[0] = CHECK_ROBOT_TYPE((0.02052873919 * 1.01170 * 0.984), 0.02053 * (1396.4/1400.0));	// Messrad rechts 0.000089494644, 0.0000530494644
	PCtrlOdo.PEncWhl_rl[1] = CHECK_ROBOT_TYPE((0.02052873919 * 1.01025 * 0.9855), 0.02053  * (1396.4/1400.0));	// Messrad links, 0.0000306589438, 0.0000696589438	

	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.P0_XYdXdYThdTh_desired[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.P0_XYdXdYThdTh_desired[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.P0_XYdXdYThdTh_desired[2] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.P0_XYdXdYThdTh_desired[3] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.P0_XYdXdYThdTh_desired[4] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.P0_XYdXdYThdTh_desired[5] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.encLR[0] = 0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.encLR[1] = 0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.swReset = FALSE;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.x0y0theta0[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.x0y0theta0[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSinWMRctrl.x0y0theta0[2] = 0.0;

	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.errorFlag = FALSE;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.errorNr = 0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.uldurd[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.uldurd[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.XYtheta[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.XYtheta[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.XYtheta[2] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.vomega[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.vomega[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.XRYRtheta[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.XRYRtheta[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.XRYRtheta[2] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlVelBUS[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlVelBUS[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlVelBUS[2] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlVelBUS[3] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlPosBUS[0] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlPosBUS[1] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlPosBUS[2] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlPosBUS[3] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlPosBUS[4] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlPosBUS[5] = 0.0;
	WMRctrlVelPosOdo0_rtZBUSouWMRctrl.ou_ctrlPosBUS[6] = 0.0;
}

/**************************************************************************
***   Funktionsname:     SetGlobalPosition   		                 		***
***   Funktion:          setzt die übergebene Positonswerte		     		***
***   Übergabe-Para.:    Keine                                          ***
***   Rückgabe-Para.:    Keine                                          ***
***   Erstellt:          Zauner Michael (30-10-2009)	                 	***
***   Änderungen:         											     				***
**************************************************************************/
void SetGlobalPosition(void)
{
	float fX, fY, fPhi;
	
	// ******************************
	// Werte übernehmen
	// ******************************
	fX = ((float)(uiXOffset)) / 1000.0;
	fY = ((float)(uiYOffset)) / 1000.0;
	fPhi = ((float)(uiWinkelOffset)) * M_PI / 1800.0;
	
	// ******************************
	// Werte der Odometrie neu setzen
	// ******************************
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.swReset = TRUE;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.x0y0theta0[0] = fX;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.x0y0theta0[1] = fY;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.x0y0theta0[2] = fPhi;

	// ******************************
	// Werte der Trajektorienvorgabe neu setzen
	// ******************************
	paramOutputAntrieb.fX = fX;
	paramOutputAntrieb.fY = fY;
	paramOutputAntrieb.fPhi = fPhi;
	
	// ******************************
	// Inkrementzähler resetieren
	// ******************************
	slIncrementMotor1 = 0;
	slIncrementMotor2 = 0;
	//   printf("SET X: %f (%d)  Y: %f (%d)  Phi: %f (%d)\r",WMRctrlVelPosOdo0_U.in_WMRctrlBUS.x0y0theta0[0],uiXOffset,WMRctrlVelPosOdo0_U.in_WMRctrlBUS.x0y0theta0[1],uiYOffset,WMRctrlVelPosOdo0_U.in_WMRctrlBUS.x0y0theta0[2],uiWinkelOffset);
	
}

/**************************************************************************
***   Funktionsname:    getINC                                          ***
***   Erstellt von Michael Zauner                               	   	***
***   Beschreibung:     Liefert die Anzahl der bewegten Inkremente      ***
***                                                                     ***
***   Parameter: R              Inkremente Rechts                       ***
***   Parameter: L              Inkremente Links                        ***
***                                                                     ***
***   Rückgabeparameter:  keine                                         ***
**************************************************************************/
void getINC(int32_t *R,int32_t *L)
{
	static int32_t L_static = 0,R_static = 0,Ln = 0,Rn = 0,Ln_1 = 0,Rn_1 = 0,dL = 0,dR = 0;  //Deklaration der Statischen Variablen
	char text[100];
	
	// **********************************************
	// Timer für linken Inkrementgeber auslesen
	// **********************************************
	Ln_1 = Ln;
	Ln = (int32_t)TCC1.CNT;

	// **********************************************
	// Timer für rechten Inkrementgeber auslesen
	// **********************************************
	Rn_1 = Rn;
	Rn = (int32_t)TCD1.CNT;

	
	// **********************************************
	// Überlauf
	// **********************************************
	if((Ln < ((MAX_INCS_PER_ROT_MOT_L * 4) / 3)) && (Ln_1 > ((MAX_INCS_PER_ROT_MOT_L * 8) / 3)))
	dL = Ln - Ln_1 + (MAX_INCS_PER_ROT_MOT_L * 4);
	// **********************************************
	// Unterlauf
	// **********************************************
	else if((Ln_1 < ((MAX_INCS_PER_ROT_MOT_L * 4) / 3)) && (Ln > ((MAX_INCS_PER_ROT_MOT_L * 8) / 3)))
	dL = Ln - Ln_1 - (MAX_INCS_PER_ROT_MOT_L * 4);
	// **********************************************
	// ansonsten
	// **********************************************
	else
	dL = Ln - Ln_1;
	

	// **********************************************
	// Überlauf
	// **********************************************
	if((Rn < ((MAX_INCS_PER_ROT_MOT_R * 4) / 3)) && (Rn_1 > ((MAX_INCS_PER_ROT_MOT_R * 8) / 3)))
	dR = Rn - Rn_1 + (MAX_INCS_PER_ROT_MOT_R * 4);
	// **********************************************
	// Unterlauf
	// **********************************************
	else if((Rn_1 < ((MAX_INCS_PER_ROT_MOT_R * 4) / 3)) && (Rn > ((MAX_INCS_PER_ROT_MOT_R * 8) / 3)))
	dR = Rn - Rn_1 - (MAX_INCS_PER_ROT_MOT_R * 4);
	// **********************************************
	// ansonsten
	// **********************************************
	else
	dR = Rn - Rn_1;
	
	
	// **********************************************
	// aktuellen Zählerstand berechenen
	// **********************************************
	L_static+=dL;        // Inkrementalgeber invertieren, wenn notwendig (Zählrichtung verkehrt)
	R_static-=dR;        // Inkrementalgeber invertieren, wenn notwendig (Zählrichtung verkehrt)

	// **********************************************
	// aktuellen Zählerstand ausgeben
	// **********************************************
	*L=L_static;
	*R=R_static;

	//sprintf(text, "R:%ld (%d %ld) L:%ld (%d %ld)\r",*R, TCC1.CNT, dR, *L, TCD1.CNT, dL);
	//debugMsg(text);
}



/**************************************************************************
***   FUNKTIONNAME: InitAntrieb                                          ***
***   FUNKTION: initialisiert die Gegnererkennung                       ***
***   TRANSMIT PARAMETER: NO                                            ***
***   RECEIVE PARAMETER.: NO                                            ***
**************************************************************************/
void InitAntrieb(void)
{
	unsigned int lineCount_l = MAX_INCS_PER_ROT_MOT_L;
	unsigned int lineCount_r = MAX_INCS_PER_ROT_MOT_R;

	// zyklischer Task - Zykluszeit: 6 ms
	SET_CYCLE(ANTRIEB_TASKNBR, 6);
	SET_TASK(ANTRIEB_TASKNBR, CYCLE);
	SET_TASK_HANDLE(ANTRIEB_TASKNBR, AntriebTask);

	// Timer/Counter TCD0 initialization -> PWM
	tcd0_init();
	
	// Quadrature Decoder Initalisation
	//QDEC1  Quadrature Decoder 1 für Motor1 -> LINKS:
	QDEC_Total_Setup1(&PORTC,                   //PORTC für QDEC1
	4,                         //Pin0 als erster Pin
	false,                     //nicht Invertiert
	0,                         //Event Multiplexer 0
	EVSYS_CHMUX_PORTC_PIN4_gc, //Input Pin für EventSystem
	false,                     //kein Index Signal
	EVSYS_QDIRM_00_gc,         //00 als Index Status
	&TCC1,                     //Timer/Counter TCC1 für QDEC1
	TC_EVSEL_CH0_gc,           //EventChannel für QDEC1
	lineCount_l);                //Anzahl der Signale/Umdrehungen
	
	//QDEC2  Quadrature Decoder 2 für Motor2 -> RECHTS:
	QDEC_Total_Setup1(&PORTB,                   //PORTB für QDEC1
	0,                         //Pin0 als erster Pin
	false,                     //nicht Invertiert
	2,                         //Event Multiplexer 2
	EVSYS_CHMUX_PORTB_PIN0_gc, //Input Pin für EventSystem
	false,                     //kein Index Signal
	EVSYS_QDIRM_00_gc,         //00 als Index Status
	&TCD1,                     //Timer/Counter TCD1 für QDEC2
	TC_EVSEL_CH2_gc,           //EventChannel für QDEC2
	lineCount_r);                //Anzahl der Signale/Umdrehungen

	// Initialize model
	Init_WMRctrlVelPosOdo0();
	WMRctrlVelPosOdo0_initialize(&WMRctrlVelPosOdo0_B, &WMRctrlVelPosOdo0_DWork, &WMRctrlVelPosOdo0_U, &WMRctrlVelPosOdo0_Y);
	
	// Beschleunigugnen setzen
	paramComAntrieb.fAmaxUs = 1.0; //1.5;//0.75; //0.5; //1.1;	// 1.4;
	paramComAntrieb.fAmaxDs = 1.0; //1.5;//0.75;//0.5; //1.0;	// 1.3;
	paramComAntrieb.fAlphaMaxUs = 10.0; //15.0;// 4.0; //4.0; //M_PI * 2.0;	// * 2.5;
	paramComAntrieb.fAlphaMaxDs = 10.0; //15.0;// 4.0;// 4.0; //M_PI * 2.0;    // * 2.5;
	
	// Regler sperren
	ucSetGlobalPosition = 0;
	ucStop = 0;
	
	// Motoren freigeben
	MOTOR_1_ENABLE;
	MOTOR_2_ENABLE;
	
}


/* ************************************************************** */
/*! \brief Set motor voltage.
*
*  Set dedicated voltage considering the battery voltage.
*
*  \param motorPoint pointer to motor.
*
*/
/* ************************************************************** */
convData_t SetU_Antrieb(float Voltage)
{
	float uBat;
	convData_t PWM;
	
	/* get accumulator voltage */
	uBat = getAccumulatorVoltage();
	
	/* ***************************************
	saturate the voltage to the battery voltage/limit
	***************************************  */
	if(Voltage < -uBat)
	Voltage = -uBat;
	if(Voltage > uBat)
	Voltage = uBat;
	
	/* ***************************************
	BTN7970
	***************************************  */
	if(Voltage < 0.0)
	{
		PWM.uint16[0] = (uint16_t)(((float)(TCD0.PER)) * (1.0 - (fabs(Voltage) / uBat)));
		PWM.uint16[1] = TCD0.PER;
	}
	else
	{
		PWM.uint16[1] = (uint16_t)(((float)(TCD0.PER)) * (1.0 - (Voltage / uBat)));
		PWM.uint16[0] = TCD0.PER;
	}
	
	return(PWM);
}



/**************************************************************************
***   FUNCTIONNAME:        AntriebTask                                  ***
***   FUNCTION:            Motion Control                               ***
***   TRANSMIT-PARAMETER:  NO                                           ***
***   RECEIVE-PARAMETER:   NO                                           ***
**************************************************************************/
unsigned char AntriebTask(void)
{
	static unsigned char ucResetOdo = 0, ausgabe = 0;
	signed int siULeft, siURight;
	convData_t leftPWM, rightPWM;
	char text[100];

	SET_CYCLE(ANTRIEB_TASKNBR, 6);
	SET_TASK(ANTRIEB_TASKNBR, CYCLE);
	
	
	// ***************************************************************************
	// ***   G L O B A L E   X / Y - P O S I T I O N   U N D   W I N K E L     ***
	// ***                             S E T Z E N                             ***
	// ***************************************************************************
	if(ucSetGlobalPosition == 1)
	{
		ucSetGlobalPosition = 0;
		SetGlobalPosition();
		ucResetOdo = 3;
	}
	
	
	// ************************************************************************
	// ***                       B E O B A C H T E R                        ***
	// ************************************************************************

	// ************************************************************************
	// *****                P O S I T I O N S R E G L E R                ******
	// ************************************************************************
	
	// ************************************************************************
	// *****         G E S C H W I N D I G K E I T S R E G L E R         ******
	// ************************************************************************
	// *******************************************
	// Positions- und Winkel-Vorgabe berechnen und setzen
	// *******************************************
	genTrajAntrieb();
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.P0_XYdXdYThdTh_desired[0] = paramOutputAntrieb.fX;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.P0_XYdXdYThdTh_desired[1] = paramOutputAntrieb.fY;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.P0_XYdXdYThdTh_desired[2] = paramOutputAntrieb.fVx;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.P0_XYdXdYThdTh_desired[3] = paramOutputAntrieb.fVy;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.P0_XYdXdYThdTh_desired[4] = paramOutputAntrieb.fPhi;
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.P0_XYdXdYThdTh_desired[5] = paramOutputAntrieb.fW;

	#ifdef _DEBUG_GEN_TRTAJ_
	sprintf(text, "%.3f;%.3f;%.3f;%.3f;%.1f;%.1f\r",paramOutputAntrieb.fX,paramOutputAntrieb.fY,paramOutputAntrieb.fVx,paramOutputAntrieb.fVy,RAD2DEG(paramOutputAntrieb.fPhi),RAD2DEG(paramOutputAntrieb.fW));
	debugMsg(text);
	#endif
	
	// *******************************************
	// Inkrementgeber auslesen
	// *******************************************
	// Messräder
	getINC(&slIncrementMotor1, &slIncrementMotor2);
	/* set encoder input to closed loop controller */
	// matehmatisch Richtig !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.encLR[_RIGHT_] = (real32_T)(GET_RIGHT_INCS) * CHECK_ROBOT_TYPE(-5.0,-1.0);
 	WMRctrlVelPosOdo0_U.in_WMRctrlBUS.encLR[_LEFT_] = (real32_T)(GET_LEFT_INCS) * CHECK_ROBOT_TYPE(-1.0,-1.0);
	// matehmatisch falsch!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//WMRctrlVelPosOdo0_U.in_WMRctrlBUS.encLR[_RIGHT_] = (real32_T)(GET_RIGHT_INCS) * CHECK_ROBOT_TYPE(1.0,1.0);
	//WMRctrlVelPosOdo0_U.in_WMRctrlBUS.encLR[_LEFT_] = (real32_T)(GET_LEFT_INCS) * CHECK_ROBOT_TYPE(5.0,1.0);
	
	// *******************************************
	// Odometrie und Regler abarbeiten
	// *******************************************
	WMRctrlVelPosOdo0_step(&WMRctrlVelPosOdo0_B, &WMRctrlVelPosOdo0_DWork, &WMRctrlVelPosOdo0_U, &WMRctrlVelPosOdo0_Y);
	
	// #####################################################################
	// Hiermit kann eine Spannung direkt an die Antriebsräder gelegt werden.
	// Dadurch kann die Drehrichtung (Spannung) der Antriebsräder überprüft
	// und gegebenfalls geändert (invertiert) werden.
	// !!! MUSS IM LAUFENDEN BETRIEB AUSKOMMENTIERT WERDEN !!!
	// #####################################################################
	
      //WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_LEFT_] = 0.0;
      //WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_RIGHT_] = 0.0;
// 	
	// #####################################################################
	// Nur zur Anzeige, ob der Task (AntriebTask) läuft.
	// !!! MUSS IM LAUFENDEN BETRIEB AUSKOMMENTIERT WERDEN !!!
	// #####################################################################
	
	//sprintf(text,"AntriebTask\r");
	//debugMsg(text);
	
	// *******************************************
	// Spannungen gegebenfalls invertieren
	// *******************************************
	// matehmatisch Richtig !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 	WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_LEFT_] *= CHECK_ROBOT_TYPE(1.0,1.0);
 	WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_RIGHT_] *= CHECK_ROBOT_TYPE(-1.0,-1.0);

	// matehmatisch falsch!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 	WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_LEFT_] *= CHECK_ROBOT_TYPE(-1.0,-1.0);
// 	WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_RIGHT_] *= CHECK_ROBOT_TYPE(1.0,1.0);
	
	//WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_LEFT_] *= (-1.0);
	//WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_RIGHT_] *= (1.0);
	
	// *******************************************
	// Werte am Motor setzen
	// *******************************************
	//    siULeft = (MAX_PWM_VALUE/2) + (signed int)(WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_LEFT_] * (MAX_PWM_VALUE/2) / getAccumulatorVoltage());
	//    if(siULeft > MAX_PWM_VALUE) siULeft = MAX_PWM_VALUE;
	//    else if(siULeft < 0) siULeft = 0;
	//
	//    siURight = (MAX_PWM_VALUE/2) + (signed int)(WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_RIGHT_] * (MAX_PWM_VALUE/2) / getAccumulatorVoltage());
	//    if(siURight > MAX_PWM_VALUE) siURight = MAX_PWM_VALUE;
	//    else if(siURight < 0) siURight = 0;
	leftPWM = SetU_Antrieb(WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_LEFT_]);
	rightPWM = SetU_Antrieb(WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_RIGHT_]);

	// *******************************************
	// PWM für Motoren ausgeben
	// *******************************************
	/* set voltage to right motor */
	SetPWM0(&TCD0, CH_A, rightPWM.uint16[0]);
	SetPWM0(&TCD0, CH_B, rightPWM.uint16[1]);
	/* set voltage to left motor */
	SetPWM0(&TCD0, CH_C, leftPWM.uint16[0]);
	SetPWM0(&TCD0, CH_D, leftPWM.uint16[1]);
	
	// *******************************************
	// wenn die Odometrie resetiert wird Motren stoppen
	// *******************************************
	if((ucResetOdo) || (ucStop == 1))
	{
		MOTOR_1_BRAKE;
		MOTOR_2_BRAKE;
	}
	else
	{
		MOTOR_1_ENABLE;
		MOTOR_2_ENABLE;
	}

	// *******************************************
	// Positions-Istwerte ausgeben
	// *******************************************
	siXPosition = (signed int)(WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.XYtheta[0] * 10000.0);
	siYPosition = (signed int)(WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.XYtheta[1] * 10000.0);
	siWinkel = (signed int)(WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.XYtheta[2] * 1800.0 / M_PI);
	siV = (signed int) (paramOutputAntrieb.fVn * 1000.0);
	// Positionsistwert alle 60 ms zum Hauptprozessor übertragen
	if(ausgabe++ >= 10) // 2
	{
		posAntrieb(siXPosition, siYPosition, siWinkel, siV);
		ausgabe = 0;
	}
	
	// *******************************************
	// Positions-Istwerte für Schleppfehler zwischenspeichern
	// *******************************************
	istX_ = WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.XYtheta[0];
	istY_ = WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.XYtheta[1];
	istPhi_ = WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.XYtheta[2];



	#ifdef _DEBUG_INCS_
	static i =0;
	if (i++>10)
	{ i=0;
			// Wie verhält sich Odometrie?
			sprintf(text, "#R:%5.1f L:%5.1f X:%2.3f Y:%2.3f P:%3.1f\r\n*",(float)WMRctrlVelPosOdo0_U.in_WMRctrlBUS.encLR[_RIGHT_],(float)WMRctrlVelPosOdo0_U.in_WMRctrlBUS.encLR[_LEFT_],istX_,istY_,istPhi_*180.0/M_PI);
			//zum Einstellen des Antriebs
			//sprintf(text, "%.3f;%.3f;%.1f;%.3f;%.3f;%.1f;%.1f;%.1f\r",paramOutputAntrieb.fX,
			//paramOutputAntrieb.fY,
			//RAD2DEG(paramOutputAntrieb.fPhi),
			//istX_,
			//istY_, 															  RAD2DEG(istPhi_),
			//WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_LEFT_],
			//WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.uldurd[_RIGHT_]);
			debugMsg(text);
	}

	#endif

	#ifdef _DEBUG_REGLER_
	sprintf(uart, "%5.4f;%5.4f;\r",    paramOutputAntrieb.fPhi,
	WMRctrlVelPosOdo0_Y.ou_WMRctrlBUS.XYtheta[2]
	);
	#endif
	
	// *******************************************
	// wenn die Positionswerte resetiert wurden -> Reseteingang zurücksetzen
	// *******************************************
	if(ucResetOdo != 0)
	{
		ucResetOdo--;
		if(ucResetOdo == 0) WMRctrlVelPosOdo0_U.in_WMRctrlBUS.swReset = FALSE;
	}

	return(CYCLE);
}

