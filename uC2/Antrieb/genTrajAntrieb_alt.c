/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      GENTRAJ.C
Version :  V 1.0
Date    :  03.03.2010
Author  :  MICHAEL ZAUNER

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : ATmega644
Program type        : Application
Clock frequency     : 20,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024

                 Copyright (c) 2006 by FH-Wels
                     All Rights Reserved.  
****************************************************************/

#define _GENTRAJ_ANTRIEB_EXTERN

#include <avr/io.h>
#include "genTrajAntrieb.h"
#include <math.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include "WMRctrlVelPosOdo0.h"         /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "antrieb.h"
#include "rrt_receivedata.h"
#include "path_clothoid.h"
#include "path_math.h"

#define MIT_SCHLEPPFEHLER

unsigned char schleppFehler = 0;

float errorTime = 0.0;


// Debugging
float global_t = 0.0;
uint8_t zaehler = 0;

float fsign(float a)
{
	return((a < 0) ? -1 : 1);
}

/**************************************************************************
***   Funktionsname:     genTraj                                    		***
***   Funktion:          generiert Trajektorie 15ms                    	***
***   Übergabe-Para.:    Keine                                          ***
***   Rückgabe-Para.:    Keine                                          ***
***   Erstellt:          Zauner Michael (21-06-2010)	                 	***
***   Änderungen:         											     				***
**************************************************************************/ 
void genTrajAntrieb(void)
{
	uint8_t text[200];
	
   float fDeltaX, fDeltaY, fPhiist, fS0, fPhi0, fTds_4, fTus_4, fW0, fPhi_2,fVmax_1, vMax_0;  
   signed int siZeitKorrektur;
   float omega0;
	
	uint8_t ii = 0;
	
	float vectorA[2] = {0.0, 0.0};			// 1. vector (startPoint - supportPoint) of the clothoid
	float vectorB[2] = {0.0, 0.0};			// 2. vector (finalPoint - supportPoint) of the clothoid
		
	float clothoidAlpha = 0.0;					// Angle alpha
	float clothoidT = 0.0;						// Tangential length T
	float clothoidT_old = 0.0;					// old tangential length
	float clothoidParameter = 0.0;			// Clothoid parameter A
	float clothoidArcLength = 0.0;			// Clothoid arc length s*
		
	// [0] ... velocity v0 (at start point of the clothoid)
	// [1] ... velocity v* (at final point of the clothoid)
	float clothoidVelocities[2] = {0.0, 0.0};
	float clothoidVelocities_old[2] = {0.0, 0.0};
		
	float T = 0.0;									
	float N = 0.0;
	int16_t n = 0;
	// to calculate if w is CW or CCW
	float k_w;
	float alpha, beta;										
	float phi;
	float accelerationa0 = 0.0;
	
	// Debugging
	//float zaehler = 0.0;
	global_t += T_ABTAST_;
   
   //****************************************************************
   //****    Nach Ende der Bewegung: Pos -> Endpunkt ausgeben    ****
   //****                            Speed -> Null ausgeben      ****
   //****************************************************************
   if(paramComAntrieb.ucState == _MOTION_READY_)
   {  
      paramOutputAntrieb.fSn = 0.0;
      paramOutputAntrieb.fSn_1 = 0.0; 
      paramOutputAntrieb.fPhin = 0.0;
      paramOutputAntrieb.fPhin_1 = 0.0; 
		errorTime = 0.0;
      
      schleppFehler = 0;
             
      // *********************************************************
      // es müssen noch Bewegungen abgearbeitet
      // -> auf _UP_SLOPE bzw. _MAX_SPEED schalten
      // *********************************************************
      if(indexCalcAntrieb > 0)
      {
         // normal motion -> POS_REL, POS_ABS, TURN_REL, TURN_ABS, KREISBOGEN
			if (paramCalcAntrieb[indexCalcOutAntrieb].clothoid == 0)
         {
				if(paramCalcAntrieb[indexCalcOutAntrieb].fTus > T_UPSLOPE_MIN_)
				{
					paramComAntrieb.ucState = _UP_SLOPE_;
				}
				else
				{
					paramComAntrieb.ucState = _MAX_SPEED_;
				}
         }
			// clothoid first part
			else if (paramCalcAntrieb[indexCalcOutAntrieb].clothoid == 1)
			{
				paramComAntrieb.ucState = _CLOTHOID_1_;
			} 
			// clothoid second part
			else if (paramCalcAntrieb[indexCalcOutAntrieb].clothoid == 2)
			{
				paramComAntrieb.ucState = _CLOTHOID_2_;
			}
			
      }
      // *********************************************************
      // bei neuer Bewegung oder noch eine Bewegung abgearbeitet werden muss
      // -> auf _NEW_MOTION schalten
      // *********************************************************
      else if(indexInputAntrieb > 0)
      {
         paramComAntrieb.ucState = _NEW_MOTION_; 
      }
      else
      {
         paramOutputAntrieb.fVn = 0.0;  
         paramOutputAntrieb.fWn = 0.0;  
      }
   } 
   
   //****************************************************************
   //****   neue Bewegung: Zeiten für Up- bzw. Down-Slope und    ****
   //****                  maximale Geschwindigkeit ausrechnen   ****
   //****************************************************************
   if(paramComAntrieb.ucState == _NEW_MOTION_)
   {                                 
      // *********************************
      // Bewegung berechnen
      // *********************************
      switch(paramInputAntrieb[indexInputOutAntrieb].ucType)
      {
         // **********************************
         // Relativposition (Translation) 
         // **********************************
         case _POS_REL:
         {
            fS0 = paramInputAntrieb[indexInputOutAntrieb].fSsoll;
            paramCalcAntrieb[indexCalcInAntrieb].fS0 = fS0 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);

            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
            // *****************************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fS0) > SPEED_LIMIT_FACTOR_) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fS0 * SPEED_LIMIT_FACTOR_  * (float)fsign(paramCalcAntrieb[indexCalcInAntrieb].fS0);
            // *****************************************
            // Geschindigkeits-Minimal Begrenzung
            // *****************************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < V_MIN) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = V_MIN  * (float)fsign(paramCalcAntrieb[indexCalcInAntrieb].fS0);
            
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // ***************************************** 
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxUs+pow(fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart),2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // *****************************************
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxDs+pow(fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart),2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
            // *****************************************
            // Upslope und Downslope-Zeit mittels max. Beschleunigung berechnen
            // *****************************************
            fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAmaxUs;
            fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVend) / paramComAntrieb.fAmaxDs;
            // *****************************************
            // Upslope und Downslope-Zeit in Struktur übergeben 
            // (!= ... gültige Zeit, == 0 ... Up- bzw. Downslope disabled
            // *****************************************
            if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
               paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

            if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)   
               paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4; 
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;
               
            // *****************************************
            // Upslope und Downslope-Zeit auf ganzzahlige vielfache von T_ABTAST umrechnen
            // *****************************************
            if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1); 
            }
               
            if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
            } 
                                  
            // *****************************************
            // tmax brechnen
            // *****************************************
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fS0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart)) / 2.0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVend)) / 2.0)) / 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax); 
                                              
            siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1);

            // *****************************************
            // Parameter auf Calc-Struktur übergeben
            // *****************************************
            paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fWmax = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fVstart = paramInputAntrieb[indexInputOutAntrieb].fVstart;
            // Vmax neu ermitteln (wegen der umrechnung auf ganzzahlige T_ABTAST !!!!)
            paramCalcAntrieb[indexCalcInAntrieb].fVmax = ((2 * fabs(fS0) - 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart) *  paramCalcAntrieb[indexCalcInAntrieb].fTus -  
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVend) *  paramCalcAntrieb[indexCalcInAntrieb].fTds) / 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax)) *
                                           (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            paramCalcAntrieb[indexCalcInAntrieb].fVend = paramInputAntrieb[indexInputOutAntrieb].fVend;
            paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
				paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;

            // Index der Calc-Struktur aktualisieren
            indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
            indexCalcAntrieb++;
             
            break;
         }
         // **********************************
         // Absolutposition (Translation) 
         // **********************************
         case _POS_ABS:
         {
            fDeltaX = paramInputAntrieb[indexInputOutAntrieb].fXsoll - paramOutputAntrieb.fX;
            fDeltaY = paramInputAntrieb[indexInputOutAntrieb].fYsoll - paramOutputAntrieb.fY;
         
            // ***************************
            // Weg ermitteln
            // ***************************
            fS0 = sqrt(pow(fDeltaX, 2.0) + pow(fDeltaY, 2.0))  * (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
         
            // ***************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
            // ***************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fS0) > SPEED_LIMIT_FACTOR_) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fS0 * SPEED_LIMIT_FACTOR_;
            // *****************************************
            // Geschindigkeits-Minimal Begrenzung
            // *****************************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < V_MIN) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = V_MIN  * (float)fsign(fS0);

            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // ***************************************** 
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxUs+pow(paramInputAntrieb[indexInputOutAntrieb].fVstart,2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // *****************************************
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxDs+pow(paramInputAntrieb[indexInputOutAntrieb].fVend,2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
         
            // ***************************
            // Winkel ermitteln (Phi = atan(dY/dX) - Phiist)
            // ***************************
            fPhiist = paramOutputAntrieb.fPhi;
            // ***************************
            // Winkel auf 0° bis 360° begrenzen
            // ***************************
            while(fPhiist > (2 * M_PI)) fPhiist -= (2 * M_PI);
            while(fPhiist < 0.0) fPhiist += (2 * M_PI);
               
            if(fS0 < 0.0) fPhiist += M_PI;
            if(fPhiist > (2 * M_PI)) fPhiist -= (2 * M_PI);  
            if(fabs(fS0) > 0.015) fPhi0 = atan2(fDeltaY, fDeltaX) - fPhiist;
            else  fPhi0 = 0.0;
                                                            
            // ***************************
            // Phi auf ±180.0° begrenzen
            // ***************************
            if(fPhi0 < -M_PI) fPhi0 += (2 * M_PI);
            else if(fPhi0 > M_PI) fPhi0 -= (2 * M_PI); 

            paramCalcAntrieb[indexCalcInAntrieb].fPhi0 = fPhi0;
               
            // ******************************************
            // Werte auf die Calc-Struktur schreiben
            // ******************************************
            // Drehung
            // ******************************************
            if(fabs(fPhi0) > 0.0001)
            {
               omega0 = W_MIN * (float)fsign(fPhi0) + fPhi0 * K_WINKEL_;
               if(fabs(omega0) < W_MIN)
               omega0 = W_MIN * (float)fsign(fPhi0);   
               if(fabs(omega0) > W_MAX)
               omega0 = W_MAX * (float)fsign(fPhi0);   
               // ***************************
               // mit max. Winkelbeschleunigung
               // ***************************
               paramCalcAntrieb[indexCalcInAntrieb].fTus = fabs(omega0) / paramComAntrieb.fAlphaMaxUs;
               if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
               {
                  siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
                  paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1); 
               }
                                 
               paramCalcAntrieb[indexCalcInAntrieb].fTds = fabs(omega0) / paramComAntrieb.fAlphaMaxDs;
               if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
               {
                  siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
                  paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
               } 
                  
               paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fPhi0) - 
                                              (paramCalcAntrieb[indexCalcInAntrieb].fTus * fabs(omega0) / 2.0) - 
                                              (paramCalcAntrieb[indexCalcInAntrieb].fTds * fabs(omega0) / 2.0)) / 
                                              (fabs(omega0));
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1); 

               paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
               paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
               paramCalcAntrieb[indexCalcInAntrieb].fWmax = 2 * /*fabs(fPhi0)*/ fPhi0 / (2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax + paramCalcAntrieb[indexCalcInAntrieb].fTds + paramCalcAntrieb[indexCalcInAntrieb].fTus); 
               paramCalcAntrieb[indexCalcInAntrieb].fVstart = 0.0;
               paramCalcAntrieb[indexCalcInAntrieb].fVmax = 0.0;
               paramCalcAntrieb[indexCalcInAntrieb].fVend = 0.0;
               paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
					paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;
                
               // Index der Calc-Struktur aktualisieren
               indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
               indexCalcAntrieb++;
            }

            // ******************************************
            // Strecke
            // ******************************************
            paramCalcAntrieb[indexCalcInAntrieb].fS0 = fS0;
            // über max. Beschleunigung
            fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAmaxUs;
            fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVend) / paramComAntrieb.fAmaxDs;
            if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
               paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

            if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)   
               paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4; 
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;
                 
            if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1); 
            }
              
            if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
            } 
                  
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fS0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart)) / 2.0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVend)) / 2.0)) / 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax); 

            siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1);
            
            paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fWmax = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fVstart = paramInputAntrieb[indexInputOutAntrieb].fVstart;
            paramCalcAntrieb[indexCalcInAntrieb].fVmax = ((2 * fabs(fS0) - 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart) *  paramCalcAntrieb[indexCalcInAntrieb].fTus -  
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVend) *  paramCalcAntrieb[indexCalcInAntrieb].fTds) / 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax)) *
                                           (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            paramCalcAntrieb[indexCalcInAntrieb].fVend = paramInputAntrieb[indexInputOutAntrieb].fVend;
            paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
				paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;

            // Index der Calc-Struktur aktualisieren
            indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
            indexCalcAntrieb++;
               
            break;
         }
         // **********************************
         // Relativposition (Rotation) 
         // **********************************
         case _TURN_REL:
         {
            fPhi0 = DEG2RAD(paramInputAntrieb[indexInputOutAntrieb].fPhiSoll);
            paramInputAntrieb[indexInputOutAntrieb].fVmax = DEG2RAD(paramInputAntrieb[indexInputOutAntrieb].fVmax); 
            paramCalcAntrieb[indexCalcInAntrieb].fPhi0 = fPhi0 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            // ***************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
            // ***************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fPhi0) > K_WINKEL_) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = paramCalcAntrieb[indexCalcInAntrieb].fPhi0 * K_WINKEL_;                 
            // *****************************************
            // Geschindigkeits-Minimal Begrenzung
            // *****************************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < W_MIN) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = W_MIN  * (float)fsign(paramCalcAntrieb[indexCalcInAntrieb].fPhi0);
               
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // ***************************************** 
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fPhi0)*paramComAntrieb.fAlphaMaxUs+pow(paramInputAntrieb[indexInputOutAntrieb].fVstart,2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // *****************************************
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fPhi0)*paramComAntrieb.fAlphaMaxDs+pow(paramInputAntrieb[indexInputOutAntrieb].fVend,2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
            
            // mit max. Winkelbeschleunigung
            fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAlphaMaxUs;
            fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAlphaMaxDs;
            if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
               paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

            if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)   
               paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4; 
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;

            if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1); 
            }
               
            if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
            } 

            paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fPhi0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart)) / 2.0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVend)) / 2.0)) / 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax);

            siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1); 

            paramCalcAntrieb[indexCalcInAntrieb].fWstart = paramInputAntrieb[indexInputOutAntrieb].fVstart;
            paramCalcAntrieb[indexCalcInAntrieb].fWmax = ((2 * fabs(fPhi0) - 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart) *  paramCalcAntrieb[indexCalcInAntrieb].fTus -  
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVend) *  paramCalcAntrieb[indexCalcInAntrieb].fTds) / 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax)) *
                                           (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax); 
            paramCalcAntrieb[indexCalcInAntrieb].fWend = paramInputAntrieb[indexInputOutAntrieb].fVend;
            paramCalcAntrieb[indexCalcInAntrieb].fVstart = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fVmax = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fVend = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
				paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;

            // Index der Calc-Struktur aktualisieren
            indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
            indexCalcAntrieb++;
            
            break;
         }
         // **********************************
         // Absolutposition (Rotation)
         // **********************************
         case _TURN_ABS:
         {
            fPhiist = paramOutputAntrieb.fPhi;
            // Winkel auf 0° bis 360° begrenzen
            while(fPhiist > (2 * M_PI)) fPhiist -= (2 * M_PI);
            while(fPhiist < 0.0) fPhiist += (2 * M_PI);
            fPhi0 = DEG2RAD(paramInputAntrieb[indexInputOutAntrieb].fPhiSoll) - fPhiist;
            if(fPhi0 > M_PI)
               fPhi0 -= 2*M_PI;
            else if(fPhi0 < -M_PI)
               fPhi0 += 2*M_PI;
            paramCalcAntrieb[indexCalcInAntrieb].fPhi0 = fPhi0;
            // ***************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
            // ***************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fPhi0) > K_WINKEL_) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fPhi0 * K_WINKEL_;
            // *****************************************
            // Geschindigkeits-Minimal Begrenzung
            // *****************************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < W_MIN) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = W_MIN  * (float)fsign(paramCalcAntrieb[indexCalcInAntrieb].fPhi0);

            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // ***************************************** 
            fVmax_1 = sqrt(RAMP_LIMIT_*fPhi0*paramComAntrieb.fAlphaMaxUs+pow(paramInputAntrieb[indexInputOutAntrieb].fVstart,2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // *****************************************
            fVmax_1 = sqrt(RAMP_LIMIT_*fPhi0*paramComAntrieb.fAlphaMaxDs+pow(paramInputAntrieb[indexInputOutAntrieb].fVend,2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
            // mit max. Winkelbeschleunigung
            fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAlphaMaxUs;
            fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAlphaMaxDs;
            if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
               paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

            if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)   
               paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4; 
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;

            if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1); 
            }
               
            if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
            }
                
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fPhi0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart)) / 2.0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVend)) / 2.0)) / 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax);

            siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1); 

            paramCalcAntrieb[indexCalcInAntrieb].fWstart = paramInputAntrieb[indexInputOutAntrieb].fVstart;
            paramCalcAntrieb[indexCalcInAntrieb].fWmax = (2 * fabs(fPhi0) - 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart) *  paramCalcAntrieb[indexCalcInAntrieb].fTus -  
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVend) *  paramCalcAntrieb[indexCalcInAntrieb].fTds) / 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax) * 
                                           (float)fsign(fPhi0);
            paramCalcAntrieb[indexCalcInAntrieb].fWend = paramInputAntrieb[indexInputOutAntrieb].fVend;
            paramCalcAntrieb[indexCalcInAntrieb].fVstart = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fVmax = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].fVend = 0.0;
            paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
				paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;

            // Index der Calc-Struktur aktualisieren
            indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
            indexCalcAntrieb++;
            
            break;
         }
         // **********************************
         // Kreisbogen 
         // **********************************
         case _KREISBOGEN:
         {
            fS0 = fabs(DEG2RAD(paramInputAntrieb[indexInputOutAntrieb].fPhiSoll)) * paramInputAntrieb[indexInputOutAntrieb].fRsoll * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);

            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
            // *****************************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fS0) > SPEED_LIMIT_FACTOR_) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fS0 * SPEED_LIMIT_FACTOR_;
            // *****************************************
            // Geschindigkeits-Minimal Begrenzung
            // *****************************************
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < V_MIN) 
               paramInputAntrieb[indexInputOutAntrieb].fVmax = V_MIN  * (float)fsign(fS0);
            
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // ***************************************** 
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxUs+pow(fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart),2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
				
            // *****************************************
            // Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
            // Upslope kontrollieren
            // *****************************************
            fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxDs+pow(fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart),2.0));
            if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1) 
            {
               paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            }
				          
            fW0 = (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) / paramInputAntrieb[indexInputOutAntrieb].fRsoll) * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fPhiSoll);    
            paramCalcAntrieb[indexCalcInAntrieb].fS0 = fS0;
            paramCalcAntrieb[indexCalcInAntrieb].fPhi0 = DEG2RAD(paramInputAntrieb[indexInputOutAntrieb].fPhiSoll);
               
            fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAmaxUs;
            fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVend) / paramComAntrieb.fAmaxDs;
            if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
               paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

            if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)   
               paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4; 
            else
               paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;
            
            if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1); 
            }
              
            if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
            {
               siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
               paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
            } 

            paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fS0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart)) / 2.0) - 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVend)) / 2.0)) / 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax);

            siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
            paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1); 

            paramCalcAntrieb[indexCalcInAntrieb].fWstart = fW0 * fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart/paramInputAntrieb[indexInputOutAntrieb].fVmax);
            paramCalcAntrieb[indexCalcInAntrieb].fWend = fW0 * fabs(paramInputAntrieb[indexInputOutAntrieb].fVend/paramInputAntrieb[indexInputOutAntrieb].fVmax);
            paramCalcAntrieb[indexCalcInAntrieb].fVstart = paramInputAntrieb[indexInputOutAntrieb].fVstart;
            paramCalcAntrieb[indexCalcInAntrieb].fVmax = ((2 * fabs(fS0) - 
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart) *  paramCalcAntrieb[indexCalcInAntrieb].fTus -  
                                           fabs(paramInputAntrieb[indexInputOutAntrieb].fVend) *  paramCalcAntrieb[indexCalcInAntrieb].fTds) / 
                                           (paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax)) * 
                                           (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            paramCalcAntrieb[indexCalcInAntrieb].fWmax = fW0 * fabs(paramCalcAntrieb[indexCalcInAntrieb].fVmax) / fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax);
            paramCalcAntrieb[indexCalcInAntrieb].fVend = paramInputAntrieb[indexInputOutAntrieb].fVend;
            paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
				paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;

            // Index der Calc-Struktur aktualisieren
            indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
					indexCalcAntrieb++;
           
            break;
         }
			case _CLOTHOID:
			{
				// *****************************************************************************************
				// *****************************************************************************************
				// **********
				// **********  more then one way point -> calculate clothoid
				// **********
				// *****************************************************************************************
				// *****************************************************************************************
				
				vMax_0 = paramInputAntrieb[indexInputOutAntrieb].fVmax;
				
				if (paramInputAntrieb[indexInputOutAntrieb].destinationPointsCount > 1)
				{
					for (ii = 0; ii < (paramInputAntrieb[indexInputOutAntrieb].destinationPointsCount); ii++)
					{		
						paramInputAntrieb[indexInputOutAntrieb].fVmax = vMax_0;
						
						fDeltaX = trayPoints[ii].x - trayPoints[ii-1].x;
						fDeltaY = trayPoints[ii].y - trayPoints[ii-1].y;
						
						// ***********************************************************************************
						// ***********   calculate first angle to turn to the path
						// ***********************************************************************************
						if (ii == 0)
						{
							fDeltaX = trayPoints[0].x - paramOutputAntrieb.fX;
							fDeltaY = trayPoints[0].y - paramOutputAntrieb.fY;
							
							// ***************************
							// Weg ermitteln
							// ***************************
							fS0 = sqrt(pow(fDeltaX, 2.0) + pow(fDeltaY, 2.0))  * (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
					
							// ***************************
							// Winkel ermitteln (Phi = atan(dY/dX) - Phiist)
							// ***************************
							fPhiist = paramOutputAntrieb.fPhi;
							// ***************************
							// Winkel auf 0° bis 360° begrenzen
							// ***************************
							while(fPhiist > (2 * M_PI)) fPhiist -= (2 * M_PI);
							while(fPhiist < 0.0) fPhiist += (2 * M_PI);
					
							if(fS0 < 0.0) fPhiist += M_PI;
							if(fPhiist > (2 * M_PI)) fPhiist -= (2 * M_PI);
							if(fabs(fS0) > 0.015) fPhi0 = atan2(fDeltaY, fDeltaX) - fPhiist;
							else  fPhi0 = 0.0;
					
							// ***************************
							// Phi auf ±180.0° begrenzen
							// ***************************
							if(fPhi0 < -M_PI) fPhi0 += (2 * M_PI);
							else if(fPhi0 > M_PI) fPhi0 -= (2 * M_PI);

							paramCalcAntrieb[indexCalcInAntrieb].fPhi0 = fPhi0;
							
							// ******************************************
							// Werte auf die Calc-Struktur schreiben
							// ******************************************
							// Drehung
							// ******************************************
							if(fabs(fPhi0) > 0.0001)
							{
								omega0 = W_MIN * (float)fsign(fPhi0) + fPhi0 * K_WINKEL_;
								if(fabs(omega0) < W_MIN)
								omega0 = W_MIN * (float)fsign(fPhi0);
								if(fabs(omega0) > W_MAX)
								omega0 = W_MAX * (float)fsign(fPhi0);
								
								// ***************************
								// mit max. Winkelbeschleunigung
								// ***************************
								paramCalcAntrieb[indexCalcInAntrieb].fTus = fabs(omega0) / paramComAntrieb.fAlphaMaxUs;
								if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
								{
									siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
									paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1);
								}
						
								paramCalcAntrieb[indexCalcInAntrieb].fTds = fabs(omega0) / paramComAntrieb.fAlphaMaxDs;
								if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
								{
									siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
									paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
								}
						
								paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fPhi0) -
								(paramCalcAntrieb[indexCalcInAntrieb].fTus * fabs(omega0) / 2.0) -
								(paramCalcAntrieb[indexCalcInAntrieb].fTds * fabs(omega0) / 2.0)) /
								(fabs(omega0));
								siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
								paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1);

								paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
								paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
								paramCalcAntrieb[indexCalcInAntrieb].fWmax = 2 * /*fabs(fPhi0)*/ fPhi0 / (2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax + paramCalcAntrieb[indexCalcInAntrieb].fTds + paramCalcAntrieb[indexCalcInAntrieb].fTus);
								paramCalcAntrieb[indexCalcInAntrieb].fVstart = 0.0;
								paramCalcAntrieb[indexCalcInAntrieb].fVmax = 0.0;
								paramCalcAntrieb[indexCalcInAntrieb].fVend = 0.0;
								paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
								paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;
						
								// Index der Calc-Struktur aktualisieren
								indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
								indexCalcAntrieb++;
							}
						}
						
						// ***********************************************************************************
						// ********** last way point
						// ***********************************************************************************
						if (ii == (paramInputAntrieb[indexInputOutAntrieb].destinationPointsCount - 1))
						{
							fDeltaX = trayPoints[ii-1].x - trayPoints[ii].x;
							fDeltaY = trayPoints[ii-1].y - trayPoints[ii].y;
							
							// ********************************************
							// ****  calculate way
							// ********************************************
							fS0 = (sqrt(pow(fDeltaX, 2.0) + pow(fDeltaY, 2.0)) - clothoidT_old)  * (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
							
							// ***************************
							// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
							// ***************************
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fS0) > SPEED_LIMIT_FACTOR_)
							paramInputAntrieb[indexInputOutAntrieb].fVmax = fS0 * SPEED_LIMIT_FACTOR_;
			
							// *****************************************
							// Geschindigkeits-Minimal Begrenzung
							// *****************************************
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < V_MIN)
							paramInputAntrieb[indexInputOutAntrieb].fVmax = V_MIN  * (float)fsign(fS0);
							
							// *****************************************
							// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
							// Upslope kontrollieren
							// *****************************************
							fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxUs+pow(clothoidVelocities_old[0],2.0));
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1)
							{
								paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
							}
							
							// *****************************************
							// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
							// Upslope kontrollieren
							// *****************************************
							fVmax_1 = sqrt(RAMP_LIMIT_* fabs(fS0)*paramComAntrieb.fAmaxDs);
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1)
							{
								paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
							}
													
							// ******************************************
							// Strecke
							// ******************************************
							paramCalcAntrieb[indexCalcInAntrieb].fS0 = fS0;
							// über max. Beschleunigung
							fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - clothoidVelocities_old[0]) / paramComAntrieb.fAmaxUs;
							fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) / paramComAntrieb.fAmaxDs;
							if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
							paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
							else
							paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

							if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)
							paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4;
							else
							paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;
							
							if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
							{
								siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
								paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1);
							}
							
							if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
							{
								siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
								paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
							}
							
							paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fS0) -
							(paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(clothoidVelocities_old[0])) / 2.0) -
							(paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax)) / 2.0)) /
							fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax);

							siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
							paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1);
							
							paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWmax = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fVstart = clothoidVelocities_old[0] * (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
							paramCalcAntrieb[indexCalcInAntrieb].fVmax = ((2 * fabs(fS0) - fabs(clothoidVelocities_old[0]) *  paramCalcAntrieb[indexCalcInAntrieb].fTus) /
							(paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax)) *
							(float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
							paramCalcAntrieb[indexCalcInAntrieb].fVend = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
							paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;
														
//sprintf(text,"LP vs:%3.3f;vm:%3.3f;ve:%3.3f;s0:%3.3f;Tu:%3.3f;Tm:%3.3f;Td:%3.3f\r\n",
//paramCalcAntrieb[indexCalcInAntrieb].fVstart,
//paramCalcAntrieb[indexCalcInAntrieb].fVmax,
//paramCalcAntrieb[indexCalcInAntrieb].fVend,
//fS0,
//paramCalcAntrieb[indexCalcInAntrieb].fTus,
//paramCalcAntrieb[indexCalcInAntrieb].fTmax,
//paramCalcAntrieb[indexCalcInAntrieb].fTds);
//debugMsg(text);

							// Index der Calc-Struktur aktualisieren
							indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
							indexCalcAntrieb++;	
						} 
						// ***********************************************************************************
						// ********** all points in between the path
						// ***********************************************************************************						
						else
						{
							// ****************************************
							// Calculation of vectorA and vectorB
							// ****************************************
							// first clothoid
							if (ii == 0)
							{
								vectorA[X] = paramOutputAntrieb.fX - trayPoints[ii].x;	// startPoint[X] - supportPoint[X];
								vectorA[Y] = paramOutputAntrieb.fY - trayPoints[ii].y;	// startPoint[Y] - supportPoint[Y];
								vectorB[X] = trayPoints[ii+1].x - trayPoints[ii].x;		// finalPoint[X] - supportPoint[X];
								vectorB[Y] = trayPoints[ii+1].y - trayPoints[ii].y;		// finalPoint[Y] - supportPoint[Y];
							}
							// all between
							else
							{
								vectorA[X] = trayPoints[ii-1].x - trayPoints[ii].x;		// startPoint[X] - supportPoint[X];
								vectorA[Y] = trayPoints[ii-1].y - trayPoints[ii].y;		// startPoint[Y] - supportPoint[Y];
								vectorB[X] = trayPoints[ii+1].x - trayPoints[ii].x;		// finalPoint[X] - supportPoint[X];
								vectorB[Y] = trayPoints[ii+1].y - trayPoints[ii].y;		// finalPoint[Y] - supportPoint[Y];
							}
	
							// ****************************************
							// Calculation of the clothoid
							// ****************************************
							// Calculates the angle alpha of the clothoid
							clothoidAlpha = Angle2D(vectorA, vectorB);
							
							// Correction of the angle
							if (clothoidAlpha > M_PI)	clothoidAlpha = (2 * M_PI) - clothoidAlpha;
							
							
							// Calculates the tangential length T of the clothoid
							clothoidT = CalculateClothoidT(vectorA, vectorB);
							
							// Calculates the parameter A of the clothoid
							clothoidParameter = CalculateClothoidParameter(clothoidAlpha, clothoidT);
							
							// Calculates the arc length s* of the clothoid
							clothoidArcLength = CalculateClothoidArcLength(clothoidAlpha, clothoidParameter);
							
							// Calculates the velocities v0 (at start point) and v* (at final point) of the clothoid
							// return -> clothoidVelocities
							CalculateClothoidVelocities(clothoidParameter, clothoidArcLength, clothoidVelocities);
							
							//sprintf(text,"alpha:%3.1f;cT:%3.3f;A:%3.3f;s:%3.3f;v0:%3.3f;vs:%3.3f;T:%3.3f;a:%3.3f;b:%3.3f;k_w:%3.3f\r\n", (clothoidAlpha*180/M_PI), clothoidT, clothoidParameter, clothoidArcLength, clothoidVelocities[0], clothoidVelocities[1], T, alpha, beta, k_w);
							//debugMsg(text);
							
							// ********************************************
							// ****  calculate way
							// ********************************************
							fS0 = (sqrt(pow(fDeltaX, 2.0) + pow(fDeltaY, 2.0))  - clothoidT - clothoidT_old)  * (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
							
							// ***************************
							// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
							// ***************************
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fS0) > SPEED_LIMIT_FACTOR_)
							paramInputAntrieb[indexInputOutAntrieb].fVmax = fS0 * SPEED_LIMIT_FACTOR_;
							
							// *****************************************
							// Geschindigkeits-Minimal Begrenzung
							// *****************************************
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < V_MIN)
							paramInputAntrieb[indexInputOutAntrieb].fVmax = V_MIN  * (float)fsign(fS0);
							
							// Saturate the clothoid velocity v0 to v_max
							if (fabs(clothoidVelocities[0]) > fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax))
							clothoidVelocities[0] = paramInputAntrieb[indexInputOutAntrieb].fVmax;
							// Set the sign of the clothoid velocity v0
							else
							clothoidVelocities[0] *= (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
							
							// Saturate the clothoid velocity v* to v_max
							if (fabs(clothoidVelocities[1]) > fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax))
							clothoidVelocities[1] = paramInputAntrieb[indexInputOutAntrieb].fVmax;
							// Set the sign of the clothoid velocity v*
							else
							clothoidVelocities[1] *= (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
							
							// *****************************************
							// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
							// Upslope kontrollieren
							// *****************************************
							fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxUs+pow(clothoidVelocities_old[0],2.0));
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1)
							{
								paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
							}
							
							// *****************************************
							// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
							// Upslope kontrollieren
							// *****************************************
							fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxDs+pow(clothoidVelocities[0],2.0));
							if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1)
							{
								paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
							}
							
							// ******************************************
							// Strecke
							// ******************************************
							paramCalcAntrieb[indexCalcInAntrieb].fS0 = fS0;
							// über max. Beschleunigung
							fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - clothoidVelocities_old[0]) / paramComAntrieb.fAmaxUs;
							fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - clothoidVelocities[0]) / paramComAntrieb.fAmaxDs;
							if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
							paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
							else
							paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

							if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)
							paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4;
							else
							paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;
							
							if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
							{
								siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
								paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1);
							}
							
							if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
							{
								siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
								paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
							}
							
							paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fS0) -
							(paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(clothoidVelocities_old[0])) / 2.0) -
							(paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(clothoidVelocities[0])) / 2.0)) /
							fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax);
							
							siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
							paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1);
							
							paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWmax = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fVstart = clothoidVelocities_old[0];
							paramCalcAntrieb[indexCalcInAntrieb].fVmax = ((2 * fabs(fS0) -
							fabs(clothoidVelocities_old[0]) *  paramCalcAntrieb[indexCalcInAntrieb].fTus -
							fabs(clothoidVelocities[0]) *  paramCalcAntrieb[indexCalcInAntrieb].fTds) /
							(paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax)) *
							(float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
							paramCalcAntrieb[indexCalcInAntrieb].fVend = clothoidVelocities[0];
							paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
							paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;
							
							//sprintf(text,"vs:%3.3f;vm:%3.3f;ve:%3.3f;s0:%3.3f;Tu:%3.3f;Tm:%3.3f;Td:%3.3f\r\n",
							//paramCalcAntrieb[indexCalcInAntrieb].fVstart,
							//paramCalcAntrieb[indexCalcInAntrieb].fVmax,
							//paramCalcAntrieb[indexCalcInAntrieb].fVend,
							//fS0,
							//paramCalcAntrieb[indexCalcInAntrieb].fTus,
							//paramCalcAntrieb[indexCalcInAntrieb].fTmax,
							//paramCalcAntrieb[indexCalcInAntrieb].fTds);
							//debugMsg(text);
							
							// Index der Calc-Struktur aktualisieren
							indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
							indexCalcAntrieb++;
							
							// ****************************************
							// Ensure the integrity (Ganzzahligkeit)
							// ****************************************
							// Calculates not integer (ganzzahlige) time T
							T = (2.0 * clothoidArcLength) / (fabs(clothoidVelocities[0]) + fabs(clothoidVelocities[1]));
							
							N = T / ((float)T_ABTAST_ANTRIEB / 1000.0);
							n = (int16_t)(N) + 1;
							
							// Calculates integer (ganzzahlige) time T
							T = ((float)T_ABTAST_ANTRIEB / 1000.0) * (float)n;
							
							// Calculates integer (ganzzahlige) velocity v*
							clothoidVelocities[1] = (((2.0 * clothoidArcLength) / T) - fabs(clothoidVelocities[0])) * (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
							
							// *****************************************
							// Calculates the sense of rotation
							// *****************************************
							ChangeOrientation2D(vectorA, vectorA);
							
							alpha = AngleToXAxis2D(vectorA);
							beta = AngleToXAxis2D(vectorB);
							
							phi = beta - alpha;
							// if phi is smaller than 0.0 -> phi += 2 * PI
							while (phi < 0.0) phi += (2 * M_PI);
							// phi < PI -> w is CCW
							if(phi < M_PI)
							k_w = 1.0;
							// phi > PI -> w is CW
							else
							k_w = -1.0;
							
							// ********************************************
							// ****  store clothoid 1 and 2
							// ********************************************
							paramCalcAntrieb[indexCalcInAntrieb].fTus = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fTds = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fTmax = T;
							paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWmax = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fVstart = clothoidVelocities[0];
							paramCalcAntrieb[indexCalcInAntrieb].fVmax = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fVend = clothoidVelocities[1];
							paramCalcAntrieb[indexCalcInAntrieb].A = clothoidParameter;
							paramCalcAntrieb[indexCalcInAntrieb].s_star = clothoidArcLength;
							paramCalcAntrieb[indexCalcInAntrieb].T = clothoidT;
							paramCalcAntrieb[indexCalcInAntrieb].alpha = clothoidAlpha;
							paramCalcAntrieb[indexCalcInAntrieb].K_W = k_w;
							paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
							paramCalcAntrieb[indexCalcInAntrieb].clothoid = 1;
							
							//sprintf(text,"T:%3.3f;v0:%3.3f;vs:%3.3f;T:%3.3f;a:%3.3f;b:%3.3f;k_w:%3.3f\r\n",
							//paramCalcAntrieb[indexCalcInAntrieb].T,
							//paramCalcAntrieb[indexCalcInAntrieb].fVstart,
							//paramCalcAntrieb[indexCalcInAntrieb].fVend,
							//paramCalcAntrieb[indexCalcInAntrieb].fTmax,
							//alpha, beta, k_w);
							//debugMsg(text);
							
							// Index der Calc-Struktur aktualisieren
							indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
							indexCalcAntrieb++;

							paramCalcAntrieb[indexCalcInAntrieb].fTus = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fTds = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fTmax = T;
							paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fWmax = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fVstart = clothoidVelocities[0];
							paramCalcAntrieb[indexCalcInAntrieb].fVmax = 0.0;
							paramCalcAntrieb[indexCalcInAntrieb].fVend = clothoidVelocities[1];
							paramCalcAntrieb[indexCalcInAntrieb].A = clothoidParameter;
							paramCalcAntrieb[indexCalcInAntrieb].s_star = clothoidArcLength;
							paramCalcAntrieb[indexCalcInAntrieb].T = clothoidT;
							paramCalcAntrieb[indexCalcInAntrieb].alpha = clothoidAlpha;
							paramCalcAntrieb[indexCalcInAntrieb].K_W = k_w;
							paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
							paramCalcAntrieb[indexCalcInAntrieb].clothoid = 2;
							
							// Index der Calc-Struktur aktualisieren
							indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
							indexCalcAntrieb++;
							
							// Save the old tangential length
							clothoidT_old = clothoidT;
							// Save the old clothoid velocities
							clothoidVelocities_old[0] = clothoidVelocities[0];
							clothoidVelocities_old[1] = clothoidVelocities[1];
					
						}					
					}		
				} 
				// *****************************************************************************************
				// *****************************************************************************************
				// **********
				// **********  only one way point -> POS_ABS
				// **********
				// *****************************************************************************************
				// *****************************************************************************************
				else
				{
					fDeltaX = trayPoints[0].x - paramOutputAntrieb.fX;
					fDeltaY = trayPoints[0].y - paramOutputAntrieb.fY;
            
					// ***************************
					// Weg ermitteln
					// ***************************
					fS0 = sqrt(pow(fDeltaX, 2.0) + pow(fDeltaY, 2.0))  * (float)(fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax));
            
					// ***************************
					// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für den Weg zu groß ist)
					// ***************************
					if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax/fS0) > SPEED_LIMIT_FACTOR_)
					paramInputAntrieb[indexInputOutAntrieb].fVmax = fS0 * SPEED_LIMIT_FACTOR_;
					
					// *****************************************
					// Geschindigkeits-Minimal Begrenzung
					// *****************************************
					if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) < V_MIN)
					paramInputAntrieb[indexInputOutAntrieb].fVmax = V_MIN  * (float)fsign(fS0);

					// *****************************************
					// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
					// Upslope kontrollieren
					// *****************************************
					fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxUs+pow(paramInputAntrieb[indexInputOutAntrieb].fVstart,2.0));
					if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1)
					{
						paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
					}
					// *****************************************
					// Geschindigkeits- Sicherheitsüberwachung (wenn Geschwindigkeit für die max. Beschleunigung zu groß ist)
					// Upslope kontrollieren
					// *****************************************
					fVmax_1 = sqrt(RAMP_LIMIT_*fabs(fS0)*paramComAntrieb.fAmaxDs+pow(paramInputAntrieb[indexInputOutAntrieb].fVend,2.0));
					if(fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) > fVmax_1)
					{
						paramInputAntrieb[indexInputOutAntrieb].fVmax = fVmax_1 * (float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
					}
            
					// ***************************
					// Winkel ermitteln (Phi = atan(dY/dX) - Phiist)
					// ***************************
					fPhiist = paramOutputAntrieb.fPhi;
					// ***************************
					// Winkel auf 0° bis 360° begrenzen
					// ***************************
					while(fPhiist > (2 * M_PI)) fPhiist -= (2 * M_PI);
					while(fPhiist < 0.0) fPhiist += (2 * M_PI);
            
					if(fS0 < 0.0) fPhiist += M_PI;
					if(fPhiist > (2 * M_PI)) fPhiist -= (2 * M_PI);
					if(fabs(fS0) > 0.015) fPhi0 = atan2(fDeltaY, fDeltaX) - fPhiist;
					else  fPhi0 = 0.0;
            
					// ***************************
					// Phi auf ±180.0° begrenzen
					// ***************************
					if(fPhi0 < -M_PI) fPhi0 += (2 * M_PI);
					else if(fPhi0 > M_PI) fPhi0 -= (2 * M_PI);

					paramCalcAntrieb[indexCalcInAntrieb].fPhi0 = fPhi0;
            
					// ******************************************
					// Werte auf die Calc-Struktur schreiben
					// ******************************************
					// Drehung
					// ******************************************
					if(fabs(fPhi0) > 0.0001)
					{
						omega0 = W_MIN * (float)fsign(fPhi0) + fPhi0 * K_WINKEL_;
						if(fabs(omega0) < W_MIN)
						omega0 = W_MIN * (float)fsign(fPhi0);
						if(fabs(omega0) > W_MAX)
						omega0 = W_MAX * (float)fsign(fPhi0);

						// ***************************
						// mit max. Winkelbeschleunigung
						// ***************************
						paramCalcAntrieb[indexCalcInAntrieb].fTus = fabs(omega0) / paramComAntrieb.fAlphaMaxUs;
						if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
						{
							siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
							paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1);
						}
	            
						paramCalcAntrieb[indexCalcInAntrieb].fTds = fabs(omega0) / paramComAntrieb.fAlphaMaxDs;
						if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
						{
							siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
							paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
						}
	            
						paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fPhi0) -
						(paramCalcAntrieb[indexCalcInAntrieb].fTus * fabs(omega0) / 2.0) -
						(paramCalcAntrieb[indexCalcInAntrieb].fTds * fabs(omega0) / 2.0)) /
						(fabs(omega0));
						siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
						paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1);

						paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
						paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
						paramCalcAntrieb[indexCalcInAntrieb].fWmax = 2 * /*fabs(fPhi0)*/ fPhi0 / (2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax + paramCalcAntrieb[indexCalcInAntrieb].fTds + paramCalcAntrieb[indexCalcInAntrieb].fTus);
						paramCalcAntrieb[indexCalcInAntrieb].fVstart = 0.0;
						paramCalcAntrieb[indexCalcInAntrieb].fVmax = 0.0;
						paramCalcAntrieb[indexCalcInAntrieb].fVend = 0.0;
						paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
						paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;
						
//sprintf(text,"I: ws:%3.3f;wm:%3.3f;we:%3.3f;p0:%3.3f;Tu:%3.3f;Tm:%3.3f;Td:%3.3f\r\n",
//paramCalcAntrieb[indexCalcInAntrieb].fWstart,
//paramCalcAntrieb[indexCalcInAntrieb].fWmax,
//paramCalcAntrieb[indexCalcInAntrieb].fWend,
//fPhi0,
//paramCalcAntrieb[indexCalcInAntrieb].fTus,
//paramCalcAntrieb[indexCalcInAntrieb].fTmax,
//paramCalcAntrieb[indexCalcInAntrieb].fTds);
//debugMsg(text);
	            
						// Index der Calc-Struktur aktualisieren
						indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
							indexCalcAntrieb++;
					}

					// ******************************************
					// Strecke
					// ******************************************
					paramCalcAntrieb[indexCalcInAntrieb].fS0 = fS0;
					// über max. Beschleunigung
					fTus_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVstart) / paramComAntrieb.fAmaxUs;
					fTds_4 = fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax - paramInputAntrieb[indexInputOutAntrieb].fVend) / paramComAntrieb.fAmaxDs;
					if(paramInputAntrieb[indexInputOutAntrieb].fTus >= T_UD_AUTO_GRENZE_)
					paramCalcAntrieb[indexCalcInAntrieb].fTus = fTus_4;
					else
					paramCalcAntrieb[indexCalcInAntrieb].fTus = paramInputAntrieb[indexInputOutAntrieb].fTus;

					if(paramInputAntrieb[indexInputOutAntrieb].fTds >= T_UD_AUTO_GRENZE_)
					paramCalcAntrieb[indexCalcInAntrieb].fTds = fTds_4;
					else
					paramCalcAntrieb[indexCalcInAntrieb].fTds = paramInputAntrieb[indexInputOutAntrieb].fTds;
            
					if(paramCalcAntrieb[indexCalcInAntrieb].fTus > 0.0001)
					{
						siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTus / T_ABTAST_);
						paramCalcAntrieb[indexCalcInAntrieb].fTus = T_ABTAST_ * ((float)siZeitKorrektur + 1);
					}
            
					if(paramCalcAntrieb[indexCalcInAntrieb].fTds > 0.0001)
					{
						siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTds / T_ABTAST_);
						paramCalcAntrieb[indexCalcInAntrieb].fTds = T_ABTAST_ * ((float)siZeitKorrektur + 1);
					}
            
					paramCalcAntrieb[indexCalcInAntrieb].fTmax = (fabs(fS0) -
					(paramCalcAntrieb[indexCalcInAntrieb].fTus * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart)) / 2.0) -
					(paramCalcAntrieb[indexCalcInAntrieb].fTds * (fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax) + fabs(paramInputAntrieb[indexInputOutAntrieb].fVend)) / 2.0)) /
					fabs(paramInputAntrieb[indexInputOutAntrieb].fVmax);

					siZeitKorrektur = (signed int)(paramCalcAntrieb[indexCalcInAntrieb].fTmax / T_ABTAST_);
					paramCalcAntrieb[indexCalcInAntrieb].fTmax = T_ABTAST_ * ((float)siZeitKorrektur + 1);
            
					paramCalcAntrieb[indexCalcInAntrieb].fWstart = 0.0;
					paramCalcAntrieb[indexCalcInAntrieb].fWend = 0.0;
					paramCalcAntrieb[indexCalcInAntrieb].fWmax = 0.0;
					paramCalcAntrieb[indexCalcInAntrieb].fVstart = paramInputAntrieb[indexInputOutAntrieb].fVstart;
					paramCalcAntrieb[indexCalcInAntrieb].fVmax = ((2 * fabs(fS0) -
					fabs(paramInputAntrieb[indexInputOutAntrieb].fVstart) *  paramCalcAntrieb[indexCalcInAntrieb].fTus -
					fabs(paramInputAntrieb[indexInputOutAntrieb].fVend) *  paramCalcAntrieb[indexCalcInAntrieb].fTds) /
					(paramCalcAntrieb[indexCalcInAntrieb].fTus + paramCalcAntrieb[indexCalcInAntrieb].fTds + 2 * paramCalcAntrieb[indexCalcInAntrieb].fTmax)) *
					(float)fsign(paramInputAntrieb[indexInputOutAntrieb].fVmax);
					paramCalcAntrieb[indexCalcInAntrieb].fVend = paramInputAntrieb[indexInputOutAntrieb].fVend;
					paramCalcAntrieb[indexCalcInAntrieb].gegnerErkennung =  paramInputAntrieb[indexInputOutAntrieb].gegnerErkennung;
					paramCalcAntrieb[indexCalcInAntrieb].clothoid = 0;
					
					
//sprintf(text,"II: vs:%3.3f;vm:%3.3f;ve:%3.3f;s0:%3.3f;Tu:%3.3f;Tm:%3.3f;Td:%3.3f\r\n",
//paramCalcAntrieb[indexCalcInAntrieb].fVstart,
//paramCalcAntrieb[indexCalcInAntrieb].fVmax,
//paramCalcAntrieb[indexCalcInAntrieb].fVend,
//fS0,
//paramCalcAntrieb[indexCalcInAntrieb].fTus,
//paramCalcAntrieb[indexCalcInAntrieb].fTmax,
//paramCalcAntrieb[indexCalcInAntrieb].fTds);
//debugMsg(text);					

					// Index der Calc-Struktur aktualisieren
					indexCalcInAntrieb = ((++indexCalcInAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcInAntrieb);
						indexCalcAntrieb++;					
				}
							
				break;
			}				
      }

      // Index der Input-Struktur aktualisieren
      indexInputOutAntrieb = ((++indexInputOutAntrieb >= MAX_IN_ANTRIEB) ? 0 : indexInputOutAntrieb);
      if(indexInputAntrieb > 0) indexInputAntrieb--;
        
      paramComAntrieb.fT = 0.0;
      if(paramCalcAntrieb[indexCalcOutAntrieb].fTus > T_UPSLOPE_MIN_) 
      {
        paramComAntrieb.ucState = _UP_SLOPE_;
      }
      else
      {
         paramComAntrieb.ucState = _MAX_SPEED_;
      }
   } 
	
	
	
	errorTime += T_ABTAST_;
	
	if((motionIR_Received == 1) && (errorTime > 0.018))
	{
		paramComAntrieb.ucState = _MOTION_INTERRUPT_;
		motionIR_Received = 0;
	}		
  
   // ******************************************************************
   // ******************************************************************
   // generiert das Weg(Winkel) - Geschwindigkeits(Winkelgeschw.) Profil
   // ******************************************************************
   // ******************************************************************
   switch(paramComAntrieb.ucState)                                                 
   {  
      //****************************************************************
      //****    Up-Slope ausgeben: im ersten Teil des Weges eine    ****
      //****                       Rampe ausgeben                   ****
      //****************************************************************
      case _UP_SLOPE_:
      { 
         // **********************************************
         // Weg/Geschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fSn_1 = paramOutputAntrieb.fSn;
         paramOutputAntrieb.fSn = paramCalcAntrieb[indexCalcOutAntrieb].fVstart * paramComAntrieb.fT + 
                           (paramCalcAntrieb[indexCalcOutAntrieb].fVmax - paramCalcAntrieb[indexCalcOutAntrieb].fVstart) *
                           pow(paramComAntrieb.fT, 2.0) / 2 / paramCalcAntrieb[indexCalcOutAntrieb].fTus; 
         
			paramOutputAntrieb.fVn_1 = paramOutputAntrieb.fVn;                  
         paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVstart + 
                           (paramCalcAntrieb[indexCalcOutAntrieb].fVmax - paramCalcAntrieb[indexCalcOutAntrieb].fVstart) *
                           paramComAntrieb.fT / paramCalcAntrieb[indexCalcOutAntrieb].fTus;
         
         // Geschwindigkeit auf Vmax begrenzen
         if((fabs(paramOutputAntrieb.fVn) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVmax)) && 
			   (fabs(paramOutputAntrieb.fVn_1) < fabs(paramOutputAntrieb.fVn)))
            paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVmax;
         if((fabs(paramOutputAntrieb.fVn) <= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVmax)) &&
				(fabs(paramOutputAntrieb.fVn_1) > fabs(paramOutputAntrieb.fVn)))
				paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVmax;
            
         // **********************************************
         // Winkel/Winkelgeschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fPhin_1 = paramOutputAntrieb.fPhin;
         paramOutputAntrieb.fPhin = paramCalcAntrieb[indexCalcOutAntrieb].fWstart * paramComAntrieb.fT + 
                             (paramCalcAntrieb[indexCalcOutAntrieb].fWmax - paramCalcAntrieb[indexCalcOutAntrieb].fWstart) *
                             pow(paramComAntrieb.fT, 2.0) / 2 / paramCalcAntrieb[indexCalcOutAntrieb].fTus; 

         paramOutputAntrieb.fWn = paramCalcAntrieb[indexCalcOutAntrieb].fWstart + 
                           (paramCalcAntrieb[indexCalcOutAntrieb].fWmax - paramCalcAntrieb[indexCalcOutAntrieb].fWstart) *
                           paramComAntrieb.fT / paramCalcAntrieb[indexCalcOutAntrieb].fTus; 

         // Winkelgeschwindigkeit auf Wmax begrenzen
         if(fabs(paramOutputAntrieb.fWn) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fWmax)) 
            paramOutputAntrieb.fWn = paramCalcAntrieb[indexCalcOutAntrieb].fWmax;

         // **********************************************
         // aktuelle Weg- und Winkelwerte zwischenspeichern
         // **********************************************
         paramCalcAntrieb[indexCalcOutAntrieb].fSold = paramOutputAntrieb.fSn;
         paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = paramOutputAntrieb.fPhin;
         paramCalcAntrieb[indexCalcOutAntrieb].fVold = paramOutputAntrieb.fVn;
         paramCalcAntrieb[indexCalcOutAntrieb].fWold = paramOutputAntrieb.fWn;
         
         if(paramComAntrieb.fT >= paramCalcAntrieb[indexCalcOutAntrieb].fTus)
         {
            if(paramCalcAntrieb[indexCalcOutAntrieb].fTmax > T_MAX_MIN_)
            {
               paramComAntrieb.ucState = _MAX_SPEED_;
               paramCalcAntrieb[indexCalcOutAntrieb].fSI = paramOutputAntrieb.fSn;
               paramCalcAntrieb[indexCalcOutAntrieb].fPhiI = paramOutputAntrieb.fPhin;
               paramCalcAntrieb[indexCalcOutAntrieb].fTmax -= (paramComAntrieb.fT - paramCalcAntrieb[indexCalcOutAntrieb].fTus);
            }
            else
            {
               paramComAntrieb.ucState = _DOWN_SLOPE_; 
               paramCalcAntrieb[indexCalcOutAntrieb].fSII = paramOutputAntrieb.fSn;
               paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = paramOutputAntrieb.fPhin;
               paramCalcAntrieb[indexCalcOutAntrieb].fTds -= (paramComAntrieb.fT - paramCalcAntrieb[indexCalcOutAntrieb].fTus);
            }
            paramComAntrieb.fT = 0.0;  
         }
			
			errorTime = 0.0;
         paramComAntrieb.fT += T_ABTAST_;
         
         break;
      } 
      
      //****************************************************************
      //****    Down-Slope ausgeben: im letzten Teil des Weges eine ****
      //****                         Rampe ausgeben                 ****
      //****************************************************************
      case _DOWN_SLOPE_:
      {
         // **********************************************
         // Weg/Geschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fSn_1 = paramOutputAntrieb.fSn;
         paramOutputAntrieb.fSn = paramCalcAntrieb[indexCalcOutAntrieb].fSII + 
                           paramCalcAntrieb[indexCalcOutAntrieb].fVmax * paramComAntrieb.fT - 
                           (paramCalcAntrieb[indexCalcOutAntrieb].fVmax - paramCalcAntrieb[indexCalcOutAntrieb].fVend) *
                           pow(paramComAntrieb.fT, 2.0) / 2 / paramCalcAntrieb[indexCalcOutAntrieb].fTds; 
         // Weg auf S0 begrenzen
         if(fabs(paramOutputAntrieb.fSn) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fS0)) 
            paramOutputAntrieb.fSn = paramCalcAntrieb[indexCalcOutAntrieb].fS0;
	
			paramOutputAntrieb.fVn_1 = paramOutputAntrieb.fVn;

         paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVmax - 
                           (paramCalcAntrieb[indexCalcOutAntrieb].fVmax - paramCalcAntrieb[indexCalcOutAntrieb].fVend) *
                           paramComAntrieb.fT / paramCalcAntrieb[indexCalcOutAntrieb].fTds;  

         // Geschwindigkeit auf Vend begrenzen
         if((fabs(paramOutputAntrieb.fVn) <= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVend)) &&
				(fabs(paramOutputAntrieb.fVn_1) > fabs(paramOutputAntrieb.fVn))) 
            paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVend;
         if((fabs(paramOutputAntrieb.fVn) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVend)) &&
         (fabs(paramOutputAntrieb.fVn_1) < fabs(paramOutputAntrieb.fVn)))
				paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVend;
         if((fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVend) < 0.001) && ((paramOutputAntrieb.fVn * (float)fsign(paramCalcAntrieb[indexCalcOutAntrieb].fVold)) < 0.0))
            paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVend;
         // Geschwindigkeit auf Vmax begrenzen
         //if((fabs(paramOutputAntrieb.fVn) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVmax)) &&
				//(paramOutputAntrieb.fVn_1 < paramOutputAntrieb.fVn))
				//paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVmax;
         //if((fabs(paramOutputAntrieb.fVn) <= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVmax)) &&
				//(paramOutputAntrieb.fVn_1 > paramOutputAntrieb.fVn))
				//paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVmax;

         // **********************************************
         // Winkel/Winkelgeschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fPhin_1 = paramOutputAntrieb.fPhin;
         paramOutputAntrieb.fPhin = paramCalcAntrieb[indexCalcOutAntrieb].fPhiII +
                             paramCalcAntrieb[indexCalcOutAntrieb].fWmax * paramComAntrieb.fT - 
                             (paramCalcAntrieb[indexCalcOutAntrieb].fWmax - paramCalcAntrieb[indexCalcOutAntrieb].fWend) *
                             pow(paramComAntrieb.fT, 2.0) / 2 / paramCalcAntrieb[indexCalcOutAntrieb].fTds; 
         // Winkel auf Phi0 begrenzen
         if(fabs(paramOutputAntrieb.fPhin) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fPhi0)) 
            paramOutputAntrieb.fPhin = paramCalcAntrieb[indexCalcOutAntrieb].fPhi0;


         paramOutputAntrieb.fWn = paramCalcAntrieb[indexCalcOutAntrieb].fWmax - 
                           (paramCalcAntrieb[indexCalcOutAntrieb].fWmax - paramCalcAntrieb[indexCalcOutAntrieb].fWend) *
                           paramComAntrieb.fT / paramCalcAntrieb[indexCalcOutAntrieb].fTds; 
             
         // Winkelgeschwindigkeit auf Wend begrenzen
         if(fabs(paramOutputAntrieb.fWn) <= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fWend)) 
            paramOutputAntrieb.fWn = paramCalcAntrieb[indexCalcOutAntrieb].fWend;
         if((fabs(paramCalcAntrieb[indexCalcOutAntrieb].fWend) < 0.0001) && ((paramOutputAntrieb.fWn * (float)fsign(paramCalcAntrieb[indexCalcOutAntrieb].fWold)) < 0.0))
            paramOutputAntrieb.fWn = paramCalcAntrieb[indexCalcOutAntrieb].fWend;

         // **********************************************
         // aktuelle Weg- und Winkelwerte zwischenspeichern
         // **********************************************
         paramCalcAntrieb[indexCalcOutAntrieb].fSold = paramOutputAntrieb.fSn;
         paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = paramOutputAntrieb.fPhin;
         paramCalcAntrieb[indexCalcOutAntrieb].fVold = paramOutputAntrieb.fVn;
         paramCalcAntrieb[indexCalcOutAntrieb].fWold =paramOutputAntrieb.fWn;

         if(paramComAntrieb.fT >= paramCalcAntrieb[indexCalcOutAntrieb].fTds)
         {
         // Index der Calc-Struktur aktualisieren 
            paramCalcAntrieb[indexCalcOutAntrieb].fTus = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fTds = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fTmax = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVstart = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVend = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVmax = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWstart = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWend = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWmax = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fSold = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fSI = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fSII = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVold = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhin = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhiI = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWold = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].A = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].alpha = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].clothoid = 0;
				paramCalcAntrieb[indexCalcOutAntrieb].K_W = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].T = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].s_star = 0.0;				
            
            indexCalcOutAntrieb = ((++indexCalcOutAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcOutAntrieb);
            if(indexCalcAntrieb > 0) indexCalcAntrieb--;
 
            if((indexCalcAntrieb == 0) && (indexInputAntrieb == 0))
               statusAntrieb(MOTION_OK);				
            
				errorTime = 0.0;
			   paramComAntrieb.ucState = _MOTION_READY_;
            paramComAntrieb.fT = 0.0;  
         }

         paramComAntrieb.fT += T_ABTAST_;
     
         break;
      }
      //****************************************************************
      //****    maximale Speed ausgeben: Geschwindigkeit konstant   ****
      //****                             halten                     ****
      //****************************************************************
      case _MAX_SPEED_:
      {
         // **********************************************
         // Weg/Geschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fSn_1 = paramOutputAntrieb.fSn;
         paramOutputAntrieb.fSn = paramCalcAntrieb[indexCalcOutAntrieb].fSI + 
                           paramCalcAntrieb[indexCalcOutAntrieb].fVmax * paramComAntrieb.fT; 

         paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVmax;
           
         // **********************************************
         // Winkel/Winkelgeschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fPhin_1 = paramOutputAntrieb.fPhin;
         paramOutputAntrieb.fPhin = paramCalcAntrieb[indexCalcOutAntrieb].fPhiI +
                             paramCalcAntrieb[indexCalcOutAntrieb].fWmax * paramComAntrieb.fT; 

         paramOutputAntrieb.fWn = paramCalcAntrieb[indexCalcOutAntrieb].fWmax; 
                 
         // **********************************************
         // aktuelle Weg- und Winkelwerte zwischenspeichern
         // **********************************************
         paramCalcAntrieb[indexCalcOutAntrieb].fSold = paramOutputAntrieb.fSn;
         paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = paramOutputAntrieb.fPhin;
         paramCalcAntrieb[indexCalcOutAntrieb].fVold = paramOutputAntrieb.fVn;
         paramCalcAntrieb[indexCalcOutAntrieb].fWold = paramOutputAntrieb.fWn;

         if(paramComAntrieb.fT >= paramCalcAntrieb[indexCalcOutAntrieb].fTmax)
         {          
				if(paramCalcAntrieb[indexCalcOutAntrieb].fTds > T_DOWNSLOPE_MIN_) 
            {
               paramComAntrieb.ucState = _DOWN_SLOPE_; 
               paramCalcAntrieb[indexCalcOutAntrieb].fSII = paramOutputAntrieb.fSn;
               paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = paramOutputAntrieb.fPhin;
               paramCalcAntrieb[indexCalcOutAntrieb].fTds -= (paramComAntrieb.fT - paramCalcAntrieb[indexCalcOutAntrieb].fTmax);
            }
            else
            {
               // Index der Calc-Struktur aktualisieren 
               paramCalcAntrieb[indexCalcOutAntrieb].fTus = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fTds = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fTmax = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fVstart = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fVend = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fVmax = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fWstart = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fWend = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fWmax = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fSold = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fSI = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fSII = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fVold = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fPhin = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fPhiI = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = 0.0;
               paramCalcAntrieb[indexCalcOutAntrieb].fWold = 0.0;
					paramCalcAntrieb[indexCalcOutAntrieb].A = 0.0;
					paramCalcAntrieb[indexCalcOutAntrieb].alpha = 0.0;
					paramCalcAntrieb[indexCalcOutAntrieb].clothoid = 0;
					paramCalcAntrieb[indexCalcOutAntrieb].K_W = 0.0;
					paramCalcAntrieb[indexCalcOutAntrieb].T = 0.0;
					paramCalcAntrieb[indexCalcOutAntrieb].s_star = 0.0; 
					           
               // Weg auf S0 begrenzen
               if(fabs(paramOutputAntrieb.fSn) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fS0)) 
                  paramOutputAntrieb.fSn = paramCalcAntrieb[indexCalcOutAntrieb].fS0; 

               // Winkel auf Phi0 begrenzen
               if(fabs(paramOutputAntrieb.fPhin) >= fabs(paramCalcAntrieb[indexCalcOutAntrieb].fPhi0)) 
                  paramOutputAntrieb.fPhin = paramCalcAntrieb[indexCalcOutAntrieb].fPhi0;
                  
               indexCalcOutAntrieb = ((++indexCalcOutAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcOutAntrieb);
               if(indexCalcAntrieb > 0) indexCalcAntrieb--; 
               
               if((indexCalcAntrieb == 0) && (indexInputAntrieb == 0))
                  statusAntrieb(MOTION_OK);
               
               paramComAntrieb.ucState = _MOTION_READY_;
            }
				errorTime = 0.0;				
            paramComAntrieb.fT = 0.0;  
         }

         paramComAntrieb.fT += T_ABTAST_;
        
         break;
      }
      //****************************************************************
      //****   Bewegung abbrechen: Bewegung abbrechen und zu Null   ****
      //****                       fahren                           ****
      //****************************************************************
      case _MOTION_INTERRUPT_:
      {   
         // **********************************************
         // Motion IR initialisieren
         // **********************************************
         if(paramComAntrieb.ucInitMotionInterrupt == 1)
         {
            paramComAntrieb.ucInitMotionInterrupt = 0;
            // t auf Null setzen
            paramComAntrieb.fT = 0.0;  
            // wenn Geschwindigkeit != 0 -> Tinterrupt auf Vold / Adsmax setzen 
            if((fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVold) > 0.001) && (fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVold) > fabs(paramCalcAntrieb[indexCalcOutAntrieb].fWold)))
               paramComAntrieb.fTinterrupt = fabs(paramCalcAntrieb[indexCalcOutAntrieb].fVold) / paramComAntrieb.fAmaxDs / 1.5;
            // wenn Winkelgeschwindigkeit != 0 -> Tinterrupt auf Wold / Alphadsmax setzen 
            else if(fabs(paramCalcAntrieb[indexCalcOutAntrieb].fWold) > 0.0001)
               paramComAntrieb.fTinterrupt = fabs(paramCalcAntrieb[indexCalcOutAntrieb].fWold) / paramComAntrieb.fAlphaMaxDs / 1.5;
            // ansosnten tinterrupt auf eine Abtastzeit setzen
            else
               paramComAntrieb.fTinterrupt = T_ABTAST_;
                  
         }
         // **********************************************
         // Weg/Geschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fSn_1 = paramOutputAntrieb.fSn;
         paramOutputAntrieb.fSn = paramCalcAntrieb[indexCalcOutAntrieb].fSold + 
                           paramCalcAntrieb[indexCalcOutAntrieb].fVold * paramComAntrieb.fT - 
                           paramCalcAntrieb[indexCalcOutAntrieb].fVold *
                           pow(paramComAntrieb.fT, 2.0) / 2 / paramComAntrieb.fTinterrupt; 

         paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVold - 
                           paramCalcAntrieb[indexCalcOutAntrieb].fVold *
                           paramComAntrieb.fT / paramComAntrieb.fTinterrupt;  
         
         // Geschwindigkeit auf 0 begrenzen
         if((paramOutputAntrieb.fVn * (float)fsign(paramCalcAntrieb[indexCalcOutAntrieb].fVold)) < 0.0) 
            paramOutputAntrieb.fVn = 0.0;
            
         // **********************************************
         // Winkel/Winkelgeschwindigkeit berechnen
         // **********************************************
         paramOutputAntrieb.fPhin_1 = paramOutputAntrieb.fPhin;
         paramOutputAntrieb.fPhin = paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld +
                             paramCalcAntrieb[indexCalcOutAntrieb].fWold * paramComAntrieb.fT - 
                             paramCalcAntrieb[indexCalcOutAntrieb].fWold *
                             pow(paramComAntrieb.fT, 2.0) / 2 / paramComAntrieb.fTinterrupt; 

         paramOutputAntrieb.fWn = paramCalcAntrieb[indexCalcOutAntrieb].fWold - 
                           paramCalcAntrieb[indexCalcOutAntrieb].fWold *
                           paramComAntrieb.fT / paramComAntrieb.fTinterrupt; 

         // Winkelgeschwindigkeit auf 0 begrenzen
         if((paramOutputAntrieb.fWn * (float)fsign(paramCalcAntrieb[indexCalcOutAntrieb].fWold)) < 0.0) 
            paramOutputAntrieb.fWn = 0.0;
         
         if(paramComAntrieb.fT >= paramComAntrieb.fTinterrupt)
         {
            paramComAntrieb.ucState = _MOTION_READY_;
            
            statusAntrieb(MOTION_ERROR_IR);

            // Index der Calc-Struktur aktualisieren 
            paramCalcAntrieb[indexCalcOutAntrieb].fTus = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fTds = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fTmax = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVstart = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVend = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVmax = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWstart = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWend = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWmax = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fSold = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fSI = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fSII = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fVold = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhin = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhiI = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = 0.0;
            paramCalcAntrieb[indexCalcOutAntrieb].fWold = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].A = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].alpha = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].clothoid = 0;
				paramCalcAntrieb[indexCalcOutAntrieb].K_W = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].T = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].s_star = 0.0;
				            
            // nach dem Abbruch der Bewegung alle noch anstehenden 
            // Bewegungen löschen
            indexCalcAntrieb = 0;
            indexCalcOutAntrieb = 0;
            indexCalcInAntrieb = 0;
            indexInputAntrieb = 0;
            indexInputOutAntrieb = 0;
            indexInputInAntrieb = 0;
				errorTime = 0.0;
            paramComAntrieb.fT = 0.0;  
         }

         paramComAntrieb.fT += T_ABTAST_;
			    
         break;
      } 
      //****************************************************************
      //****   Clothoid: first part                                 ****
      //****************************************************************
		case _CLOTHOID_1_:
		{
			paramCalcAntrieb[indexCalcOutAntrieb].fVold = paramOutputAntrieb.fVn; 
			paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVstart - (paramCalcAntrieb[indexCalcOutAntrieb].fVstart - paramCalcAntrieb[indexCalcOutAntrieb].fVend) * paramComAntrieb.fT / paramCalcAntrieb[indexCalcOutAntrieb].fTmax;
			
			paramOutputAntrieb.fSn_1 = paramOutputAntrieb.fSn;
			paramOutputAntrieb.fSn += (paramCalcAntrieb[indexCalcOutAntrieb].fVold + paramOutputAntrieb.fVn) / 2.0 * T_ABTAST_;
					
			accelerationa0 = ((pow(paramCalcAntrieb[indexCalcOutAntrieb].fVstart, 2.0) - pow(paramCalcAntrieb[indexCalcOutAntrieb].fVend, 2.0)) / (2.0 * paramCalcAntrieb[indexCalcOutAntrieb].s_star));

			paramOutputAntrieb.fWn = (1 / pow(paramCalcAntrieb[indexCalcOutAntrieb].A, 2.0)) * ((pow(paramCalcAntrieb[indexCalcOutAntrieb].fVstart, 2.0) * paramComAntrieb.fT) - ((3.0 / 2.0) * paramCalcAntrieb[indexCalcOutAntrieb].fVstart * accelerationa0 * pow(paramComAntrieb.fT, 2.0)) + ((1.0 / 2.0) * pow(accelerationa0, 2.0) * pow(paramComAntrieb.fT, 3.0)));
		
			paramOutputAntrieb.fPhin_1 = paramOutputAntrieb.fPhin;
			paramOutputAntrieb.fPhin += paramOutputAntrieb.fWn * T_ABTAST_ * paramCalcAntrieb[indexCalcOutAntrieb].K_W;
					
			// **********************************************
			// aktuelle Weg- und Winkelwerte zwischenspeichern
			// **********************************************
			paramCalcAntrieb[indexCalcOutAntrieb].fSold = paramOutputAntrieb.fSn;
			paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = paramOutputAntrieb.fPhin;
			paramCalcAntrieb[indexCalcOutAntrieb].fVold = paramOutputAntrieb.fVn;
			paramCalcAntrieb[indexCalcOutAntrieb].fWold =paramOutputAntrieb.fWn;

         if(paramComAntrieb.fT >= paramCalcAntrieb[indexCalcOutAntrieb].fTmax)
         {
	         // Index der Calc-Struktur aktualisieren
	         paramCalcAntrieb[indexCalcOutAntrieb].fTus = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fTds = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fTmax = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fVstart = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fVend = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fVmax = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fWstart = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fWend = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fWmax = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fSold = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fSI = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fSII = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fVold = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fPhin = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fPhiI = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = 0.0;
	         paramCalcAntrieb[indexCalcOutAntrieb].fWold = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].A = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].alpha = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].clothoid = 0;
				paramCalcAntrieb[indexCalcOutAntrieb].K_W = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].T = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].s_star = 0.0;
	         
	         indexCalcOutAntrieb = ((++indexCalcOutAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcOutAntrieb);
	         if(indexCalcAntrieb > 0) indexCalcAntrieb--;
	         
	         if((indexCalcAntrieb == 0) && (indexInputAntrieb == 0))
					statusAntrieb(MOTION_OK);
	         
				errorTime = 0.0;
			   paramComAntrieb.ucState = _MOTION_READY_;
	         paramComAntrieb.fT = 0.0;
         }

         paramComAntrieb.fT += T_ABTAST_;
			
			break;	
		}
      //****************************************************************
      //****   Clothoid: second part                                ****
      //****************************************************************
		case _CLOTHOID_2_:
		{
			paramCalcAntrieb[indexCalcOutAntrieb].fVold = paramOutputAntrieb.fVn;
			paramOutputAntrieb.fVn = paramCalcAntrieb[indexCalcOutAntrieb].fVstart - (paramCalcAntrieb[indexCalcOutAntrieb].fVstart - paramCalcAntrieb[indexCalcOutAntrieb].fVend) * (paramCalcAntrieb[indexCalcOutAntrieb].fTmax - paramComAntrieb.fT) / paramCalcAntrieb[indexCalcOutAntrieb].fTmax;
			
			paramOutputAntrieb.fSn_1 = paramOutputAntrieb.fSn;
			paramOutputAntrieb.fSn += (paramCalcAntrieb[indexCalcOutAntrieb].fVold + paramOutputAntrieb.fVn) / 2.0 * T_ABTAST_;
			
			accelerationa0 = ((pow(paramCalcAntrieb[indexCalcOutAntrieb].fVstart, 2.0) - pow(paramCalcAntrieb[indexCalcOutAntrieb].fVend, 2.0)) / (2.0 * paramCalcAntrieb[indexCalcOutAntrieb].s_star));

			paramOutputAntrieb.fWn = (1 / pow(paramCalcAntrieb[indexCalcOutAntrieb].A, 2.0)) * ((pow(paramCalcAntrieb[indexCalcOutAntrieb].fVstart, 2.0) * (paramCalcAntrieb[indexCalcOutAntrieb].fTmax - paramComAntrieb.fT)) - ((3.0 / 2.0) * paramCalcAntrieb[indexCalcOutAntrieb].fVstart * accelerationa0 * pow((paramCalcAntrieb[indexCalcOutAntrieb].fTmax - paramComAntrieb.fT), 2.0)) + ((1.0 / 2.0) * pow(accelerationa0, 2.0) * pow((paramCalcAntrieb[indexCalcOutAntrieb].fTmax - paramComAntrieb.fT), 3.0)));
			
			paramOutputAntrieb.fPhin_1 = paramOutputAntrieb.fPhin;
			paramOutputAntrieb.fPhin += paramOutputAntrieb.fWn * T_ABTAST_ * paramCalcAntrieb[indexCalcOutAntrieb].K_W;
			
			// **********************************************
			// aktuelle Weg- und Winkelwerte zwischenspeichern
			// **********************************************
			paramCalcAntrieb[indexCalcOutAntrieb].fSold = paramOutputAntrieb.fSn;
			paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = paramOutputAntrieb.fPhin;
			paramCalcAntrieb[indexCalcOutAntrieb].fVold = paramOutputAntrieb.fVn;
			paramCalcAntrieb[indexCalcOutAntrieb].fWold =paramOutputAntrieb.fWn;
			
			if(paramComAntrieb.fT >= paramCalcAntrieb[indexCalcOutAntrieb].fTmax)
			{
				// Index der Calc-Struktur aktualisieren
				paramCalcAntrieb[indexCalcOutAntrieb].fTus = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fTds = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fTmax = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fVstart = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fVend = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fVmax = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fWstart = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fWend = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fWmax = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fSold = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fSI = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fSII = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fVold = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fPhin = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fPhiI = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].fWold = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].A = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].alpha = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].clothoid = 0;
				paramCalcAntrieb[indexCalcOutAntrieb].K_W = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].T = 0.0;
				paramCalcAntrieb[indexCalcOutAntrieb].s_star = 0.0;
				
				indexCalcOutAntrieb = ((++indexCalcOutAntrieb >= MAX_CALC_ANTRIEB) ? 0 : indexCalcOutAntrieb);
				if(indexCalcAntrieb > 0) indexCalcAntrieb--;
				
				if((indexCalcAntrieb == 0) && (indexInputAntrieb == 0))
					statusAntrieb(MOTION_OK);
				
				errorTime = 0.0;
				
				paramComAntrieb.ucState = _MOTION_READY_;
				paramComAntrieb.fT = 0.0;
			}

			paramComAntrieb.fT += T_ABTAST_;
			
			break;
		}			
   }  
   
   // **************************************************
   // Wenn nicht Motion-Interrupt aktiv ist 
   // -> Init Motion IR auf 1 setzen, damit bei Motion IR die Parameter initialisiert werden
   // **************************************************
   if(paramComAntrieb.ucState != _MOTION_INTERRUPT_) paramComAntrieb.ucInitMotionInterrupt = 1;

#ifdef MIT_SCHLEPPFEHLER 
   // **************************************************
   // Schleppfehlererkennung
   // -> Wenn der Schleppfehler in X- bzw. Y-Richtung um 6 cm oder der Winkelfehler größer
   //    30° ist, wird die Bewegung abgebrochen
   // **************************************************
   if((fabs(paramOutputAntrieb.fX - istX_) > SCHLEPP_FEHLER_TRANS_) || 
      (fabs(paramOutputAntrieb.fY - istY_) > SCHLEPP_FEHLER_TRANS_) || 
      (fabs(paramOutputAntrieb.fPhi - istPhi_) > SCHLEPP_FEHLER_ROT_))
   {
      if(schleppFehler > 10)
      {
         paramComAntrieb.ucState = _MOTION_READY_;
         
         statusAntrieb(MOTION_ERROR_SCHLEPP);
         
         paramOutputAntrieb.fPhi = istPhi_;
         paramOutputAntrieb.fW = 0.0;
         paramOutputAntrieb.fX = istX_;
         paramOutputAntrieb.fY = istY_;
         paramOutputAntrieb.fVx = 0.0;
         paramOutputAntrieb.fVy = 0.0;
        
         // Index der Calc-Struktur aktualisieren 
         paramCalcAntrieb[indexCalcOutAntrieb].fTus = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fTds = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fTmax = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fVstart = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fVend = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fVmax = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fWstart = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fWend = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fWmax = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fSold = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fSI = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fSII = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fVold = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fPhin = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fPhiOld = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fPhiI = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fPhiII = 0.0;
         paramCalcAntrieb[indexCalcOutAntrieb].fWold = 0.0;
			paramCalcAntrieb[indexCalcOutAntrieb].A = 0.0;
			paramCalcAntrieb[indexCalcOutAntrieb].alpha = 0.0;
			paramCalcAntrieb[indexCalcOutAntrieb].clothoid = 0;
			paramCalcAntrieb[indexCalcOutAntrieb].K_W = 0.0;
			paramCalcAntrieb[indexCalcOutAntrieb].T = 0.0;
			paramCalcAntrieb[indexCalcOutAntrieb].s_star = 0.0;
			
         paramOutputAntrieb.fSn = 0.0;
         paramOutputAntrieb.fSn_1 = 0.0; 
         paramOutputAntrieb.fPhin = 0.0;
         paramOutputAntrieb.fPhin_1 = 0.0; 
         
         paramOutputAntrieb.fVn = 0.0;  
         paramOutputAntrieb.fWn = 0.0;
			
         // nach dem Abbruch der Bewegung alle noch anstehenden 
         // Bewegungen löschen
         indexCalcAntrieb = 0;
         indexCalcOutAntrieb = 0;
         indexCalcInAntrieb = 0;
         indexInputAntrieb = 0;
         indexInputOutAntrieb = 0;
         indexInputInAntrieb = 0;
			
			errorTime = 0.0;
         paramComAntrieb.fT = 0.0;
      }
      else
         schleppFehler++;
   }
   else
   {
      if(schleppFehler > 0)
         schleppFehler--;   
   }
#endif  
   
   // **********************************
   // Wegprofil ausgeben
   // **********************************
   paramOutputAntrieb.fPhi += (paramOutputAntrieb.fPhin - paramOutputAntrieb.fPhin_1);
   fPhi_2 = (paramOutputAntrieb.fPhin - paramOutputAntrieb.fPhin_1) / 2.0;
   paramOutputAntrieb.fW = paramOutputAntrieb.fWn;
   paramOutputAntrieb.fX += (paramOutputAntrieb.fSn - paramOutputAntrieb.fSn_1) * cos(paramOutputAntrieb.fPhi - fPhi_2);
   paramOutputAntrieb.fY += (paramOutputAntrieb.fSn - paramOutputAntrieb.fSn_1) * sin(paramOutputAntrieb.fPhi - fPhi_2);
   paramOutputAntrieb.fVx = paramOutputAntrieb.fVn * cos(paramOutputAntrieb.fPhi - fPhi_2);
   paramOutputAntrieb.fVy = paramOutputAntrieb.fVn * sin(paramOutputAntrieb.fPhi - fPhi_2);
	
	//// Debugging              
	////if(paramComAntrieb.fT < 0.05)
	////{
		//if(zaehler++ >= 0)
		//{
			//sprintf(text,"%1.3f;%1.3f;%1.3f;%1.3f;%1.3f;%1.3f;%d;%1.3f;%1.3f\r", paramOutputAntrieb.fX, paramOutputAntrieb.fY, paramOutputAntrieb.fVx, paramOutputAntrieb.fVy, paramOutputAntrieb.fPhi,paramOutputAntrieb.fW,paramComAntrieb.ucState,paramComAntrieb.fT,global_t);
			//debugMsg(text);
			//zaehler = 0;
		//}
////	}
}

/**************************************************************************
***   Funktionsname:     getAngle                                  		***
***   Funktion:          gibt den Winkel zwischen 0° und 359.99° zurück	***
***   Übergabe-Para.:    Keine                                          ***
***   Rückgabe-Para.:    float Winkel                                   ***
***   Erstellt:          Zauner Michael (01-02-2011)	                 	***
***   Änderungen:         											     				***
**************************************************************************/ 
float getAngle(void)
{
   float Winkel;
   
   Winkel = RAD2DEG(paramOutputAntrieb.fPhi);
   while(Winkel > 360.0) Winkel -= 360.0;
   return(Winkel);
}
