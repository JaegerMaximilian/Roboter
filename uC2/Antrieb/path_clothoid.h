/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      PATH_CLOTHOID.h
Version :  V 1.0
Date    :  26.02.2014
Author  :  SCHLAGER MANUEL

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
Data Stack size     : 1024

                 Copyright (c) 2014 by FH-Wels
                     All Rights Reserved.  
****************************************************************/

#ifndef _PATH_CLOTHOID_H
#define _PATH_CLOTHOID_H

#ifndef _PATH_CLOTHOID_EXTERN
	#define _PATH_CLOTHOID_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
#define GRAVITY_OF_EARTH	9.80665			// Gravity of earth (m/s2)
#define T_DEFAULT				0.25				// Default value for tangential length T (25cm)

// Limits for calculating the optimal velocities v0 (at start point) and v* (at final point) of the clothoid
#define OMEGA_PUNKT_MAX		(3 * M_PI)		// 1. Limit: omegaPunkt_MAX  (2 * M_PI)	
#define MU_MAX					0.8				// 2. Limit: µ_MAX  0.5
#define A_MAX					3.5				// 3. Limit: a_MAX  2.5

/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
// Methods for calculating the clothoid
float CalculateClothoidT(float *vectorA, float *vectorB);
float CalculateClothoidParameter(float clothoidAlpha, float clothoidT);
float CalculateClothoidArcLength(float clothoidAlpha, float clothoidParameter);
void CalculateClothoidVelocities(float clothoidParameter, float clothoidArcLength, float* clothoidVelocitiesResult);

// Methods for calculating the clothoidParameter approximately
float f_sinFresnel(float x);
float f_cosFresnel(float x);
float Simpson_Rule(float (*f)(float), float a, float b, int n);

#endif