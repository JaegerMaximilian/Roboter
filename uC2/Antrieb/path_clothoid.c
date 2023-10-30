/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      PATH_CLOTHOID.c
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

#define _PATH_CLOTHOID_EXTERN

#include <avr/io.h>  
#include "global.h" 
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "path_clothoid.h"
#include "path_math.h"
#include "define.h"
#include "rrt_receivedata.h"

/*********************************************************************************************
*** FUNCTIONNAME		  : CalculateT																			 ***
*** FUNCTION			  : Calculates the tangential length T 										 ***
*** RECEIVE-PARAMETER  : vectorA ... 1. vector (startPoint - supportPoint) of the clothoid ***
***							 vectorB ... 2. vector (finalPoint - supportPoint) of the clothoid ***
*** TRANSMIT-PARAMETER : Tangential length T																 ***
*** -------------------------------------------------------------------------------------- ***
*** Version				  : V 1.0																				 ***
*** Date					  : 28.0.2014																			 ***
*** Author				  : SCHLAGER MANUEL, ZAUNER MICHAEL												 ***
*********************************************************************************************/
float CalculateClothoidT(float *vectorA, float *vectorB)
{
	float normVectorA = 0.0;
	float normVectorB = 0.0;
	
	// Calculates the norm of vectorA and vectorB
	normVectorA = Norm2D(vectorA);
	normVectorB = Norm2D(vectorB);
	
	if(normVectorA <= normVectorB)
	{
		// Distance must always be smaller then the half way from one point to the next point
		// If not, it would be possible that two clothoids intersect
		if(T_DEFAULT > (normVectorA / 3.0))
			return (normVectorA / 3.0);
		else
			return (T_DEFAULT);
	}
	else
	{
		// Distance must always be smaller then the half way from one point to the next point
		// If not, it would be possible that two clothoids intersect
		if(T_DEFAULT > (normVectorB / 3.0))
			return (normVectorB / 3.0);
		else
			return (T_DEFAULT);
	}
}

/*****************************************************************
*** FUNCTIONNAME		  : CalculateClothoidParameter				***
*** FUNCTION			  : Calculates the clothoid parameter A	***
*** RECEIVE-PARAMETER  : clothoidAlpha ... Angle alpha			***
***							 clothoidT ....... Tangential length T	***
*** TRANSMIT-PARAMETER : Clothoid parameter A						***
*** ---------------------------------------------------------- ***
*** Version				  : V 1.1											***
*** Date					  : 14.03.2014										***
*** Author				  : SCHLAGER MANUEL								***
*****************************************************************/
float CalculateClothoidParameter(float clothoidAlpha, float clothoidT)
{
	int numberOfSamplingPoints = 128;
	
	return ((clothoidT * tan(clothoidAlpha / 2.0)) / (Simpson_Rule(f_sinFresnel, 0.0, sqrt(M_PI - clothoidAlpha), numberOfSamplingPoints) + tan(clothoidAlpha / 2.0) * Simpson_Rule(f_cosFresnel, 0.0, sqrt(M_PI - clothoidAlpha), numberOfSamplingPoints)));
}

/**********************************************************************
*** FUNCTIONNAME		  : CalculateClothoidArcLength					  ***
*** FUNCTION			  : Calculates the clothoid arc length s*  	  ***
*** RECEIVE-PARAMETER  : clothoidAlpha ....... Angle alpha			  ***
***							 clothoidParameter ... Clothoid parameter	A ***						     							 
*** TRANSMIT-PARAMETER : Clothoid arc length s*							  ***
*** --------------------------------------------------------------- ***
*** Version				  : V 1.0												  ***
*** Date					  : 13.03.2014											  ***
*** Author				  : SCHLAGER MANUEL									  ***
**********************************************************************/
float CalculateClothoidArcLength(float clothoidAlpha, float clothoidParameter)
{
	return (clothoidParameter * (sqrt(M_PI - clothoidAlpha)));
}

/*****************************************************************************************************************
*** FUNCTIONNAME		  : CalculateClothoidVelocities																				***
*** FUNCTION			  : Calculates the velocities v0 (at start point) and v* (at final point) of the clothoid ***
*** RECEIVE-PARAMETER  : clothoidParameter ... Clothoid parameter A															***
***							 clothoidArcLength ... Clothoid arc length s*														***
*** TRANSMIT-PARAMETER : clothoidVelocitiesResult:																					***
***								[0] ... velocities v0 (at start point) of the clothoid										***
***								[1] ... velocities v* (at final point) of the clothoid										***
*** ---------------------------------------------------------------------------------------------------------- ***
*** Version				  : V 1.1																											***
*** Date					  : 14.03.2014																										***
*** Author				  : SCHLAGER MANUEL																								***
*****************************************************************************************************************/
void CalculateClothoidVelocities(float clothoidParameter, float clothoidArcLength, float* clothoidVelocitiesResult)
{
	// Velocity flags
	float velocityFlag1 = 0.0;
	float velocityFlag2 = 0.0;
	// Velocity v0 (at start point) of the clothoid
	float velocityStart1 = 0.0;
	float velocityStart2 = 0.0;
	// Velocity v* (at final point) of the clothoid
	float velocityEnd1 = 0.0;
	float velocityEnd2 = 0.0;
	// Acceleration a0
	float accelerationa0 = 0.0;
	
	// t*,1 t*,2 for time measurement (comparison of time)
	float t1 = 0.0;
	float t2 = 0.0;
	
	// ****************************************
	// 1. Case
	// ****************************************
	
	// Step 1: Calculate velocity v0 (at start point) of the clothoid 
	velocityFlag1 = sqrt(pow(clothoidParameter, 2.0) * OMEGA_PUNKT_MAX);
	velocityFlag2 = sqrt((4.0 * pow(clothoidParameter, 2.0) * MU_MAX * GRAVITY_OF_EARTH) / clothoidArcLength);
	
	// Saving smaller velocity v0 -> velocityStart1
	if(velocityFlag1 <= velocityFlag2)
		velocityStart1 = velocityFlag1;
	else
		velocityStart1 = velocityFlag2;
	
	// Step 2: Calculate velocity v* (at final point) of the clothoid
	velocityFlag1 = velocityStart1 / sqrt(2);
	velocityFlag2 = sqrt(pow(velocityStart1, 2.0) - ((pow(velocityStart1, 4.0) * clothoidArcLength) / (4.0 * pow(clothoidParameter, 2.0) * MU_MAX * GRAVITY_OF_EARTH)));
	
	// Saving smaller velocity v* -> velocityEnd1
	if(velocityFlag1 <= velocityFlag2)
		velocityEnd1 = velocityFlag1;
	else
		velocityEnd1 = velocityFlag2;
		
	// Step 3: Calculate acceleration a0 in dependency of velocity v0 and v*
	accelerationa0 = ((pow(velocityStart1, 2.0) - pow(velocityEnd1, 2.0)) / (2.0 * clothoidArcLength));
	
	// Step 4: Calculate a new velocity v0 in dependency of A_MAX if necessary
	if(accelerationa0 > A_MAX)
	{
		velocityStart1 = sqrt(2.0 * A_MAX * clothoidArcLength + pow(velocityEnd1, 2.0));
	}
	
	// Step 5: Calculate time t*,1
	t1 = ((2.0 * clothoidArcLength) / (velocityStart1 + velocityEnd1));
	
	// ****************************************
	// 2. Case
	// ****************************************
	
	if((pow(clothoidParameter, 2.0) * OMEGA_PUNKT_MAX) > ((pow(clothoidParameter, 2.0) * MU_MAX * GRAVITY_OF_EARTH) / (clothoidArcLength)))
	{
		// Step 1: Calculate velocity v0 (at start point) of the clothoid
		velocityFlag1 = sqrt(pow(clothoidParameter, 2.0) * OMEGA_PUNKT_MAX);
		velocityFlag2 = sqrt((2.0 * pow(clothoidParameter, 2.0) * MU_MAX * GRAVITY_OF_EARTH) / clothoidArcLength);
		
		// Saving smaller velocity v0 -> velocityStart2
		if(velocityFlag1 <= velocityFlag2)
			velocityStart2 = velocityFlag1;
		else
			velocityStart2 = velocityFlag2;
		
		// Step 2: Calculate velocity v* (at final point) of the clothoid	
		velocityEnd2 = sqrt((pow(clothoidParameter, 2.0) * MU_MAX * GRAVITY_OF_EARTH) / clothoidArcLength);
		
		// Step 3: Calculate acceleration a0 in dependency of velocity v0 and v*
		accelerationa0 = ((pow(velocityStart2, 2.0) - pow(velocityEnd2, 2.0)) / (2.0 * clothoidArcLength));
		
		// Step 4: Calculate a new velocity v0 in dependency of A_MAX if necessary
		if(accelerationa0 > A_MAX)
		{
			velocityStart2 = sqrt(2.0 * A_MAX * clothoidArcLength + pow(velocityEnd2, 2.0));
		}
		
		// Step 5: Calculate time t*,2
		t2 = ((2.0 * clothoidArcLength) / (velocityStart2 + velocityEnd2));
	}
	
	// Step: Finishing
	// If the 2. Case doesn't happen because of the if-Clause => t2 = 0.0 or the time t*,1 <= t*,2
	if((t2 == 0.0) || (t1 <= t2))
	{
		clothoidVelocitiesResult[0] = velocityStart1;
		clothoidVelocitiesResult[1] = velocityEnd1;
	}
	else
	{	
		clothoidVelocitiesResult[0] = velocityStart2;
		clothoidVelocitiesResult[1] = velocityEnd2;
	}
}

/**************************************************************************
*** FUNCTIONNAME		  : f_sinFresnel												***
*** FUNCTION			  : Definition of the function SinusFresnel (SF)	***
*** RECEIVE-PARAMETER  : x ... Value of the function							***
*** TRANSMIT-PARAMETER : Value of SinusFresnel in dependency of value x ***
*** ------------------------------------------------------------------- ***
*** Version				  : V 1.1														***
*** Date					  : 14.03.2014													***
*** Author				  : SCHLAGER MANUEL											***
**************************************************************************/
float f_sinFresnel(float x)
{
	return (sin(0.5*(x*x)));
}

/******************************************************************************
*** FUNCTIONNAME		  : f_cosFresnel													 ***
*** FUNCTION			  : Definition of the function for CosinusFresnel (CF) ***
*** RECEIVE-PARAMETER  : x ... Value of the function								 ***
*** TRANSMIT-PARAMETER : Value of CosinusFresnel in dependency of value x   ***
*** ----------------------------------------------------------------------- ***
*** Version				  : V 1.1															 ***
*** Date					  : 14.03.2014														 ***
*** Author				  : SCHLAGER MANUEL												 ***
******************************************************************************/
float f_cosFresnel(float x)
{
	return (cos(0.5*(x*x)));
}

/*************************************************************************************************
*** FUNCTIONNAME		  : Simpson_Rule (Simpson-Regel)													     ***
*** FUNCTION			  : Numeric Integration with Simpson's rule									     ***
*** RECEIVE-PARAMETER  : *f ... Pointer to function to be integrated: float f (float x)	     ***
***						     a ... Lower Limit of the integration interval 						     ***
***						 	  b ... Upper Limit of the integration interval							     ***
***							  n ... Number of sampling points												  ***
*** TRANSMIT-PARAMETER : Approximated value of the integral of function f(x) in interval [a,b] ***
*** ------------------------------------------------------------------------------------------ ***
*** Version				  : V 1.1																					  ***
*** Date					  : 26.02.2014																			     ***
*** Author				  : SCHLAGER MANUEL																	     ***
*************************************************************************************************/
float Simpson_Rule(float (*f)(float), float a, float b, int n)
{
	int i = 0;
	float h = (b - a) / (2 * n);
	float sum0 = 0.0, sum1 = 0.0, sum2 = 0.0;

	for (i = 0; i <= 2 * n; i++)
	{
		if ((i == 0) || (i == 2 * n))			// Sum f[a] + Sum f[b]
			sum0 += f(a + i * h);
		if (i%2 == 1)								// 4 * Sum
			sum1 += f(a + i * h);
		if ((i%2 == 0) && (i != 2 * n))		// 2 * Sum
			sum2 += f(a + i * h);
	}
	
	return (h / 3) * (sum0 + 4 * sum1 + 2 * sum2);
}