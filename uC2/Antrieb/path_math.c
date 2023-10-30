/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      PATH_MATH.c
Version :  V 1.0
Date    :  05.03.2014
Author  :  INFANGER SIMON, SCHLAGER MANUEL, ZAUNER MICHAEL 


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

#define _PATH_MATH_EXTERN

#include <avr/io.h> 
#include "global.h"  
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "path_math.h"
#include "define.h"

/***************************************************************************************************
*** FUNCTIONNAME		  : AngleToXAxis2D (Winkel zwischen der X-Achse und Vektor)				       ***
*** FUNCTION			  : Calculates the angle between the unit vector of the X-axis and a vector ***
*** RECEIVE-PARAMETER  : vector ... 2-dimensional vector														 ***
*** TRANSMIT-PARAMETER : Angle between the unit vector and a vector										 ***
*** -------------------------------------------------------------------------------------------- ***
*** Version				  : V 1.0																						 ***
*** Date					  : 05.03.2014																					 ***
*** Author				  : SCHLAGER MANUEL, ZAUNER MICHAEL														 ***
***************************************************************************************************/
float AngleToXAxis2D(float* vector)
{
	float phi;
	float e_x[2] = {1, 0};
	float delta_x, delta_y;
	
	// Calculating phi_max -> Angle between the X-axis and the vector: Initial point <-> Destination point
	phi = acos(Dot2D(e_x, vector) / (Norm2D(e_x) * Norm2D(vector)));
	
	delta_y = vector[Y_POS] - e_x[Y_POS];
	delta_x = vector[X_POS] - e_x[X_POS];
	
	// If necessary, correction of the angle
	if((delta_y < 0.0) && (delta_x < 0.0))
		phi = 2 * M_PI - phi;
	if((delta_y < 0.0) && (delta_x > 0.0))
		phi = 2 * M_PI - phi;

	return (phi);
}

/********************************************************************************
*** FUNCTIONNAME		  : Angle2D (Winkel zwischen Vektoren)							***
*** FUNCTION			  : Calculates the angle between the vectorA and vectorB ***
*** RECEIVE-PARAMETER  : vectorA ... 2-dimensional vector							***
***						    vectorB ... 2-dimensional vector							***
*** TRANSMIT-PARAMETER : Angle (rad/s) between the vectors                    ***
*** ------------------------------------------------------------------------- ***
*** Version				  : V 1.0														      ***
*** Date					  : 05.03.2014												         ***
*** Author				  : SCHLAGER MANUEL, ZAUNER MICHAEL								***
********************************************************************************/
float Angle2D(float* vectorA, float* vectorB)
{
	float alpha = 0.0, alpha_1 = 0.0, alpha_2 = 0.0;

	alpha_1 = AngleToXAxis2D(vectorA);		// Angle between the vectorA and the unit vector e_x
	alpha_2 = AngleToXAxis2D(vectorB);		// Angle between the vectorB and the unit vector e_x

	alpha = (alpha_2 - alpha_1);				// Angle between the vectorA and vectorB

	if(alpha < 0)
		alpha += 2 * M_PI;

	return (alpha);
}

/********************************************************************************
*** FUNCTIONNAME		  : Dot2D (Skalarprodukt)										   ***
*** FUNCTION			  : Calculates the dot product of a 2-dimensional vector ***
*** RECEIVE-PARAMETER  : vectorA ... 2-dimensional vector							***
***						    vectorB ... 2-dimensional vector							***
*** TRANSMIT-PARAMETER : Scalar of vector A nad vectorB                       ***
*** ------------------------------------------------------------------------- ***
*** Version				  : V 1.1														      ***
*** Date					  : 03.03.2014												         ***
*** Author				  : INFANGER SIMON							  						***
********************************************************************************/
float Dot2D(float* vectorA, float* vectorB)
{
	return (vectorA[X_POS] * vectorB[X_POS] + vectorA[Y_POS] * vectorB[Y_POS]);
}

/*************************************************************************
*** FUNCTIONNAME		  : Norm2D (Betrag) 										  ***
*** FUNCTION			  : Calculates the norm of a 2-dimensional vector ***
*** RECEIVE-PARAMETER  : vector ... 2-dimensional vector					  ***
*** TRANSMIT-PARAMETER : Norm of the vector                            ***
*** ------------------------------------------------------------------ ***
*** Version				  : V 1.1													  ***
*** Date					  : 03.03.2014												  ***
*** Author				  : INFANGER SIMON						    			  ***
*************************************************************************/
float Norm2D(float* vector)
{
	return ((float)(sqrt(pow(vector[X_POS], 2.0) + pow(vector[Y_POS], 2.0))));
}

/************************************************************************
*** FUNCTIONNAME		  : Normalize2D (Einheitsvektor)  					 ***
*** FUNCTION			  : Creates a unit vector					          ***
*** RECEIVE-PARAMETER  : vector ... 2-dimensional vector					 ***
***						  : vectorResult ... 2-dimensional result vector ***
*** TRANSMIT-PARAMETER : vectorResult ... Unit vector                 ***
*** ----------------------------------------------------------------- ***
*** Version				  : V 1.0													 ***
*** Date					  : 05.03.2014											    ***
*** Author				  : INFANGER SIMON    								    ***
************************************************************************/
void Normalize2D(float* vector, float* vectorResult)
{
	float norm = Norm2D(vector);
	
	vectorResult[X_POS] = vector[X_POS] / norm;
	vectorResult[Y_POS] = vector[Y_POS] / norm;
}

/************************************************************
*** FUNCTIONNAME		  : Scale2D (Skalierung)				 ***
*** FUNCTION			  : Creates a scaled vector			 ***
*** RECEIVE-PARAMETER  : vector ... 2-dimensional vector  ***
***						    factor ... Multiplication factor ***
*** TRANSMIT-PARAMETER : vectorResult ... Scaled vector   ***
*** ----------------------------------------------------- ***
*** Version				  : V 1.0								    ***
*** Date					  : 05.03.2014								 ***
*** Author				  : INFANGER SIMON    					 ***
************************************************************/
void Scale2D(float* vector, float factor, float* vectorResult)
{
	vectorResult[X_POS] = vector[X_POS] * factor;
	vectorResult[Y_POS] = vector[Y_POS] * factor;
}

/*****************************************************************************
*** FUNCTIONNAME		  : VectorChangeOrientation2D (Orientierung umkehren) ***
*** FUNCTION			  : Changes the orientiation of a vector					***
*** RECEIVE-PARAMETER  : vector ... 2-dimensional vector						   ***
*** TRANSMIT-PARAMETER : vectorResult ... Changed vector                   ***
*** ---------------------------------------------------------------------- ***
*** Version				  : V 1.0														   ***
*** Date					  : 05.03.2014													   ***
*** Author				  : INFANGER SIMON    										   ***
*****************************************************************************/
void ChangeOrientation2D(float* vector, float* vectorResult)
{
	vectorResult[X_POS] = -vector[X_POS];
	vectorResult[Y_POS] = -vector[Y_POS];
}

/*******************************************************************
*** FUNCTIONNAME		  : NormalVector2D (Normalvektor bilden)	  ***
*** FUNCTION			  : Generates the normal vector of a vector ***
*** RECEIVE-PARAMETER  : vector ... 2-dimensional vector			  ***
*** TRANSMIT-PARAMETER : vectorResult ... Normal vector          ***
*** ------------------------------------------------------------ ***
*** Version				  : V 1.0											  ***
*** Date					  : 17.03.2014										  ***
*** Author				  : SCHLAGER MANUEL    							  ***
*******************************************************************/
void NormalVector2D(float* vector, float* vectorResult)
{
	vectorResult[X_POS] = -vector[Y_POS];
	vectorResult[Y_POS] = vector[X_POS];
}