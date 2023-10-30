/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      PATH_MATH.h
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

#ifndef _PATH_MATH_H
#define _PATH_MATH_H

#ifndef _PATH_MATH_EXTERN
	#define _PATH_MATH_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
#define X_POS		0
#define Y_POS		1

/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
float AngleToXAxis2D(float* vector);
float Angle2D(float* vectorA, float* vectorB);
float Dot2D(float* vectorA, float* vectorB);
float Norm2D(float* vector);
void Normalize2D(float* vector, float* vectorResult);
void Scale2D(float* vector, float factor, float* vectorResult);
void ChangeOrientation2D(float* vector, float* vectorResult);
void NormalVector2D(float* vector, float* vectorResult);

#endif