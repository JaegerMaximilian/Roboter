/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      ANTRIEB.H
Version :  V 1.0
Date    :  07.03.2011
Author  :  MICHAEL ZAUNER

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


#ifndef _ANTRIEB_H
#define _ANTRIEB_H



#ifndef _ANTRIEB_EXTERN
   #define _ANTRIEB_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                         ***
**************************************************************************/
#define T_ABTAST_ANTRIEB		6		// AntriebTask()

_ANTRIEB_EXTERN float istX_, istY_, istPhi_;
_ANTRIEB_EXTERN unsigned int uiXOffset, uiYOffset, uiWinkelOffset;
_ANTRIEB_EXTERN unsigned char ucSetGlobalPosition; 
_ANTRIEB_EXTERN signed int siXPosition, siYPosition, siWinkel, siV;


/**************************************************************************
***                        Prototypen-Definition                        ***
**************************************************************************/
void Init_WMRctrlVelPosOdo0(void);
void SetGlobalPosition(void);
void getINC(long*R,long*L);
void InitAntrieb(void);
unsigned char AntriebTask(void);


#endif
