/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      LCDSPIELFARBE.H
Version :  V 1.0
Date    :  07.09.2010
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
#ifndef START_H
#define START_H

#ifndef _START_EXTERN
#define _START_EXTERN extern
#endif



/**************************************************************************
***            Variablen-Definition                                     ***
**************************************************************************/


/**************************************************************************
***            Prototypen-Definition                                    ***
**************************************************************************/
void InitStart(void);
unsigned char StartTask(void);

#endif
