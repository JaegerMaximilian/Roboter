/***************************************************************
  
              ************************************
              ** FH OBEROESTERREICH CAMPUS WELS **
              ************************************
  
Project :  MODULARER ROBOTER FOR TOURNAMENTS
Modul:     EVERYONE
File:      RECEIVETASK.H
Version :  V 1.0
Date    :  25.02.2008
Author  :  MUCKENHUMER BERNHARD

Comments: 

Last edit: 
Programmchange: 

                *)....
                *).....

Chip type           : ATXmega256a3
Program type        : Application
Clock frequency     : 32,000000 MHz
Memory model        : Small
External SRAM size  : 0
Data Stack size     : 1024

                 Copyright (c) 2008 by FH-Wels
                     All Rights Reserved.  
****************************************************************/


#ifndef RRTLAN_RECEIVETASK_H
#define RRTLAN_RECEIVETASK_H


#ifndef RRTLAN_RECEIVETASK_EXTERN
#define RRTLAN_RECEIVETASK_EXTERN extern
#endif

/**************************************************************************
***                        Variablen-Definition                               ***
**************************************************************************/
 

/**************************************************************************
***                          Prototypen-Definition                           ***
**************************************************************************/
void InitReceivetask(void);
unsigned char ReceiveTask(void);



#endif


