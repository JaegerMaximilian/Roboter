/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Includes general definitions, e.g. task-numbers,... .
 *
 *      
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. 
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 *
 *
 * \par Documentation
 *      The file includes general definitions.
 *
 * \author
 *      Michael Zauner
 *      RRT (University of Applied Sciences Upper Austria)  http://rrt.fh-wels.at \n
 *      Support email: roboracing@fh-wels.at
 *
 * $Revision: 1 $
 * $Date: 2012-09-11  $  \n
 *
 * Copyright (c) 2012, RRT (University of Applied Sciences Upper Austria) All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of RRT may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RRT (University of Applied Sciences Upper Austria) 
 * "AS IS" AND ANY EXPRESS OR IMPLIED  * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/ 


#ifndef _DEFINE_H
#define _DEFINE_H

#include "global.h"



/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
#define _MASTER_


#define TRUE      1
#define FALSE     0

#define INPUT     0
#define OUTPUT    1

#define ON        1
#define OFF       0

#define STATUS_LED	




/* task numbers */
#define DEBUG_TASKNBR							0
#define MOTOR_TASKNBR							1
#define EA_EDIPTFT_TASKNBR						2
#define EA_EDIPTFT_SBUF_TASKNBR					3
#define SERIAL_RECEIVE_TASKNBR					4
#define TIMEOUT_TASKNBR							5
#define TIMEOUTMANAGER_TASKNBR					6
#define TRANSMITTING_TASKNBR_1					7
#define RRTLAN_VALUE_TASKNBR					8


#define GUI_TASKNBR								13
#define EA_EDIPTFT_SBUF_CHECK_TASKNBR			14

// LCD
#define DEBUG_MSG_PORTNBR			201
#define EINSTELLUNG_PORTNBR			202
#define VALUE_PORTNBR				203




#endif
