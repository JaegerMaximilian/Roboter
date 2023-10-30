/*
 * spielZeit.h
 *
 * Created: 10.12.2019 12:40:59
 *  Author: P20087
 */ 


#ifndef SPIELZEIT_H_
#define SPIELZEIT_H_

/* intern/extern switch */
#ifndef _SPIELZEIT_EXTERN
   #define _SPIELZEIT_EXTERN extern
#endif

/* **************************** */
/* ***     enumerators      *** */
/* **************************** */
#define MAX_SPIELZEIT		100

/* **************************** */
/* ***      prototypes      *** */
/* **************************** */
void spielZeit_init();
uint8_t spielZeitTask();


#endif /* SPIELZEIT_H_ */