/*
 * KiWatch.h
 *
 * Created: 15.11.2023 20:09:37
 *  Author: marku
 */ 


#ifndef KIWATCH_H_
#define KIWATCH_H_

#include <stdint.h>
#include "global.h"
#include "Pfadplanung.h"

void InitKiWatch(void);
uint8_t KiWatchTask(void);

#define MaxCountInArea 10


#endif /* KIWATCH_H_ */