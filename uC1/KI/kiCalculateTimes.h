/*
 * kiWatchRobotPosition.h
 *
 * Created: 15.11.2023 20:30:26
 *  Author: marku
 */ 


#ifndef KIWATCHROBOTPOSITION_H_
#define KIWATCHROBOTPOSITION_H_

#include <stdint.h>
#include "global.h"
#include "Pfadplanung.h"



void InitKiCalculateTimesTask(void);
uint8_t KiCalculateTimesTask(void);


#endif /* KIWATCHROBOTPOSITION_H_ */