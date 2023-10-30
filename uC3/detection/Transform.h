/*
 * Transform.h
 *
 * Created: 24.04.2020 15:17:00
 *  Author: Schalk Rene
 */ 


#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include "datapoint.h"
#include "datapoint_da.h"
#include "rplidar.h"
#include "clusters.h"
#include "global.h"

// Transformiert einen gescanten Punkt und fügt ihn in das Array mit den Datenpunkten ein, wenn er sich
//im vorher definierten Arbeitsbereich befindet
int transform(rpLidar_Scan_t* scan);

objectdetectionoutput_t oldoutputdetection;
objectdetectionoutput_t olddetectionotherrobot;

uint16_t initcenter[4][2];

//Initialisierung der Objekterkennung im Multitasking System
void evaluation_Init();

#endif /* TRANSFORM_H_ */