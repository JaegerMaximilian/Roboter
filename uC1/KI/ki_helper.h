/*
 * ki_helper.h
 *
 * Created: 15.11.2023 19:14:37
 *  Author: marku
 */ 


#ifndef KI_HELPER_H_
#define KI_HELPER_H_

#include <stdint.h>
#include "global.h"
#include "Pfadplanung.h"
#include "ki.h"

point_t AddMiddlePoint(point_t start, point_t ziel);
void ChangePrioToYellow(void);
void ActivatePlantAsObstacle(void);
uint8_t DriveBack(uint8_t distance, uint8_t speed);
float CalcDistance(point_t firstPoint, point_t secondPoint);
void RePrioritisePlantTasks(void);
void CalcOpenPlants(void);
void CalcOpenParkPositions(void);
void CalcOpenPlanter(void);
void Repreoritise_SolarPanels(void);
uint8_t CalcTimeRemainingPlants(void);


// ****************************************************
// Dot2D, Norm2D, AngleToXAxis2D from path_math.c (µC2)
// ****************************************************
float Dot2D(float* vectorA, float* vectorB);
float Norm2D(float* vector);
float AngleToXAxis2D(float* vector);

// ****************************************************
// GetPx, GetPy for shooting the mammoth
// ****************************************************
int16_t GetPx(int16_t Mx, int16_t My);
int16_t GetPy(int16_t Mx, int16_t My);

#endif /* KI_HELPER_H_ */