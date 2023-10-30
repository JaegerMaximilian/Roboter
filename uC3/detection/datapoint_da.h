/*
 * datapoint_da.h
 *
 * Created: 24.04.2020 21:21:34
 *  Author: Richard
 */ 


#ifndef DATAPOINT_DA_H_
#define DATAPOINT_DA_H_

#include "datapoint.h"

//Struktur f�r die Datapointarrays
struct datapoint_ar_t{
	
	struct datapoint_t datapoint_ar[50];
	int count;
};


//Verwendete Arrays der Objekterkennung
struct datapoint_ar_t* Outputdata, *sorted_datapoints, *arrayptr;

//Initialisieren der Arrays in welchen die Datenpunkte gespeichert werden
void InitDataPoint();
//Initialisieren des Arrays f�r die sortierten Datanpunkte
void InitSortedDatapoints();
//L�schen der Arrays
int delete_all_arrays();

#endif /* DATAPOINT_DA_H_ */