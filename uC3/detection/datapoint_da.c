/*
 * datapoint_da.c
 *
 * Created: 24.04.2020 21:21:19
 *  Author: Richard
 */ 

#include "datapoint_da.h"
#include <stdlib.h>
#include <stdio.h>
#include "usart.h"
#include "global.h"
#include "clusters.h"
#include "Transform.h"

//Initialisieren der Arrays in welchen die Datenpunkte gespeichert werden
void InitDataPoint()
{

	arrayptr = (struct datapoint_ar_t*) malloc(sizeof(struct datapoint_ar_t));
	arrayptr->count = 0;
	
//  objectcenter.center[0][0] = 2850;
//  objectcenter.center[0][1] = 150;
//  	
//  objectcenter.center[1][0] = 2150;
//  objectcenter.center[1][1] = 500;
//  	
//  objectcenter.center[2][0] = 2000;
//  objectcenter.center[2][1] = 1300;
//  	
//  objectcenter.center[3][0] = 2800;
//  objectcenter.center[3][1] = 1100;

	initcenter[0][0] = 133;
	initcenter[0][1] = 1042;
 
	initcenter[1][0] = 229;
	initcenter[1][1] = 1346;
 
	initcenter[2][0] = 2867;
	initcenter[2][1] = 1042;
 
	initcenter[3][0] = 2771;
	initcenter[3][1] = 1346;

	objectcenter.count = 3;
}

//Initialisieren des Arrays für die sortierten Datanpunkte
void InitSortedDatapoints()
{
	sorted_datapoints = (struct datapoint_ar_t*)malloc(sizeof(struct datapoint_ar_t));
	sorted_datapoints->count = 0;
}

//Löschen der Arrays
int delete_all_arrays()
{
	
	arrayptr->count = 0;
	sorted_datapoints->count = 0;
	
	for(uint8_t i =0; i<firstclust.counter; i++)
	{
		
		firstclust.data[i].pos = 0;
		
		for(uint8_t k = 0; k<firstclust.data[i].length; k++)
		{
			
			firstclust.data[i].cluster_pos[k] = 0;
		}
		
		firstclust.data[i].length = 0;
	}
}