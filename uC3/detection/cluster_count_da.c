/*
 * cluster_count_da.c
 *
 * Created: 28.07.2020 19:34:39
 *  Author: Rene
 */ 


#include "cluster_count_da.h"
#include <stdlib.h>
#include <stdio.h>
#include "usart.h"
#include "rrt_transmittingtask.h"

void InitClustArray(struct cluster_counter_ar_t* ClustArray)
{
	ClustArray = malloc(sizeof(struct cluster_counter_ar_t));
	ClustArray->count = 0;
}

int add_clust( struct cluster_counter_ar_t* arptr, uint8_t data){
	
	//char text[100];
	if(arptr->count == 0) {
		
		struct cluster_counter_t* ptr = malloc(sizeof(struct cluster_counter_t));  //(arptr->count + 1) * sizeof(struct datapoint_t)
		if(ptr == 0) {
			
			return 0;
		}
		arptr->cluster_counter_ar = ptr;
		arptr->cluster_counter_ar[0].pos = data; 
		arptr->count++;
	}
	else {
		
		struct cluster_counter_t* ptr = realloc(arptr->cluster_counter_ar, (arptr->count + 1) * sizeof(struct cluster_counter_t));
		arptr->cluster_counter_ar = ptr; 
		if (ptr == 0) {
			
			return 0;
		}

		arptr->cluster_counter_ar[arptr->count].pos = data;
		arptr->count++;
		
	}
	
	// 	sprintf(text,"%.d;%.d\r\n", arptr->datapoint_ar[arptr->count-1].x, arptr->datapoint_ar[arptr->count-1].y);
	//  debugMsg(text);
	
	return 1;
}

int delete_clust_array(struct cluster_counter_ar_t* arptr){
	
	free(arptr);
}