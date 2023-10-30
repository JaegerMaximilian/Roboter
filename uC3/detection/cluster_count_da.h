/*
 * cluster_count_da.h
 *
 * Created: 28.07.2020 19:27:48
 *  Author: Rene
 */ 


#ifndef CLUSTER_COUNT_DA_H_
#define CLUSTER_COUNT_DA_H_


#include "cluster_count.h"

struct cluster_counter_ar_t{
	
	struct cluster_counter_t* cluster_counter_ar;
	int count;
};

void initClustArray(struct cluster_counter_ar_t* ClustArray);
void InitOutput();
int addclust( struct cluster_counter_ar_t* arptr, struct cluster_counter_t* data);



#endif /* CLUSTER_COUNT_DA_H_ */