/*
 * clusters_vkl.h
 *
 * Created: 09.05.2020 16:10:59
 *  Author: Richard
 */ 


#ifndef CLUSTERS_VKL_H_
#define CLUSTERS_VKL_H_

#include "clusters.h"
#include "datapoint_da.h"

struct node_t {
	struct cluster data[3];
    int counter;
};

struct node_t* MakeNode(struct cluster* data);              
void Append(struct node_t** ptr, struct node_t* newP);		
int Makeandappendnode(struct node_t** ptr, struct cluster* data);
void deleteNode(struct node_t** listroot, struct cluster data);

#endif /* CLUSTERS_VKL_H_ */