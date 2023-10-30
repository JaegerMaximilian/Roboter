/*
 * cluster.h
 *
 * Created: 11.05.2020 15:34:05
 *  Author: Schalk Rene
 */ 


#ifndef CLUSTER_H_
#define CLUSTER_H_


/*! \brief Structure of the clusters */
struct cluster{
	
	uint16_t locx, locy;
	uint8_t pos, length;
	uint8_t cluster_pos[50];
};

#endif /* CLUSTER_H_ */