/*
 * datapoint.h
 *
 * Created: 24.04.2020 20:46:33
 *  Author: Richard
 */ 


#ifndef DATAPOINT_H_
#define DATAPOINT_H_

#include <stdlib.h>
#include <avr/io.h>

struct datapoint_t
{
	
	uint16_t x, y, distance, angle; // position in the coordinate system
};



#endif /* DATAPOINT_H_ */