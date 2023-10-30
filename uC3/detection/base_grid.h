/*
 * base_gridh.h
 *
 * Created: 12.05.2020 16:27:28
 *  Author: Richard
 */ 


#ifndef BASE_GRIDH_H_
#define BASE_GRIDH_H_


struct grid{
	
	int xcoordinate, ycoordinate;
};

struct grid_ar{
	
	struct grid* points;
	int elements;
};

struct grid_ar* grid_points;


#endif /* BASE_GRIDH_H_ */