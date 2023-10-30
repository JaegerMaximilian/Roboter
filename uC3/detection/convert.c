/*
 * convert.c
 *
 * Created: 12.05.2020 15:32:04
 *  Author: Richard
 */ 

#include "convert.h"
#include <stdlib.h>
#include "usart.h"
#include <util\delay.h>
#include "rrt_transmittingtask.h"

void convert_points(struct grid_ar* gridpoints){
	
	int16_t i, k, dx, dy;
	char text[100];
	
	for(i=0; i < sorted_datapoints->count; i++){
		
		//sprintf(text,"\r\n%.d\r\n", i);
		//debugMsg(text);
		//_delay_ms(20);
		
		for (k=0; k<gridpoints->elements; k++){
			
			
	
				
			dx = (int16_t)sorted_datapoints->datapoint_ar[i].x-(int16_t)gridpoints->points[k].xcoordinate;
		
			dy = (int16_t)sorted_datapoints->datapoint_ar[i].y-(int16_t)gridpoints->points[k].ycoordinate;
		
			
			sprintf(text,"%.d;%.d\r\n", dx, dy);
  			//debugMsg(text);	
			
			_delay_ms(20);
			
			if((dx <= 50) && (dy <= 50) && (dx >= -50) && (dy >= -50)){
			
				sprintf(text,"\r\n%.d\r\n\r\n", i);
				//debugMsg(text);
				//_delay_ms(20);
				sorted_datapoints->datapoint_ar[i].x=gridpoints->points[k].xcoordinate;
				sorted_datapoints->datapoint_ar[i].y=gridpoints->points[k].ycoordinate;
				break;
			}
		}
	}
}