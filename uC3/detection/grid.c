/*
 * grid.c
 *
 * Created: 12.05.2020 15:32:24
 *  Author: Rene
 */ 

#include "grid.h"
#include <stdlib.h>
#include "base_grid.h"
#include "usart.h"
#include <util\delay.h>
#include "rrt_transmittingtask.h"

int initialize_grid(){
	
	int k, aktx=100, akty=100;
	 
	grid_points = malloc(sizeof(struct grid));
	grid_points->elements = 0;

	struct grid_ar* ptr = realloc(grid_points->points, 551 * sizeof(struct grid));
	grid_points->points = ptr; // des woa a safety first soch richtig?
	
	grid_points->elements=551;
	
	if (ptr == 0) {
			
		return 0;
	}
	 
	 for(k=0; k<601; k++){
		
		 grid_points->points[k].xcoordinate=aktx;
		 grid_points->points[k].ycoordinate=akty;
		 
		 aktx=aktx+100;
		 
		 if(aktx==3000){
		 
			aktx=100;
			akty=akty+100;
		 }
	 }
		 
		 char text[100];
		 
// 		 for(int i=0; i<(29*19-1); i++){
// 			 
// 			 sprintf(text,"%.d;%.d\r\n", grid_points->points[i].xcoordinate, grid_points->points[i].ycoordinate);
// 			 debugMsg(text);
// 			 _delay_ms(10);
// 		 }

	 
	if(grid_points->points[(29*19)-1].xcoordinate!=0){
		
		return 1;
	}
	return 0;
}