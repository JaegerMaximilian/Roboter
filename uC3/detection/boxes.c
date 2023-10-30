/*
 * boxes.c
 *
 * Created: 07.07.2020 14:18:07
 *  Author: Rene
 */ 

#include "boxes.h"
#include "usart.h"
#include "clusters_vkl.h"
#include <util/delay.h>
#include "rrt_transmittingtask.h"
#include <stdio.h>

int comparexasc( const void* a, const void* b) //nach x in aufsteigender reihenfolge
{

	const struct datapoint_t* dataPoint1 = (struct datapoint_t*) a;
	const struct datapoint_t* dataPoint2 = (struct datapoint_t*) b;

	if ( dataPoint1->x == dataPoint2->x ) return 0;
	else if ( dataPoint1->x < dataPoint2->x ) return -1;
	else return 1;
}
int comparexdesc( const void* a, const void* b)//nach x in absteigender reihenfolge
{
	
	const struct datapoint_t* dataPoint1 = (struct datapoint_t*) a;
	const struct datapoint_t* dataPoint2 = (struct datapoint_t*) b;

	if ( dataPoint1->x == dataPoint2->x ) return 0;
	else if ( dataPoint1->x > dataPoint2->x ) return -1;
	else return 1;
}
int compareyasc( const void* a, const void* b) //nach y in aufsteigender reihenfolge
{

	const struct datapoint_t* dataPoint1 = (struct datapoint_t*) a;
	const struct datapoint_t* dataPoint2 = (struct datapoint_t*) b;

	if ( dataPoint1->y == dataPoint2->y ) return 0;
	else if ( dataPoint1->y < dataPoint2->y ) return -1;
	else return 1;
}
int compareydesc( const void* a, const void* b)//nach y in absteigender reihenfolge
{
	
	const struct datapoint_t* dataPoint1 = (struct datapoint_t*) a;
	const struct datapoint_t* dataPoint2 = (struct datapoint_t*) b;

	if ( dataPoint1->y == dataPoint2->y ) return 0;
	else if ( dataPoint1->y > dataPoint2->y ) return -1;
	else return 1;
}


//
//
//void create_hor_strline_bel_left(struct datapoint_ar_t* clustdata, int xdistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){ // horizontal box cause of straight line or small angle from below 
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//
	//
	//char text[100];
//
	//sprintf(text,"%.d; %.d\r\n", clustdata->datapoint_ar[current_cluster->data.pos].x, current_cluster->data.pos);
	////debugMsg(text);
//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//if(xdistance>200){
		//xdistance=200;
	//}
	//
	//xdistance=xdistance+100;
	//
	//double down= fabs((uint32_t)100*(uint32_t)100), up= fabs((uint32_t)xdistance*(uint32_t)xdistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//
	//
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(xact==clustdata->datapoint_ar[current_cluster->data.pos].x+xdistance){
			//
			//yact=yact+100;
			//xact=clustdata->datapoint_ar[current_cluster->data.pos].x;
		//}
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//sprintf(text,"%.d\r\n", i);
		////debugMsg(text);
		//
		//xact=xact+100;
		//
		//add(dataoutput, &dat);
	//}
//
//}
//void create_hor_strline_bel_right(struct datapoint_ar_t* clustdata, int xdistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//char text[100];
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexdesc);
	//
	//if(xdistance>200){
		//xdistance=200;
	//}
	//
	//xdistance=xdistance+100;
	//
	//double down = fabs((uint32_t)100*(uint32_t)100),  up = fabs((uint32_t)xdistance*(uint32_t)xdistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(xact==clustdata->datapoint_ar[current_cluster->data.pos].x-xdistance){
			//
			//yact=yact+100;
			//xact=clustdata->datapoint_ar[current_cluster->data.pos].x;
		//}
		//
		//sprintf(text,"%.d\r\n", i);
		////debugMsg(text);
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//xact=xact-100;
		//
		//add(dataoutput, &dat);
	//}
//}
//void create_ang_peak_bel(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareyasc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x+100;
		//dat.y=dat.y+100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-200;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-300;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y+100;
			//}
		//}
	//}
//}
//
//
//void create_hor_strline_abv_left(struct datapoint_ar_t* clustdata, int xdistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){ // horizontal box cause of straight line or small angle from above
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//
	//if(xdistance>200){
		//xdistance=200;
	//}
	//
	//xdistance=xdistance+100;
	//
	//double down= fabs((uint32_t)100*(uint32_t)100), up= fabs((uint32_t)xdistance*(uint32_t)xdistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(xact==clustdata->datapoint_ar[current_cluster->data.pos].x+xdistance){
			//
			//yact=yact-100;
			//xact=clustdata->datapoint_ar[current_cluster->data.pos].x;
		//}
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//xact=xact+100;
		//
		//add(dataoutput, &dat);
	//}
//
//}
//void create_hor_strline_abv_right(struct datapoint_ar_t* clustdata, int xdistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){ 
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexdesc);
	//
	//if(xdistance>200){
		//xdistance=200;
	//}
	//
	//xdistance=xdistance+100;
	//
	//double down= fabs((uint32_t)100*(uint32_t)100), up= fabs((uint32_t)xdistance*(uint32_t)xdistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(xact==clustdata->datapoint_ar[current_cluster->data.pos].x-xdistance){
			//
			//yact=yact-100;
			//xact=clustdata->datapoint_ar[current_cluster->data.pos].x;
		//}
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//xact=xact-100;
		//
		//add(dataoutput, &dat);
	//}
//
//}
//void create_ang_peak_abv(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareydesc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x+100;
		//dat.y=dat.y-100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-200;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-300;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-100;
			//}
		//}
	//}
//}
//
//
//void create_hor_strline_left_bel(struct datapoint_ar_t* clustdata, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){ // horizontal box cause of straight line or small angle from left
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareyasc);
	//
	//if(ydistance>200){
		//ydistance=200;
	//}
	//
	//ydistance=ydistance+100;
	//
	//double down= fabs((uint32_t)100*(uint32_t)100), up= fabs((uint32_t)ydistance*(uint32_t)ydistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(yact==clustdata->datapoint_ar[current_cluster->data.pos].y+ydistance){
			//
			//xact=xact+100;
			//yact=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//yact=yact+100;
		//
		//add(dataoutput, &dat);
	//}
//
//}
//void create_hor_strline_left_abv(struct datapoint_ar_t* clustdata, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){ // horizontal box cause of straight line or small angle from left
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareydesc);
	//
	//if(ydistance>200){
		//ydistance=200;
	//}
	//
	//ydistance=ydistance+100;
	//
	//double down= fabs((uint32_t)100*(uint32_t)100), up= fabs((uint32_t)ydistance*(uint32_t)ydistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(yact==clustdata->datapoint_ar[current_cluster->data.pos].y-ydistance){
			//
			//xact=xact+100;
			//yact=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//yact=yact-100;
		//
		//add(dataoutput, &dat);
	//}
//}
//void create_ang_peak_left(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x+100;
		//dat.y=dat.y+100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-200;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x+100;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-300;
			//}
		//}
	//}
//}
//
//
//void create_hor_strline_right_bel(struct datapoint_ar_t* clustdata, int ydistance, struct datapoint_ar_t* dataoutput, struct node_t* current_cluster){ // horizontal box cause of straight line or small angle from above
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//char text[100];
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareyasc);
	//
	//if(ydistance>200){
		//
		//ydistance=200;
	//}
	//
	//ydistance=ydistance+100;
	//
	//double down= fabs((uint32_t)100*(uint32_t)100), up= fabs((uint32_t)ydistance*(uint32_t)ydistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//sprintf(text,"d%.2f; %.d; %.d\r\n", up, ydistance, number_of_points);
	////debugMsg(text);
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(yact>=clustdata->datapoint_ar[current_cluster->data.pos].y+ydistance){
			//
			//xact=xact-100;
			//yact=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//sprintf(text,"n%.d; %.d; %.d; %.d; %.d\r\n", i, clustdata->datapoint_ar[current_cluster->data.pos].x, clustdata->datapoint_ar[current_cluster->data.pos].y, xact, yact);
		////debugMsg(text);
		//
		//yact=yact+100;
		//
		//add(dataoutput, &dat);
	//}
//
//}
//void create_hor_strline_right_abv(struct datapoint_ar_t* clustdata, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){ // horizontal box cause of straight line or small angle from above
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareydesc);
	//
	//if(ydistance>200){
		//ydistance=200;
	//}
	//
	//ydistance=ydistance+100;
	//
	//double down= fabs((uint32_t)100*(uint32_t)100), up= fabs((uint32_t)ydistance*(uint32_t)ydistance);
	//
	//uint8_t number_of_points;
	//
	//number_of_points = up/down;
	//
	//int xact=clustdata->datapoint_ar[current_cluster->data.pos].x, yact= clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//for(int i=0; i<number_of_points; i++){
		//
		//if(yact==clustdata->datapoint_ar[current_cluster->data.pos].y-ydistance){
			//
			//xact=xact-100;
			//yact=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//
		//dat.x=xact;
		//dat.y=yact;
		//
		//yact=yact-100;
		//
		//add(dataoutput, &dat);
	//}
//}
//void create_ang_peak_right(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.pos+current_cluster->data.length,sizeof(struct datapoint_t),comparexdesc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x-100;
		//dat.y=dat.y+100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-200;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-100;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-300;
			//}
		//}
	//}
//}
//
//
//void create_ang_diag_bel_left(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//char text[100];
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//sprintf(text,"n%.d; %.d\r\n", current_cluster->data.pos, current_cluster->data.length);
	//debugMsg(text);
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x+100;
		//dat.y=dat.y-100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y+200;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x+100;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y+300;
			//}
		//}
	//}
//}
//void create_ang_diag_bel_right(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.pos+current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x+100;
		//dat.y=dat.y+100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-200;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-300;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y+100;
			//}
		//}
	//}
//}
//void create_ang_diag_abv_left(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x+100;
		//dat.y=dat.y+100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-200;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x+100;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-300;
			//}
		//}
	//}
//}
//void create_ang_diag_abv_right(struct datapoint_ar_t* clustdata,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//struct datapoint_t dat;
	//dat.angle=0;
	//dat.distance=0;
	//
	//qsort(&clustdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x;
	//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
	//
	//add(dataoutput, &dat);
	//
	//for(int i=0; i<8; i++){
		//
		//dat.x=dat.x+100;
		//dat.y=dat.y-100;
		//
		//add(dataoutput, &dat);
		//
		//if(i==1){
			//
			//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-200;
			//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y;
		//}
		//else{
			//
			//if(i==4){
				//
				//dat.x=clustdata->datapoint_ar[current_cluster->data.pos].x-300;
				//dat.y=clustdata->datapoint_ar[current_cluster->data.pos].y-100;
			//}
		//}
	//}
//}
//
//
//void robot_left_from_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance, struct datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//if(xdistance==0){ //horizontalbox building from the left side
		//
		//create_hor_strline_left_bel(clusterdata, ydistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareyasc);
		//
		//if(clusterdata->datapoint_ar[current_cluster->data.pos].x == clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length-1].x){ // one peak toward the robot
			//
			//create_ang_peak_left(clusterdata, dataoutput, current_cluster);
		//}
		//else{ //robot has a small angle therefore a horizontal box will be created
			//
			//create_hor_strline_left_bel(clusterdata, ydistance, dataoutput, current_cluster);
		//}
	//}
//}
//void robot_right_from_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance, struct datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//if(xdistance==0){ //horizontalbox building from the right side
		//
		//create_hor_strline_right_bel(clusterdata, ydistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareyasc);
		//
		//if(clusterdata->datapoint_ar[current_cluster->data.pos].x == clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length-1].x){ // one peak toward the robot
			//
			//create_ang_peak_right(clusterdata, dataoutput, current_cluster);
		//}
		//else{ //robot has a small angle therefore a horizontal box will be created
			//
			//create_hor_strline_right_bel(clusterdata, ydistance, dataoutput, current_cluster);
		//}
	//}
//}
//
//
//void robot_middle_below_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//
	//
	//if(ydistance==0){ //horizontalbox building from the left side
		//
		//create_hor_strline_bel_left(clusterdata, ydistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
		//
		//if(clusterdata->datapoint_ar[current_cluster->data.pos].y == clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length-1].y){ // one peak toward the robot
			//
			//create_ang_peak_bel(clusterdata, dataoutput, current_cluster);
		//}
		//else{ //robot has a small angle therefore a horizontal box will be created
			//
			//create_hor_strline_bel_left(clusterdata, ydistance, dataoutput, current_cluster);
		//}
	//}
//}
//void robot_middle_above_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance, struct datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//if(ydistance==0){ //horizontalbox building from the left side
		//
		//create_hor_strline_abv_left(clusterdata, ydistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
		//
		//if(clusterdata->datapoint_ar[current_cluster->data.pos].y == clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length-1].y){ // one peak toward the robot
			//
			//create_ang_peak_abv(clusterdata, dataoutput, current_cluster);
		//}
		//else{ //robot has a small angle therefore a horizontal box will be created
			//
			//create_hor_strline_abv_left(clusterdata, ydistance, dataoutput, current_cluster);
		//}
	//}
//}
//
//
//
//
//void robot_left_below_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//if(xdistance>ydistance){
		//
		//create_hor_strline_bel_left(clusterdata, xdistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//if(xdistance<ydistance){
			//
			//create_hor_strline_left_bel(clusterdata, xdistance, dataoutput, current_cluster);
		//}
		//else{ // both dimensions equal
			//
			//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
			//
			//int m=0;
			//
			//for(int i=current_cluster->data.pos; i<current_cluster->data.pos+current_cluster->data.length;i++){
				//
				//if((clusterdata->datapoint_ar[i].x==clusterdata->datapoint_ar[i+1].x) && (clusterdata->datapoint_ar[i].y==clusterdata->datapoint_ar[i+1].y)){}
				//else{
					//
					//if((clusterdata->datapoint_ar[i+1].x==clusterdata->datapoint_ar[i].y+100) && (clusterdata->datapoint_ar[i+1].y==clusterdata->datapoint_ar[i].y-100)){}
					//else{
						//
						//m=1;
					//}
				//}
			//}
			//
			//if(m==0){ // diagonal lines
				//
				//create_ang_diag_bel_left(clusterdata, dataoutput, current_cluster);
			//}
			//else{ //horizontal box
				//
				//create_hor_strline_left_abv(clusterdata, ydistance, dataoutput, current_cluster);
			//}
		//}
	//}
//}
//void robot_right_below_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//if(xdistance>ydistance){
		//
		//create_hor_strline_bel_right(clusterdata, xdistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//if(xdistance<ydistance){
			//
			//create_hor_strline_right_bel(clusterdata, xdistance, dataoutput, current_cluster);
		//}
		//else{ // both dimensions equal
			//
			//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos], current_cluster->data.length, sizeof(struct datapoint_t),comparexasc);
			//
			//int m=0;
			//
			//for(int i=current_cluster->data.pos; i<current_cluster->data.pos+ current_cluster->data.length;i++){
				//
				//if((clusterdata->datapoint_ar[i].x==clusterdata->datapoint_ar[i+1].x) && (clusterdata->datapoint_ar[i].y==clusterdata->datapoint_ar[i+1].y)){}
				//else{
					//
					//if((clusterdata->datapoint_ar[i+1].x==clusterdata->datapoint_ar[i].y+100) && (clusterdata->datapoint_ar[i+1].y==clusterdata->datapoint_ar[i].y+100)){}
					//else{
						//
						//m=1;
					//}
				//}
			//}
			//
			//if(m==0){ // diagonal lines
				//
				//create_ang_diag_bel_right(clusterdata, dataoutput, current_cluster);
			//}
			//else{ //horizontal box
				//
				//create_hor_strline_right_abv(clusterdata, ydistance, dataoutput, current_cluster);
			//}
		//}
	//}
//}
//void robot_left_above_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//if(xdistance>ydistance){
		//
		//create_hor_strline_abv_left(clusterdata, xdistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//if(xdistance<ydistance){
			//
			//create_hor_strline_left_abv(clusterdata, xdistance, dataoutput, current_cluster);
		//}
		//else{ // both dimensions equal
			//
			//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
			//
			//int m=0;
			//
			//for(int i=current_cluster->data.pos; i<current_cluster->data.length+current_cluster->data.pos;i++){
				//
				//if((clusterdata->datapoint_ar[i].x==clusterdata->datapoint_ar[i+1].x) && (clusterdata->datapoint_ar[i].y==clusterdata->datapoint_ar[i+1].y)){}
				//else{
					//
					//if((clusterdata->datapoint_ar[i+1].x==clusterdata->datapoint_ar[i].y+100) && (clusterdata->datapoint_ar[i+1].y==clusterdata->datapoint_ar[i].y+100)){}
					//else{
						//
						//m=1;
					//}
				//}
			//}
			//
			//if(m==0){ // diagonal lines
				//
				//create_ang_diag_abv_left(clusterdata, dataoutput, current_cluster);
			//}
			//else{ //horizontal box
				//
				//create_hor_strline_left_bel(clusterdata, ydistance, dataoutput, current_cluster);
			//}
		//}
	//}
//}
//void robot_right_above_object(struct datapoint_ar_t* clusterdata, int xdistance, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//if(xdistance>ydistance){
		//
		//create_hor_strline_abv_right(clusterdata, xdistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//if(xdistance<ydistance){
			//
			//create_hor_strline_right_abv(clusterdata, xdistance, dataoutput, current_cluster);
		//}
		//else{ // both dimensions equal
			//
			//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],clusterdata->count,sizeof(struct datapoint_t),comparexasc);
			//
			//int m=0;
			//
			//for(int i=current_cluster->data.pos; i<current_cluster->data.pos+current_cluster->data.length-1;i++){
				//
				//if((clusterdata->datapoint_ar[i].x==clusterdata->datapoint_ar[i+1].x) && (clusterdata->datapoint_ar[i].y==clusterdata->datapoint_ar[i+1].y)){}
				//else{
					//
					//if((clusterdata->datapoint_ar[i+1].x==clusterdata->datapoint_ar[i].y+100) && (clusterdata->datapoint_ar[i+1].y==clusterdata->datapoint_ar[i].y-100)){}
					//else{
						//
						//m=1;
					//}
				//}
			//}
			//
			//if(m==0){ // diagonal lines
				//
				//create_ang_diag_abv_right(clusterdata, dataoutput, current_cluster);
			//}
			//else{ //horizontal box
				//
				//create_hor_strline_right_bel(clusterdata, ydistance, dataoutput, current_cluster);
			//}
		//}
	//}
//}
//


//
//void robot_below_object(struct datapoint_ar_t* clusterdata, int robx, int xdistance, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
//
	//
//
	//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//
	//if( clusterdata->datapoint_ar[current_cluster->data.pos].x > robx){ // robot is on the left side below the object
		//
		//robot_left_below_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//if(clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length].x< robx){ //robot is on the right side below the object
			//
			//robot_right_below_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
		//}
		//else{ // robot is under the object
			//
			//robot_middle_below_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
		//}
	//}
//}
//void robot_above_object(struct datapoint_ar_t* clusterdata, int robx, int xdistance, int ydistance,struct  datapoint_ar_t* dataoutput, struct node_t* current_cluster){
	//
	//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],clusterdata->count,sizeof(struct datapoint_t),comparexasc);
	//
	//if( clusterdata->datapoint_ar[current_cluster->data.pos].x > robx){ // robot is on the left side below the object
		//
		//robot_left_above_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//if(clusterdata->datapoint_ar[clusterdata->count-1].x< robx){ //robot is on the right side below the object
			//
			//robot_right_above_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
		//}
		//else{ // robot is under the object
			//
			//robot_middle_above_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
		//}
	//}
//}
//
//
//
//void output_control(struct datapoint_ar_t* outputdata){
	//
	//char text[100];
	//
	//for(int i=0; i<outputdata->count; i++){
		//
		//sprintf(text,"%.d;%.d\r\n", outputdata->datapoint_ar[i].x, outputdata->datapoint_ar[i].y);
		//debugMsg(text);
		//_delay_ms(10);
	//}
//}
//
//
//void getshape(struct datapoint_ar_t* datapoints, struct node_t* listroot, int robx, int roby){
	//
	//struct node_t* current_cluster=listroot;
	//struct datapoint_ar_t* clusterdata=datapoints;
	//int xdistance, ydistance;
	//
	//char text[100];
	//
	//InitOutput();
	//struct datapoint_ar_t* dataoutput=NULL;
	//
	//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
	//xdistance=clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length].x-clusterdata->datapoint_ar[current_cluster->data.pos].x;
	//
	//
	//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareyasc);
	//ydistance=clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length].y-clusterdata->datapoint_ar[current_cluster->data.pos].y;
//
	//sprintf(text,"dd %.d; %.d\r\n", current_cluster->data.pos, clusterdata->datapoint_ar[current_cluster->data.pos].x);
	////debugMsg(text);
//
	////debugMsg(text);
	//
	//if(datapoints->datapoint_ar[current_cluster->data.pos].y>roby){ // robot below the object
		//
//
		//
		//robot_below_object(clusterdata, robx, xdistance, ydistance, dataoutput, current_cluster);
	//}
	//else{
		//
		//if(datapoints->datapoint_ar[current_cluster->data.pos+current_cluster->data.length-1].y<roby){ // robot above the object
			//
			//robot_above_object(clusterdata, robx, xdistance, ydistance, dataoutput, current_cluster);
		//}
		//else{ //robot either left or right from the object
			//
			//qsort(&datapoints[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
			//
			//if(datapoints->datapoint_ar[current_cluster->data.pos].x>robx){ // robot on the left side
				//
				//robot_left_from_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
			//}
			//else{ //robot can only be on the right side now
				//
				//robot_right_from_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
			//}
		//}
	//}
	//if(current_cluster->next!=NULL){
		//do{
		//
			//current_cluster=current_cluster->next;
		//
			//clusterdata->datapoint_ar[current_cluster->data.pos]=datapoints->datapoint_ar[current_cluster->data.pos];
			//clusterdata->count=current_cluster->data.length;
		//
			//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),comparexasc);
			//xdistance=clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length].x-clusterdata->datapoint_ar[current_cluster->data.pos].x;
	//
	//
			//qsort(&clusterdata->datapoint_ar[current_cluster->data.pos],current_cluster->data.length,sizeof(struct datapoint_t),compareyasc);
			//ydistance=clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length].y-clusterdata->datapoint_ar[current_cluster->data.pos].y;
		//
			//sprintf(text,"dd %.d;%.d\r\n", clusterdata->datapoint_ar[current_cluster->data.pos+current_cluster->data.length].y, clusterdata->datapoint_ar[current_cluster->data.pos].y);
			////debugMsg(text);
		//
			//if(datapoints->datapoint_ar[current_cluster->data.pos].y>roby){ // robot below the object
			//
				//robot_below_object(clusterdata, robx, xdistance, ydistance, dataoutput, current_cluster);
			//}
			//else{
			//
				//if(datapoints->datapoint_ar[current_cluster->data.pos+current_cluster->data.length-1].y<roby){ // robot above the object
				//
					//robot_above_object(clusterdata, robx, xdistance, ydistance, dataoutput, current_cluster);
				//}
				//else{ //robot either left or right from the object
				//
					//qsort(&datapoints[0],datapoints->count,sizeof(struct datapoint_t),comparexasc);
				//
					//if(datapoints->datapoint_ar[current_cluster->data.pos].x>robx){ // robot on the left side
				//
						//robot_left_from_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
					//}
					//else{ //robot can only be on the right side now
					//
						//robot_right_from_object(clusterdata, xdistance, ydistance, dataoutput, current_cluster);
					//}
				//}
			//}
		//
		//} while(current_cluster->next!=0);
	//}
	//
	//output_control(dataoutput);
//}