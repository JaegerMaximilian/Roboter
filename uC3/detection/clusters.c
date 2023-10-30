/*
 * clusters.c
 *
 * Created: 09.05.2020 15:47:59
 *  Author: Schalk Rene
 */ 


#include "clusters.h"
#include "clusters_vkl.h"
#include "datapoint_da.h"
#include "math.h"
#include <stdlib.h>
#include "usart.h"
#include <stdio.h>
#include <util\delay.h>
#include "cluster_count_da.h"
#include "avr\wdt.h"
#include "ports.h"
#include "global.h"
#include "Transform.h"

struct cluster compcluster;



uint8_t init = 0;

/* ************************************************************** */
/*! \brief Clusters the points in the datapoint array
 *
 *  Function Adds a point to a cluster if the point is too far away from the existing clusters a new cluster will be generated
 *
 *  \param arptr A pointer to the datapoint array the function has to evaluate
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
int8_t find_clusters(struct datapoint_ar_t* arptr){
		
	struct cluster clust;
	
	//char text[20];
	
  	if(Robx != 0 && Roby != 0)
	{
		if(init == 0)
		{
	
			for(uint8_t i=0; i<3; i++)
			{
			
				if((abs(objectcenter.center[i][0]-Robx) <= 100 && abs(objectcenter.center[i][1]-Roby) <= 100 || init == 1))
				{
				
					objectcenter.center[i][0] = initcenter[i+1][0];
					objectcenter.center[i][1] = initcenter[i+1][1];
				
// 					firstclust.data[i].locx = objectcenter.center[i][0];
// 					firstclust.data[i].locy = objectcenter.center[i][1];
				
					init = 1;
				}
				else
				{
				
					objectcenter.center[i][0] = initcenter[i][0];
					objectcenter.center[i][1] = initcenter[i][1];
					
// 					firstclust.data[i].locx = objectcenter.center[i][0];
// 					firstclust.data[i].locy = objectcenter.center[i][1];
				}
			}
			
			init = 1;
		}
		
		uint16_t distx1, distx2, distx3, disty1, disty2, disty3, dist1, dist2, dist3;
		
		for(uint8_t i = 0; i<arrayptr->count; i++)
		{
			
			distx1 = abs(arrayptr->datapoint_ar[i].x - objectcenter.center[0][0]);
			distx2 = abs(arrayptr->datapoint_ar[i].x - objectcenter.center[1][0]);
			distx3 = abs(arrayptr->datapoint_ar[i].x - objectcenter.center[2][0]);
			
			disty1 = abs(arrayptr->datapoint_ar[i].y - objectcenter.center[0][1]);
			disty2 = abs(arrayptr->datapoint_ar[i].y - objectcenter.center[1][1]);
			disty3 = abs(arrayptr->datapoint_ar[i].y - objectcenter.center[2][1]);
			
			dist1 = sqrt(pow(distx1, 2)+pow(disty1, 2));
			dist2 = sqrt(pow(distx2, 2)+pow(disty2, 2));
			dist3 = sqrt(pow(distx3, 2)+pow(disty3, 2));
			
			if(dist1 < 1000 || dist2 < 1000 || dist3 < 1000)
			{
				
				if(dist1 <= dist2 && dist1 <= dist3)
				{
								
// 					newmedx = (uint16_t)((int16_t)firstclust.data[0].locx + (((int16_t)arptr->datapoint_ar[i].x - (int16_t)firstclust.data[0].locx)/(int16_t)firstclust.data[0].length));
// 					newmedy = (uint16_t)((int16_t)firstclust.data[0].locy + (((int16_t)arptr->datapoint_ar[i].y - (int16_t)firstclust.data[0].locy)/(int16_t)firstclust.data[0].length));
				
					firstclust.data[0].cluster_pos[firstclust.data[0].length] = i;
					
// 					firstclust.data[0].locx = newmedx;
// 					firstclust.data[0].locy = newmedy;
				
					firstclust.data[0].length++;
				}
				else
				{
				
					if(dist2 <= dist3 && dist2 < dist1)
					{

// 						newmedx = (uint16_t)((int16_t)firstclust.data[1].locx + (((int16_t)arptr->datapoint_ar[i].x - (int16_t)firstclust.data[1].locx)/(int16_t)firstclust.data[1].length));
// 						newmedy = (uint16_t)((int16_t)firstclust.data[1].locy + (((int16_t)arptr->datapoint_ar[i].y - (int16_t)firstclust.data[1].locy)/(int16_t)firstclust.data[1].length));
					
						firstclust.data[1].cluster_pos[firstclust.data[1].length] = i;
					
// 						firstclust.data[1].locx = newmedx;
// 						firstclust.data[1].locy = newmedy;
					
						firstclust.data[1].length++;
					}
					else
					{

// 						newmedx = (uint16_t)((int16_t)firstclust.data[2].locx + (((int16_t)arptr->datapoint_ar[i].x - (int16_t)firstclust.data[2].locx)/(int16_t)firstclust.data[2].length));
// 						newmedy = (uint16_t)((int16_t)firstclust.data[2].locy + (((int16_t)arptr->datapoint_ar[i].y - (int16_t)firstclust.data[2].locy)/(int16_t)firstclust.data[2].length));
					
						firstclust.data[2].cluster_pos[firstclust.data[2].length] = i;
					
// 						firstclust.data[2].locx = newmedx;
// 						firstclust.data[2].locy = newmedy;
					
						firstclust.data[2].length++;
					}
				}
			}
		}
		
		uint32_t newmedx, newmedy;
		
		for(uint8_t i = 0; i<3; i++)
		{
		
			if(firstclust.data[i].length>0)
			{
			
				newmedx = 0;
				newmedy = 0;
					
				for(uint8_t k = 0; k<firstclust.data[i].length; k++)
				{
				
					newmedx += (uint32_t)arrayptr->datapoint_ar[firstclust.data[i].cluster_pos[k]].x;
					newmedy += (uint32_t)arrayptr->datapoint_ar[firstclust.data[i].cluster_pos[k]].y;
				}
			
				firstclust.data[i].locx = newmedx/firstclust.data[i].length;
				firstclust.data[i].locy = newmedy/firstclust.data[i].length;
			}
			else
			{
				
				firstclust.data[i].locx = 65535;
				firstclust.data[i].locy = 65535;
			}
			
// 			sprintf(&text,"\n\M; %d; %d\n", firstclust.data[i].locx, firstclust.data[i].locy);
// 			debugMsg(text);
		}
		
		firstclust.counter = 3;
		
		 for(uint8_t i = 1; i<firstclust.counter; i++)
		 {
			 
			 firstclust.data[i].pos = firstclust.data[i-1].pos + firstclust.data[i-1].length;
		 }
		return 1;
	}
	
	return 0;
	
// 	uint8_t clustered = 0, k, i;
// 	int16_t distance, xdist, ydist, newmedx, newmedy; 
// 	//uint8_t clustup=0;
// 	
// 	clust.locx= arptr->datapoint_ar[0].x;
// 	clust.locy= arptr->datapoint_ar[0].y;
// 	clust.length=1fvfvvffffffffffffffffffffffffffffffv;
// 	clust.pos=0;,,
// 	clust.cluster_pos[0]=0;
// 	
// 	firstclust.counter = 0;
// 	firstclust.data[firstclust.counter] = clust;
// 	firstclust.counter++;
// 	
// 	//Clustern der Punkte
// 	for(i=1; i < arptr->count; i++){
// 		
// 		clustered = 0; 
// 		
// 		for(k = 0; k<firstclust.counter; k++)
// 		{
// 			
// 			xdist = abs(arptr->datapoint_ar[i].x-firstclust.data[k].locx);
// 			ydist = abs(arptr->datapoint_ar[i].y-firstclust.data[k].locy);
// 			
// 			distance = ((xdist <= ydist) ? xdist/2 : xdist) + ((xdist <= ydist) ? ydist : ydist/2);
// 			
// 			//Alternative Variante zur Berechnung der Distanz
// 			//distance = sqrt(xdist*xdist+ydist*ydist);
// 
// 			//Distanz<25cm: Punkt ist Teil des Clusters
// 			if(distance < 250){
// 				
// 				firstclust.data[k].length++;
// 				
// 				newmedx = (uint16_t)((int16_t)firstclust.data[k].locx + (((int16_t)arptr->datapoint_ar[i].x - (int16_t)firstclust.data[k].locx)/(int16_t)firstclust.data[k].length));
// 				newmedy = (uint16_t)((int16_t)firstclust.data[k].locy + (((int16_t)arptr->datapoint_ar[i].y - (int16_t)firstclust.data[k].locy)/(int16_t)firstclust.data[k].length));
// 				
// 				//Alternative Variante zur Berechnung des neuen Clustermittelpunkts
// // 				newmedx=(((uint32_t)((uint32_t)firstclust.data[k].length-1)*(uint32_t)firstclust.data[k].locx)+(uint32_t)arptr->datapoint_ar[i].x)/(uint32_t)(firstclust.data[k].length);
// // 				newmedy=(((uint32_t)((uint32_t)firstclust.data[k].length-1)*(uint32_t)firstclust.data[k].locy)+(uint32_t)arptr->datapoint_ar[i].y)/(uint32_t)(firstclust.data[k].length);
// 				
// 				firstclust.data[k].locx=newmedx;
// 				firstclust.data[k].locy=newmedy;
// 				firstclust.data[k].cluster_pos[firstclust.data[k].length-1]=i;
// 				
// 				clustered = 1;
// 				
// 				break;
// 			}
// 		}
// 		
// 		//Konnte der Punkt nicht geclustert werden, wird ein neuer Cluster mit dem Punkt erstellt
// 		if(clustered == 0)
// 		{
// 			
// 			clust.locx = arptr->datapoint_ar[i].x;
// 			clust.locy = arptr->datapoint_ar[i].y;
// 			clust.length = 1;
// 			clust.cluster_pos[0] = i;
// 			clust.pos = 0;
// 			
// 			firstclust.data[firstclust.counter] = clust;
// 			firstclust.counter++;
// 		}
// 		
// 		if(firstclust.counter == 0)
// 		{
// 			
// 			return -1;
// 		}
// 		return 1;
// 	}
// 	
// 	firstclust.data[0].pos=0;
// 	
// 	//Berechnung der Position des ersten Punktes des Clusters im sortierten Array

}

/* ************************************************************** */
/*! \brief Sorts the points of the datapoint array
 *
 *  Function Sorts the points of the datapoint array by the clusters in a separate array
 *
 *  \param arptr Pointer to the array that has to be sorted
 *
 *  \param sorted_points Pointer to the array where the sorted points will be stored
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
void sort_points(struct datapoint_ar_t* arptr, struct datapoint_ar_t* sorted_points){
	
	for(uint8_t i=0; i<firstclust.counter; i++)
	{
		
		for(uint8_t k=0; k<firstclust.data[i].length; k++)
		{
			
			sorted_points->datapoint_ar[sorted_points->count] = arptr->datapoint_ar[firstclust.data[i].cluster_pos[k]];
			sorted_points->count++;
		}
	}
	

}

/* ************************************************************** */
/*! \brief Outputs the clusters
 *
 *  The output is a flow of strings which outputs the center point of the cluster and the points that belong to that cluster
 *
 *  \param arptr Pointer to the datapoint array where the points are already sorted after by the clusters
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
//Ausgabe der Cluster zu Debugzwecken
void clusteroutput(struct datapoint_ar_t* arptr){
	
	char text[20];
	
	sprintf(text,"\n\nclusteroutput\n");
	debugMsg(text);
	
	for(uint8_t i = 0; i<firstclust.counter; i++)
	{
		sprintf(text,"\n\nnaechster cluster; %d; %d; %d\n", firstclust.data[i].pos, firstclust.data[i].length, i+1);
		debugMsg(text);
		
		sprintf(&text,"\n\Mitte; %d; %d\n", firstclust.data[i].locx, firstclust.data[i].locy);
		debugMsg(text);
		
		for(uint8_t k=firstclust.data[i].pos; k<firstclust.data[i].pos+firstclust.data[i].length; k++)
		{
			
			sprintf(&text,"%.d;%.d; %.d\r\n", arptr->datapoint_ar[k].x, arptr->datapoint_ar[k].y, k);
			debugMsg(text);
			//_delay_ms(10);
		}
	}
		
	sprintf(text,"\n\nende\n");
	debugMsg(text);
}

/* ************************************************************** */
/*! \brief Clusters the datapoints
 *
 *  Evaluates the datapoint array and creates clusters as well as an array with sorted points
 *
 *  \param arptr Pointer to the datapoint array which has to be clustered
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
//Aufruf der Funktionen zum Clustern
int8_t clusterpoints(struct datapoint_ar_t* arptr){
	
	int8_t errorcluster = find_clusters(arptr);
	
	//Wird woascheinlich nimma braucht
// 	if(errorcluster != 0)
// 	{
// 		
// 		sort_points(arptr, sorted_datapoints);
// 	}
	
	//Nur zum debuggen
	//clusteroutput(sorted_datapoints);
	
	return errorcluster;
}
