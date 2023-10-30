/*
 * Transform.c
 *
 * Created: 24.04.2020 15:16:44
 *  Author: Schalk Rene
 *
 * This module is for the transformation of the coordinates of the sensor output to the coordinate system the robot uses 
 *
 *
 */ 


#include <stdlib.h>
//#include "rm.h"
#include "rplidar.h"
#include <math.h>
#include "usart.h"
#include <stdio.h>
#include "clusters_vkl.h"
#include <util/delay.h>
#include "avr/wdt.h"
#include "multitask.h"
#include "define.h"
#include <time.h>
#include "ports.h"
#include "Transform.h"
#include "global.h"
#include "rrt_transmittingtask.h"

float center[3][2];
int zaehl=0;
uint8_t clustcount, active_detection=0;
uint16_t numberofpoints = 0/*, numberofits=0*/;

uint8_t maxscans1 = 50;

/* ************************************************************** */
/*! \brief Sort a datapoint Array
 *
 *  Function Sort a datapoint array by the y values of the datapoints in ascending order
 *
 *  \param a First datapoint
 *  \param b Second datapoint
 *
 *  \retval 0 ... y values are equal, -1 ... The y value of a is higher, 1 ... The y value of b is higher
 *  \version 12.4.2021
 */
/* ************************************************************** */
 int compareyasc( const void* a, const void* b) 
 {

 	const struct datapoint_t* dataPoint1 = (struct datapoint_t*) a;
 	const struct datapoint_t* dataPoint2 = (struct datapoint_t*) b;

 	if ( dataPoint1->y == dataPoint2->y ) return 0;
 	else if ( dataPoint1->y < dataPoint2->y ) return -1;
 	else return 1;
 }


/* ************************************************************** */
/*! \brief Transforms a datapoint from polar coordinates into cartesian coordinates
 *
 *  Function Transform a scanned datapoint into cartesian coordinates with a maximum value of 3400
 *
 *  \param scan The scanned point from the LIDAR
 *
 *  \retval A datapoint with the transformed coordinates
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
struct datapoint_t transformpoint(rpLidar_Scan_t* scan){ 
	
	struct datapoint_t data;
	int xvalue, yvalue;
	
	data.angle = scan->angle;
	data.distance=scan->distance;
		
	//Koordinatentransformation
	xvalue = Robx + (int16_t)(scan->distance*cos(((float)(Roba)+(scan->angle+90))*M_PI/180));
	yvalue = Roby + (int16_t)(scan->distance*sin(((float)(Roba)+(scan->angle+90))*M_PI/180));

 	if(xvalue>3400)
	{
		
		xvalue=3400;
	}
	if(yvalue>2200)
	{
		
		yvalue=2200;
	}
	
	data.x=xvalue;
	data.y=yvalue;
	
	return data;
}


/* ************************************************************** */
/*! \brief Transforms a datapoint and adds it to the datapoint array 
 *
 *  Function Transforms a datapoint and checks if it is in the designated area, in which case it adds it to the datapoint array
 *
 *  \param scan The scanned point from the LIDAR
 *
 *  \retval 1 ... The datapoint was added to the array, 2 ... The datapoint is outsidde of the designated area
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
int transform(rpLidar_Scan_t* scan){
	
	struct datapoint_t point;
	//char text[20];
	
// 	if(!(scan->angle>51.19 && scan->angle<55.75) && !(scan->angle>123.73 && scan->angle<126.32) && !(scan->angle>182.93 && scan->angle<187.03) && !(scan->angle>351.49 && scan->angle<354.45))
// 	{
// 		
 		point = transformpoint(scan);
// 	}
		
	// das verwendete Feld ist das Spielfeld der Eurobot mit einem Einzug von 5cm
 	if((point.x<2950 && point.y <1950 && point.x > 50 && point.y > 50) && numberofpoints<maxscans1){

		numberofpoints++;

		arrayptr->datapoint_ar[arrayptr->count] = point;
		arrayptr->count++;						

 		return 1;
 	}

	return 0;
}

/* ************************************************************** */
/*! \brief Evaluates the position of the center point of an object
 *
 *  Function Evaluates the position of the center point of an object by using a vector that connects the robot and the centerpoint of the corresponding point cloud scanned by the LIDAR
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
int8_t linearapproximation()
{
	
	float vec[3][2], sum, addvec[2];  
	
	clustcount = 0;
	
	for(uint8_t i=0; i<firstclust.counter; i++)
	{
		
		if(firstclust.data[i].locx != 65535 && firstclust.data[i].locy != 65535)
		{
			
			// Nur Cluster mit mehr als 5 Punkten werden weiter verarbeitet
			if(firstclust.data[i].length>5)
			{
				
				//Die Vektoren zwischen Roboter und Clustermittelpunkt werden erstellt
				vec[i][0] = (float) ((int16_t)firstclust.data[i].locx - (int16_t)Robx);
				vec[i][1] = (float) ((int16_t)firstclust.data[i].locy - (int16_t)Roby);
				
				//Länge der Vektoren und diese normieren zu können
				sum=sqrt((float)(vec[i][0]*vec[i][0])+(float)(vec[i][1]*vec[i][1]));
				
				// Berechnung der jeweiligen Verschiebungen mit Einheitsvektoren
				addvec[0] = (float)vec[i][0] * (150.0/sum);
				addvec[1] = (float)vec[i][1] * (150.0/sum);
				
				// Der Mittelpunkt des Objekts ist der Clustermittelpunkt + Verschiebung
				center[i][0] = addvec[0] + (float)firstclust.data[i].locx;
				center[i][1] = addvec[1] + (float)firstclust.data[i].locy;
				
				clustcount++;
			}
			else
			{
				
				center[i][0] = (float) firstclust.data[i].locx;
				center[i][1] = (float) firstclust.data[i].locy;
				clustcount++;
			}
		}
		else
		{
			
			center[i][0] = 65535;
			center[i][1] = 65535;
			clustcount++;
		}
		
	}
	
	if(clustcount > 3)
	{
		
		return -1;
	}
	
	return 1;
}

/* ************************************************************** */
/*! \brief Evaluates the variance of the sensor
 *
 *  Function evaluates the variance of the output based on the sensormodel of the sensor
 *
 *  \param distance The distance of the point
 *
 *  \retval The variance based on the distance
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
float getsensormodel(int distance)
{
	
	//return (float)(6.492*(float)exp(0.011*(float)distance));
	return (float) ((8.217*(float)pow(10,-9))*(float)pow((float)distance,2.891))+0.5485;
}

/* ************************************************************** */
/*! \brief Sends the output of the detection to the main unit
 *
 *  Function Stores the output in a structure and sends it to the other unit for further evaluation
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
void dataout( int8_t errorclust, int8_t erroroutput)
{
	
	//char text[20];
	int xdist, ydist;
	
	if(errorclust == 65535 || erroroutput == 65535)
	{
		
		outputdetection = oldoutputdetection;
	}
	else
	{
		
		for(int i = 0; i<3; i++)
		{
			
			outputdetection.center[i][0] = 0;
			outputdetection.center[i][1] = 0;
			
			outputdetection.variance[i] = 0;
		}
		
		outputdetection.count = clustcount;
		
		for (uint8_t i = 0; i<outputdetection.count; i++)
		{
			
			if(center[i][0] != 65535 && center[i][1] != 65535)
			{
				
				outputdetection.center[i][0] = (uint16_t) center[i][0];
				outputdetection.center[i][1] = (uint16_t) center[i][1];
			
				xdist = abs(outputdetection.center[i][0]-Robx);
				ydist = abs(outputdetection.center[i][1]-Roby);
			
				//Neue Variante für die Berechnung der Distanz
				outputdetection.variance[i] =  getsensormodel((int) sqrt(pow((double)xdist,2)+pow((double)ydist,2)));
			
				if((atan((Roby-outputdetection.center[i][1])/(Robx-outputdetection.center[i][1]))*180/M_PI+Roba)>85 && (atan((Roby-outputdetection.center[i][0])/(Robx-outputdetection.center[i][2]))*180/M_PI+Roba)<100)
				{
				
					outputdetection.variance[i] = outputdetection.variance[i]*10;
				}
				//Alternative Variante für die Berechnung der Distanz
				//outputdata.distance[i] = ((xdist <= ydist) ? xdist/2 : xdist) + ((xdist <= ydist) ? ydist : ydist/2);
			}
			else
			{
				
				outputdetection.center[i][0] = oldoutputdetection.center[i][0];
				outputdetection.center[i][1] = oldoutputdetection.center[i][1];
				
				outputdetection.variance[i] = oldoutputdetection.variance[i];
			}
		}
	}

	if (clustcount)
	{
		sendEnemyDataRaw_to_WIFI_RRTLAN();
	}
	
}

/* ************************************************************** */
/*! \brief The Kalman algorithm
 *
 *  Function Merges the output of both units using a kalman-filter and sends the result to the main unit if the count variable of the outputs is not equal the algorithm will take the data from the greateroutput parameter
 *
 *  \param greateroutput Detectionoutput with a greater or equal count
 *
 *  \param smalleroutput Detectionoutput with a smaller or equal count
 *
 *  \version 11.5.2021
 */
/* ************************************************************** */
void getobjectcenters(objectdetectionoutput_t* greateroutput, objectdetectionoutput_t* smalleroutput)
{
	
	float Kt;
	//char text [50];
	
	uint16_t xdist, ydist, distance;
	uint8_t m[3], n;
	
	objectcenter.count = 0;
	
	//Kalmanfilterung wird für jedes Objekt durchgeführt, dabei werden die alten Positionen nicht mit eingerechnetweshalb der Prädiktorschritt wegfällt
	for(uint8_t k=0; k<3; k++)
	{
		
		m[k]=0;
	}
	
	for (uint8_t k = 0; k<greateroutput->count; k++)
	{
		
		if(!(abs(greateroutput->center[k][0]-Robx)<150 && abs(greateroutput->center[k][1]-Roby)<150) && !(greateroutput->center[k][0] == 0 && greateroutput->center[k][1] == 0))
		{

			n=0;
			for (uint8_t i = 0; i<smalleroutput->count; i++)
			{
				
				if(!(abs(smalleroutput->center[i][0]-Robx)<150 && abs(smalleroutput->center[i][1]-Roby)<150) && !(smalleroutput->center[i][0] == 0 && smalleroutput->center[i][1] == 0) && m[i]==0 && n==0)
				{
						
					xdist = abs(greateroutput->center[k][0]-smalleroutput->center[i][0]);
					ydist = abs(greateroutput->center[k][1]-smalleroutput->center[i][1]);
			
					distance = ((xdist <= ydist) ? xdist/2 : xdist) + ((xdist <= ydist) ? ydist : ydist/2);
			
					if(distance <150 )
					{
				
						//Berechnung der Kalmangewinnung
						if(greateroutput->variance[k] < 0.5 && smalleroutput->variance[i] < 0.5)
						{
					
							Kt=0.5;
						}
						else
						{
					
							Kt = (float)greateroutput->variance[k]/(float)(smalleroutput->variance[i]+greateroutput->variance[k]);
						}
				
						//Berechnung der neuen Mittelpunkte
						objectcenter.center[objectcenter.count][0] = greateroutput->center[k][0] + (uint16_t)(Kt * (float)(smalleroutput->center[i][0]-greateroutput->center[k][0]));
						objectcenter.center[objectcenter.count][1] = greateroutput->center[k][1] + (uint16_t)(Kt * (float)(smalleroutput->center[i][1]-greateroutput->center[k][1]));
				
						m[i]=1;
						n=1;
						objectcenter.count++;
						break;
					}
				}
			}
		
			if (n==0)
			{
			
				objectcenter.center[objectcenter.count][0] = greateroutput->center[k][0];
				objectcenter.center[objectcenter.count][1] = greateroutput->center[k][1];
				
				objectcenter.count++;
			}
			
			if(objectcenter.center[objectcenter.count-1][0] == 0 && objectcenter.center[objectcenter.count-1][1] == 0)
			{
				
				objectcenter.center[objectcenter.count-1][0] = greateroutput->center[k][0];
				objectcenter.center[objectcenter.count-1][1] = greateroutput->center[k][1];
			}
			
			if((abs(objectcenter.center[objectcenter.count-1][0]-Robx) < 200 && abs(objectcenter.center[objectcenter.count-1][1]-Roby) < 200))
			{
				
				if((abs(greateroutput->center[k][0]-Robx) > 300 && abs(greateroutput->center[k][1]-Roby) > 300))
				{
								
					objectcenter.center[objectcenter.count-1][0] = greateroutput->center[k][0];
					objectcenter.center[objectcenter.count-1][1] = greateroutput->center[k][1];
				}
				else
				{
					
					if((abs(oldoutputdetection.center[k][0]-Robx) > 300 && abs(oldoutputdetection.center[k][1]-Roby) > 300))
					{
						
						objectcenter.center[objectcenter.count-1][0] = oldoutputdetection.center[k][0];
						objectcenter.center[objectcenter.count-1][1] = oldoutputdetection.center[k][1];
					}
					else
					{
						
						int16_t x, y;
						
						x = (int16_t)objectcenter.center[objectcenter.count-1][0] - (int16_t)Robx;
						y = (int16_t)objectcenter.center[objectcenter.count-1][1] - (int16_t)Roby;
						
						if(abs(x) > 300)
						{
							
							objectcenter.center[objectcenter.count-1][0] = (uint16_t)((int16_t)objectcenter.center[objectcenter.count-1][0] + x);
						}
						else
						{
							
							objectcenter.center[objectcenter.count-1][0] = (uint16_t)((int16_t)objectcenter.center[objectcenter.count-1][0] + x*(int16_t)(abs((float)500/(float)x)));
						}
						
						if(abs(y) > 300)
						{
							
							objectcenter.center[objectcenter.count-1][1] = (uint16_t)((int16_t)objectcenter.center[objectcenter.count-1][1] + y);
						}
						else
						{
							
							objectcenter.center[objectcenter.count-1][1] = (uint16_t)((int16_t)objectcenter.center[objectcenter.count-1][1] + y*(int16_t)(abs((float)500/(float)y)));
						}
					}
				}
			}
  		}
	}
	
//   	sprintf(text,"%d; %d; %d; %d; %d; %d;\n", objectcenter.center[0][0], objectcenter.center[0][1], objectcenter.center[1][0], objectcenter.center[1][1], objectcenter.center[2][0], objectcenter.center[2][1]);
//   	debugMsg(text);
}

/* ************************************************************** */
/*! \brief merges the output of both units
 *
 *  Function Merges the output of both units using a kalman-filter and sends the result to the main unit
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
void Kalman()
{
			
	//char text[20];
	//Das Ergebnis des Objekterkennungsalgorithmus des zweiten Roboters
	//detectionotherrobot  = outputdetection;/*= getotherrobotoutput()*/
	
	
//   	for(int i =0; i<5; i++)
//   	{
//   		
//   		outputdetection.center[i][0] = 100;
//   		outputdetection.center[i][1] = 200;
//   		
//   		outputdetection.variance[i] = i;
//   	}
//   	
//   	for(int i =0; i<5; i++)
//   	{
//   		
//   		detectionotherrobot.center[i][0] = 150;
//   		detectionotherrobot.center[i][1] = 300;
//   		
//   		detectionotherrobot.variance[i] = 4-i;
//   	}
// 	
// 	
// 	detectionotherrobot.center[3][0] = 200;
// 	detectionotherrobot.center[3][1] = 2000;
// 		
// 	detectionotherrobot.variance[3] = 1;
// 	
//   	outputdetection.count = 5;
//   	detectionotherrobot.count = 5;
	
	//choose the output with the larger size for the kalman filter
	
	//detectionotherrobot = outputdetection;
	
// 	for(int k = 0; k<detectionotherrobot.count; k++)
// 	{
// 		
// 		detectionotherrobot.center[k][0] = detectionotherrobot.center[k][0]+10;
// 		detectionotherrobot.center[k][1] = detectionotherrobot.center[k][1]+10;
// 	}
	
	if(outputdetection.count == 0 && detectionotherrobot.count == 0)
	{
		
		getobjectcenters(&oldoutputdetection, &olddetectionotherrobot);
	}
	else
	{
		
// 		if(detectionotherrobot.count>outputdetection.count)
// 		{
// 			
// 			getobjectcenters(&detectionotherrobot, &outputdetection);
// 		}
// 		else
// 		{
			
			getobjectcenters(&outputdetection, &detectionotherrobot);
//		}
	}
	
	sendEnemyData_to_Pathplaner_RRTLAN();
}

/* ************************************************************** */
/*! \brief The object detection algorithm
 *
 *  Function Evaluates the scanned datapoints and sends the output to the other unit, then joins the output of both units and sends the result to the main controller. This operation will be activated every 10 ms
 *
 *  \retval The time until the object detection has to evaluate the result again
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
uint8_t evaluate()
{

	//Einstellung des Zeitintervalls zum Aufrufen der Funktion	
	SET_CYCLE(Evaluation, 100);

	//Wurden genügend Punkte detektiert startet die Objekterkennung
	if((numberofpoints>maxscans1-1) && (!active_detection)){

		//char text[20];
 		SET_PIN(LED_PORT, LED1);
 		
		//Sortieren der Punkte nach y um zu verhindern, dass das geamte Array in der Mitte eines Objektes anfängt
		//qsort(&arrayptr->datapoint_ar[0],arrayptr->count,sizeof(struct datapoint_t),compareyasc);
	
		// Löschen von Firstclust 
		firstclust.counter=0;		
		// Verhindern, dass die Erkennung zwei Mal gestartet wird
 		active_detection=1;

		//Clustern der Datenpunkte und Sortieren in ein weiteres Array
		int8_t errorclust = clusterpoints(arrayptr);
		
		if(errorclust != 0)
		{
			
			//Berechnung der Objektmittelpunkte anhand der Clustermittelpunkte	
			int8_t erroroutput = linearapproximation();
		
			//Senden der Ergebnisse an die zweite Einheit
			dataout( errorclust, erroroutput);
	
			//Implementierung des Kalmanfilters
			Kalman();
			
			oldoutputdetection = outputdetection;
			olddetectionotherrobot = detectionotherrobot;
		}
		
		//Löschen der verwendeten Arrays
		delete_all_arrays();
				
		numberofpoints=0;
		//numberofits++;
		
		CLR_PIN(LED_PORT, LED1);
  	}

	active_detection=0;
	
	return(CYCLE);
}

/* ************************************************************** */
/*! \brief Initialization of the object detection
 *
 *  Function Initializes the object detection in the multi tasking system
 *
 *  \version 12.4.2021
 */
/* ************************************************************** */
void evaluation_Init()
{
		
	/* cyclic task - cycle time: 10 ms */
	SET_CYCLE(Evaluation, 100);
	SET_TASK(Evaluation, CYCLE);
	SET_TASK_HANDLE(Evaluation, evaluate);
}