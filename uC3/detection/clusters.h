/*
 * clusters.h
 *
 * Created: 09.05.2020 15:48:30
 *  Author: Richard
 */ 



#ifndef CLUSTERS_H_
#define CLUSTERS_H_

#include <stdlib.h>
#include <avr/io.h>
#include "cluster.h"
#include "datapoint_da.h"
#include "clusters_vkl.h"

struct node_t firstclust;

//Erstellen der Cluster und zuordnen der Punkte zu den Clustern
int8_t find_clusters(struct datapoint_ar_t* arptr);
//Sortieren der Datenpunkte in einem zweiten nach den Clustern
void sort_points(struct datapoint_ar_t* arptr, struct datapoint_ar_t* sorted_points);
//Aufruf der Funktionen zum Clustern
int8_t clusterpoints(struct datapoint_ar_t* arptr);
//Ausgabe der Cluster zu Debugzwecken
void clusteroutput(struct datapoint_ar_t* arptr);

#endif /* CLUSTERS_H_ */