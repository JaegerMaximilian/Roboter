/*
 * obstacle.h
 *
 * Created: 03.06.2021 09:45:05
 *  Author: P20087
 */ 


#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#ifndef _OBSTACLE_EXTERN
	#define _OBSTACLE_EXTERN extern
#endif

/* ***************************** */
/*			ENUMERATES			 */
/* ***************************** */
/* number of scan-array-elements */
#define OBSTACLE_SCAN_ARRAY_LENGTH		200
/* length of obstacle-list -> number of possible obstacles */
#define OBSTACLE_LIST_LENGTH			5
/* length of scan-list in obstacle-list -> number of possible points for one obstacle */
#define OBSTACLE_POINT_LENGTH			50
/* maximal different between two angles [°] */
#define OBSTACLE_MAX_DIFF_ANGLE			10
/* maximal different between two distances [mm] */
#define OBSTACLE_MAX_DIFF_DISTANCE		100
/* minimal amount of points describing a obstacle */
#define OBSTACLE_MIN_NBR_POINTS			2
/* maximum size of an obstacle [mm] */
#define OBSTACLE_MAX_SIZE				400

/* offset (x-position) between lidar-position and odometrie [mm] */
#define OBSTACLE_LIDAR_OFFSET_X			60.0
/* offset (y-position) between lidar-position and odometrie [mm] */
#define OBSTACLE_LIDAR_OFFSET_Y			0.0

/* dimensions of playground */
#define OBSTACLE_DIM_PLAYGROUND_X		2900
#define OBSTACLE_DIM_PLAYGROUND_Y		1900

/* buffer of valid scan-points (valid means, all point the are on playground - 2m x 3m table) */
typedef struct
{
	/* x-coordination of scan-point */
	uint16_t x;
	/* y-coordination of scan-point */
	uint16_t y;
	/* angular (from lidar-scan) of scan-point */
	uint16_t phi;
	/* measured distance (from lidar-scan) of scan-point */
	uint16_t r;
	
	uint16_t timeStamp;
	
} OBSTACLE_scanPoint_t;

_OBSTACLE_EXTERN OBSTACLE_scanPoint_t scanArray[OBSTACLE_SCAN_ARRAY_LENGTH];


/* obstacle-list */
typedef struct
{
	/* liost of scan-points of the obstacle */
	OBSTACLE_scanPoint_t scan[OBSTACLE_POINT_LENGTH];
	/* number of scan-points */
	uint8_t nbr;
	/* dimension of the obstacle [mm] */
	uint16_t dim;
	/* expansion in x-direction */
	uint16_t deltaX;
	/* expantion in y-direction */
	uint16_t deltaY;
	/* vector in the direction of the center of the obstacle */
	float vec[2];
	/* number of obstacles -> is need when a object is splitted into two obstacles */
	uint8_t nbrObstacle;
	/* x-coordiantion of the center of the obstacle */
	uint16_t centerX[2];
	/* y-coordiantion of the center of the obstacle */
	uint16_t centerY[2];
} OBSTACLE_list_t;

/* obstacle list -> here all detected obstacles are stored */
_OBSTACLE_EXTERN OBSTACLE_list_t obstacleList[OBSTACLE_LIST_LENGTH];

/* ***************************** */
/*			PROTOTYPES			 */
/* ***************************** */
void OBSTACLE_InitDetection();
void OBSTACLE_DetectionTask();


#endif /* OBSTACLE_H_ */