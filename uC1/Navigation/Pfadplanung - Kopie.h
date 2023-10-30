/*
 * Pfadplanung.h
 *
 * Created: 04.12.2019 10:34:08
 *  Author: paste
 */ 


#ifndef PFADPLANUNG_H
#define PFADPLANUNG_H

/* ***************************** */
/*			PROTOTYPES			 */
/* ***************************** */

/*					     3000 mm
	 ---------------------------------------------
	|											  |
	|    x  x  x  x  x  x  x  x  x  x  x  x  x    |
	|											  |
	|    x  x  x  x  x  x  x  x  x  x  x  x  x    |  
	|					  100					  |  
	|    x  x  x  x  x  x  x  x  x  x  x  x  x    |  
	|						100					  |  2000 mm
	|    x  x  x  x  x  x  x  x  x  x  x  x  x    |  
	|											  |  
	|    x  x  x  x  x  x  x  x  x  x  x  x  x    |    
	|											  |  
	|200 x  x  x  x  x  x  x  x  x  x  x  x  x    |
	|   200							     		  |
	 --------------------------------------------

*/
/* dimensions of the playground - 3000 x 2000 mm */
#define PATH_PLAYGROUND_DIM_X		3000
#define PATH_PLAYGROUND_DIM_Y		2000
/* non-trafficable area at the edge of the field of play - 200 mm */
#define PATH_NON_TRAFFICLE_AREA		200
/* resolution of the grid (straight) - 100 mm */
#define PATH_GRID_RESOLUTION		100
/* resolution of the grid (diagonally) - 141 mm -> 100 * SQR(2) */
#define PATH_GRID_RESOLUTION_DIAGONALLY		141
/* dimension of grid in x and y */
#define PATH_GRID_DIM_X				(((PATH_PLAYGROUND_DIM_X - (2 * PATH_NON_TRAFFICLE_AREA)) / PATH_GRID_RESOLUTION) + 1)
#define PATH_GRID_DIM_Y				(((PATH_PLAYGROUND_DIM_Y - (2 * PATH_NON_TRAFFICLE_AREA)) / PATH_GRID_RESOLUTION) + 1)
/* INF for occupancy grid */
#define PATH_INFINITY				255
/* FREE for occupancy grid */
#define PATH_FREE					1
/* zone 1 .. 3 around an obsacle */
#define PATH_OBSTACLE_ZONE_1		2
#define PATH_OBSTACLE_ZONE_2		3
#define PATH_OBSTACLE_ZONE_3		5
/* point has not been inspected */
#define PATH_NOT_INSPECTED			32767
/* point has been inspected */
#define PATH_INSPECTED				65535
/* size of obstacle list -> */
/* [0] .. second robot, [1] .. enemy robot 1, [2] .. enemy robot 2 */
/* [3 ... 9] .. seven "static" obstacles */
#define PATH_OBSTACLE_LIST_LENGTH	10
/* obstacle indentifier (0,1,2) ... robots, (255) ... static obstacles */
#define PATH_SECOND_ROBOT			0
#define PATH_ENEMY_ROBOT_1			1
#define PATH_ENEMY_ROBOT_2			2
#define PATH_STATIC_OBSTACLE		255
/* indentifier for list-element is empty */
#define PATH_OBSTACLE_LIST_EMPTY	-1
/* size of segment list -> each obstacle consists of four segments*/
#define PATH_SEGMENT_LIST_LENGTH	(PATH_OBSTACLE_LIST_LENGTH * 4)

/* point on playground (x,y) */
typedef struct {
	int16_t Xpos;
	int16_t Ypos;
} point_t;

/* point in occupancy grid */
typedef struct {
	int8_t Xpos;
	int8_t Ypos;
} matrixpoint_t;

typedef struct  
{
	/*
		oo oo XY
		oo oo oo
		xy oo oo
	*/
	/* lower-left corner of the obstacle */
	matrixpoint_t xy;
	/* upper-right corner of the obstacle */
	matrixpoint_t XY;
} obstacle_t;

typedef struct
{
	/* x-component of the vector */
	int8_t x;
	/* y-component of the vector */
	int8_t y;	
} vector_t;

typedef struct
{
	/* vector - conects two points in a object */
	vector_t v;
	/* origion of the vector */
	matrixpoint_t o;	
} segment_t;

typedef struct
{
	/* occupancy grid: [i][j] = 1 -> area is free (low probability for obstacle) */
	/*                 [i][j] = 255 -> area is occupied (high probability for obstacle) */
	uint8_t occupancyGrid[PATH_GRID_DIM_X][PATH_GRID_DIM_Y];
	/* l: [i][j] = != -1 -> distance between starting point and point (i,j) */
	/*    [i][j] = 32767 (2^15) -> point has not been inspected */
	/*    [i][j] = -1 -> point is already inspected */
 	uint16_t l[PATH_GRID_DIM_X][PATH_GRID_DIM_Y];
 	/* p: [i][j] = previous point - from this point the point (i,j) is reachable */
 	matrixpoint_t p[PATH_GRID_DIM_X][PATH_GRID_DIM_Y];
	/* reduced point-list */
	matrixpoint_t P[PATH_GRID_DIM_X + PATH_GRID_DIM_Y];
	/* used elements in P */
	uint8_t dimP;
	/* flatten point-list */
	matrixpoint_t F[PATH_GRID_DIM_X + PATH_GRID_DIM_Y];
	/* flatten point-list */
	matrixpoint_t finalList[PATH_GRID_DIM_X];

	/* obstacle list */
	obstacle_t obstacleList[PATH_OBSTACLE_LIST_LENGTH];
	/* segment-list */
	segment_t segmentList[PATH_SEGMENT_LIST_LENGTH];
	uint8_t segmentListLength;
	/* in this list the real coordinates are stored */
	point_t pointList[PATH_GRID_DIM_X]; 
} path_t;

path_t path;


uint8_t PATH_DriveToAbsPos(int16_t vMax, point_t start, point_t ziel, uint8_t gegner);
matrixpoint_t PATH_FindNearestMatrixPoint(point_t pos);
void PATH_ResetOccupancyGrid();
void PATH_Set_ObstacleInOccupancyGrid(point_t obstacle);
uint8_t PATH_A_Star(matrixpoint_t start, matrixpoint_t goal);
uint16_t PATH_calcHeuristic(matrixpoint_t actual, matrixpoint_t goal);
uint8_t  PATH_ReducePoints(matrixpoint_t start, matrixpoint_t goal);
void PATH_FlattenTrajectory(matrixpoint_t start, matrixpoint_t goal);
uint8_t PATH_AddItemToObstacleList(uint8_t type, int8_t x, int8_t y, int8_t X, int8_t Y);
uint8_t PATH_DeleteItemFromObstacleList(uint8_t type, int8_t x, int8_t y, int8_t X, int8_t Y);
void PATH_DeleteObstacleList();
obstacle_t PATH_Get_ObstacleLimits(point_t obstacle);
void PATH_Set_ObstacleListInOccupancyGrid();
void PATH_SetSegmentList();
uint8_t PATH_CalcIntersection(matrixpoint_t A1, matrixpoint_t A2, segment_t B);
void PATH_ShortenPath(matrixpoint_t start, matrixpoint_t goal);
void PATH_StoreRealPoints(matrixpoint_t goal, point_t realGoal);

#endif /* PFADPLANUNG_H */