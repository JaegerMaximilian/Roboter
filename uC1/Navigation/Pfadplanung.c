/*
* Pfadplanung.c
*
* Created: 04.12.2019 10:33:52
*  Author: paste
*/

#define PFADPLANUNG_EXTERN

#include <avr/io.h>
#include "Pfadplanung.h"
#include "ki.h"
#include <stdlib.h>
#include <math.h>
#include "command.h"
#include "usart.h"
#include <stdio.h>
#include "observation.h"



/**********************************************************
NAME: PATH_DriveTo
FUNC.: gibt den Weg zu einem bestimmten Punkt vor
IN: -
OUT: -
VERSION: 1.0 - Michael Zauner (24.04.2020)
**********************************************************/
uint8_t PATH_DriveToAbsPos(point_t start, point_t ziel, point_t *pointList, uint8_t *pointNr)
{
	matrixpoint_t startMP, zielMP;
	obstacle_t robot;
	float s;
	
	/* calculate the distance to drive */
	s = sqrtf(pow(((float)start.Xpos - (float)ziel.Xpos), 2.0) + pow(((float)start.Ypos - (float)ziel.Ypos), 2.));
	
	/* if the distance to drive is smaller as 300 mm -> drive direct to the goal */
	if (s < 300.0)
	{
		pointList[0].Xpos = ziel.Xpos;
		pointList[0].Ypos = ziel.Ypos;
		*pointNr = 1;
		return(1);
	}
	/* start path-planner */
	else
	{
		/* get grid-coordinates of the start-point */
		startMP = PATH_FindNearestMatrixPoint(start);
		/* get grid-coordinates of the goal-point */
		zielMP = PATH_FindNearestMatrixPoint(ziel);
		
		//char text1[100];
		//sprintf(text1, "Start: (%d/%d) Ziel: (%d/%d)\n", startMP.Xpos, startMP.Ypos, zielMP.Xpos,zielMP.Ypos);
		//writeString_usart(&WIFI_IF, text1);

		
		/* reset occupancy grid - set the all matrix-elements to 1 */
		PATH_ResetOccupancyGrid();
		
		/* delete the obstacle-list */
		PATH_DeleteObstacleList();

		/* put the other robots in the occupancy grid */
		for (uint8_t i = 0 ; i < 5; i++)
		{
			if ((enemyRobot[i].Xpos < PATH_PLAYGROUND_DIM_X) && (enemyRobot[i].Xpos > 0) && (enemyRobot[i].Ypos < PATH_PLAYGROUND_DIM_Y) && (enemyRobot[i].Ypos > 0))
			{
				robot = PATH_Get_ObstacleLimits(enemyRobot[i]);
				PATH_AddItemToObstacleList(i, robot.xy.Xpos, robot.xy.Ypos, robot.XY.Xpos, robot.XY.Ypos);
			}
		}		
		
		// Statische Hindernisse werden hinzugefügt
		PATH_AddItemListToObstacleList(StaticObsticalList);
		
		/* set all obstacles in occypancy grid */
		PATH_Set_ObstacleListInOccupancyGrid();
		
		/* start A* algorithm -> returns 1 if a path has been found and otherwise 0 */
		if (PATH_A_Star(startMP, zielMP))
		{
			/* delete all points excluding the corner-points */
			path.dimP = PATH_ReducePoints(startMP, zielMP);
			/* flatten the trajectory */
			PATH_FlattenTrajectory(startMP, zielMP);
			/* make a list of all segments of the objects (vectors between the edges) */
			PATH_SetSegmentList();
			/* shorten the path - delete unnecessary corners */
			PATH_ShortenPath(startMP, zielMP);
			/* recalculate the real coordinates of the matrix-points */
			path.dimP = PATH_StoreRealPoints(zielMP, ziel);
			/* execute path */
			for (uint8_t i = 0 ; i < path.dimP; i++)
			{
				pointList[i].Xpos = path.pointList[i].Xpos;
				pointList[i].Ypos = path.pointList[i].Ypos;
			}
			*pointNr = path.dimP;
			return(1);
		}
	}
	return(0);
}


/**********************************************************
NAME:		PATH_FindNearestMatrixPoint
FUNC.:		calculate from the coordinates the nearest
position (index) in the grid
IN:			pos ... position on playground (x/y)
OUT:		index in grid (x/y)
VERSION:	1.0 - Michael Zauner (24.04.2020)
**********************************************************/
matrixpoint_t PATH_FindNearestMatrixPoint(point_t pos)
{
	volatile matrixpoint_t gridPoint;
	
	/* add the (PATH_GRID_RESOLUTION / 2) to the pos -> so the result is rounded correctly */
	/* example1: X = 1010, X += 50, X = 1060, X /= 100 -> result = 10 - OK! 1010 is closer to 1000 */
	/* example1: X = 1070, X += 50, X = 1120, X /= 100 -> result = 11 - OK! 1070 is closer to 1100 */
	pos.Xpos += (PATH_GRID_RESOLUTION / 2);
	pos.Ypos += (PATH_GRID_RESOLUTION / 2);
	
	/* calculate nearest grid-point */
	gridPoint.Xpos = (int8_t)((pos.Xpos - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION);
	gridPoint.Ypos = (int8_t)((pos.Ypos - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION);
	
	/* limit grid-point in x */
	gridPoint.Xpos = ((gridPoint.Xpos >= PATH_GRID_DIM_X) ? PATH_GRID_DIM_X-1 : gridPoint.Xpos);
	gridPoint.Xpos = ((gridPoint.Xpos < 0) ? 0 : gridPoint.Xpos);

	/* limit grid-point in y */
	gridPoint.Ypos = ((gridPoint.Ypos >= PATH_GRID_DIM_Y) ? PATH_GRID_DIM_Y-1 : gridPoint.Ypos);
	gridPoint.Ypos = ((gridPoint.Ypos < 0) ? 0 : gridPoint.Ypos);
	
	/* return grid-point */
	return(gridPoint);
	
}

/**********************************************************
NAME:		PATH_ResetOccupancyGrid
FUNC.:		set all elements in the occupancy grid to FREE
IN:			-
OUT:		-
VERSION:	1.0 - Michael Zauner (24.04.2020)
**********************************************************/
void PATH_ResetOccupancyGrid()
{
	/* set all elements of the occupancy grid to FREE */
	for (uint8_t x = 0; x < PATH_GRID_DIM_X; x++)
	{
		for (uint8_t y = 0; y < PATH_GRID_DIM_Y; y++)
		{
			path.occupancyGrid[x][y] = PATH_FREE;
			/* set all points to not inspected */
			path.l[x][y] = PATH_NOT_INSPECTED;
		}
	}
}


/**********************************************************
NAME:		PATH_A_Star
FUNC.:		calculate a path between start-point and goal-point
IN:			start ... start-point in matrix
goal ... goal-point in matrix
OUT:		0 ... no path has been found
1 ... path has been calculated
VERSION:	1.0 - Michael Zauner (04.05.2020)
**********************************************************/
uint8_t PATH_A_Star(matrixpoint_t start, matrixpoint_t goal)
{
	/* set execute status to "no path found" */
	uint8_t A_Star_executeStatus = 0;
	/* initialize the minimum path to maximum-value -> PATH_NOT_INSPECTED */
	volatile uint16_t min_l;
	volatile int8_t ii, jj;
	/* path-costs */
	volatile uint16_t C;
	volatile matrixpoint_t actual;
	volatile uint16_t h = PATH_calcHeuristic(actual, start);
	/* limit the searching-region */
	matrixpoint_t min, max;

	
	/* ******* */
	/* reset p */
	/* ******* */
	for (uint8_t i = 0; i < PATH_GRID_DIM_X; i++)
	{
		for (uint8_t j = 0; j < PATH_GRID_DIM_Y; j++)
		{
			path.p[i][j].Xpos = 0;
			path.p[i][j].Ypos = 0;
		}
	}
	
	/* calculate the path from goal to start (so you get the list of points in the right order!) */
	/* set goal-point to 0 */
	path.l[goal.Xpos][goal.Ypos] = 0;
	min = goal;
	max = goal;
	
	/* ************************ */
	/* execute the A*-algorithm */
	/* ************************ */
	/* the loop counter guarantees a termination if all fields were examined but no path was found */
	for (uint16_t loopCounter = 0; loopCounter < (PATH_GRID_DIM_X * PATH_GRID_DIM_Y); loopCounter++)
	{
		/* set flag-indices to an invalid value */
		ii = -1;
		jj = -1;
		/* set flag for minimum l-value to maximum */
		min_l = PATH_NOT_INSPECTED;
		
		/* search the minimum path-length (path from start-point + heuristic) */
		for (uint8_t i = min.Xpos; i <= max.Xpos; i++)
		{
			for (uint8_t j = min.Ypos; j <= max.Ypos; j++)
			{
				/* inspect only if cell is uninspected */
				if (path.l[i][j] != PATH_INSPECTED)
				{
					actual.Xpos = i;
					actual.Ypos = j;
					h = PATH_calcHeuristic(actual, start);
					
					if ((path.l[i][j] + h) < min_l)
					{
						min_l = path.l[i][j] + h;
						
						/* store the current indexes */
						ii = i;
						jj = j;
					}
				}
			}
		}

		/* set new limits for searching- region */
		min.Xpos = (((min.Xpos > 0) && ((ii-1) < min.Xpos) && (ii > 0)) ? (ii-1) : min.Xpos);
		min.Ypos = (((min.Ypos > 0) && ((jj-1) < min.Ypos) && (jj > 0)) ? (jj-1) : min.Ypos);
		max.Xpos = (((max.Xpos < (PATH_GRID_DIM_X - 1)) && ((ii+1) > max.Xpos)) ? (ii+1) : max.Xpos);
		max.Ypos = (((max.Ypos < (PATH_GRID_DIM_Y - 1)) && ((jj+1) > max.Ypos)) ? (jj+1) : max.Ypos);
		
		/* if no new matix-point has been found -> no valid path can be found */
		if ((ii == -1) && (jj == -1))
		{
			return (0);
		}

		/* termination condition: */
		/* -> if the goal-point is entered in the USED list, the shortest way to destination found */
		if ((ii == start.Xpos) && (jj == start.Ypos))
		{
			return (1);
		}
		/* otherwise: find all connections to the current point */
		else
		{
			/* check the connected grid-elements (o) around the current point (X) */
			/*			o o o
			o X o
			o o o             */
			for (int8_t m = -1; m <= 1; m++)
			{
				for (int8_t n = -1; n <= 1; n++)
				{
					/* check if the indices are valid */
					if (((ii + m) >= 0) && ((ii + m) < PATH_GRID_DIM_X) && ((jj + n) >= 0) && ((jj + n) < PATH_GRID_DIM_Y))
					{
						/* check if the matrix-point is already inspected */
						if (path.l[ii+m][jj+n] != PATH_INSPECTED)
						{
							/*    o
							o X o
							o    */
							if ((m == 0) || (n == 0))
							{
								C = PATH_GRID_RESOLUTION * (uint16_t)(path.occupancyGrid[ii+m][jj+n]);
							}
							/* o   o
							X
							o   o  */
							else
							{
								C = PATH_GRID_RESOLUTION_DIAGONALLY * (uint16_t)(path.occupancyGrid[ii+m][jj+n]);
							}
							/* if the path to the point (ii+m,jj+n) is shorter than the actual path */
							/* -> overwrite the current value and set the current point (ii,jj) as previous point */
							if ((path.l[ii+m][jj+n] > (path.l[ii][jj] + C)) && ((m != 0) || (n != 0)))
							{
								path.l[ii+m][jj+n] = (path.l[ii][jj] + C);
								path.p[ii+m][jj+n].Xpos = ii;
								path.p[ii+m][jj+n].Ypos = jj;
							}
						}
					}
				}
			}
		}
		/* set matrix-point as inspected (only if a valid point has been found) */
		path.l[ii][jj] = PATH_INSPECTED;
	}
	/* return result */
	return(A_Star_executeStatus);
}


/**********************************************************
NAME:		PATH_calcHeuristic
FUNC.:		calculates the estimated route to the goal
IN:			actual ... actual point in matrix
goal ... goal-point in matrix
OUT:		calculated heuristic
VERSION:	1.0 - Michael Zauner (04.05.2020)
**********************************************************/
uint16_t PATH_calcHeuristic(matrixpoint_t actual, matrixpoint_t goal)
{
	uint16_t dx = abs(actual.Xpos - goal.Xpos);
	uint16_t dy = abs(actual.Ypos - goal.Ypos);
	
	/* returns estimated distance to goal-point [mm] */
	return ((dx > dy) ? ((dx * PATH_GRID_RESOLUTION) + ((dy * PATH_GRID_RESOLUTION) / 2)) : ((dy * PATH_GRID_RESOLUTION) + ((dx * PATH_GRID_RESOLUTION) / 2)));
}


/**********************************************************
NAME:		PATH_ReducePoints
FUNC.:		delete all unnecessary points
IN:			start ... start-point in matrix
goal ... goal-point in matrix
OUT:		number of used elements in P
VERSION:	1.0 - Michael Zauner (04.05.2020)
**********************************************************/
uint8_t PATH_ReducePoints(matrixpoint_t start, matrixpoint_t goal)
{
	/* indices to point-lists */
	uint8_t i = start.Xpos, j = start.Ypos, k = 0;
	/* auxiliary variables */
	uint8_t ii, jj, iii, dx1, dx2, dy1, dy2, Dx, Dy;
	
	/* set the point-list to an invalid value */
	for (uint8_t m = 0; m < (sizeof(path.P) / sizeof(matrixpoint_t)); m++)
	{
		path.P[m].Xpos = 255;
		path.P[m].Ypos = 255;
	}
	
	/* set first point in list */
	path.P[k].Xpos = i;
	path.P[k].Ypos = j;
	
	/* set point-list-index to the second element */
	k = 1;
	
	while (1)
	{
		/* readout the indices of the next point */
		ii = path.p[i][j].Xpos;
		jj = path.p[i][j].Ypos;
		
		/* calculate the differnt in x and y between the current point and the next point */
		dx1 = i - ii;
		dy1 = j - jj;

		/* calculate the different in x and y between the next point and the next but one point */
		dx2 = path.p[i][j].Xpos - path.p[ii][jj].Xpos;
		dy2 = path.p[i][j].Ypos - path.p[ii][jj].Ypos;

		/* if the diferent in x or y is unequal -> (ii,jj) = corner-point */
		if ((dx1 != dx2) || (dy1 != dy2))
		{
			/* calculate the differnt between the last stored point and the corner point in x and y */
			Dx = abs(path.p[i][j].Xpos - path.P[k-1].Xpos);
			Dy = abs(path.p[i][j].Ypos - path.P[k-1].Ypos);
			/* if the different is bigger than 1 in one of both dimensions or at least 1 in both dimensions */
			/*  -> store point to reduced point-list */
			if (((Dx > 1) || (Dy > 1)) || ((Dx == 1) && (Dy == 1)))
			{
				/* store corner-point */
				path.P[k] = path.p[i][j];
				/* increment index */
				k++;
			}
		}

		/* when the examination has reached the goal point -> break */
		if ((path.p[ii][jj].Xpos == goal.Xpos) && (path.p[ii][jj].Ypos == goal.Ypos))
		{
			/* store goal-point to point-list */
			path.P[k] = path.p[ii][jj];
			break;
		}
		
		/* readout the next point in list */
		/* buffer store index in x-direction */
		iii = path.p[i][j].Xpos;
		/* get new index in y-direction */
		j = path.p[i][j].Ypos;
		/* set new index in x-direction */
		i = iii;
	}
	
	/* return the count of the used elments in P */
	return (k + 1);
}


/**********************************************************
NAME:		PATH_FlattenTrajectory
FUNC.:		delete all unnecessary points
IN:			start ... start-point in matrix
goal ... goal-point in matrix
OUT:		-
VERSION:	1.0 - Michael Zauner (04.05.2020)
**********************************************************/
void PATH_FlattenTrajectory(matrixpoint_t start, matrixpoint_t goal)
{
	/* indices to point-lists */
	uint8_t k = 0;
	/* auxiliary variables */
	uint8_t m, n, dx1, dx2, dy1, dy2;

	/* set first point in list */
	path.F[k] = start;
	/* set index of the F-list (flatten trajectory) to the second element */
	k = 1;

	/* set index of the P-list (reduced points) to the second element */
	m = 1;

	while (1)
	{
		/* execute the flatten-algorithm as long as points in the P-list */
		if (m < path.dimP)
		{
			/* calculate delta between current point and previous point */
			dx1 = path.P[m].Xpos - path.P[m-1].Xpos;
			dy1 = path.P[m].Ypos - path.P[m-1].Ypos;
			/* calculate delta between next point and previous point */
			dx2 = path.P[m+1].Xpos - path.P[m-1].Xpos;
			dy2 = path.P[m+1].Ypos - path.P[m-1].Ypos;
			/* set the extra-increment-variable to zero */
			n = 0;
			
			/* if the change between the to deltas is bigger than 2 in x or y */
			/*  -> store point to flatten list */
			if ((abs(dx1-dx2) > 1) || (abs(dy1-dy2) > 1))
			{
				path.F[k] = path.P[m];
				k++;
			}
			/* otherwide: */
			/*  -> store the next point (current point has been deleted!) */
			else
			{
				path.F[k] = path.P[m+1];
				/* increment F-list index */
				k++;
				/* set the extra-increment-variable to one */
				n = 1;
			}
		}
		/* otherwise: store goal-point to list -> algorithm is finished */
		else
		{
			path.F[k] = goal;
			break;
		}
		// if the examination has reached the goal-point -> algorithm is finished
		if ((path.P[k-1].Xpos == goal.Xpos) && (path.P[k-1].Ypos == goal.Ypos))
		{
			break;
		}
		/* calculate the next P-list-index */
		m = m + n + 1;
	}
}


/**********************************************************
NAME:		PATH_AddItemToObstacleList
FUNC.:		add an obstacle to the list
IN:			type ... (0,1,2) robots; (255) static obstacles
xy ... lower-left corner
XY ... upper-right corner
OUT:		1 .. object has been stored to list
0 .. object has not been stored to list
VERSION:	1.0 - Michael Zauner (07.05.2020)
**********************************************************/
uint8_t PATH_AddItemToObstacleList(uint8_t type, int8_t x, int8_t y, int8_t X, int8_t Y)
{
	/* store robot to list */
	if (type != PATH_STATIC_OBSTACLE)
	{
		path.obstacleList[type].xy.Xpos = x;
		path.obstacleList[type].xy.Ypos = y;
		path.obstacleList[type].XY.Xpos = X;
		path.obstacleList[type].XY.Ypos = Y;
		return(1);
	}
	/* store static object to list */
	else
	{
		/* search for free place in list */
		for (uint8_t i = 5; i < PATH_OBSTACLE_LIST_LENGTH; i++)
		{
			/* store the list elements */
			if ((path.obstacleList[i].xy.Xpos == PATH_OBSTACLE_LIST_EMPTY) && (path.obstacleList[i].XY.Xpos == PATH_OBSTACLE_LIST_EMPTY))
			{
				path.obstacleList[i].xy.Xpos = x;
				path.obstacleList[i].xy.Ypos = y;
				path.obstacleList[i].XY.Xpos = X;
				path.obstacleList[i].XY.Ypos = Y;
				return(1);
			}
		}
	}
	
	/* no list element was free -> return error */
	return(0);
}



/**********************************************************
NAME:		PATH_AddItemListToObstacleList
FUNC.:		add an obstacle to the list
IN:			item ... pointer to item-list
OUT:		-
VERSION:	1.0 - Michael Zauner (21.04.2021)
**********************************************************/
void PATH_AddItemListToObstacleList(StaticObstical_t *item)
{
	for (uint8_t j = 0; j < PATH_STATIC_OBSTICAL_LIST_LENGHT; j++)
	{
		/* check if the static obstacle is enabled */
		if (item[j].enableObstical == 1)
		{
			PATH_AddItemToObstacleList(PATH_STATIC_OBSTACLE, item[j].x, item[j].y, item[j].X, item[j].Y);
		}
	}
}

/**********************************************************
NAME:		PATH_DeleteItemFromObstacleList
FUNC.:		delete an obstacle to the list
IN:			type ... (0,1,2) robots; (255) static obstacles
xy ... lower-left corner
XY ... upper-right corner
OUT:		1 .. object has been deleted from list
0 .. object has not been deleted from list
VERSION:	1.0 - Michael Zauner (07.05.2020)
**********************************************************/
uint8_t PATH_DeleteItemFromObstacleList(uint8_t type, int8_t x, int8_t y, int8_t X, int8_t Y)
{
	/* delete robot from list */
	if (type != PATH_STATIC_OBSTACLE)
	{
		path.obstacleList[type].xy.Xpos = PATH_OBSTACLE_LIST_EMPTY;
		path.obstacleList[type].xy.Ypos = PATH_OBSTACLE_LIST_EMPTY;
		path.obstacleList[type].XY.Xpos = PATH_OBSTACLE_LIST_EMPTY;
		path.obstacleList[type].XY.Ypos = PATH_OBSTACLE_LIST_EMPTY;
		return(1);
	}
	/* delete static object from list */
	else
	{
		/* search for item in list */
		for (uint8_t i = 3; i < PATH_OBSTACLE_LIST_LENGTH; i++)
		{
			/* delete the list elements */
			if ((path.obstacleList[i].xy.Xpos == x) && (path.obstacleList[i].xy.Ypos == y) && (path.obstacleList[i].XY.Xpos == X) && (path.obstacleList[i].XY.Ypos == Y))
			{
				path.obstacleList[i].xy.Xpos = PATH_OBSTACLE_LIST_EMPTY;
				path.obstacleList[i].xy.Ypos = PATH_OBSTACLE_LIST_EMPTY;
				path.obstacleList[i].XY.Xpos = PATH_OBSTACLE_LIST_EMPTY;
				path.obstacleList[i].XY.Ypos = PATH_OBSTACLE_LIST_EMPTY;
				return(1);
			}
		}
	}
	
	/* no list element was free -> return error */
	return(0);
}

/**********************************************************
NAME:		PATH_DeleteObstacleList
FUNC.:		delete an obstacle to the list
IN:			-
OUT:		-
VERSION:	1.0 - Michael Zauner (07.05.2020)
**********************************************************/
void PATH_DeleteObstacleList()
{
	/* delete the whole list */
	for (uint8_t i = 0; i < PATH_OBSTACLE_LIST_LENGTH; i++)
	{
		path.obstacleList[i].xy.Xpos = PATH_OBSTACLE_LIST_EMPTY;
		path.obstacleList[i].xy.Ypos = PATH_OBSTACLE_LIST_EMPTY;
		path.obstacleList[i].XY.Xpos = PATH_OBSTACLE_LIST_EMPTY;
		path.obstacleList[i].XY.Ypos = PATH_OBSTACLE_LIST_EMPTY;
	}
}

/**********************************************************
NAME:		PATH_Get_ObstacleLimits
FUNC.:		returns the limits of the robot
IN:			obstacle ... coordinates of the obsacle (center)
OUT:		limits of robot
VERSION:	1.0 - Michael Zauner (08.05.2020)
**********************************************************/
obstacle_t PATH_Get_ObstacleLimits(point_t obstacle)
{
	matrixpoint_t obstacleInGrid;
	obstacle_t robot;
	
	/* insert the obstacle only if the position is valid */
	if ((obstacle.Xpos > 0) && (obstacle.Ypos > 0))
	{
		/* get the nearest grid coordinates of the obstacle */
		obstacleInGrid = PATH_FindNearestMatrixPoint(obstacle);
		
		/* set lower limit */
		robot.xy.Xpos = (((obstacleInGrid.Xpos - PATH_OBSTACLE_SIZE) < 0) ? 0 : (obstacleInGrid.Xpos - PATH_OBSTACLE_SIZE));
		robot.xy.Ypos = (((obstacleInGrid.Ypos - PATH_OBSTACLE_SIZE) < 0) ? 0 : (obstacleInGrid.Ypos - 3));
		/* set upper limit */
		robot.XY.Xpos = (((obstacleInGrid.Xpos + PATH_OBSTACLE_SIZE) >= PATH_GRID_DIM_X) ? (PATH_GRID_DIM_X - 1) : (obstacleInGrid.Xpos + PATH_OBSTACLE_SIZE));
		robot.XY.Ypos = (((obstacleInGrid.Ypos + PATH_OBSTACLE_SIZE) >= PATH_GRID_DIM_Y) ? (PATH_GRID_DIM_Y - 1) : (obstacleInGrid.Ypos + PATH_OBSTACLE_SIZE));
	}
	else
	{
		/* set lower limit */
		robot.xy.Xpos = PATH_OBSTACLE_LIST_EMPTY;
		robot.xy.Ypos = PATH_OBSTACLE_LIST_EMPTY;
		/* set upper limit */
		robot.XY.Xpos = PATH_OBSTACLE_LIST_EMPTY;
		robot.XY.Ypos = PATH_OBSTACLE_LIST_EMPTY;
	}
	return(robot);
}

/**********************************************************
NAME:		PATH_Set_ObstacleListInOccupancyGrid
FUNC.:		set all obstacles in the occupancy grid
IN:			-
OUT:		-
VERSION:	1.0 - Michael Zauner (08.05.2020)
**********************************************************/
void PATH_Set_ObstacleListInOccupancyGrid()
{
	/* set all elements from obstacle-list  */
	for (int8_t k = 0;  k < PATH_OBSTACLE_LIST_LENGTH; k++)
	{
		if (path.obstacleList[k].xy.Xpos != PATH_OBSTACLE_LIST_EMPTY)
		{
			/* i ... x-coordinate of occupancy grid */
			for (uint8_t i = path.obstacleList[k].xy.Xpos; i <= path.obstacleList[k].XY.Xpos; i++)
			{
				/* j ... y-coordinate of occupancy grid */
				for (uint8_t j = path.obstacleList[k].xy.Ypos; j <= path.obstacleList[k].XY.Ypos; j++)
				{
					path.occupancyGrid[i][j] = PATH_INFINITY;
					/* set occupied points to already inspected */
					path.l[i][j] = PATH_INSPECTED;
				}
			}
		}
	}
}


/**********************************************************
NAME:		PATH_SetSegmentList
FUNC.:		sets all obstacle-boundaries in the segment-list (works only with rectangles)
IN:			-
OUT:		-
VERSION:	1.0 - Michael Zauner (08.05.2020)
**********************************************************/
void PATH_SetSegmentList()
{
	/* index of the segment-list */
	uint8_t j = 0;
	
	/* clear segment list */
	for (uint8_t i = 0; i < PATH_SEGMENT_LIST_LENGTH; i++)
	{
		path.segmentList[i].o.Xpos = 0;
		path.segmentList[i].o.Ypos = 0;
		path.segmentList[i].v.x = 0;
		path.segmentList[i].v.y = 0;
	}
	
	/* check all obstacles */
	for (uint8_t i = 0; i < PATH_OBSTACLE_LIST_LENGTH; i++)
	{
		/* if obstacle-list-element is valid -> calculate all vectors */
		if (path.obstacleList[i].xy.Xpos != PATH_OBSTACLE_LIST_EMPTY)
		{
			/*
			Xy       XY
			o-------o
			|		|
			|		|
			o-------o
			xy      xY
			*/
			/* vector xy - xY */
			path.segmentList[j].o.Xpos = path.obstacleList[i].xy.Xpos;
			path.segmentList[j].o.Ypos = path.obstacleList[i].xy.Ypos;
			path.segmentList[j].v.x = path.obstacleList[i].XY.Xpos - path.obstacleList[i].xy.Xpos;
			path.segmentList[j++].v.y = 0;

			/* vector xY - XY */
			path.segmentList[j].o.Xpos = path.obstacleList[i].XY.Xpos;
			path.segmentList[j].o.Ypos = path.obstacleList[i].xy.Ypos;
			path.segmentList[j].v.x = 0;
			path.segmentList[j++].v.y = path.obstacleList[i].XY.Ypos - path.obstacleList[i].xy.Ypos;
			
			/* vector XY - Xy */
			path.segmentList[j].o.Xpos = path.obstacleList[i].XY.Xpos;
			path.segmentList[j].o.Ypos = path.obstacleList[i].XY.Ypos;
			path.segmentList[j].v.x = path.obstacleList[i].xy.Xpos - path.obstacleList[i].XY.Xpos;
			path.segmentList[j++].v.y = 0;

			/* vector Xy - xy */
			path.segmentList[j].o.Xpos = path.obstacleList[i].xy.Xpos;
			path.segmentList[j].o.Ypos = path.obstacleList[i].XY.Ypos;
			path.segmentList[j].v.x = 0;
			path.segmentList[j++].v.y = path.obstacleList[i].xy.Ypos - path.obstacleList[i].XY.Ypos;
		}
	}
	path.segmentListLength = j;
}

/**********************************************************
NAME:		sign
FUNC.:		returns the sign of the number
IN:			a ... number
OUT:		0 ... a = 0; 1 ... a > 0; -1 ... a < 0
VERSION:	1.0 - Michael Zauner (12.05.2020)
**********************************************************/
int8_t sign(int16_t a)
{
	return ((a < 0) ? -1 : ((a == 0) ? 0 : 1));
}

/**********************************************************
NAME:		PATH_CalcIntersection
FUNC.:		calculate if an intersection between to vectors exists
IN:			A1 ... starting-point of vector A
A2 ... ending-poimt of vector A
B ... vector B
OUT:		0 ... no intersection; 1 ... intersection
VERSION:	1.0 - Michael Zauner (12.05.2020)
**********************************************************/
uint8_t PATH_CalcIntersection(matrixpoint_t A1, matrixpoint_t A2, segment_t B)
{
	/* vector A */
	segment_t A;
	vector_t C;
	/*  */
	int16_t detK, zAlpha, zBeta;
	
	/* set vector A */
	A.v.x = A2.Xpos - A1.Xpos;
	A.v.y = A2.Ypos - A1.Ypos;
	/* set origin of vectr A */
	A.o.Xpos = A1.Xpos;
	A.o.Ypos = A1.Ypos;
	
	/* system of equations to calculate an intersection */
	/* Ax*Alpha - Dx*Beta = Cx */
	/* Ay*Alpha - Dy*Beta = Cy */
	detK = A.v.x * B.v.y - A.v.y * B.v.x;
	
	/* if detK == 0 -> vector A and B are parallel */
	if (detK != 0)
	{
		/* calculate vector C - vector points from the origin from vector A zo the origin of vector B */
		C.x = B.o.Xpos - A.o.Xpos;
		C.y = B.o.Ypos - A.o.Ypos;
		
		/* calcualte the numerator of Alpha and Beta */
		zAlpha =  C.x * B.v.y - C.y * B.v.x;
		zBeta =  C.x * A.v.y - C.y * A.v.x;
		
		/* intersection only at: (0 < Alpha < 1) and (0 < Beta < 1) - Alpha = zAlpha/detK and Beta = zBeta/detK */
		/*  -> zAlpha, detK and zBeta must have the same sign */
		/*  -> |zAlpha| < |detK| and |zBeta| < |detK| */
		/* otherwise there is no intersection -> return 0 */
		//		if (((sign(detK) != sign(zAlpha)) && (zAlpha != 0)) || ((sign(detK) != sign(zBeta)) && (zBeta != 0)) || (abs(detK) < abs(zAlpha)) || (abs(detK) < abs(zBeta)) && (zAlpha != 0) && (zBeta != 0))
		if (((sign(detK) != sign(zAlpha)) && (zAlpha != 0)) || ((sign(detK) != sign(zBeta)) && (zBeta != 0)) || (((abs(detK) < abs(zAlpha)) || (abs(detK) < abs(zBeta))) && (zAlpha != 0) && (zBeta != 0)))
		{
			return(0);
		}
		/* intersection has been detected -> return 1 */
		else
		{
			return(1);
		}
	}
	
	/* no intersection */
	return (0);
}


/**********************************************************
NAME:		PATH_ShortenPath
FUNC.:		shorten the path to a final path
IN:			start ... start-point in matrix
goal ... goal-point in matrix
OUT:		-
VERSION:	1.0 - Michael Zauner (12.05.2020)
**********************************************************/
void PATH_ShortenPath(matrixpoint_t start, matrixpoint_t goal)
{
	/* actual point (AP) from this point all conections to the rest */
	/* of the points of the path will be checked */
	matrixpoint_t AP;
	/* index to the final-list */
	uint8_t i = 0, k = 1;
	
	/* set starting-point as actual point */
	AP = start;
	
	/* reset final-list */
	for (uint8_t m = 0; m < PATH_GRID_DIM_X; m++)
	{
		path.finalList[m].Xpos = 0;
		path.finalList[m].Ypos = 0;
	}

	/* if the next point is the goal-point -> store point in list and break */
	while ((path.F[k].Xpos != goal.Xpos) || (path.F[k].Ypos != goal.Ypos))
	{
		/* select next point from list */
		k++;
		
		/* check an intersection between the actual path-segment with all obstacles */
		for (uint8_t j = 0; j < path.segmentListLength; j++)
		{
			/* if an intersection has been found */
			/*  -> store NP to final-list and proceed with this point AP = NP */
			if (PATH_CalcIntersection(AP, path.F[k], path.segmentList[j]) == 1)
			{
				/* if there has been an intersection -> select the former point from list */
				k--;
				path.finalList[i].Xpos = path.F[k].Xpos;
				path.finalList[i].Ypos = path.F[k].Ypos;
				i++;
				AP = path.F[k];
				/* switch to the next point in list */
				k++;
				break;
			}
		}
	}

	/* store goal-point to final-list */
	path.finalList[i].Xpos = path.F[k].Xpos;
	path.finalList[i].Ypos = path.F[k].Ypos;
}


/**********************************************************
NAME:		PATH_StoreRealPoints
FUNC.:		stores the real coordinates
IN:			goal ... goal-point in matrix
OUT:		nbr ... number of points
VERSION:	1.0 - Michael Zauner (12.05.2020)
**********************************************************/
uint8_t PATH_StoreRealPoints(matrixpoint_t goal, point_t realGoal)
{
	uint8_t nbr = 0;

	/* reset final-list */
	for (uint8_t m = 0; m < PATH_GRID_DIM_X; m++)
	{
		path.pointList[m].Xpos = 0;
		path.pointList[m].Ypos = 0;
	}
	
	/* proceed until the goal-point */
	while ((path.finalList[nbr].Xpos != goal.Xpos) || (path.finalList[nbr].Ypos != goal.Ypos))
	{
		/* calculate the real position and store it to the point-list */
		path.pointList[nbr].Xpos = PATH_NON_TRAFFICLE_AREA + PATH_GRID_RESOLUTION * (int16_t)(path.finalList[nbr].Xpos);
		path.pointList[nbr].Ypos = PATH_NON_TRAFFICLE_AREA + PATH_GRID_RESOLUTION * (int16_t)(path.finalList[nbr].Ypos);
		nbr++;
	}
	/* store goal-point to final-list */
	path.pointList[nbr] = realGoal;
	/* return number of points */
	return(nbr+1);
}


/**********************************************************
NAME:		PATH_SetStaticObstacle
FUNC.:		stores a static obstacle
IN:			nbr ... obstacle number
xy ... lower-left corner
XY ... upper-right corner
OUT:		-
VERSION:	1.0 - Michael Zauner (24.05.2023)
**********************************************************/
void PATH_SetStaticObstacle(uint8_t nbr, int16_t x, int16_t y, int16_t X, int16_t Y)
{
	int16_t xInGrid, yInGrid, XInGrid, YInGrid;
	
	/* calculate the grid-position from the lower x-coordinate */
	xInGrid = (x - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION;
	xInGrid = ((xInGrid < 0) ? 0 : xInGrid);
	xInGrid = ((xInGrid >= PATH_GRID_DIM_X) ? (PATH_GRID_DIM_X-1) : xInGrid);

	/* calculate the grid-position from the lower y-coordinate */
	yInGrid = (y - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION;
	yInGrid = ((yInGrid < 0) ? 0 : yInGrid);
	yInGrid = ((yInGrid >= PATH_GRID_DIM_Y) ? (PATH_GRID_DIM_Y-1) : yInGrid);

	/* calculate the grid-position from the upper x-coordinate */
	XInGrid = (X - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION;
	XInGrid = ((XInGrid < 0) ? 0 : XInGrid);
	XInGrid = ((XInGrid >= PATH_GRID_DIM_X) ? (PATH_GRID_DIM_X-1) : XInGrid);

	/* calculate the grid-position from the upper y-coordinate */
	YInGrid = (Y - PATH_NON_TRAFFICLE_AREA) / PATH_GRID_RESOLUTION;
	YInGrid = ((YInGrid < 0) ? 0 : YInGrid);
	YInGrid = ((YInGrid >= PATH_GRID_DIM_Y) ? (PATH_GRID_DIM_Y-1) : YInGrid);

	/* set obstacle */
	PATH_SET_OBSTICAL(nbr,xInGrid,yInGrid,XInGrid,YInGrid);

}

///**********************************************************
//NAME:		PATH_Set_ObstacleInOccupancyGrid
//FUNC.:		set all elements in the occupancy grid to FREE
//IN:			obstacle ... coordinates of the obsacle (center)
//OUT:		-
//VERSION:	1.0 - Michael Zauner (24.04.2020)
//**********************************************************/
//void PATH_Set_ObstacleInOccupancyGrid(point_t obstacle)
//{
//volatile matrixpoint_t obstacleInGrid;
//uint8_t cost[4] = {PATH_INFINITY, PATH_INFINITY, PATH_INFINITY, PATH_OBSTACLE_ZONE_1};
//
///* insert the obstacle only if the position is valid */
//if ((obstacle.Xpos > 0) && (obstacle.Ypos > 0))
//{
///* get the nearest grid coordinates of the obstacle */
//obstacleInGrid = PATH_FindNearestMatrixPoint(obstacle);
//
///* set center of the obstacle as occupied */
///*
//c ... center of the obstacle
//o ... obstacle -> PATH_INFINITY
//x ... area closest to the ostacle -> PATH_INFINITY (PATH_ZONE_3)
//u ... area closer to the ostacle -> PATH_INFINITY (PATH_ZONE_2)
//n ... area close to the ostacle -> PATH_ZONE_1
//
//n n n n n n n n n
//n u u u u u u u n
//n u x x x x x u n
//n u x o o o x u n
//n u x o c o x u n
//n u x o o o x u n
//n u x x x x x u n
//n u u u u u u u n
//n n n n n n n n n
//*/
///* k ... zone around the obstacle (1 .. obstacle; 2 .. closest; 3 .. closer; 4 .. close  */
//for (int8_t k = 1;  k < 5; k++)
//{
//uint8_t breakX = ((obstacleInGrid.Xpos < (PATH_GRID_DIM_X - k)) ? (obstacleInGrid.Xpos + k) : (PATH_GRID_DIM_X - 1));
//uint8_t breakY = ((obstacleInGrid.Ypos < (PATH_GRID_DIM_Y - k)) ? (obstacleInGrid.Ypos + k) : (PATH_GRID_DIM_Y - 1));
//
///* i ... x-coordinate of occupancy grid */
//for (uint8_t i = ((obstacleInGrid.Xpos >= k) ? (obstacleInGrid.Xpos - k) : 0); i <= breakX; i++)
//{
///* j ... y-coordinate of occupancy grid */
//for (uint8_t j = ((obstacleInGrid.Ypos >= k) ? (obstacleInGrid.Ypos - k) : 0); j <= breakY; j++)
//{
///* set occupancy grid (only if the cost is higher than the actual cost) */
//if (path.occupancyGrid[i][j] < cost[k-1])
//{
//path.occupancyGrid[i][j] = cost[k-1];
///* set occupied points to already inspected */
//if (cost[k-1] == PATH_INFINITY)
//{
//path.l[i][j] = PATH_INSPECTED;
//}
//}
//}
//}
//}
//}
//}






