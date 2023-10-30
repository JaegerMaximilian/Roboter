/*
 * filter.c
 *
 * Created: 24.04.2020 15:21:30
 *  Author: Schalk Rene
 *
 *
 * This module prefilters the data for the object detection
 *
 */ 


#include <avr/io.h>
#include "rplidar.h"
#include "define.h"
#include "global.h"
#include "multitask.h"
#include "usart.h"
#include <string.h>
#include "timer.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>




	
	
	void test() {
		int* ptr = malloc(sizeof(int));
		free(ptr);
	}
	