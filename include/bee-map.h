/*************************************************************************
    > File Name: bee-map.cpp
    > Author: Yang Tian
    > Email: tianyang9310@gmail.com 
    > Created Time: Sun 07 Feb 2016 01:26:31 PM EST
 ************************************************************************/
#ifndef BEEMAP_H
#define BEEMAP_H

#include <stdio.h>
#include <stdlib.h>

// Codes from STR class of Stanford
typedef struct {
	int resolution, size_x, size_y;
	float offset_x, offset_y;
	int min_x, max_x, min_y, max_y;
	float **prob;
} map_type;

void new_hornetsoft_map(map_type *map, int size_x, int size_y);

int read_beesoft_map(const char *mapName, map_type *map);
#endif
