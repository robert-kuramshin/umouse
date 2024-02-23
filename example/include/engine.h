#ifndef ENGINE_H
#define ENGINE_H

#include "stdio.h"
#include "map.h"

#define HEIGHT (MAZE_HEIGHT)
#define WIDTH (MAZE_WIDTH)

void printGraph();

void buildGraph(float[HEIGHT - 1][WIDTH], float[HEIGHT][WIDTH - 1]);

int* getShortestDistancePath(int, int);

char* getPathInstructions(int[MAZE_HEIGHT * MAZE_WIDTH]);
#endif