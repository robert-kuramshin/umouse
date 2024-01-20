// 16 x 16
#ifndef MAP_H
#define MAP_H

#include "stdio.h"

#define MAZE_WIDTH (16)
#define MAZE_HEIGHT (16)
#define CELL_WIDHT_MM (180)

// for absoulte orientation of mouse within maze
enum orientation {
    ORIGHT,
    ODOWN,
    OLEFT,
    OUP,
};

// for relative direction with respect to mouse orientation
enum direction {
    DRIGHT,
    DBACKWARDS,
    DLEFT,
    DFORWARD,
};

typedef struct state {
    enum direction or;     // direction the robot is facing
    int x;                 // grid cell occupied x
    int y;               // grid cell occupied y
    int dist_in_cell_mm;     // position within cell from bounding wall (in dir)
} state_t;

void printMaze();

state_t mouseGetState();

void mouseUpdateWall(float confidence, int dir);

void mouseUpdateDir(int dir);

void mouseUpdateOdom(int distance_mm);

#endif