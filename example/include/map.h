// 16 x 16
#ifndef MAP_H
#define MAP_H

#include "stdio.h"

#include "string.h"
#include <cstdint>

#define MAZE_WIDTH (16)
#define MAZE_HEIGHT (16)
#define CELL_WIDHT_MM (168)

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
    int ori;     // direction the robot is facing
    int x;                 // grid cell occupied x
    int y;               // grid cell occupied y
    float dist_in_cell_mm;     // position within cell from bounding wall (in dir)
} state_t;

void printMaze();

state_t mouseGetState();

void mouseUpdateWall(int8_t confidence, int dir);

void mouseUpdateDir(int dir);

void mouseUpdateOri(int ori);

void mouseUpdateOdom(float distance_mm);

int isMouseInDestinationZone();

int mouseCanGoRight();

int mouseCanGoLeft();

int8_t* getVWalls();

int8_t* getHWalls();

void setVWalls(int8_t[MAZE_HEIGHT][MAZE_WIDTH - 1]);

void setHWalls(int8_t[MAZE_HEIGHT - 1][MAZE_WIDTH]);
#endif