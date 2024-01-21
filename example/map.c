#include "map.h"

float h_walls[MAZE_HEIGHT-1][MAZE_WIDTH-1] = {0};
float v_walls[MAZE_HEIGHT-1][MAZE_WIDTH-1] = {0};

// assumes pos at 0,0 facing right, middle of cell
state_t g_state = {
    DRIGHT,5,5,CELL_WIDHT_MM/2
}; // global micrmomouse state

void printMaze()
{
    printf("Mouse pos (%d,%d) orientation: %d distance_in_cell_mm:%d\n",
           g_state.x, g_state.y, g_state.or, g_state.dist_in_cell_mm);
    for (int x = 0; x< MAZE_HEIGHT-1;x++)
    {
        printf("v|");
        for(int y = 0; y < MAZE_WIDTH-1;y++)
        {
            //print vertication
            if (x == g_state.x && y == g_state.y)
            {
                printf("X");
            } 
            printf(" %2.0f", v_walls[x][y]);
        }
        printf("\n");
        printf("h_ ");
        for(int y = 0; y < MAZE_WIDTH-1;y++)
        {
            //print vertication
            printf("%2.0f ", h_walls[x][y]);
        }
        printf("\n");
    }
    printf("\n");
    printf("\n");
}
/*
 - - - - -
| X | | | |
 - - - - -
| X | | | |
*/

state_t mouseGetState()
{
    return g_state;
}

// dir -> DLEFT, DRIGHT, Forawrd. Confidence -> [0,1]
void mouseUpdateWall(float confidence, int dir)
{
    if (confidence > 0)
    {
        printf("voting wall to the ");
    } else {
        printf("voting gap to the ");
    }
    if (dir == DRIGHT) {
        printf("right\n");
    } else if (dir == DLEFT){
        printf("left\n");
    } else if (dir == DFORWARD) {
        printf("ahead\n");
    }
    int x = g_state.x;
    int y = g_state.y;
    int facing = g_state.or ;

    switch (facing)
    {
    case ORIGHT:
        switch (dir)
        {
        case DRIGHT:
            h_walls[x][y] += confidence;
            break;
        case DLEFT:
            if (x - 1 < 0)
                break; // uhoh
            h_walls[x - 1][y] += confidence;
            break;
        case DFORWARD:
            v_walls[x][y] += confidence;
            break;
        }
        break;
    case ODOWN:
        switch (dir)
        {
        case DRIGHT:
            if (y - 1 < 0)
                break; // uhoh
            v_walls[x][y - 1] += confidence;
            break;
        case DLEFT:
            v_walls[x][y] += confidence;
            break;
        case DFORWARD:
            h_walls[x][y] += confidence;
            break;
        }
        break;
    case OLEFT:
        switch (dir)
        {
        case DRIGHT:
            if (x - 1 < 0)
                break; // uhoh
            h_walls[x - 1][y] += confidence;
            break;
        case DLEFT:
            h_walls[x][y] += confidence;
            break;
        case DFORWARD:
            if (y - 1 < 0)
                break; // uhoh
            v_walls[x][y - 1] += confidence;
            break;
        }
        break;
    case OUP:
        switch (dir)
        {
        case DRIGHT:
            v_walls[x][y] += confidence;
            break;
        case DLEFT:
            if (y - 1 < 0)
                break; // uhoh
            v_walls[x][y - 1] += confidence;
            break;
        case DFORWARD:
            if (x - 1 < 0)
                break; // uhoh
            h_walls[x - 1][y] += confidence;
            break;
        }
    }
}

// dir -> DRIGHT, DLEFT
void mouseUpdateDir(int dir)
{
    if (dir == DRIGHT)
    {
        g_state.or = (g_state.or + 1) % 4;
    }
    else
    {
        g_state.or = (g_state.or +3) % 4;
    }
    // assume that we are in the moddle of the cell after the turn
    g_state.dist_in_cell_mm = CELL_WIDHT_MM / 2;
}

void mouseUpdateOdom(int distance_mm)
{
    // add distance to dist_in_cell_mm
    // while dist >= cell_width
    // update x,y
    g_state.dist_in_cell_mm += distance_mm;
    while (g_state.dist_in_cell_mm > CELL_WIDHT_MM)
    {
        switch (g_state.or)
        {
        case ORIGHT:
            g_state.y += 1;
            break;
        case ODOWN:
            g_state.x += 1;
            break;
        case OLEFT:
            g_state.y -= 1;
            break;
        case OUP:
            g_state.x -= 1;
            break;
        }
        g_state.dist_in_cell_mm -= CELL_WIDHT_MM;
    }
}
