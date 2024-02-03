#include "map.h"
#include "logflash.h"

float h_walls[MAZE_HEIGHT - 1][MAZE_WIDTH] = {0};
float v_walls[MAZE_HEIGHT][MAZE_WIDTH - 1] = {0};

// assumes pos at 0,0 facing right, middle of cell
state_t g_state = {
    DRIGHT, 0, 0, CELL_WIDHT_MM / 2}; // global micrmomouse state

void printMaze()
{
    lfprintf("Mouse pos (%d,%d) orientation: %d distance_in_cell_mm:%f\n",
           g_state.x, g_state.y, g_state.or, g_state.dist_in_cell_mm);
    for (int x = 0; x < MAZE_HEIGHT; x++)
    {
        lfprintf("v|");
        for (int y = 0; y < MAZE_WIDTH - 1; y++)
        {
            // print vertication
            if (x == g_state.x && y == g_state.y)
            {
                lfprintf("X");
            }
            lfprintf(" %2.0f", v_walls[x][y]);
        }
        lfprintf("\n");
        if (x == MAZE_HEIGHT - 1)
        {
            break;
        }
        lfprintf("h| ");
        for (int y = 0; y < MAZE_WIDTH; y++)
        {
            // print vertication
            lfprintf("%2.0f ", h_walls[x][y]);
        }
        lfprintf("\n");
    }
    lfprintf("\n");
    lfprintf("\n");
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

int mouseCanGoRight()
{

    int x = g_state.x;
    int y = g_state.y;
    int facing = g_state.or ;
    switch (facing)
    {
    case ORIGHT:
        if (x == MAZE_HEIGHT - 1)
            return 0;
        if (h_walls[g_state.x][g_state.y] < 0)
        {
            return 1;
        };
        return 0;
    case ODOWN:
        if (y - 1 < 0)
            return 0; // uhoh
        if (v_walls[x][y - 1] < 0)
            return 1;
        return 0;
    case OLEFT:
        if (x - 1 < 0)
            return 0;
        if (h_walls[x - 1][y] < 0)
            return 1;
        return 0;
    case OUP:
        if (y == MAZE_WIDTH - 1)
            return 0;
        if (v_walls[x][y] < 0)
            return 1;
        return 0;
    }
    return 0;
}

int mouseCanGoLeft()
{
    if (h_walls[g_state.x - 1][g_state.y] < 0)
    {
        return 1;
    }
    return 0;

    int x = g_state.x;
    int y = g_state.y;
    int facing = g_state.or ;

    switch (facing)
    {
    case ORIGHT:
            if (x - 1 < 0)
                return 0; // uhoh
            if (h_walls[x - 1][y] < 0)
                return 1;
            return 0;
    case ODOWN:
            if (y == MAZE_WIDTH - 1)
                return 0;
            if (v_walls[x][y] < 0)
                return 1;
            return 0;
    case OLEFT:
            if (x == MAZE_HEIGHT - 1)
                return 0;
            if (h_walls[x][y] < 0)
                return 1;
            return 0;
    case OUP:
            if (y - 1 < 0)
                return 0;
            if (v_walls[x][y - 1] < 0){
                return 1;
            }
            return 0;
    }
}

// dir -> DLEFT, DRIGHT, Forawrd. Confidence -> [0,1]
void mouseUpdateWall(float confidence, int dir)
{
    if (confidence > 0)
    {
        lfprintf("voting wall to the ");
    }
    else
    {
        lfprintf("voting gap to the ");
    }
    if (dir == DRIGHT)
    {
        lfprintf("right\n");
    }
    else if (dir == DLEFT)
    {
        lfprintf("left\n");
    }
    else if (dir == DFORWARD)
    {
        lfprintf("ahead\n");
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
            if (x == MAZE_HEIGHT - 1)
                break;
            h_walls[x][y] += confidence;
            break;
        case DLEFT:
            if (x - 1 < 0)
                break; // uhoh
            h_walls[x - 1][y] += confidence;
            break;
        case DFORWARD:
            if (y == MAZE_WIDTH - 1)
                break;
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
            if (y == MAZE_WIDTH - 1)
                break;
            v_walls[x][y] += confidence;
            break;
        case DFORWARD:
            if (x == MAZE_HEIGHT - 1)
                break;
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
            if (x == MAZE_HEIGHT - 1)
                break;
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
            if (y == MAZE_WIDTH - 1)
                break;
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
        g_state.or = (g_state.or +1) % 4;
    }
    else
    {
        g_state.or = (g_state.or +3) % 4;
    }
    // assume that we are in the moddle of the cell after the turn
    g_state.dist_in_cell_mm = CELL_WIDHT_MM / 2;
}

void mouseUpdateOdom(float distance_mm)
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

    if (g_state.x >= MAZE_HEIGHT)
    {
        lfprintf("We have hit the maze height at position %d, %d\n", g_state.x, g_state.y);
        g_state.x = MAZE_HEIGHT - 1;
    }
    if (g_state.y >= MAZE_WIDTH)
    {
        lfprintf("We have hit the maze width at position %d, %d\n", g_state.x, g_state.y);
        g_state.y = MAZE_WIDTH - 1;
    }
    if (g_state.x < 0)
    {
        lfprintf("We have hit the maze height at position %d, %d\n", g_state.x, g_state.y);
        g_state.x = 0;
    }
    if (g_state.y < 0)
    {
        lfprintf("We have hit the maze width at position %d, %d\n", g_state.x, g_state.y);
        g_state.y = 0;
    }
}
