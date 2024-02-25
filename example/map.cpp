#include "map.h"
#include "logflash.h"

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))


int8_t h_walls[MAZE_HEIGHT - 1][MAZE_WIDTH] = {0};
int8_t v_walls[MAZE_HEIGHT][MAZE_WIDTH - 1] = {0};

// assumes pos at 0,0 facing right, middle of cell
state_t g_state = {
    DRIGHT, 0, 0, CELL_WIDHT_MM / 2}; // global micrmomouse state

int8_t* getVWalls() {
    return (int8_t *)v_walls;
}

int8_t* getHWalls() {
    return (int8_t *)h_walls;
}

void setVWalls(int8_t walls[MAZE_HEIGHT][MAZE_WIDTH - 1]) {
    memcpy(v_walls, walls, MAZE_HEIGHT*(MAZE_WIDTH - 1));
}

void setHWalls(int8_t walls[MAZE_HEIGHT - 1][MAZE_WIDTH]) {
    memcpy(h_walls, walls, (MAZE_HEIGHT - 1)*MAZE_WIDTH);
}

void printMaze()
{
    lfprintf("Mouse pos (%d,%d) orientation: %d distance_in_cell_mm:%f\n",
           g_state.x, g_state.y, g_state.ori, g_state.dist_in_cell_mm);
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
            lfprintf(" %d", v_walls[x][y]);
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
            lfprintf("%d ", h_walls[x][y]);
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
    int facing = g_state.ori ;
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
    int facing = g_state.ori ;

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
        if (v_walls[x][y - 1] < 0)
        {
            return 1;
        }
        return 0;
    }
}

void add_bound_h(int x, int y, int8_t confidence)
{
    h_walls[x][y] = max(min(h_walls[x][y] + confidence, INT8_MAX), INT8_MIN);
}

void add_bound_v(int x, int y, int8_t confidence)
{
    v_walls[x][y] = max(min(v_walls[x][y] + confidence, INT8_MAX), INT8_MIN);
}

// dir -> DLEFT, DRIGHT, Forawrd. Confidence -> [0,1]
void mouseUpdateWall(int8_t confidence, int dir)
{
    int x = g_state.x;
    int y = g_state.y;
    int facing = g_state.ori;

    if (g_state.dist_in_cell_mm > 91 || g_state.dist_in_cell_mm < 89)
    {
        return;
    }

    switch (facing)
    {
    case ORIGHT:
        switch (dir)
        {
        case DRIGHT:
            if (x == MAZE_HEIGHT - 1)
                break;
            add_bound_h(x,y,confidence);
            break;
        case DLEFT:
            if (x - 1 < 0)
                break; // uhoh
            add_bound_h(x-1,y,confidence);
            break;
        case DFORWARD:
            if (y == MAZE_WIDTH - 1)
                break;
            add_bound_v(x,y,confidence);
            break;
        }
        break;
    case ODOWN:
        switch (dir)
        {
        case DRIGHT:
            if (y - 1 < 0)
                break; // uhoh
            add_bound_v(x,y-1,confidence);
            break;
        case DLEFT:
            if (y == MAZE_WIDTH - 1)
                break;
            add_bound_v(x,y,confidence);
            break;
        case DFORWARD:
            if (x == MAZE_HEIGHT - 1)
                break;
            add_bound_h(x,y,confidence);
            break;
        }
        break;
    case OLEFT:
        switch (dir)
        {
        case DRIGHT:
            if (x - 1 < 0)
                break; // uhoh
            add_bound_h(x-1,y,confidence);
            break;
        case DLEFT:
            if (x == MAZE_HEIGHT - 1)
                break;
            add_bound_h(x,y,confidence);
            break;
        case DFORWARD:
            if (y - 1 < 0)
                break; // uhoh
            add_bound_v(x,y-1,confidence);
            break;
        }
        break;
    case OUP:
        switch (dir)
        {
        case DRIGHT:
            if (y == MAZE_WIDTH - 1)
                break;
            add_bound_v(x,y,confidence);
            break;
        case DLEFT:
            if (y - 1 < 0)
                break; // uhoh
            add_bound_v(x,y-1,confidence);
            break;
        case DFORWARD:
            if (x - 1 < 0)
                break; // uhoh
            add_bound_h(x-1,y,confidence);
            break;
        }
    }
}

// dir -> DRIGHT, DLEFT
void mouseUpdateDir(int dir)
{
    if (dir == DRIGHT)
    {
        g_state.ori = (g_state.ori +1) % 4;
    }
    else
    {
        g_state.ori = (g_state.ori +3) % 4;
    }
    // assume that we are in the moddle of the cell after the turn
    g_state.dist_in_cell_mm = CELL_WIDHT_MM / 2;
}

void mouseUpdateOri(int ori)
{
    g_state.ori = ori;
    // assume that we are in the moddle of the cell after the turn
    g_state.dist_in_cell_mm = CELL_WIDHT_MM / 2;
}

// call this once every iter of control loop?
int isMouseInDestinationZone() {
    int first_x_dest = ((int) MAZE_HEIGHT / 2) - 1;
    int first_y_dest = ((int) MAZE_WIDTH / 2) - 1;
    if ((g_state.x == first_x_dest || g_state.x == first_x_dest + 1) && (g_state.y == first_y_dest || g_state.y == first_y_dest + 1)) {
        return 1;
    }
    return -1;
}

void mouseUpdateOdom(float distance_mm)
{
    // add distance to dist_in_cell_mm
    // while dist >= cell_width
    // update x,y
    g_state.dist_in_cell_mm += distance_mm;
    while (g_state.dist_in_cell_mm > CELL_WIDHT_MM)
    {
        switch (g_state.ori)
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
