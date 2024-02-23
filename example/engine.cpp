#include "engine.h"
#include "map.h"

float graph[HEIGHT * WIDTH][HEIGHT * WIDTH];

void printGraph()
{
    for (int i = 0; i < HEIGHT; i++)
    {
        for (int j = 0; j < WIDTH; j++)
        {
            printf("%2.0f", graph[i][j]);
        }
        printf("\n");
    }
}

/*
Example:
3x3 maze


Example:
4x4 maze v and h walls:

v_walls[4][3] = {
    {0, 0, -1},
    {1, 0, -1},
    {1, 0, 1},
    {-1, -1, -1}
}
h_walls[3][4] = {
    {1, 1, 1, -1},
    {0, -1, -1, -1},
    {1, 1, 1, -1}
}

-> G = {
    {0, 1, 2, 3},
    {-1, 6, 5, 4},
    {-1, 7, 6, 5},
    {9, 8, 7, 6}
}

v_walls[5][4] = {
    {0, 0, 0, -1},
    {-1, -1, -1, -1},
    {3, -1, -1, -1},
    {3, -1, 3, 3},
    {-1, -1, -1, -1}
}


h_walls[4][5] = {
    {3, 3, 3, 3, -1},
    {3, 3, 3, 3, -1},
    {-1, -1, -1, -1},
    {-1, 3, 3, -1, 3}
}

*/

void buildGraph(float h_walls[HEIGHT - 1][WIDTH], float v_walls[HEIGHT][WIDTH - 1])
{
    // linear in number of cells in the maze :yikes: -> so 256 cells in competition
    for (int i = 0; i < HEIGHT * WIDTH; i++) {
        graph[i][i] = 1;
        // check the next cell on vertical axis
        if (i < HEIGHT * WIDTH - 1) {
            if (i % WIDTH < WIDTH - 1) { // last vertical cell in a maze doesnt have another vertical neighbor
                if (v_walls[i / HEIGHT][i % WIDTH] <= 0) {
                    graph[i][i + 1] = 1;
                    graph[i + 1][i] = 1;
                }
            }
        }
        // all the way to the end of the second last row, check below cell
        if (i < (HEIGHT - 1) * WIDTH) {
            if (h_walls[i / HEIGHT][i % WIDTH] <= 0) {
                graph[i][i + WIDTH] = 1;
                graph[i + WIDTH][i] = 1;
            }
        }
    }
}

int dfs(float graph[HEIGHT * WIDTH][HEIGHT * WIDTH], int visited[HEIGHT * WIDTH], int path[HEIGHT * WIDTH], int curr_i, int start, int target) {
    visited[start] = 1;
    printf("Node: %d\n", start);
    for (int i =0; i < HEIGHT * WIDTH; i++) {
        if (graph[start][i] == 1 && visited[i] == -1) {
            if (i != target) {
                path[curr_i] = i;
                if (dfs(graph, visited, path, curr_i + 1, i, target) == 0) {
                    continue;
                } else {
                    return 1;
                }
            }
            else {
                path[curr_i] = target;
                printf("We found target! %d\n", target);
                i = HEIGHT*WIDTH + 1;
                return 1;
            }
        }
    }
    //printf("We added nothing!\n");
    curr_i = curr_i - 1;
    return 0;
}


int* getShortestDistancePath(int start, int target)
{
    // returns a path from start to finish.
    // its all unweighted imo since cells are the same size.
    // so really its balancing finding the shortest path && tweaking on num turns we do.
    // rn this just finds any one path which is shortest (could be multilpe shortest paths)
    // TODO(prathamdesai): I also need to have some way of storing a path, and then returning it;
    int visited[MAZE_HEIGHT * MAZE_WIDTH] = {0};
    int queue[MAZE_HEIGHT * MAZE_WIDTH] = {0};
    
    for (int i = 0; i < MAZE_HEIGHT * MAZE_WIDTH; i++)
    {
        visited[i] = -1;
    }
    queue[0] = start;
    int i = 0;
    int k = 1;
    visited[start] = 1;
    int curr;
    int path[MAZE_HEIGHT * MAZE_WIDTH] = {0};
    printf("begin!\n");
    int success = dfs(graph, visited, path, 1, start, target);
    return path;
    // while (i < MAZE_HEIGHT * MAZE_WIDTH)
    // {
    //     curr = queue[i];
    //     visited[curr] = 1;
    //     printf("node %d\n", curr);
    //     for (int j = 0; j < MAZE_HEIGHT * MAZE_WIDTH; j++)
    //     {
    //         if ((graph[curr][j] == 1) && visited[j] == -1)
    //         {
    //             if (j == target)
    //             {
    //                 printf("node: %d\n", j);
    //                 printf("found a path to target!");
    //                 return;
    //             }
    //             else
    //             {
    //                 printf("adding node %d\n", j);
    //                 queue[k] = j;
    //                 k = k + 1;
    //             }
    //         }
    //     }
    //     i = i + 1;
    // }
    // printf("no path found?!");
    // return visited;
}

char* getPathInstructions(int* path) {
    // Then here we make the instructions for the mouse.
    char instructions[sizeof(path)/sizeof(path[0]) - 1] = {0};
    char orientation = ORIGHT;
    for (int i = 0; i < sizeof(path)/sizeof(path[0]); i++) {
        // vertical cell movement
        if (path[i + 1] - path[i] == 1) {
            if (orientation == ORIGHT){
                instructions[i] = 'S';
                orientation = ORIGHT;
            } else if (orientation == OUP) {
                instructions[i] = 'R';
                orientation = ORIGHT;
            } else if (orientation == ODOWN) {
                instructions[i] = 'L';
                orientation = ORIGHT;
            } else if (orientation == OLEFT) {
                instructions[i] = 'B';
                orientation = OLEFT;
            }
        } else if (path[i + 1] - path[i] == -1) {
            if (orientation == OLEFT) {
                instructions[i] = 'S';
                orientation = OLEFT;
            } else if (orientation == OUP) {
                instructions[i] = 'L';
                orientation = OLEFT;
            } else if (orientation == ODOWN) {
                instructions[i] = 'R';
                orientation = OLEFT;
            } else if (orientation == ORIGHT) {
                instructions[i] = 'B';
                orientation = ORIGHT;
            }
        }
        // horizontal cell movement
        if (path[i + 1] - path[i] == WIDTH) {
            if (orientation == ORIGHT){
                instructions[i] = 'R';
                orientation = ODOWN;
            } else if (orientation == OUP) {
                instructions[i] = 'B';
                orientation = OUP;
            } else if (orientation == ODOWN) {
                instructions[i] = 'S';
                orientation = ODOWN;
            } else if (orientation == OLEFT) {
                instructions[i] = 'L';
                orientation = OLEFT;
            }
        } else if (path[i + 1] - path[i] == -WIDTH) {
            if (orientation == OLEFT) {
                instructions[i] = 'R';
                orientation = OUP;
            } else if (orientation == OUP) {
                instructions[i] = 'S';
                orientation = OUP;
            } else if (orientation == ODOWN) {
                instructions[i] = 'B';
                orientation = ODOWN;
            } else if (orientation == ORIGHT) {
                instructions[i] = 'L';
                orientation = OUP;
            }
        }
        // diagonal movement?????
        ;
        printf("%c\n", instructions[i]);
    }
    return instructions;
}