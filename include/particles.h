#ifndef PARTICLES_H
#define PARTICLES_H
#define MAX_SPHERES_PER_CELL 1000
#define GRID_RES 17
#define CELL_SIZE ((BOUNDARY * 2.0f) / GRID_RES)
#define M_PI 3.14159265358979323846
#define GRAVITY         -0.8f
#define SPHERE_RADIUS   0.12f
#define RESTITUTION     0.65f
#define TIME_STEP       0.1f
#define WALL_TRANSPARENCY 0.3f

// Total number of spheres = GRID_SIZE^3
#define GRID_SIZE       20
#define SPACING         0.001f
#define BOUNDARY        3.9f
#define MAX_SPHERES (GRID_SIZE * GRID_SIZE * GRID_SIZE)
#define DRAG_RADIUS 0.5f
#define MAX_DRAGGED 100

// Spatial Partitioning Definitions

typedef struct {
    int count;
    int indices[MAX_SPHERES_PER_CELL];
} GridCell;

GridCell grid[GRID_RES][GRID_RES][GRID_RES];

typedef struct {
    float x, y, z;
    float vx, vy, vz;
} Sphere;

#endif