#ifndef PFS_H
#define PFS_H

#include <stdbool.h>
#include "particles.h"

#define MAX_SPHERES_PER_CELL 100000000000
#define GRID_RES 10000000
#define CELL_SIZE ((BOUNDARY * 2.0f) / GRID_RES)
#define M_PI 3.14159265358979323846
#define GRAVITY         -0.8f
#define SPHERE_RADIUS   0.12f
#define RESTITUTION     0.65f
#define TIME_STEP       0.1f
#define WALL_TRANSPARENCY 0.3f

// Total number of spheres = GRID_SIZE^3
#define GRID_SIZE       200000
#define SPACING         0.001f
#define BOUNDARY        3.9f
#define MAX_SPHERES (GRID_SIZE * GRID_SIZE * GRID_SIZE)
#define DRAG_RADIUS 0.5f
#define MAX_DRAGGED 100


void init();
void initializeSpheres();
void applyPhysics();
void display();
void idle();
void reshape(int width, int height);
void resetSimulation();
void keyboard(int key, int x, int y);
void keyPress(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void mouseMotion(int x, int y);
/*
void clearGrid();
void buildSpatialGrid();
void resolveCollisionsSpatial();
bool raySphereIntersect(float *rayOrigin, float *rayDir, float *sphereCenter, float sphereRadius, float *intersection);
bool intersectRayPlane(const float rayOrigin[3], const float rayDir[3], const float planePoint[3], const float planeNormal[3], float intersection[3]);
*/
bool isDragged(int index);
void handleWallCollisions(Sphere *s);

typedef struct {
    int count;
    int indices[MAX_SPHERES_PER_CELL];
} GridCell;

typedef struct {
    char* velocities;
    int val;
} Item;




#endif