#ifndef PFS_H
#define PFS_H

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
void clearGrid();
void buildSpatialGrid();
void resolveCollisionsSpatial();
bool raySphereIntersect(float *rayOrigin, float *rayDir, float *sphereCenter, float sphereRadius, float *intersection);
bool intersectRayPlane(const float rayOrigin[3], const float rayDir[3], const float planePoint[3], const float planeNormal[3], float intersection[3]);

#endif