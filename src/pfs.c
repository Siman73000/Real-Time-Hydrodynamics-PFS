#include <GL/glut.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include "fast_sqrt.h"
#include "pfs.h"
#include "octree.h"
#include "particles.h"

bool showMenu = false;
int particleCount = 500;
int draggedIndices[MAX_DRAGGED];
int numDragged = 0;
float dragOffsets[MAX_DRAGGED][3];
float dragPlanePoint[3];
float dragPlaneNormal[3];

Sphere spheres[MAX_SPHERES];
int numSpheres = 0;

// Camera and light settings
float camDist = 15.0f;
float camAngleX = 30.0f, camAngleY = 30.0f;
bool isDragging = false;
bool toggle = false;
GLfloat lightPos[] = {5.0f, 10.0f, 5.0f, 1.0f};

// Global maximum velocity used for color interpolation
float MAX_VELOCITY = -0.001f;

// Display list for optimized sphere rendering
GLuint sphereList;

void initializeSpheres() {
    int count = particleCount;

    // Compute the grid dimension as the cube root (rounded up)
    int gridDim = (int)ceil(pow((double)particleCount, 1.0 / 3.0));
    // Center the grid around the origin
    float offset = (gridDim - 1) * SPACING / 2.0f;
    numSpheres = 0;
    
    for (int x = 0; x < gridDim && numSpheres < count; x++) {
        for (int y = 0; y < gridDim && numSpheres < count; y++) {
            for (int z = 0; z < gridDim && numSpheres < count; z++) {
                Sphere s;
                s.x = x * SPACING - offset;
                s.y = y * SPACING - offset;
                s.z = z * SPACING - offset;
                s.vx = 0.0f;
                s.vy = 0.0f;
                s.vz = 0.0f;
                spheres[numSpheres++] = s;
            }
        }
    }
}

/* Update sphere position/velocity using velocity Verlet integration.
   Gravity is the only acceleration, and the y-component update is done via inline assembly. */
void updateSphere(Sphere* s, float dt) {
    // Acceleration (only gravity on y)
    float ax = 0.0f, ay = GRAVITY, az = 0.0f;
    // Update position: pos = pos + v*dt + 0.5*a*dt^2
    s->x += s->vx * dt + 0.5f * ax * dt * dt;
    s->y += s->vy * dt + 0.5f * ay * dt * dt;
    s->z += s->vz * dt + 0.5f * az * dt * dt;
    
    // Update velocity: for y-component, using inline assembly.
    float old_vy = s->vy;
    float dt_val = dt;
    float gravity_val = GRAVITY;
    asm volatile (
        "movss %1, %%xmm0\n\t"   // xmm0 = gravity_val
        "mulss %2, %%xmm0\n\t"   // xmm0 = gravity_val * dt
        "addss %3, %%xmm0\n\t"   // xmm0 = old_vy + (gravity_val * dt)
        "movss %%xmm0, %0\n\t"   // store result back into s->vy
        : "=m" (s->vy)
        : "x" (gravity_val), "x" (dt_val), "x" (old_vy)
        : "xmm0"
    );
}

/* ---------------------------------------------------------------------------
   Removed grid-based spatial partitioning functions:
   clearGrid(), buildSpatialGrid(), and the associated spatial hash globals
   have been removed in favor of using the octree approach defined in octree.h.
   --------------------------------------------------------------------------- */

// Utility: Compute a ray (origin & direction) from the current mouse position.
void computeRay(int x, int y, float rayOrigin[3], float rayDir[3]) {
    GLint viewport[4];
    GLdouble modelview[16], projection[16];
    GLdouble ox, oy, oz;
    GLdouble ax, ay, az;
    
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    
    // Unproject at near (0.0) and far (1.0) depths to form the ray.
    gluUnProject(x, viewport[3]-y, 0.0, modelview, projection, viewport, &ox, &oy, &oz);
    gluUnProject(x, viewport[3]-y, 1.0, modelview, projection, viewport, &ax, &ay, &az);
    
    rayOrigin[0] = (float)ox;
    rayOrigin[1] = (float)oy;
    rayOrigin[2] = (float)oz;
    
    rayDir[0] = (float)(ax - ox);
    rayDir[1] = (float)(ay - oy);
    rayDir[2] = (float)(az - oz);
    
    // Normalize rayDir.
    float len = fast_sqrt(rayDir[0]*rayDir[0] + rayDir[1]*rayDir[1] + rayDir[2]*rayDir[2]);
    if (len > 0) {
        rayDir[0] /= len;
        rayDir[1] /= len;
        rayDir[2] /= len;
    }
}

// Utility: Intersect a ray with a plane defined by dragPlanePoint and dragPlaneNormal.
bool intersectRayPlane(const float rayOrigin[3], const float rayDir[3],
                        const float planePoint[3], const float planeNormal[3],
                        float intersection[3]) {
    float denom = rayDir[0]*planeNormal[0] + rayDir[1]*planeNormal[1] + rayDir[2]*planeNormal[2];
    if (fabs(denom) < 1e-6)
        return false;
    float t = ((planePoint[0]-rayOrigin[0])*planeNormal[0] +
               (planePoint[1]-rayOrigin[1])*planeNormal[1] +
               (planePoint[2]-rayOrigin[2])*planeNormal[2]) / denom;
    if (t < 0)
        return false;
    intersection[0] = rayOrigin[0] + rayDir[0]*t;
    intersection[1] = rayOrigin[1] + rayDir[1]*t;
    intersection[2] = rayOrigin[2] + rayDir[2]*t;
    return true;
}

Item* linearSearch(Item* item, size_t size, const char* velocities) {
    for (size_t i = 0; i < size; i++) {
        if (strcmp(item[i].velocities, velocities) == 0) {
            return &item[i];
        }
    }
    return NULL;
}

/* ---------------------------------------------------------------------------
   Collision Resolution using Octree
   Instead of building a spatial hash grid, we now call the octree-based routine.
   --------------------------------------------------------------------------- */
 
// In applyPhysics(), we now call resolveCollisionsOctree (declared in octree.h)
// which builds the octree, queries neighbors, and resolves collisions.
 
/* Update physics:
   1. Update each sphere’s position/velocity.
   2. Handle wall collisions.
   3. Resolve collisions using the octree.
*/
void applyPhysics() {
    for (int i = 0; i < numSpheres; i++) {
        if (!isDragged(i)) {
            updateSphere(&spheres[i], TIME_STEP);
            handleWallCollisions(&spheres[i]);
        }
    }
    resolveCollisionsOctree(spheres, numSpheres);
}

void handleWallCollisions(Sphere *s) {
    if (s->x - SPHERE_RADIUS < -BOUNDARY) { s->x = -BOUNDARY + SPHERE_RADIUS; s->vx = -s->vx * RESTITUTION; }
    if (s->x + SPHERE_RADIUS > BOUNDARY)  { s->x = BOUNDARY - SPHERE_RADIUS; s->vx = -s->vx * RESTITUTION; }
    if (s->z - SPHERE_RADIUS < -BOUNDARY) { s->z = -BOUNDARY + SPHERE_RADIUS; s->vz = -s->vz * RESTITUTION; }
    if (s->z + SPHERE_RADIUS > BOUNDARY)  { s->z = BOUNDARY - SPHERE_RADIUS; s->vz = -s->vz * RESTITUTION; }
    if (s->y - SPHERE_RADIUS < -BOUNDARY) { s->y = -BOUNDARY + SPHERE_RADIUS; s->vy = -s->vy * RESTITUTION; }
}

float getOriginDepth() {
    GLint viewport[4];
    GLdouble modelview[16], projection[16];
    GLdouble winX, winY, winZ;
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    // Get window coordinates for the origin (0,0,0)
    gluProject(0.0, 0.0, 0.0, modelview, projection, viewport, &winX, &winY, &winZ);
    return (float)winZ;
}

bool isDragged(int index) {
    for (int i = 0; i < numDragged; i++) {
        if (draggedIndices[i] == index)
            return true;
    }
    return false;
}

void drawTransparentWalls() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.5f, 0.5f, 0.8f, WALL_TRANSPARENCY);
    
    glBegin(GL_QUADS);
        glVertex3f(-BOUNDARY, -BOUNDARY, -BOUNDARY);
        glVertex3f(-BOUNDARY,  BOUNDARY, -BOUNDARY);
        glVertex3f(-BOUNDARY,  BOUNDARY,  BOUNDARY);
        glVertex3f(-BOUNDARY, -BOUNDARY,  BOUNDARY);
        
        glVertex3f( BOUNDARY, -BOUNDARY, -BOUNDARY);
        glVertex3f( BOUNDARY,  BOUNDARY, -BOUNDARY);
        glVertex3f( BOUNDARY,  BOUNDARY,  BOUNDARY);
        glVertex3f( BOUNDARY, -BOUNDARY,  BOUNDARY);
    glEnd();
    
    glDisable(GL_BLEND);
}

void drawText(const char *text, float x, float y) {
    glRasterPos2f(x, y);
    for (const char *c = text; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
}

void drawMenu() {
    glColor3f(0.2f, 0.2f, 0.2f);
    glBegin(GL_QUADS);
        glVertex2f(50, 50);
        glVertex2f(250, 50);
        glVertex2f(250, 200);
        glVertex2f(50, 200);
    glEnd();
    glColor3f(1.0f, 1.0f, 1.0f);
    drawText("1 - ", 70, 170);
    drawText("2 - ", 70, 140);
    drawText("3 - ", 70, 110);
    drawText("4 - ", 70, 80);
    
    // Text options using GLUT bitmap fonts
    glColor3f(1.0f, 1.0f, 1.0f);
    
    glRasterPos2f(70, 170);
    const char *opt1 = "1 - 500 Particles";
    for (const char *c = opt1; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
    
    glRasterPos2f(70, 140);
    const char *opt2 = "2 - 5000 Particles";
    for (const char *c = opt2; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
    
    glRasterPos2f(70, 110);
    const char *opt3 = "3 - 8000 Particles";
    for (const char *c = opt3; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
    
    glRasterPos2f(70, 80);
    const char *opt4 = "4 - Color Toggle";
    for (const char *c = opt4; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
}

void toggleColor() {
    for (int i = 0; i < numSpheres; i++) {
        glPushMatrix();
        glTranslatef(spheres[i].x, spheres[i].y, spheres[i].z);
        float velocityMagnitude = fast_sqrt(spheres[i].vx * spheres[i].vx +
                                             spheres[i].vy * spheres[i].vy +
                                             spheres[i].vz * spheres[i].vz);
        float velocityRatio = velocityMagnitude / MAX_VELOCITY;
        float red = velocityRatio;
        float blue = 1.0f - velocityRatio;
        float green = 0.3f * (1.0f - velocityRatio);
        glColor3f(red, green, blue);
        glCallList(sphereList);
        glPopMatrix();
    }
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    float camX = camDist * sin(camAngleX * M_PI / 180.0f) * cos(camAngleY * M_PI / 180.0f);
    float camY = camDist * sin(camAngleY * M_PI / 180.0f);
    float camZ = camDist * cos(camAngleX * M_PI / 180.0f) * cos(camAngleY * M_PI / 180.0f);
    
    gluLookAt(camX, camY, camZ,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    
    // Render floor (without lighting)
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_QUADS);
        glVertex3f(-BOUNDARY, -BOUNDARY, -BOUNDARY);
        glVertex3f( BOUNDARY, -BOUNDARY, -BOUNDARY);
        glVertex3f( BOUNDARY, -BOUNDARY,  BOUNDARY);
        glVertex3f(-BOUNDARY, -BOUNDARY,  BOUNDARY);
    glEnd();
    if (!toggle) {
        glClearColor(0.6, 0.8, 1.0, 1.0);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
        glColor3f(1.0f, 1.0f, 0.94f);
        glEnable(GL_LIGHTING);
        for (int i = 0; i < numSpheres; i++) {
            glPushMatrix();
            glTranslatef(spheres[i].x, spheres[i].y, spheres[i].z);
            glCallList(sphereList);
            glPopMatrix();
        }
    }
    else if (toggle) {
        toggleColor();
    }
    if (showMenu) {
        // Save current projection and modelview matrices.
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        int width = glutGet(GLUT_WINDOW_WIDTH);
        int height = glutGet(GLUT_WINDOW_HEIGHT);
        gluOrtho2D(0, width, 0, height);
        
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        
        // Disable depth testing and lighting so text renders on top.
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        
        // Draw the menu overlay.
        drawMenu();
        
        // Restore matrices.
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    
    glutSwapBuffers();
}

void idle() {
    applyPhysics();
    glutPostRedisplay();
}

void resetSimulation() {
    numSpheres = 0;
    initializeSpheres();
}

void keyboard(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP:    camAngleY += 2.0f; break;
        case GLUT_KEY_DOWN:  camAngleY -= 2.0f; break;
        case GLUT_KEY_LEFT:  camAngleX -= 2.0f; break;
        case GLUT_KEY_RIGHT: camAngleX += 2.0f; break;
        case 13: resetSimulation(); break;
    }
    glutPostRedisplay();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (float)width / (float)height, 1.0, 50.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyPress(unsigned char key, int x, int y) {
    if (showMenu) {
        switch (key) {
            case '1':
                particleCount = 500;
                resetSimulation();
                showMenu = false;
                break;
            case '2':
                particleCount = 5000;
                resetSimulation();
                showMenu = false;
                break;
            case '3':
                particleCount = 8000;
                resetSimulation();
                showMenu = false;
                break;
            case '4':
                toggle = !toggle;
                toggleColor();
                showMenu = false;
                break;
            case 'f': // Hide menu if already open
                showMenu = false;
                break;
            default:
                break;
        }
    }
    else {
        switch (key) {
            case 'w': camDist -= 1.0f; break;
            case 's': camDist += 1.0f; break;
            case 'f': showMenu = true; break;  // Toggle menu on
            case 13:  resetSimulation(); break;
            default:
                break;
        }
    }
    glutPostRedisplay();
}

void screenToWorld(int x, int y, float *worldX, float *worldY, float *worldZ) {
    GLint viewport[4];
    GLdouble modelview[16], projection[16];
    GLdouble wx, wy, wz;
    
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    
    // Use a fixed depth value (e.g., 0.5) to intersect a plane in the scene.
    gluUnProject(x, viewport[3] - y, 0.5, modelview, projection, viewport, &wx, &wy, &wz);
    *worldX = (float)wx;
    *worldY = (float)wy;
    *worldZ = (float)wz;
}

void getCameraPosition(float camPos[3]) {
    float camX = camDist * sin(camAngleX * M_PI / 180.0f) * cos(camAngleY * M_PI / 180.0f);
    float camY = camDist * sin(camAngleY * M_PI / 180.0f);
    float camZ = camDist * cos(camAngleX * M_PI / 180.0f) * cos(camAngleY * M_PI / 180.0f);
    camPos[0] = camX; camPos[1] = camY; camPos[2] = camZ;
}

void mouseMotion(int x, int y) {
    if (isDragging) {
        float rayOrigin[3], rayDir[3];
        computeRay(x, y, rayOrigin, rayDir);
        float newClumpCenter[3];
        if (intersectRayPlane(rayOrigin, rayDir, dragPlanePoint, dragPlaneNormal, newClumpCenter)) {
            // Update positions of dragged spheres using their stored offsets.
            for (int i = 0; i < numDragged; i++) {
                int idx = draggedIndices[i];
                spheres[idx].x = newClumpCenter[0] + dragOffsets[i][0];
                spheres[idx].y = newClumpCenter[1] + dragOffsets[i][1];
                spheres[idx].z = newClumpCenter[2] + dragOffsets[i][2];
                // Reset velocities.
                spheres[idx].vx = 0.0f;
                spheres[idx].vy = 0.0f;
                spheres[idx].vz = 0.0f;
            }
            // Update dragPlanePoint to the new clump center.
            dragPlanePoint[0] = newClumpCenter[0];
            dragPlanePoint[1] = newClumpCenter[1];
            dragPlanePoint[2] = newClumpCenter[2];
        }
    }
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            float clickX, clickY, clickZ;
            {
                GLint viewport[4];
                GLdouble modelview[16], projection[16];
                GLdouble wx, wy, wz;
                glGetIntegerv(GL_VIEWPORT, viewport);
                glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
                glGetDoublev(GL_PROJECTION_MATRIX, projection);
                // Use depth at the origin (where spheres are) rather than 0.5.
                float depth = getOriginDepth();
                gluUnProject(x, viewport[3]-y, depth, modelview, projection, viewport, &wx, &wy, &wz);
                clickX = (float)wx; 
                clickY = (float)wy; 
                clickZ = (float)wz;
            }
            
            numDragged = 0;
            // Select all spheres within DRAG_RADIUS of the click point.
            for (int i = 0; i < numSpheres; i++) {
                float dx = spheres[i].x - clickX;
                float dy = spheres[i].y - clickY;
                float dz = spheres[i].z - clickZ;
                float dist = fast_sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < DRAG_RADIUS && numDragged < MAX_DRAGGED) {
                    draggedIndices[numDragged] = i;
                    numDragged++;
                }
            }
            if (numDragged > 0) {
                isDragging = true;
                // Compute the clump center.
                float clumpCenter[3] = {0, 0, 0};
                for (int i = 0; i < numDragged; i++) {
                    int idx = draggedIndices[i];
                    clumpCenter[0] += spheres[idx].x;
                    clumpCenter[1] += spheres[idx].y;
                    clumpCenter[2] += spheres[idx].z;
                }
                clumpCenter[0] /= numDragged;
                clumpCenter[1] /= numDragged;
                clumpCenter[2] /= numDragged;
                
                // Store clump center as the drag plane point.
                dragPlanePoint[0] = clumpCenter[0];
                dragPlanePoint[1] = clumpCenter[1];
                dragPlanePoint[2] = clumpCenter[2];
                
                // Set the drag plane normal as the normalized camera view direction.
                float camPos[3];
                getCameraPosition(camPos);
                float viewVec[3] = { -camPos[0], -camPos[1], -camPos[2] };
                float len = fast_sqrt(viewVec[0]*viewVec[0] + viewVec[1]*viewVec[1] + viewVec[2]*viewVec[2]);
                if (len > 0) {
                    dragPlaneNormal[0] = viewVec[0] / len;
                    dragPlaneNormal[1] = viewVec[1] / len;
                    dragPlaneNormal[2] = viewVec[2] / len;
                }
                
                // For each dragged sphere, store its offset relative to the clump center.
                for (int i = 0; i < numDragged; i++) {
                    int idx = draggedIndices[i];
                    dragOffsets[i][0] = spheres[idx].x - clumpCenter[0];
                    dragOffsets[i][1] = spheres[idx].y - clumpCenter[1];
                    dragOffsets[i][2] = spheres[idx].z - clumpCenter[2];
                }
            }
        } else if (state == GLUT_UP) {
            isDragging = false;
            numDragged = 0;
        }
    }
    glutPostRedisplay();
}

void init() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);
    glClearColor(0.6, 0.8, 1.0, 1.0);
    initializeSpheres();
    sphereList = glGenLists(1);
    glNewList(sphereList, GL_COMPILE);
        glutSolidSphere(SPHERE_RADIUS, 20, 20);
    glEndList();
}
