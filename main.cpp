#include <GL/glut.h>
#include <cmath>
#include <vector>
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <iostream>

#define GRAVITY -0.018f
#define SPHERE_RADIUS 0.12f
#define RESTITUTION 0.65f
#define TIME_STEP 0.1f
#define WALL_TRANSPARENCY 0.3f

const int GRID_SIZE = 15;
const float SPACING = 0.001f;
const float BOUNDARY = 2.5f;

struct Sphere {
    float x, y, z;
    float vx, vy, vz;
};

std::vector<Sphere> spheres;

float camDist = 15.0f;
float camAngleX = 30.0f, camAngleY = 30.0f;
bool isDragging = false;

// Light position
GLfloat lightPos[] = {5.0f, 10.0f, 5.0f, 1.0f};

void initializeSpheres() {
    float offset = (GRID_SIZE - 1) * SPACING / 2.0f;

    for (int x = 0; x < GRID_SIZE; x++) {
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int z = 0; z < GRID_SIZE; z++) {
                Sphere s;
                s.x = x * SPACING - offset;
                s.y = y * SPACING - offset;
                s.z = z * SPACING - offset;
                s.vx = s.vy = s.vz = 0.0f;
                spheres.push_back(s);
            }
        }
    }
}


/*
// Handles sphere-sphere collisions using impulse physics
void resolveCollisions() {
    for (size_t i = 0; i < spheres.size(); i++) {
        for (size_t j = i + 1; j < spheres.size(); j++) {
            Sphere &s1 = spheres[i];
            Sphere &s2 = spheres[j];

            float dx = s2.x - s1.x;
            float dy = s2.y - s1.y;
            float dz = s2.z - s1.z;
            float dist = sqrt(dx * dx + dy * dy + dz * dz);
            float minDist = 2 * SPHERE_RADIUS;

            if (dist < minDist) {
                // Normalize the collision normal
                float nx = dx / dist;
                float ny = dy / dist;
                float nz = dz / dist;

                // Resolve overlap by moving spheres apart
                float overlap = minDist - dist;
                s1.x -= nx * overlap * 0.5f;
                s1.y -= ny * overlap * 0.5f;
                s1.z -= nz * overlap * 0.5f;
                s2.x += nx * overlap * 0.5f;
                s2.y += ny * overlap * 0.5f;
                s2.z += nz * overlap * 0.5f;

                // Compute relative velocity along the normal
                float vx_rel = s2.vx - s1.vx;
                float vy_rel = s2.vy - s1.vy;
                float vz_rel = s2.vz - s1.vz;
                float velAlongNormal = vx_rel * nx + vy_rel * ny + vz_rel * nz;

                if (velAlongNormal > 0) continue; // Spheres are separating

                // Impulse magnitude (elastic collision)
                float impulse = -(1 + RESTITUTION) * velAlongNormal / 2.0f;

                if (velAlongNormal > 2) glColor4b(1.0f, 0.0f, 0.0f, spheres);

                // Apply impulse to velocities
                s1.vx -= impulse * nx;
                s1.vy -= impulse * ny;
                s1.vz -= impulse * nz;
                s2.vx += impulse * nx;
                s2.vy += impulse * ny;
                s2.vz += impulse * nz;
            }
        }
    }
}*/

float MAX_VELOCITY = -0.001f;

void resolveCollisions() {
    for (size_t i = 0; i < spheres.size(); i++) {
        Sphere &s1 = spheres[i];

        for (size_t j = 0; j < spheres.size(); j++) {
            if (i != j) {
                Sphere &s2 = spheres[j];

                float dx = s2.x - s1.x;
                float dy = s2.y - s1.y;
                float dz = s2.z - s1.z;
                float dist = sqrt(dx * dx + dy * dy + dz * dz);
                float minDist = 2 * SPHERE_RADIUS;

                if (dist < minDist) {
                    // Normalize the collision normal
                    float nx = dx / dist;
                    float ny = dy / dist;
                    float nz = dz / dist;

                    // Resolve overlap by moving spheres apart
                    float overlap = minDist - dist;
                    s1.x -= nx * overlap * 0.5f;
                    s1.y -= ny * overlap * 0.5f;
                    s1.z -= nz * overlap * 0.5f;
                    s2.x += nx * overlap * 0.5f;
                    s2.y += ny * overlap * 0.5f;
                    s2.z += nz * overlap * 0.5f;

                    // Compute relative velocity along the normal
                    float vx_rel = s2.vx - s1.vx;
                    float vy_rel = s2.vy - s1.vy;
                    float vz_rel = s2.vz - s1.vz;
                    float velAlongNormal = vx_rel * nx + vy_rel * ny + vz_rel * nz;

                    if (velAlongNormal > 0) continue; // Spheres are separating

                    // Update MAX_VELOCITY dynamically
                    float velocityMagnitude1 = sqrt(s1.vx * s1.vx + s1.vy * s1.vy + s1.vz * s1.vz);
                    float velocityMagnitude2 = sqrt(s2.vx * s2.vx + s2.vy * s2.vy + s2.vz * s2.vz);
                    float maxCurrentVelocity = std::max(velocityMagnitude1, velocityMagnitude2);
                    MAX_VELOCITY = std::max(MAX_VELOCITY, maxCurrentVelocity);

                    // Impulse magnitude (elastic collision)
                    float impulse = -(1 + RESTITUTION) * velAlongNormal / 2.0f;

                    // Normalize the velocity magnitude (for color interpolation)
                    float velocityRatio = maxCurrentVelocity / MAX_VELOCITY;

                    // Interpolate between blue and red based on the normalized velocity
                    float red = velocityRatio;
                    float blue = 0.0f - velocityRatio;
                    float green = 0.0f;

                    // Apply color based on velocity

                    // Apply impulse to velocities
                    s1.vx -= impulse * nx;
                    s1.vy -= impulse * ny;
                    s1.vz -= impulse * nz;
                    s2.vx += impulse * nx;
                    s2.vy += impulse * ny;
                    s2.vz += impulse * nz;
                }
            }
        }
    }
}





void handleWallCollisions(Sphere &s) {
    if (s.x - SPHERE_RADIUS < -BOUNDARY) { s.x = -BOUNDARY + SPHERE_RADIUS; s.vx = -s.vx * RESTITUTION; }
    if (s.x + SPHERE_RADIUS > BOUNDARY)  { s.x = BOUNDARY - SPHERE_RADIUS; s.vx = -s.vx * RESTITUTION; }
    if (s.z - SPHERE_RADIUS < -BOUNDARY) { s.z = -BOUNDARY + SPHERE_RADIUS; s.vz = -s.vz * RESTITUTION; }
    if (s.z + SPHERE_RADIUS > BOUNDARY)  { s.z = BOUNDARY - SPHERE_RADIUS; s.vz = -s.vz * RESTITUTION; }
    if (s.y - SPHERE_RADIUS < -BOUNDARY) { s.y = -BOUNDARY + SPHERE_RADIUS; s.vy = -s.vy * RESTITUTION; }
}

void applyPhysics() {
    for (auto &s : spheres) {
        s.vy += GRAVITY;
        s.x += s.vx;
        s.y += s.vy;
        s.z += s.vz;
        handleWallCollisions(s);
    }
    resolveCollisions();
}

void drawTransparentWalls() {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.5f, 0.5f, 0.8f, WALL_TRANSPARENCY);
    
    glBegin(GL_QUADS);
    glVertex3f(-BOUNDARY, -BOUNDARY, -BOUNDARY);
    glVertex3f(-BOUNDARY, BOUNDARY, -BOUNDARY);
    glVertex3f(-BOUNDARY, BOUNDARY, BOUNDARY);
    glVertex3f(-BOUNDARY, -BOUNDARY, BOUNDARY);
    
    glVertex3f(BOUNDARY, -BOUNDARY, -BOUNDARY);
    glVertex3f(BOUNDARY, BOUNDARY, -BOUNDARY);
    glVertex3f(BOUNDARY, BOUNDARY, BOUNDARY);
    glVertex3f(BOUNDARY, -BOUNDARY, BOUNDARY);
    glEnd();

    glDisable(GL_BLEND);
}

/*void applyPhysics() {
    for (auto &s : spheres) {
        s.vy += GRAVITY;
        s.x += s.vx;
        s.y += s.vy;
        s.z += s.vz;

        // Ground collision
        if (s.y - SPHERE_RADIUS <= -BOUNDARY) {
            s.y = -BOUNDARY + SPHERE_RADIUS;
            s.vy = -s.vy * RESTITUTION;
        }
    }

    resolveCollisions(); // Sphere-sphere collision response
}*/


/*
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    float camX = camDist * sin(camAngleX * M_PI / 180) * cos(camAngleY * M_PI / 180);
    float camY = camDist * sin(camAngleY * M_PI / 180);
    float camZ = camDist * cos(camAngleX * M_PI / 180) * cos(camAngleY * M_PI / 180);

    gluLookAt(camX, camY, camZ, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    // Render floor
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_QUADS);
    glVertex3f(-BOUNDARY, -BOUNDARY, -BOUNDARY);
    glVertex3f(BOUNDARY, -BOUNDARY, -BOUNDARY);
    glVertex3f(BOUNDARY, -BOUNDARY, BOUNDARY);
    glVertex3f(-BOUNDARY, -BOUNDARY, BOUNDARY);
    glEnd();

    // Render spheres with lighting
    glEnable(GL_LIGHTING);
    for (const auto &s : spheres) {
        glPushMatrix();
        glTranslatef(s.x, s.y, s.z);
        glutSolidSphere(SPHERE_RADIUS, 20, 20);
        glPopMatrix();
    }

    glutSwapBuffers();
}*/



void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    bool isFloating = true;

    float camX = camDist * sin(camAngleX * M_PI / 180) * cos(camAngleY * M_PI / 180);
    float camY = camDist * sin(camAngleY * M_PI / 180);
    float camZ = camDist * cos(camAngleX * M_PI / 180) * cos(camAngleY * M_PI / 180);

    gluLookAt(camX, camY, camZ, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    // Render floor (without lighting)
    glDisable(GL_LIGHTING);
    glColor3f(0.3f, 0.3f, 0.3f);
    glBegin(GL_QUADS);
    glVertex3f(-BOUNDARY, -BOUNDARY, -BOUNDARY);
    glVertex3f(BOUNDARY, -BOUNDARY, -BOUNDARY);
    glVertex3f(BOUNDARY, -BOUNDARY, BOUNDARY);
    glVertex3f(-BOUNDARY, -BOUNDARY, BOUNDARY);
    glEnd();

    // Render spheres with color based on velocity
    for (const auto &s : spheres) {
        glPushMatrix();
        glTranslatef(s.x, s.y, s.z);

        // Disable lighting and apply custom color
        glDisable(GL_LIGHTING);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Set a default color for spheres
        glEnable(GL_LIGHTING);
        glutSolidSphere(SPHERE_RADIUS, 20, 20);
        glPopMatrix();
    }

    glutSwapBuffers();
}


void drawSpheres() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);  // Enable smooth shading
    glEnable(GL_DEPTH_TEST);
    
    GLUquadric *quad = gluNewQuadric();
    gluQuadricNormals(quad, GLU_SMOOTH);

    for (const auto &s : spheres) {
        glPushMatrix();
        glTranslatef(s.x, s.y, s.z);

        float velocityMagnitude = sqrt(s.vx * s.vx + s.vy * s.vy + s.vz * s.vz);
        float velocityRatio = velocityMagnitude / MAX_VELOCITY;

        // Interpolating between blue (slow) and red (fast)
        float red = velocityRatio;
        float blue = 1.0f - velocityRatio;
        float green = 0.3f * (1.0f - velocityRatio);  // Adding some green for more gradation

        glColor3f(red, green, blue);
        
        gluSphere(quad, SPHERE_RADIUS, 32, 32); // Increase tessellation for smoothness

        glPopMatrix();
    }

    gluDeleteQuadric(quad);
    glDisable(GL_LIGHTING);
}

void idle() {
    applyPhysics();
    glutPostRedisplay();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (float)width / (float)height, 1.0, 50.0);
    glMatrixMode(GL_MODELVIEW);
}


void resetSimulation() {
    spheres.clear();
    initializeSpheres();
/*
    camDist = 15.0f;
    camAngleX = 30.0f;
    camAngleY = 30.0f; */
}

void keyboard(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP:    camAngleY += 2.0f; break;
        case GLUT_KEY_DOWN:  camAngleY -= 2.0f; break;
        case GLUT_KEY_LEFT:  camAngleX -= 2.0f; break;
        case GLUT_KEY_RIGHT: camAngleX += 2.0f; break;
        case 13:
            resetSimulation();
    }
    glutPostRedisplay();
}

void mouseMotion(int x, int y) {
    float normalizedX = (float)x / glutGet(GLUT_WINDOW_WIDTH);
    float normalizedY = 1.0f - (float)y / glutGet(GLUT_WINDOW_HEIGHT);  // Invert Y to match OpenGL coordinates

    // Define color ranges
    float topR = 0.6 + 0.2 * normalizedX;  // Slightly vary red component
    float topG = 0.9;
    float topB = 1.0 - 0.3 * normalizedY;  // Darker blue at bottom

    float bottomR = 0.2;
    float bottomG = 0.6 + 0.3 * normalizedX;  // Add more green with movement
    float bottomB = 1.0;

    //glClearColor(bottomR, smoother.ttomG, bottomB, 1.0);
    glutPostRedisplay();
}


void keyPress(unsigned char key, int x, int y) {
    switch (key) {
        case 'w': camDist -= 1.0f; break;
        case 's': camDist += 1.0f; break;
        case 13:  resetSimulation(); break;
    }
    glutPostRedisplay();
}

void init() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glShadeModel(GL_SMOOTH);
    glClearColor(0.6, 0.8, 1.0, 1.0);  // Light blue
    initializeSpheres();
}



int main(int argc, char** argv) {
    while (true) {
        if (GLUT_KEY_PAGE_DOWN) {
            glutInit(&argc, argv);
            glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
            glutInitWindowSize(800, 600);
            glutCreateWindow("3D Fluid Particle Physics Simulation");
            init();
            glutDisplayFunc(display);
            glutReshapeFunc(reshape);
            glutIdleFunc(idle);
            glutSpecialFunc(keyboard);
            glutKeyboardFunc(keyPress);
            glutMainLoop();
        }
    }
    return 0;
}
