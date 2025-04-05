#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include "octree.h"

// Global definitions
#define SPACING 1.0f
#define GRAVITY -9.81f
#define TIME_STEP 0.016f
#define BOUNDARY 10.0f
#define RESTITUTION 0.8f
#define SPHERE_RADIUS 0.5f

#ifndef MAX_SPHERES_COUNTS
#define MAX_SPHERES_COUNTS 10000000
#endif

// Global variables
int particleCount = 1000;
Sphere spheres[MAX_SPHERES_COUNTS];
int numSpheres = 0;

// Function to initialize spheres in a grid
void initializeSpheres() {
    if (particleCount > MAX_SPHERES_COUNTS) {
        printf("Error: particleCount exceeds MAX_SPHERES_COUNTS!\n");
        exit(EXIT_FAILURE);
    }

    int count = particleCount;
    int gridDim = (int)ceil(pow((double)particleCount, 1.0 / 3.0));
    float offset = (gridDim - 1) * SPACING / 2.0f;
    numSpheres = 0;

    for (int x = 0; x < gridDim && numSpheres < count; x++) {
        for (int y = 0; y < gridDim && numSpheres < count; y++) {
            for (int z = 0; z < gridDim && numSpheres < count; z++) {
                if (numSpheres >= MAX_SPHERES_COUNTS) {
                    printf("Error: Reached maximum number of spheres.\n");
                    return;
                }
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

int main() {
    // Initialize particles
    initializeSpheres();

    // Open JSON file for writing
    FILE *fp = fopen("particles.json", "w");
    if (!fp) {
        perror("Error opening file for writing");
        exit(EXIT_FAILURE);
    }

    // Write JSON opening bracket
    fprintf(fp, "[\n");

    // Use a buffer for efficient writing
    const int buffer_size = 1024 * 1024; // 1MB buffer
    char buffer[buffer_size];
    int buffer_offset = 0;

    // Write particle data in chunks to the buffer, then flush to the file periodically
    for (int i = 0; i < numSpheres; i++) {
        int bytes_written = snprintf(buffer + buffer_offset, buffer_size - buffer_offset,
                                    "  {\"x\": %.3f, \"y\": %.3f, \"z\": %.3f, \"vx\": %.3f, \"vy\": %.3f, \"vz\": %.3f}%s\n",
                                    spheres[i].x, spheres[i].y, spheres[i].z,
                                    spheres[i].vx, spheres[i].vy, spheres[i].vz,
                                    (i < numSpheres - 1) ? "," : "");
        buffer_offset += bytes_written;

        // Flush buffer to file if it's full or it's the last particle
        if (buffer_offset >= buffer_size - 1 || i == numSpheres - 1) {
            fwrite(buffer, 1, buffer_offset, fp);
            buffer_offset = 0;  // Reset buffer
        }
    }

    // Write JSON closing bracket
    fprintf(fp, "]\n");

    fclose(fp);

    printf("Particle data written to particles.json\n");
    return 0;
}
