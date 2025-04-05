#ifndef SPATIAL_HASH_H
#define SPATIAL_HASH_H

#include "particles.h"    // Contains definitions for Sphere, SPHERE_RADIUS, RESTITUTION, etc.
#include "uthash.h"       // Make sure uthash.h is in your include path

// A simple key for the spatial hash, based on grid coordinates.
typedef struct {
    int x, y, z;
} CellKey;

// The grid cell structure used in the hash table.
typedef struct {
    CellKey key;                        // The grid cell coordinates.
    int count;                          // Number of spheres in this cell.
    int indices[MAX_SPHERES_PER_CELL];  // Array of sphere indices.
    UT_hash_handle hh;                  // UTHash handle.
} GridCellStructure;

// Global pointer to the hash table (declared here so it can be shared in spatial_hash.c).
extern GridCellStructure *gridHash;

// Function prototypes.
void buildSpatialHashGrid(void);
void resolveCollisionsSpatial(void);

#endif // SPATIAL_HASH_H
