#ifndef OCTREE_H
#define OCTREE_H

#include "pfs.h"    // Provides Sphere, BOUNDARY, SPHERE_RADIUS, RESTITUTION, etc.
#include "particles.h"
#include "fast_sqrt.h"    // Provides fast_sqrt()

// Octree node: represents a cube in space.
typedef struct OctreeNode {
    float center[3];         // Center of the cube.
    float halfDimension;     // Half the side length of the cube.
    Sphere **spheres;        // Array of pointers to spheres (only valid in leaf nodes).
    int sphereCount;         // Number of spheres stored in this node.
    int capacity;            // Maximum number of spheres before subdividing.
    struct OctreeNode *children[8]; // Eight children (NULL if this is a leaf).
} OctreeNode;

// Create an octree node with given center, half-dimension and capacity.
OctreeNode* octree_create(float center[3], float halfDimension, int capacity);

// Free an octree node (and all its children).
void octree_free(OctreeNode *node);

// Insert a sphere into the octree node.
void octree_insert(OctreeNode *node, Sphere *s);

// Query the octree: find all spheres within 'radius' of sphere s.
// 'results' is an array of Sphere pointers; *resultCount returns the number found (up to maxResults).
void octree_query(OctreeNode *node, Sphere *s, float radius,
                  Sphere **results, int *resultCount, int maxResults);

// Build an octree for all spheres.
// Uses the global BOUNDARY to define the simulation cube and 'capacity' for each leaf.
OctreeNode* build_octree(Sphere *spheres, int numSpheres, int capacity);

// Resolve collisions using the octree.
// This function builds the octree, then for each sphere queries neighbors within collision radius,
// and applies collision response.
void resolveCollisionsOctree(Sphere *spheres, int numSpheres);

#endif // OCTREE_H
