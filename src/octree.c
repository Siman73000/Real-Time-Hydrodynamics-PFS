#include <stdlib.h>
#include <math.h>
#include "octree.h"
#include "pfs.c"
#include "fast_sqrt.h"

// Helper: Check if a sphere is inside this node's cube.
static int sphere_in_node(OctreeNode *node, Sphere *s) {
    return (s->x >= node->center[0] - node->halfDimension &&
            s->x <= node->center[0] + node->halfDimension &&
            s->y >= node->center[1] - node->halfDimension &&
            s->y <= node->center[1] + node->halfDimension &&
            s->z >= node->center[2] - node->halfDimension &&
            s->z <= node->center[2] + node->halfDimension);
}

// Create a new octree node.
OctreeNode* octree_create(float center[3], float halfDimension, int capacity) {
    OctreeNode *node = malloc(sizeof(OctreeNode));
    node->center[0] = center[0];
    node->center[1] = center[1];
    node->center[2] = center[2];
    node->halfDimension = halfDimension;
    node->sphereCount = 0;
    node->capacity = capacity;
    node->spheres = malloc(sizeof(Sphere*) * capacity);
    for (int i = 0; i < 8; i++) {
        node->children[i] = NULL;
    }
    return node;
}

// Free the node and its children.
void octree_free(OctreeNode *node) {
    if (!node) return;
    for (int i = 0; i < 8; i++) {
        octree_free(node->children[i]);
    }
    free(node->spheres);
    free(node);
}

// Subdivide a node into eight children.
static void octree_subdivide(OctreeNode *node) {
    float childHalf = node->halfDimension / 2.0f;
    for (int i = 0; i < 8; i++) {
        float offsetX = (i & 1) ? childHalf : -childHalf;
        float offsetY = (i & 2) ? childHalf : -childHalf;
        float offsetZ = (i & 4) ? childHalf : -childHalf;
        float childCenter[3] = {
            node->center[0] + offsetX,
            node->center[1] + offsetY,
            node->center[2] + offsetZ
        };
        node->children[i] = octree_create(childCenter, childHalf, node->capacity);
    }
    // Reinsert spheres from this node into the children.
    for (int i = 0; i < node->sphereCount; i++) {
        Sphere *s = node->spheres[i];
        for (int j = 0; j < 8; j++) {
            if (sphere_in_node(node->children[j], s)) {
                octree_insert(node->children[j], s);
                break;
            }
        }
    }
    node->sphereCount = 0; // Clear storage now that we're not a leaf.
}

// Insert a sphere into the octree.
void octree_insert(OctreeNode *node, Sphere *s) {
    if (!sphere_in_node(node, s))
        return;  // Should not happen if the octree covers the domain.

    // If leaf node and has capacity, store here.
    if (node->children[0] == NULL) {
        if (node->sphereCount < node->capacity) {
            node->spheres[node->sphereCount++] = s;
            return;
        } else {
            // Subdivide and reinsert.
            octree_subdivide(node);
        }
    }
    // Insert into the appropriate child.
    for (int i = 0; i < 8; i++) {
        if (sphere_in_node(node->children[i], s)) {
            octree_insert(node->children[i], s);
            return;
        }
    }
}

// Query the octree for spheres within a given radius of sphere s.
void octree_query(OctreeNode *node, Sphere *s, float radius,
                  Sphere **results, int *resultCount, int maxResults) {
    // Compute a conservative bound to test if the node's cube might intersect the query sphere.
    float dx = fabs(s->x - node->center[0]);
    float dy = fabs(s->y - node->center[1]);
    float dz = fabs(s->z - node->center[2]);
    float maxDist = radius + node->halfDimension * sqrt(3.0f);
    if (dx > maxDist || dy > maxDist || dz > maxDist)
        return; // No intersection.

    if (node->children[0] == NULL) { // Leaf node.
        for (int i = 0; i < node->sphereCount; i++) {
            Sphere *s2 = node->spheres[i];
            float dx_ = s2->x - s->x;
            float dy_ = s2->y - s->y;
            float dz_ = s2->z - s->z;
            float dist = fast_sqrt(dx_*dx_ + dy_*dy_ + dz_*dz_);
            if (dist <= radius && *resultCount < maxResults) {
                results[(*resultCount)++] = s2;
            }
        }
    } else {
        // Recursively query children.
        for (int i = 0; i < 8; i++) {
            if (node->children[i])
                octree_query(node->children[i], s, radius, results, resultCount, maxResults);
        }
    }
}

// Build an octree for all spheres.
OctreeNode* build_octree(Sphere *spheres, int numSpheres, int capacity) {
    // For simplicity, use a cube centered at (0,0,0) with halfDimension = BOUNDARY.
    float center[3] = { 0.0f, 0.0f, 0.0f };
    OctreeNode *root = octree_create(center, BOUNDARY, capacity);
    for (int i = 0; i < numSpheres; i++) {
        octree_insert(root, &spheres[i]);
    }
    return root;
}

void resolveCollisionsOctree(Sphere *spheres, int numSpheres) {
    int capacity = 8; // Adjust as needed.
    OctreeNode *root = build_octree(spheres, numSpheres, capacity);
    float queryRadius = 2 * SPHERE_RADIUS;
    const int maxResults = 64;
    Sphere *results[maxResults];

    for (int i = 0; i < numSpheres; i++) {
        Sphere *sA = &spheres[i];
        int resultCount = 0;
        octree_query(root, sA, queryRadius, results, &resultCount, maxResults);
        for (int j = 0; j < resultCount; j++) {
            Sphere *sB = results[j];
            // Avoid double-processing.
            if (sB < sA) continue;
            float dx_ = sB->x - sA->x;
            float dy_ = sB->y - sA->y;
            float dz_ = sB->z - sA->z;
            float dist = fast_sqrt(dx_ * dx_ + dy_ * dy_ + dz_ * dz_);
            float minDist = 2 * SPHERE_RADIUS;
            if (dist < minDist && dist > 0.0f) {
                float nx_ = dx_ / dist;
                float ny_ = dy_ / dist;
                float nz_ = dz_ / dist;
                float overlap = minDist - dist;
                sA->x -= nx_ * overlap * 0.5f;
                sA->y -= ny_ * overlap * 0.5f;
                sA->z -= nz_ * overlap * 0.5f;
                sB->x += nx_ * overlap * 0.5f;
                sB->y += ny_ * overlap * 0.5f;
                sB->z += nz_ * overlap * 0.5f;

                float vx_rel = sB->vx - sA->vx;
                float vy_rel = sB->vy - sA->vy;
                float vz_rel = sB->vz - sA->vz;
                float velAlongNormal = vx_rel * nx_ + vy_rel * ny_ + vz_rel * nz_;
                if (velAlongNormal <= 0) {
                    float impulse = -(1 + RESTITUTION) * velAlongNormal / 2.0f;
                    sA->vx -= impulse * nx_;
                    sA->vy -= impulse * ny_;
                    sA->vz -= impulse * nz_;
                    sB->vx += impulse * nx_;
                    sB->vy += impulse * ny_;
                    sB->vz += impulse * nz_;

                    float velA = fast_sqrt(sA->vx * sA->vx +
                                           sA->vy * sA->vy +
                                           sA->vz * sA->vz);
                    float velB = fast_sqrt(sB->vx * sB->vx +
                                           sB->vy * sB->vy +
                                           sB->vz * sB->vz);
                    float maxVel = (velA > velB) ? velA : velB;
                    if (maxVel > MAX_VELOCITY) {
                        MAX_VELOCITY = maxVel;
                    }
                }
            }
        }
    }
    octree_free(root);
}
