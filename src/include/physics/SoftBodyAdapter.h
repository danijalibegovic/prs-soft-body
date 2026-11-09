#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SoftWorld SoftWorld;
typedef struct SoftBody SoftBody;
typedef struct SoftRigidBody SoftRigidBody;

typedef struct SoftVec3 {
    float x;
    float y;
    float z;
} SoftVec3;

typedef struct SoftWorldStats {
    float total_ms;
    float prediction_ms;
    float constraints_ms;
    float collision_ms;
    float update_ms;
    size_t node_count;
    size_t face_count;
} SoftWorldStats;

SoftWorld* softWorldCreate(SoftVec3 gravity);
void softWorldDestroy(SoftWorld* world);
void softWorldSetGravity(SoftWorld* world, SoftVec3 gravity);
int softWorldStep(SoftWorld* world, float timestep_seconds);
int softWorldGetStats(const SoftWorld* world, SoftWorldStats* out_stats);

SoftBody* softBodyCreateCloth(SoftWorld* world,
                              SoftVec3 origin,
                              float width,
                              float height,
                              size_t segments_x,
                              size_t segments_y,
                              float total_mass);
SoftBody* softBodyCreateSphere(SoftWorld* world,
                               SoftVec3 center,
                               float radius,
                               size_t resolution,
                               float total_mass);
SoftBody* softBodyCreateCube(SoftWorld* world,
                             SoftVec3 center,
                             SoftVec3 half_extents,
                             float total_mass);
void softBodyDestroy(SoftBody* body);

SoftRigidBody* softRigidBodyCreateSphere(SoftWorld* world,
                                         SoftVec3 center,
                                         float radius,
                                         float mass);
void softRigidBodyDestroy(SoftRigidBody* body);
int softRigidBodyGetPosition(const SoftRigidBody* body, SoftVec3* out_position);

size_t softBodyGetNodeCount(const SoftBody* body);
size_t softBodyGetFaceCount(const SoftBody* body);
size_t softBodyCopyNodePositions(const SoftBody* body,
                                 SoftVec3* destination,
                                 size_t destination_count);
size_t softBodyCopyIndices(const SoftBody* body,
                           unsigned int* destination,
                           size_t destination_count);

#ifdef __cplusplus
}
#endif
