#pragma once

#include "physics/SoftBodyAdapter.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <string>

class XpbdSoftBodyApi
{
public:
    XpbdSoftBodyApi() = default;
    ~XpbdSoftBodyApi();

    XpbdSoftBodyApi(const XpbdSoftBodyApi&) = delete;
    XpbdSoftBodyApi& operator=(const XpbdSoftBodyApi&) = delete;

    bool Load();
    bool IsLoaded() const { return m_module != nullptr; }

    SoftWorld* (*softWorldCreate)(SoftVec3) = nullptr;
    void (*softWorldDestroy)(SoftWorld*) = nullptr;
    void (*softWorldSetGravity)(SoftWorld*, SoftVec3) = nullptr;
    int (*softWorldStep)(SoftWorld*, float) = nullptr;
    int (*softWorldGetStats)(const SoftWorld*, SoftWorldStats*) = nullptr;
    SoftBody* (*softBodyCreateCloth)(SoftWorld*, SoftVec3, float, float, size_t, size_t, float) = nullptr;
    SoftBody* (*softBodyCreateSphere)(SoftWorld*, SoftVec3, float, size_t, float) = nullptr;
    SoftBody* (*softBodyCreateCube)(SoftWorld*, SoftVec3, SoftVec3, float) = nullptr;
    void (*softBodyDestroy)(SoftBody*) = nullptr;
    SoftRigidBody* (*softRigidBodyCreateSphere)(SoftWorld*, SoftVec3, float, float) = nullptr;
    void (*softRigidBodyDestroy)(SoftRigidBody*) = nullptr;
    int (*softRigidBodyGetPosition)(const SoftRigidBody*, SoftVec3*) = nullptr;
    size_t (*softBodyGetNodeCount)(const SoftBody*) = nullptr;
    size_t (*softBodyGetFaceCount)(const SoftBody*) = nullptr;
    size_t (*softBodyCopyNodePositions)(const SoftBody*, SoftVec3*, size_t) = nullptr;
    size_t (*softBodyCopyIndices)(const SoftBody*, unsigned int*, size_t) = nullptr;

private:
#ifdef _WIN32
    HMODULE m_module = nullptr;
#endif
};
