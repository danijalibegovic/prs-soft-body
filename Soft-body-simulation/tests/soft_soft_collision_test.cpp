
#include "physics/SoftBodyAdapter.h"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

int main()
{
    SoftWorld* world = softWorldCreate(
        SoftVec3{0.0f, -9.81f, 0.0f}
    );

    if (!world)
        return 1;

    SoftBody* cube = softBodyCreateCube(
        world,
        SoftVec3{0.0f, 4.0f, 0.0f},
        SoftVec3{1.25f, 1.25f, 1.25f},
        4.0f
    );

    SoftBody* sphere = softBodyCreateSphere(
        world,
        SoftVec3{1.35f, 7.5f, 0.0f},
        1.75f,
        7,
        0.6f,
        1e-7f,
        2e-5f,
        1e-8f,
        12,
        0.12f,
        0.35f,
        0.01f
    );

    if (!cube || !sphere) {
        std::printf("Failed to create bodies\n");
        softWorldDestroy(world);
        return 1;
    }

    constexpr float dt = 1.0f / 120.0f;

    const std::size_t count = softBodyGetNodeCount(sphere);
    std::vector<SoftVec3> positions(count);
    for (int frame = 0; frame < 600; ++frame) {
        if (!softWorldStep(world, dt) ||
            softBodyCopyNodePositions(sphere, positions.data(), positions.size()) !=
                positions.size()) {
            std::printf("Simulation step or position read failed\n");
            softWorldDestroy(world);
            return 1;
        }
    }

    float minY = std::numeric_limits<float>::max();
    float meanY = 0.0f;
    float meanX = 0.0f;
    for (const auto& p : positions) {
        minY = std::min(minY, p.y);
        meanY += p.y;
        meanX += p.x;
    }
    meanY /= static_cast<float>(positions.size());
    meanX /= static_cast<float>(positions.size());
    std::printf("Final sphere min Y=%.3f mean Y=%.3f mean X=%.3f\n",
                minY, meanY, meanX);
    if (minY > 0.1f || meanY > 3.0f || meanX < 1.5f) {
        std::printf("FAIL: Ball remained on the cube instead of reaching the ground\n");
        softWorldDestroy(world);
        return 1;
    }

    std::printf("PASS: Ball deflected from cube and settled on ground\n");
    softWorldDestroy(world);
    return 0;
}
