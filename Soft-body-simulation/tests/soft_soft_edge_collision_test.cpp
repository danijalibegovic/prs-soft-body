
#include "physics/SoftBodyAdapter.h"

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
        SoftVec3{0.0f, 1.0f, 0.0f},
        SoftVec3{1.0f, 1.0f, 1.0f},
        10.0f
    );

    SoftBody* sphere = softBodyCreateSphere(
        world,
        SoftVec3{0.85f, 4.0f, 0.0f},
        0.5f,
        12,
        1.0f
    );

    if (!cube || !sphere) {
        std::printf("Failed to create bodies\n");
        softWorldDestroy(world);
        return 1;
    }

    const std::size_t count = softBodyGetNodeCount(sphere);
    std::vector<SoftVec3> positions(count);

    constexpr float dt = 1.0f / 120.0f;

    std::printf("Frame | Sphere min Y | Sphere max Y\n");
    std::printf("-----------------------------------\n");

    for (int frame = 0; frame < 360; ++frame) {
        if (!softWorldStep(world, dt)) {
            std::printf("Simulation step failed\n");
            softWorldDestroy(world);
            return 1;
        }

        if (frame % 20 != 0)
            continue;

        softBodyCopyNodePositions(
            sphere,
            positions.data(),
            positions.size()
        );

        
float minX = std::numeric_limits<float>::max();
float maxX = std::numeric_limits<float>::lowest();

for (const auto& p : positions) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
}

const float centerX = (minX + maxX) * 0.5f;

std::printf("Frame %d | Center X = %.4f\n",
            frame, centerX);



        float minY = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::lowest();

        for (const auto& p : positions) {
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }

        std::printf(
            "%5d | %12.4f | %12.4f\n",
            frame, minY, maxY
        );
    }





softBodyCopyNodePositions(
    sphere, positions.data(), positions.size()
);

float minX = std::numeric_limits<float>::max();
float maxX = std::numeric_limits<float>::lowest();

for (const auto& p : positions) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
}

const float finalCenterX = (minX + maxX) * 0.5f;

if (finalCenterX < 1.5f) {
    std::printf(
        "FAIL: Sphere did not move past cube edge (X=%.4f)\n",
        finalCenterX
    );
    softWorldDestroy(world);
    return 1;
}

std::printf(
    "PASS: Sphere moved past cube edge (X=%.4f)\n",
    finalCenterX
);


    softWorldDestroy(world);
    return 0;
}
