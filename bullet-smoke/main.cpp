#include "physics/SoftBodyAdapter.h"

#include <cstdio>
#include <algorithm>
#include <limits>
#include <vector>

int main()
{
    constexpr float timestep = 1.0f / 60.0f;
    SoftWorld* world = softWorldCreate({0.0f, -9.81f, 0.0f});
    if (world == nullptr)
        return 1;

    SoftBody* cube = softBodyCreateCube(
        world, {0.0f, 6.0f, 0.0f}, {1.25f, 1.25f, 1.25f}, 1.0f);
    if (cube == nullptr)
    {
        softWorldDestroy(world);
        return 1;
    }

    std::vector<SoftVec3> positions(softBodyGetNodeCount(cube));
    std::vector<unsigned int> indices(softBodyGetFaceCount(cube) * 3);
    const size_t node_count = softBodyCopyNodePositions(
        cube, positions.data(), positions.size());
    const size_t index_count = softBodyCopyIndices(
        cube, indices.data(), indices.size());

    std::FILE* csv = std::fopen("bullet_soft_body.csv", "w");
    if (csv == nullptr)
    {
        softWorldDestroy(world);
        return 1;
    }

    std::fprintf(csv,
                 "step,total_ms,prediction_ms,constraints_ms,collision_ms,update_ms,"
                 "node_count,face_count\n");
    for (int step = 1; step <= 600; ++step)
    {
        if (!softWorldStep(world, timestep))
        {
            std::fclose(csv);
            softWorldDestroy(world);
            return 1;
        }

        SoftWorldStats stats{};
        softWorldGetStats(world, &stats);
        std::fprintf(csv, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%zu,%zu\n",
                     step, stats.total_ms, stats.prediction_ms,
                     stats.constraints_ms, stats.collision_ms, stats.update_ms,
                     stats.node_count, stats.face_count);
        if (step % 60 == 0)
        {
            softBodyCopyNodePositions(cube, positions.data(), positions.size());
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();
            for (const SoftVec3& position : positions)
            {
                min_y = std::min(min_y, position.y);
                max_y = std::max(max_y, position.y);
            }
            std::printf("step=%d total_ms=%.3f nodes=%zu faces=%zu\n",
                        step, stats.total_ms, stats.node_count, stats.face_count);
            std::printf("  cube_y_range=%.3f..%.3f\n", min_y, max_y);
        }
    }

    std::printf("copied_nodes=%zu copied_indices=%zu\n", node_count, index_count);
    std::fclose(csv);
    softWorldDestroy(world);
    return 0;
}
