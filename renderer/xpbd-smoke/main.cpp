#include "XpbdSoftBodyApi.h"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

int main()
{
    XpbdSoftBodyApi api;
    if (!api.Load())
        return 1;

    SoftWorld* world = api.softWorldCreate({0.0f, -9.81f, 0.0f});
    if (world == nullptr)
    {
        std::fprintf(stderr, "XPBD smoke: world creation failed\n");
        return 1;
    }

    SoftBody* cube = api.softBodyCreateCube(
        world, {0.0f, 4.0f, 0.0f}, {1.25f, 1.25f, 1.25f}, 4.0f);
    SoftBody* ball = api.softBodyCreateSphere(
        world, {0.0f, 7.5f, 0.0f}, 1.75f, 7, 0.6f,
        1e-7f, 2e-5f, 1e-8f, 8, 0.12f, 0.35f, 0.01f);
    if (cube == nullptr || ball == nullptr)
    {
        if (cube != nullptr)
            api.softBodyDestroy(cube);
        if (ball != nullptr)
            api.softBodyDestroy(ball);
        api.softWorldDestroy(world);
        std::fprintf(stderr, "XPBD smoke: body creation failed (cube=%p, ball=%p)\n",
                     static_cast<void*>(cube), static_cast<void*>(ball));
        return 1;
    }

    std::vector<SoftVec3> positions(api.softBodyGetNodeCount(cube));
    std::vector<SoftVec3> ball_positions(api.softBodyGetNodeCount(ball));
    std::FILE* csv = std::fopen("xpbd_soft_body.csv", "w");
    if (csv == nullptr)
    {
        api.softBodyDestroy(cube);
        api.softBodyDestroy(ball);
        api.softWorldDestroy(world);
        std::fprintf(stderr, "XPBD smoke: could not open telemetry output\n");
        return 1;
    }

    std::fprintf(csv, "step,total_ms,prediction_ms,constraints_ms,collision_ms,update_ms,node_count,face_count,cube_min_y,cube_max_y,ball_min_y,ball_max_y\n");
    constexpr float timestep = 1.0f / 60.0f;
    float prediction_ms = 0.0f;
    float constraints_ms = 0.0f;
    float collision_ms = 0.0f;
    float update_ms = 0.0f;
    float total_ms = 0.0f;
    for (int step = 1; step <= 600; ++step)
    {
        if (!api.softWorldStep(world, timestep))
            break;
        SoftWorldStats stats{};
        api.softWorldGetStats(world, &stats);
        total_ms += stats.total_ms;
        prediction_ms += stats.prediction_ms;
        constraints_ms += stats.constraints_ms;
        collision_ms += stats.collision_ms;
        update_ms += stats.update_ms;
        api.softBodyCopyNodePositions(cube, positions.data(), positions.size());
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        for (const SoftVec3& position : positions)
        {
            min_y = std::min(min_y, position.y);
            max_y = std::max(max_y, position.y);
        }
        api.softBodyCopyNodePositions(ball, ball_positions.data(), ball_positions.size());
        float ball_min_y = std::numeric_limits<float>::max();
        float ball_max_y = std::numeric_limits<float>::lowest();
        for (const SoftVec3& position : ball_positions)
        {
            ball_min_y = std::min(ball_min_y, position.y);
            ball_max_y = std::max(ball_max_y, position.y);
        }
        std::fprintf(csv, "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%zu,%zu,%.6f,%.6f,%.6f,%.6f\n",
                     step, stats.total_ms, stats.prediction_ms,
                     stats.constraints_ms, stats.collision_ms, stats.update_ms,
                     stats.node_count, stats.face_count,
                     min_y, max_y, ball_min_y, ball_max_y);
        if (step % 60 == 0)
        {
            std::printf("step=%d avg_step_ms=%.3f avg_stage_ms[predict=%.3f constraints=%.3f collision=%.3f update=%.3f] nodes=%zu faces=%zu cube_y_range=%.3f..%.3f ball_y_range=%.3f..%.3f\n",
                        step, total_ms / 60.0f,
                        prediction_ms / 60.0f, constraints_ms / 60.0f,
                        collision_ms / 60.0f, update_ms / 60.0f,
                        stats.node_count, stats.face_count,
                        min_y, max_y, ball_min_y, ball_max_y);
            prediction_ms = 0.0f;
            constraints_ms = 0.0f;
            collision_ms = 0.0f;
            update_ms = 0.0f;
            total_ms = 0.0f;
        }
    }

    std::fclose(csv);
    api.softBodyDestroy(cube);
    api.softBodyDestroy(ball);
    api.softWorldDestroy(world);

    SoftWorld* rigid_world = api.softWorldCreate({0.0f, -9.81f, 0.0f});
    SoftBody* rigid_cube = api.softBodyCreateCube(
        rigid_world, {0.0f, 4.0f, 0.0f}, {1.25f, 1.25f, 1.25f}, 4.0f);
    SoftRigidBody* rigid_ball = api.softRigidBodyCreateSphere(
        rigid_world, {0.0f, 7.5f, 0.0f}, 1.75f, 0.6f);
    if (rigid_world != nullptr && rigid_cube != nullptr && rigid_ball != nullptr)
    {
        for (int step = 0; step < 600; ++step)
            api.softWorldStep(rigid_world, timestep);
        SoftVec3 rigid_position{};
        api.softRigidBodyGetPosition(rigid_ball, &rigid_position);
        std::printf("rigid-ball-control final_y=%.3f cube_nodes=%zu cube_faces=%zu\n",
                    rigid_position.y,
                    api.softBodyGetNodeCount(rigid_cube),
                    api.softBodyGetFaceCount(rigid_cube));
    }
    if (rigid_ball != nullptr)
        api.softRigidBodyDestroy(rigid_ball);
    if (rigid_cube != nullptr)
        api.softBodyDestroy(rigid_cube);
    if (rigid_world != nullptr)
        api.softWorldDestroy(rigid_world);
    return 0;
}
