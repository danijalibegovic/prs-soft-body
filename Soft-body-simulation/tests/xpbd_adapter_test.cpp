
#include "physics/SoftBodyAdapter.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <map>

void check(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

int main() {
    SoftWorld* world = nullptr;

    try {
        std::cout << "=== XPBD ADAPTER TEST ===\n";

        world = softWorldCreate({0.0f, -9.81f, 0.0f});
        check(world != nullptr, "World creation failed");

        std::cout << "[PASS] World creation\n";

        SoftBody* cube = softBodyCreateCube(
            world,
            {0.0f, 3.0f, 0.0f},
            {0.5f, 0.5f, 0.5f},
            1.0f
        );

        check(cube != nullptr, "Cube creation failed");
        check(softBodyGetNodeCount(cube) == 8,
              "Incorrect cube node count");
        check(softBodyGetFaceCount(cube) == 12,
              "Incorrect cube face count");

        std::cout << "[PASS] Cube geometry\n";

        SoftBody* sphere = softBodyCreateSphere(
            world,
            {2.0f, 3.0f, 0.0f},
            0.5f,
            7,
            1.0f,
            1e-7f,
            2e-5f,
            1e-8f,
            8,
            0.12f,
            0.35f,
            0.01f
        );

        check(sphere != nullptr, "Sphere creation failed");


        check(softBodyGetNodeCount(sphere) == 492,
              "Incorrect sphere node count");
        check(softBodyGetFaceCount(sphere) == 980,
              "Incorrect sphere face count");

        std::vector<SoftVec3> sphere_positions(softBodyGetNodeCount(sphere));
        std::vector<unsigned int> sphere_indices(softBodyGetFaceCount(sphere) * 3);
        check(softBodyCopyNodePositions(sphere, sphere_positions.data(),
                                        sphere_positions.size()) ==
                  sphere_positions.size(),
              "Failed to copy sphere positions");
        check(softBodyCopyIndices(sphere, sphere_indices.data(),
                                  sphere_indices.size()) == sphere_indices.size(),
              "Failed to copy sphere indices");

        std::map<std::pair<unsigned int, unsigned int>, size_t> edge_use_count;
        std::vector<bool> referenced(sphere_positions.size(), false);
        for (size_t i = 0; i < sphere_indices.size(); i += 3) {
            const unsigned int face[] = {
                sphere_indices[i], sphere_indices[i + 1], sphere_indices[i + 2]
            };
            for (size_t edge = 0; edge < 3; ++edge) {
                const unsigned int a = face[edge];
                const unsigned int b = face[(edge + 1) % 3];
                check(a < sphere_positions.size() && b < sphere_positions.size(),
                      "Sphere index out of range");
                ++edge_use_count[std::minmax(a, b)];
                referenced[a] = true;
            }
        }
        check(std::all_of(referenced.begin(), referenced.end(),
                          [](bool value) { return value; }),
              "Sphere contains an unreferenced simulation particle");
        check(std::all_of(edge_use_count.begin(), edge_use_count.end(),
                          [](const auto& edge) { return edge.second == 2; }),
              "Sphere is not a closed welded manifold");
        const auto vertex_count = static_cast<long long>(sphere_positions.size());
        const auto edge_count = static_cast<long long>(edge_use_count.size());
        const auto face_count = static_cast<long long>(sphere_indices.size() / 3);
        check(vertex_count - edge_count + face_count == 2,
              "Sphere Euler characteristic is not 2");

        std::cout << "[PASS] Welded closed sphere geometry\n";

                
        SoftBody* cloth = softBodyCreateCloth(
            world,
            {-2.0f, 3.0f, 0.0f},
            2.0f,
            2.0f,
            10,
            10,
            1.0f
        );

        check(cloth != nullptr, "Cloth creation failed");

        check(
            softBodyGetNodeCount(cloth) == 121,
            "Incorrect cloth node count"
        );

        check(
            softBodyGetFaceCount(cloth) == 200,
            "Incorrect cloth face count"
        );

        std::cout << "[PASS] Cloth geometry\n";

        
        SoftRigidBody* rigid = softRigidBodyCreateSphere(
            world,
            {4.0f, 2.0f, 0.0f},
            0.5f,
            1.0f
        );

        check(rigid != nullptr, "Rigid sphere creation failed");

        SoftVec3 rigid_position{};

        check(
            softRigidBodyGetPosition(rigid, &rigid_position) == 1,
            "Failed to read rigid sphere position"
        );

        check(
            std::abs(rigid_position.y - 2.0f) < 0.001f,
            "Incorrect initial rigid sphere position"
        );

        std::cout << "[PASS] Rigid sphere creation\n";



        std::vector<SoftVec3> cube_positions(
            softBodyGetNodeCount(cube)
        );

        check(
            softBodyCopyNodePositions(
                cube,
                cube_positions.data(),
                cube_positions.size()
            ) == cube_positions.size(),
            "Failed to copy cube positions"
        );

        const float initial_y = cube_positions[0].y;

                // Smoke test: pocetne prostorne koordinate kocke
        float minX = cube_positions[0].x;
        float maxX = cube_positions[0].x;
        float minY = cube_positions[0].y;
        float maxY = cube_positions[0].y;
        float minZ = cube_positions[0].z;
        float maxZ = cube_positions[0].z;

        for (const auto& p : cube_positions) {
            check(std::isfinite(p.x) &&
                std::isfinite(p.y) &&
                std::isfinite(p.z),
                "Invalid initial cube coordinates");

            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
            minZ = std::min(minZ, p.z);
            maxZ = std::max(maxZ, p.z);
        }

        std::cout << "Cube initial bounds:\n";
        std::cout << "  X: " << minX << " to " << maxX << '\n';
        std::cout << "  Y: " << minY << " to " << maxY << '\n';
        std::cout << "  Z: " << minZ << " to " << maxZ << '\n';

        const float tolerance = 0.001f;

        check(std::abs(minX + 0.5f) < tolerance &&
            std::abs(maxX - 0.5f) < tolerance &&
            std::abs(minY - 2.5f) < tolerance &&
            std::abs(maxY - 3.5f) < tolerance &&
            std::abs(minZ + 0.5f) < tolerance &&
            std::abs(maxZ - 0.5f) < tolerance,
            "Cube initial coordinates are incorrect");

        std::cout << "[PASS] Cube spatial coordinates\n";

        std::vector<unsigned int> cube_indices(
            softBodyGetFaceCount(cube) * 3
        );

        check(
            softBodyCopyIndices(
                cube,
                cube_indices.data(),
                cube_indices.size()
            ) == cube_indices.size(),
            "Failed to copy cube indices"
        );

        for (unsigned int index : cube_indices) {
            check(index < cube_positions.size(),
                  "Cube triangle index out of range");
        }

        std::cout << "[PASS] Geometry copying\n";

        check(softWorldStep(world, 1.0f / 60.0f) == 1,
              "Simulation step failed");

        check(
            softBodyCopyNodePositions(
                cube,
                cube_positions.data(),
                cube_positions.size()
            ) == cube_positions.size(),
            "Failed to read updated positions"
        );

        check(
            std::isfinite(cube_positions[0].y) &&
            cube_positions[0].y < initial_y,
            "Cube did not move under gravity"
        );

        std::cout << "[PASS] XPBD simulation step\n";

                
        check(
            softRigidBodyGetPosition(rigid, &rigid_position) == 1,
            "Failed to read updated rigid sphere position"
        );

        check(
            rigid_position.y < 2.0f,
            "Rigid sphere did not fall under gravity"
        );

        std::cout << "[PASS] Rigid sphere gravity\n";

        // Provjera kolizije s podlogom.
        for (int i = 0; i < 180; ++i) {
            check(
                softWorldStep(world, 1.0f / 60.0f) == 1,
                "Simulation step failed"
            );
        }

        check(
            softRigidBodyGetPosition(rigid, &rigid_position) == 1,
            "Failed to read rigid sphere after falling"
        );

        check(
            std::abs(rigid_position.y - 0.5f) < 0.001f,
            "Rigid sphere did not stop on ground"
        );

        std::cout << "[PASS] Rigid sphere ground collision\n";


        SoftWorldStats stats{};

        check(softWorldGetStats(world, &stats) == 1,
              "Failed to get statistics");

        check(stats.node_count == 621,
              "Incorrect world node count");

        check(stats.face_count == 1192,
              "Incorrect world face count");

        std::cout << "[PASS] Simulation statistics\n";

        // Numericki smoke test: 1000 dodatnih koraka simulacije.
        std::cout << "Running 1000-step numerical stability test...\n";

        struct BodyToCheck {
            SoftBody* body;
            const char* name;
        };

        const BodyToCheck bodies[] = {
            {cube, "cube"},
            {sphere, "sphere"},
            {cloth, "cloth"}
        };

        for (int step = 0; step < 1000; ++step) {
            check(
                softWorldStep(world, 1.0f / 60.0f) == 1,
                "Numerical stability: simulation step failed"
            );

            for (const auto& item : bodies) {
                const size_t count = softBodyGetNodeCount(item.body);
                std::vector<SoftVec3> positions(count);

                check(
                    softBodyCopyNodePositions(
                        item.body,
                        positions.data(),
                        positions.size()
                    ) == count,
                    "Numerical stability: failed to copy positions"
                );

                for (const auto& p : positions) {
                    check(
                        std::isfinite(p.x) &&
                        std::isfinite(p.y) &&
                        std::isfinite(p.z),
                        "Numerical stability: NaN or Inf detected"
                    );

                    check(
                        std::abs(p.x) < 10000.0f &&
                        std::abs(p.y) < 10000.0f &&
                        std::abs(p.z) < 10000.0f,
                        "Numerical stability: position exploded"
                    );
                }
            }

            SoftVec3 rigid_pos{};

            check(
                softRigidBodyGetPosition(rigid, &rigid_pos) == 1,
                "Numerical stability: failed to read rigid position"
            );

            check(
                std::isfinite(rigid_pos.x) &&
                std::isfinite(rigid_pos.y) &&
                std::isfinite(rigid_pos.z),
                "Numerical stability: rigid sphere has NaN or Inf"
            );

            check(
                std::abs(rigid_pos.x) < 10000.0f &&
                std::abs(rigid_pos.y) < 10000.0f &&
                std::abs(rigid_pos.z) < 10000.0f,
                "Numerical stability: rigid sphere position exploded"
            );
        }

        std::cout << "[PASS] 1000-step numerical stability test\n";

        softWorldDestroy(world);
        world = nullptr;

        std::cout << "[PASS] World destruction\n";

        SoftWorld* volume_world = softWorldCreate({0.0f, -9.81f, 0.0f});
        check(volume_world != nullptr, "Volume test world creation failed");
        SoftBody* volume_sphere = softBodyCreateSphere(
            volume_world, {0.0f, 2.0f, 0.0f}, 0.5f, 7, 0.6f,
            1e-7f, 2e-5f, 1e-8f, 8, 0.12f, 0.35f, 0.01f);
        check(volume_sphere != nullptr, "Volume test sphere creation failed");
        const auto calculate_sphere_volume = [&](SoftBody* body) {
            std::vector<SoftVec3> positions(softBodyGetNodeCount(body));
            std::vector<unsigned int> indices(softBodyGetFaceCount(body) * 3);
            check(softBodyCopyNodePositions(body, positions.data(), positions.size()) ==
                      positions.size(),
                  "Failed to read volume-test sphere positions");
            check(softBodyCopyIndices(body, indices.data(), indices.size()) ==
                      indices.size(),
                  "Failed to read volume-test sphere indices");
            double volume = 0.0;
            for (std::size_t i = 0; i < indices.size(); i += 3) {
                const auto& a = positions[indices[i]];
                const auto& b = positions[indices[i + 1]];
                const auto& c = positions[indices[i + 2]];
                volume +=
                    static_cast<double>(a.x) *
                        (static_cast<double>(b.y) * c.z -
                         static_cast<double>(b.z) * c.y) +
                    static_cast<double>(a.y) *
                        (static_cast<double>(b.z) * c.x -
                         static_cast<double>(b.x) * c.z) +
                    static_cast<double>(a.z) *
                        (static_cast<double>(b.x) * c.y -
                         static_cast<double>(b.y) * c.x);
            }
            return volume / 6.0;
        };
        const double rest_sphere_volume = calculate_sphere_volume(volume_sphere);
        for (int step = 0; step < 600; ++step)
            check(softWorldStep(volume_world, 1.0f / 60.0f) == 1,
                  "Volume test simulation step failed");
        const double landed_sphere_volume = calculate_sphere_volume(volume_sphere);
        check(std::abs(landed_sphere_volume - rest_sphere_volume) /
                  std::abs(rest_sphere_volume) < 0.03,
              "Sphere volume changed by more than 3% after landing");
        softBodyDestroy(volume_sphere);
        softWorldDestroy(volume_world);
        std::cout << "[PASS] Sphere volume after landing\n";

        std::cout << "\nAll adapter tests passed!\n";

        return 0;
    }
    catch (const std::exception& e) {
        softWorldDestroy(world);

        std::cerr << "[FAIL] " << e.what() << '\n';
        return 1;
    }
}
