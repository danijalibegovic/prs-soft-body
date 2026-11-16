#pragma once

#include "scene/Scene.h"
#include "core/MeshBuffer.h"
#include "core/Primitives.h"
#include "core/ShaderProgram.h"
#include "XpbdSoftBodyApi.h"

#include <glm/vec3.hpp>
#include <memory>
#include <vector>

struct XpbdSurfaceVertex
{
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec3 color{1.0f};
};

struct XpbdBallParams
{
    float edge_compliance = 1e-7f;       // Higher compliance = softer edge stretching.
    float bending_compliance = 2e-5f;   // Higher compliance = easier bending/more squash.
    float volume_compliance = 1e-8f;    // Higher compliance = more volume change.
    int solver_iterations = 12;         // Fewer iterations = softer/faster; more = stiffer.
    int mesh_frequency = 7;              // More segments = more particles; reset to apply.
    float fixed_timestep = 1.0f / 120.0f; // Smaller step = more stable, but more work.
    int max_substeps = 8;                // Higher cap = more catch-up work per frame.
    float total_mass = 0.6f;             // Lower mass = more responsive to external forces.
    float restitution = 0.12f;           // Higher value = bouncier ground impacts.
    float ground_friction = 0.35f;       // Higher value = less tangential motion at ground.
    float velocity_damping = 0.01f;      // Higher value = less wobble and faster settling.
};

class XpbdSoftBodyScene : public Scene
{
public:
    ~XpbdSoftBodyScene() override;

    bool Setup();
    void OnActivated() override;
    void OnImGuiRender() override;
    void OnUpdate(float deltaTime, IInputProvider& input) override;

private:
    void UpdateBodyMesh(SoftBody* body,
                        std::vector<VertexPC>& vertices,
                        MeshBuffer& mesh);
    void UpdateBallMesh();
    void ResetSimulation();
    void ResetBall();
    bool ApplyBallParameters();
    SoftBody* CreateBall();

    XpbdSoftBodyApi m_api;
    std::unique_ptr<ShaderProgram> m_shader;
    std::unique_ptr<ShaderProgram> m_ballShader;
    std::unique_ptr<MeshBuffer> m_groundMesh;
    std::unique_ptr<MeshBuffer> m_cubeMesh;
    std::unique_ptr<MeshBuffer> m_ballMesh;
    std::vector<VertexPC> m_cubeVertices;
    std::vector<XpbdSurfaceVertex> m_ballVertices;
    std::vector<unsigned int> m_ballIndices;
    std::vector<SoftVec3> m_cubePositions;
    std::vector<SoftVec3> m_ballPositions;
    SoftWorld* m_world = nullptr;
    SoftBody* m_cube = nullptr;
    SoftBody* m_ball = nullptr;
    XpbdBallParams m_ballParams;
    float m_accumulator = 0.0f;
    glm::vec3 m_playerPosition{0.0f, 3.0f, 10.0f};
    glm::vec3 m_moveDirection{0.0f};
    SoftWorldStats m_physicsStats{};
};
