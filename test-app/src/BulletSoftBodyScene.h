#pragma once

#include "scene/Scene.h"
#include "core/MeshBuffer.h"
#include "core/Primitives.h"
#include "core/ShaderProgram.h"
#include "physics/SoftBodyAdapter.h"

#include <glm/vec3.hpp>
#include <memory>
#include <vector>

struct SoftWorld;
struct SoftBody;

class BulletSoftBodyScene : public Scene
{
public:
    ~BulletSoftBodyScene() override;

    bool Setup();
    void OnActivated() override;
    void OnImGuiRender() override;
    void OnUpdate(float deltaTime, IInputProvider& input) override;

private:
    void UpdateBodyMesh(SoftBody* body,
                        std::vector<struct VertexPC>& vertices,
                        MeshBuffer& mesh);
    void ResetSimulation();

    std::unique_ptr<ShaderProgram> m_shader;
    std::unique_ptr<MeshBuffer> m_groundMesh;
    std::unique_ptr<MeshBuffer> m_cubeMesh;
    std::unique_ptr<MeshBuffer> m_ballMesh;
    std::vector<VertexPC> m_cubeVertices;
    std::vector<VertexPC> m_ballVertices;
    SoftWorld* m_world = nullptr;
    SoftBody* m_cube = nullptr;
    SoftBody* m_ball = nullptr;
    float m_accumulator = 0.0f;
    glm::vec3 m_playerPosition{0.0f, 3.0f, 10.0f};
    glm::vec3 m_moveDirection{0.0f};
    SoftWorldStats m_physicsStats{};
};
