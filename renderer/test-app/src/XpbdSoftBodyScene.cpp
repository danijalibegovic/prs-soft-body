#include "XpbdSoftBodyScene.h"

#include "core/Primitives.h"

#include <imgui.h>
#include <spdlog/spdlog.h>
#include <algorithm>

namespace
{
    std::unique_ptr<MeshBuffer> CreateDynamicMesh(const std::vector<VertexPC>& vertices,
                                                  const std::vector<unsigned int>& indices)
    {
        return std::make_unique<MeshBuffer>(
            vertices.data(),
            static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexPC)),
            static_cast<GLsizei>(vertices.size()),
            indices.data(),
            static_cast<GLsizei>(indices.size()),
            PrimitiveMeshData::CreateLayout(),
            GL_DYNAMIC_DRAW);
    }
}

XpbdSoftBodyScene::~XpbdSoftBodyScene()
{
    if (m_world != nullptr)
        m_api.softWorldDestroy(m_world);
}

void XpbdSoftBodyScene::OnActivated()
{
    ResetSimulation();
}

void XpbdSoftBodyScene::OnImGuiRender()
{
    ImGui::Begin("Physics", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::TextUnformatted("Backend: XPBD");
    ImGui::Separator();
    ImGui::Text("Simulation: %.3f ms", m_physicsStats.total_ms);
    ImGui::Text("Nodes: %zu", m_physicsStats.node_count);
    ImGui::Text("Faces: %zu", m_physicsStats.face_count);
    ImGui::TextUnformatted("Fixed timestep: 1/60 s");
    ImGui::End();
}

void XpbdSoftBodyScene::ResetSimulation()
{
    if (m_world != nullptr)
        m_api.softWorldDestroy(m_world);
    m_world = m_api.softWorldCreate({0.0f, -9.81f, 0.0f});
    m_cube = m_api.softBodyCreateCube(
        m_world, {0.0f, 4.0f, 0.0f}, {1.25f, 1.25f, 1.25f}, 4.0f);
    m_ball = m_api.softBodyCreateSphere(
        m_world, {0.0f, 7.5f, 0.0f}, 1.75f, 16, 0.6f);
    m_accumulator = 0.0f;
    m_physicsStats = {};
    if (m_cube != nullptr)
        UpdateBodyMesh(m_cube, m_cubeVertices, *m_cubeMesh);
    if (m_ball != nullptr)
        UpdateBodyMesh(m_ball, m_ballVertices, *m_ballMesh);
}

bool XpbdSoftBodyScene::Setup()
{
    SetSceneName("XPBD Soft Body");
    SetClearColor({0.025f, 0.035f, 0.055f, 1.0f});
    SetSkyboxVisible(false);
    SetAmbientLight({0.12f, 0.13f, 0.16f}, 1.0f);
    PointLight light;
    light.position = {3.5f, 8.0f, 5.0f};
    light.intensity = 35.0f;
    light.color = {1.0f, 0.92f, 0.82f};
    light.radius = 18.0f;
    light.name = "Showcase key light";
    GetLights().AddPointLight(light);

    if (!m_api.Load())
        return false;

    m_shader = std::make_unique<ShaderProgram>(
        "assets/shaders/basic.vert", "assets/shaders/basic.frag");
    if (!m_shader->IsValid())
    {
        spdlog::error("[XPBD] Showcase shader failed to load");
        return false;
    }

    Camera camera(CameraMode::FreeFly);
    camera.SetPosition({0.0f, 4.0f, 12.0f});
    camera.SetTarget({0.0f, 2.0f, 0.0f});
    SetCamera(camera);

    PrimitiveMeshData ground = GenerateQuad({
        .colorMode = ColorMode::Solid,
        .baseColor = {0.22f, 0.28f, 0.34f},
        .doubleSided = true,
    });
    m_groundMesh = std::make_unique<MeshBuffer>(ground.CreateMeshBuffer());
    RenderItem ground_item;
    ground_item.mesh = m_groundMesh.get();
    ground_item.shader = m_shader.get();
    ground_item.transform.SetScale({12.0f, 12.0f, 12.0f});
    ground_item.transform.SetRotationEulerDegrees({-90.0f, 0.0f, 0.0f});
    AddObject(ground_item);

    m_world = m_api.softWorldCreate({0.0f, -9.81f, 0.0f});
    if (m_world == nullptr)
    {
        spdlog::error("[XPBD] softWorldCreate returned null");
        return false;
    }
    m_cube = m_api.softBodyCreateCube(
        m_world, {0.0f, 4.0f, 0.0f}, {1.25f, 1.25f, 1.25f}, 4.0f);
    m_ball = m_api.softBodyCreateSphere(
        m_world, {0.0f, 7.5f, 0.0f}, 1.75f, 16, 0.6f);
    if (m_cube == nullptr || m_ball == nullptr)
    {
        spdlog::error("[XPBD] Body creation failed (cube={}, ball={})",
                      static_cast<const void*>(m_cube),
                      static_cast<const void*>(m_ball));
        if (m_cube != nullptr)
            m_api.softBodyDestroy(m_cube);
        if (m_ball != nullptr)
            m_api.softBodyDestroy(m_ball);
        m_api.softWorldDestroy(m_world);
        m_cube = nullptr;
        m_ball = nullptr;
        m_world = nullptr;
        return false;
    }

    std::vector<SoftVec3> cube_positions(m_api.softBodyGetNodeCount(m_cube));
    std::vector<unsigned int> cube_indices(m_api.softBodyGetFaceCount(m_cube) * 3);
    m_api.softBodyCopyNodePositions(m_cube, cube_positions.data(), cube_positions.size());
    m_api.softBodyCopyIndices(m_cube, cube_indices.data(), cube_indices.size());
    m_cubeVertices.resize(cube_positions.size());
    for (size_t i = 0; i < cube_positions.size(); ++i)
        m_cubeVertices[i] = {{cube_positions[i].x, cube_positions[i].y, cube_positions[i].z},
                             {1.0f, 0.35f, 0.18f}};
    m_cubeMesh = CreateDynamicMesh(m_cubeVertices, cube_indices);
    RenderItem cube_item;
    cube_item.mesh = m_cubeMesh.get();
    cube_item.shader = m_shader.get();
    cube_item.flags.castShadow = false;
    AddObject(cube_item);

    std::vector<SoftVec3> ball_positions(m_api.softBodyGetNodeCount(m_ball));
    std::vector<unsigned int> ball_indices(m_api.softBodyGetFaceCount(m_ball) * 3);
    m_api.softBodyCopyNodePositions(m_ball, ball_positions.data(), ball_positions.size());
    m_api.softBodyCopyIndices(m_ball, ball_indices.data(), ball_indices.size());
    m_ballVertices.resize(ball_positions.size());
    for (size_t i = 0; i < ball_positions.size(); ++i)
        m_ballVertices[i] = {{ball_positions[i].x, ball_positions[i].y, ball_positions[i].z},
                             {0.95f, 0.95f, 0.98f}};
    m_ballMesh = CreateDynamicMesh(m_ballVertices, ball_indices);
    RenderItem ball_item;
    ball_item.mesh = m_ballMesh.get();
    ball_item.shader = m_shader.get();
    ball_item.flags.castShadow = false;
    AddObject(ball_item);
    return true;
}

void XpbdSoftBodyScene::UpdateBodyMesh(SoftBody* body,
                                       std::vector<VertexPC>& vertices,
                                       MeshBuffer& mesh)
{
    std::vector<SoftVec3> positions(vertices.size());
    if (m_api.softBodyCopyNodePositions(body, positions.data(), positions.size()) != positions.size())
        return;
    for (size_t i = 0; i < positions.size(); ++i)
        vertices[i].position = {positions[i].x, positions[i].y, positions[i].z};
    mesh.UpdateVertexData(vertices.data(),
                          static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexPC)));
}

void XpbdSoftBodyScene::OnUpdate(float deltaTime, IInputProvider& input)
{
    UpdateStandardCameraAndPlayer(deltaTime, input, m_playerPosition, m_moveDirection);
    m_accumulator = std::min(m_accumulator + deltaTime, 0.25f);
    constexpr float fixed_timestep = 1.0f / 60.0f;
    while (m_accumulator >= fixed_timestep)
    {
        m_api.softWorldStep(m_world, fixed_timestep);
        m_api.softWorldGetStats(m_world, &m_physicsStats);
        m_accumulator -= fixed_timestep;
        UpdateBodyMesh(m_cube, m_cubeVertices, *m_cubeMesh);
        UpdateBodyMesh(m_ball, m_ballVertices, *m_ballMesh);
    }
}
