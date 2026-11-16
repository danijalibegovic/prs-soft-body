#include "XpbdSoftBodyScene.h"

#include "core/Primitives.h"

#include <imgui.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <utility>

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

    std::unique_ptr<MeshBuffer> CreateDynamicSurfaceMesh(
        const std::vector<XpbdSurfaceVertex>& vertices,
        const std::vector<unsigned int>& indices)
    {
        const VertexLayout layout({
            {0, 3, GL_FLOAT, GL_FALSE},
            {1, 3, GL_FLOAT, GL_FALSE},
            {2, 3, GL_FLOAT, GL_FALSE},
        });
        return std::make_unique<MeshBuffer>(
            vertices.data(),
            static_cast<GLsizeiptr>(vertices.size() * sizeof(XpbdSurfaceVertex)),
            static_cast<GLsizei>(vertices.size()),
            indices.data(),
            static_cast<GLsizei>(indices.size()),
            layout,
            GL_DYNAMIC_DRAW);
    }

    void UpdateSurfaceVertices(const std::vector<SoftVec3>& positions,
                               const std::vector<unsigned int>& indices,
                               std::vector<XpbdSurfaceVertex>& vertices)
    {
        for (size_t i = 0; i < positions.size(); ++i)
        {
            vertices[i].position = {
                positions[i].x, positions[i].y, positions[i].z
            };
            vertices[i].normal = {0.0f, 0.0f, 0.0f};
        }

        for (size_t i = 0; i + 2 < indices.size(); i += 3)
        {
            const unsigned int a = indices[i];
            const unsigned int b = indices[i + 1];
            const unsigned int c = indices[i + 2];
            const glm::vec3 normal = glm::cross(
                vertices[b].position - vertices[a].position,
                vertices[c].position - vertices[a].position);
            vertices[a].normal += normal;
            vertices[b].normal += normal;
            vertices[c].normal += normal;
        }

        for (auto& vertex : vertices)
        {
            const float length = glm::length(vertex.normal);
            if (length > 1e-8f)
                vertex.normal /= length;
            else
                vertex.normal = {0.0f, 1.0f, 0.0f};
        }
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
    ImGui::Text("XPBD step: %.3f ms", m_physicsStats.total_ms);
    ImGui::Text("  Predict: %.3f ms", m_physicsStats.prediction_ms);
    ImGui::Text("  Constraints: %.3f ms", m_physicsStats.constraints_ms);
    ImGui::Text("  Collisions: %.3f ms", m_physicsStats.collision_ms);
    ImGui::Text("  Velocity/update: %.3f ms", m_physicsStats.update_ms);
    ImGui::Text("Nodes: %zu", m_physicsStats.node_count);
    ImGui::Text("Faces: %zu", m_physicsStats.face_count);

    if (ImGui::CollapsingHeader("Ball", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const size_t vertex_count =
            m_ball != nullptr ? m_api.softBodyGetNodeCount(m_ball) : 0;
        const size_t triangle_count =
            m_ball != nullptr ? m_api.softBodyGetFaceCount(m_ball) : 0;
        ImGui::Text("Vertices: %zu", vertex_count);
        ImGui::Text("Triangles: %zu", triangle_count);

        bool changed = false;
        changed |= ImGui::SliderFloat(
            "Edge compliance", &m_ballParams.edge_compliance,
            1e-8f, 1e-2f, "%.1e", ImGuiSliderFlags_Logarithmic);
        changed |= ImGui::SliderFloat(
            "Bending compliance", &m_ballParams.bending_compliance,
            1e-7f, 1e-1f, "%.1e", ImGuiSliderFlags_Logarithmic);
        changed |= ImGui::SliderFloat(
            "Volume compliance", &m_ballParams.volume_compliance,
            1e-9f, 1e-3f, "%.1e", ImGuiSliderFlags_Logarithmic);
        changed |= ImGui::SliderInt(
            "Solver iterations", &m_ballParams.solver_iterations, 2, 16);
        ImGui::SliderInt(
            "Mesh frequency (Reset required)", &m_ballParams.mesh_frequency,
            1, 16);
        changed |= ImGui::SliderFloat(
            "Mass", &m_ballParams.total_mass, 0.1f, 2.0f, "%.2f");
        changed |= ImGui::SliderFloat(
            "Restitution", &m_ballParams.restitution, 0.0f, 0.6f, "%.2f");
        changed |= ImGui::SliderFloat(
            "Ground friction", &m_ballParams.ground_friction, 0.0f, 1.0f, "%.2f");
        changed |= ImGui::SliderFloat(
            "Velocity damping", &m_ballParams.velocity_damping,
            0.0f, 0.15f, "%.3f");

        float timestep_ms = m_ballParams.fixed_timestep * 1000.0f;
        if (ImGui::SliderFloat(
                "Physics step (ms)", &timestep_ms,
                1000.0f / 240.0f, 1000.0f / 60.0f, "%.2f"))
        {
            m_ballParams.fixed_timestep = timestep_ms / 1000.0f;
        }
        changed |= ImGui::SliderInt(
            "Max substeps/frame", &m_ballParams.max_substeps, 1, 16);

        if (changed && m_ball != nullptr && !ApplyBallParameters())
            spdlog::error("[XPBD] Failed to apply ball parameters");
        if (ImGui::Button("Reset ball"))
            ResetBall();
    }
    ImGui::End();
}

SoftBody* XpbdSoftBodyScene::CreateBall()
{
    return m_api.softBodyCreateSphere(
        m_world, {1.35f, 7.5f, 0.0f}, 1.75f,
        static_cast<size_t>(m_ballParams.mesh_frequency),
        m_ballParams.total_mass,
        m_ballParams.edge_compliance,
        m_ballParams.bending_compliance,
        m_ballParams.volume_compliance,
        static_cast<size_t>(m_ballParams.solver_iterations),
        m_ballParams.restitution,
        m_ballParams.ground_friction,
        m_ballParams.velocity_damping);
}

bool XpbdSoftBodyScene::ApplyBallParameters()
{
    return m_api.softBodySetSphereParameters(
        m_ball, m_ballParams.total_mass,
        m_ballParams.edge_compliance,
        m_ballParams.bending_compliance,
        m_ballParams.volume_compliance,
        static_cast<size_t>(m_ballParams.solver_iterations),
        m_ballParams.restitution,
        m_ballParams.ground_friction,
        m_ballParams.velocity_damping) != 0;
}

void XpbdSoftBodyScene::ResetBall()
{
    SoftBody* replacement = CreateBall();
    if (replacement == nullptr)
    {
        spdlog::error("[XPBD] Could not recreate ball with current parameters");
        return;
    }

    if (m_ballMesh == nullptr)
    {
        m_api.softBodyDestroy(replacement);
        spdlog::error("[XPBD] Cannot reset ball before its render mesh is initialized");
        return;
    }

    std::vector<SoftVec3> positions(m_api.softBodyGetNodeCount(replacement));
    std::vector<unsigned int> indices(
        m_api.softBodyGetFaceCount(replacement) * 3);
    if (m_api.softBodyCopyNodePositions(
            replacement, positions.data(), positions.size()) != positions.size() ||
        m_api.softBodyCopyIndices(
            replacement, indices.data(), indices.size()) != indices.size())
    {
        m_api.softBodyDestroy(replacement);
        spdlog::error("[XPBD] Failed to read recreated ball mesh data");
        return;
    }

    std::vector<XpbdSurfaceVertex> vertices(positions.size());
    for (auto& vertex : vertices)
        vertex.color = {0.95f, 0.95f, 0.98f};
    UpdateSurfaceVertices(positions, indices, vertices);
    auto replacement_mesh =
        CreateDynamicSurfaceMesh(vertices, indices);

    SoftBody* previous_ball = m_ball;
    m_ball = replacement;
    m_ballPositions = std::move(positions);
    m_ballIndices = std::move(indices);
    m_ballVertices = std::move(vertices);
    *m_ballMesh = std::move(*replacement_mesh);
    if (previous_ball != nullptr)
        m_api.softBodyDestroy(previous_ball);
}

void XpbdSoftBodyScene::ResetSimulation()
{
    if (m_world != nullptr)
        m_api.softWorldDestroy(m_world);
    m_world = m_api.softWorldCreate({0.0f, -9.81f, 0.0f});
    m_cube = m_api.softBodyCreateCube(
        m_world, {0.0f, 4.0f, 0.0f}, {1.25f, 1.25f, 1.25f}, 4.0f);
    m_ball = CreateBall();
    m_accumulator = 0.0f;
    m_physicsStats = {};
    if (m_cube != nullptr)
        UpdateBodyMesh(m_cube, m_cubeVertices, *m_cubeMesh);
    if (m_ball != nullptr)
        UpdateBallMesh();
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
    m_ballShader = std::make_unique<ShaderProgram>(
        "assets/shaders/xpbd_ball.vert", "assets/shaders/xpbd_ball.frag");
    if (!m_ballShader->IsValid())
    {
        spdlog::error("[XPBD] Ball shader failed to load");
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
    m_ball = CreateBall();
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

    m_cubePositions.resize(m_api.softBodyGetNodeCount(m_cube));
    std::vector<unsigned int> cube_indices(m_api.softBodyGetFaceCount(m_cube) * 3);
    m_api.softBodyCopyNodePositions(m_cube, m_cubePositions.data(), m_cubePositions.size());
    m_api.softBodyCopyIndices(m_cube, cube_indices.data(), cube_indices.size());
    m_cubeVertices.resize(m_cubePositions.size());
    for (size_t i = 0; i < m_cubePositions.size(); ++i)
        m_cubeVertices[i] = {{m_cubePositions[i].x, m_cubePositions[i].y, m_cubePositions[i].z},
                             {1.0f, 0.35f, 0.18f}};
    m_cubeMesh = CreateDynamicMesh(m_cubeVertices, cube_indices);
    RenderItem cube_item;
    cube_item.mesh = m_cubeMesh.get();
    cube_item.shader = m_shader.get();
    cube_item.flags.castShadow = false;
    AddObject(cube_item);

    m_ballPositions.resize(m_api.softBodyGetNodeCount(m_ball));
    m_api.softBodyCopyNodePositions(m_ball, m_ballPositions.data(), m_ballPositions.size());
    m_ballIndices.resize(m_api.softBodyGetFaceCount(m_ball) * 3);
    m_api.softBodyCopyIndices(m_ball, m_ballIndices.data(), m_ballIndices.size());
    m_ballVertices.resize(m_ballPositions.size());
    for (size_t i = 0; i < m_ballPositions.size(); ++i)
        m_ballVertices[i].color = {0.95f, 0.95f, 0.98f};
    UpdateSurfaceVertices(m_ballPositions, m_ballIndices, m_ballVertices);
    m_ballMesh = CreateDynamicSurfaceMesh(m_ballVertices, m_ballIndices);
    RenderItem ball_item;
    ball_item.mesh = m_ballMesh.get();
    ball_item.shader = m_ballShader.get();
    ball_item.flags.castShadow = false;
    AddObject(ball_item);
    return true;
}

void XpbdSoftBodyScene::UpdateBallMesh()
{
    if (m_ball == nullptr || m_ballMesh == nullptr)
        return;

    if (m_api.softBodyCopyNodePositions(
            m_ball, m_ballPositions.data(), m_ballPositions.size()) !=
        m_ballPositions.size())
        return;

    UpdateSurfaceVertices(m_ballPositions, m_ballIndices, m_ballVertices);
    m_ballMesh->UpdateVertexData(
        m_ballVertices.data(),
        static_cast<GLsizeiptr>(m_ballVertices.size() * sizeof(XpbdSurfaceVertex)));
}

void XpbdSoftBodyScene::UpdateBodyMesh(SoftBody* body,
                                       std::vector<VertexPC>& vertices,
                                       MeshBuffer& mesh)
{
    if (m_api.softBodyCopyNodePositions(body, m_cubePositions.data(),
                                        m_cubePositions.size()) !=
        m_cubePositions.size())
        return;
    for (size_t i = 0; i < m_cubePositions.size(); ++i)
        vertices[i].position = {
            m_cubePositions[i].x, m_cubePositions[i].y, m_cubePositions[i].z
        };
    mesh.UpdateVertexData(vertices.data(),
                          static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexPC)));
}

void XpbdSoftBodyScene::OnUpdate(float deltaTime, IInputProvider& input)
{
    UpdateStandardCameraAndPlayer(deltaTime, input, m_playerPosition, m_moveDirection);
    m_accumulator = std::min(m_accumulator + deltaTime, 0.25f);
    const float fixed_timestep = m_ballParams.fixed_timestep;
    const int max_substeps = m_ballParams.max_substeps;
    int substeps = 0;
    while (m_accumulator >= fixed_timestep && substeps < max_substeps)
    {
        m_api.softWorldStep(m_world, fixed_timestep);
        m_api.softWorldGetStats(m_world, &m_physicsStats);
        m_accumulator -= fixed_timestep;
        ++substeps;
    }

    if (substeps == max_substeps && m_accumulator >= fixed_timestep)
        m_accumulator = std::fmod(m_accumulator, fixed_timestep);

    if (substeps > 0)
    {
        UpdateBodyMesh(m_cube, m_cubeVertices, *m_cubeMesh);
        UpdateBallMesh();
    }
}
