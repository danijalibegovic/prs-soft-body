#include "physics/SoftBodyAdapter.h"

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include <chrono>
#include <algorithm>
#include <memory>
#include <vector>

btVector3 ToBullet(SoftVec3 value)
{
    return {value.x, value.y, value.z};
}

SoftVec3 FromBullet(const btVector3& value)
{
    return {value.x(), value.y(), value.z()};
}

struct SoftWorld;

struct SoftBody {
    SoftWorld* world = nullptr;
    btSoftBody* bullet_body = nullptr;
};

struct SoftRigidBody {
    SoftWorld* world = nullptr;
    btRigidBody* bullet_body = nullptr;
    std::unique_ptr<btCollisionShape> shape;
    std::unique_ptr<btDefaultMotionState> motion_state;
};

struct SoftWorld {
    std::unique_ptr<btSoftBodyRigidBodyCollisionConfiguration> collision_configuration;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btBroadphaseInterface> broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
    std::unique_ptr<btSoftRigidDynamicsWorld> dynamics_world;
    std::unique_ptr<btStaticPlaneShape> ground_shape;
    std::unique_ptr<btDefaultMotionState> ground_motion_state;
    std::unique_ptr<btRigidBody> ground_body;
    btSoftBodyWorldInfo world_info;
    SoftWorldStats stats{};
    std::vector<SoftBody*> bodies;
    std::vector<SoftRigidBody*> rigid_bodies;

    explicit SoftWorld(SoftVec3 gravity)
    {
        collision_configuration = std::make_unique<btSoftBodyRigidBodyCollisionConfiguration>();
        dispatcher = std::make_unique<btCollisionDispatcher>(collision_configuration.get());
        broadphase = std::make_unique<btDbvtBroadphase>();
        solver = std::make_unique<btSequentialImpulseConstraintSolver>();
        dynamics_world = std::make_unique<btSoftRigidDynamicsWorld>(
            dispatcher.get(), broadphase.get(), solver.get(), collision_configuration.get());

        dynamics_world->setGravity(ToBullet(gravity));
        world_info.m_dispatcher = dispatcher.get();
        world_info.m_broadphase = broadphase.get();
        world_info.m_sparsesdf.Initialize();
        world_info.m_gravity = ToBullet(gravity);
        world_info.air_density = 1.2f;
        world_info.water_density = 0.0f;
        world_info.water_offset = 0.0f;
        world_info.water_normal = btVector3(0, 0, 0);

        ground_shape = std::make_unique<btStaticPlaneShape>(btVector3(0, 1, 0), 0.0f);
        ground_motion_state = std::make_unique<btDefaultMotionState>(
            btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 0)));
        btRigidBody::btRigidBodyConstructionInfo ground_info(
            0.0f, ground_motion_state.get(), ground_shape.get());
        ground_body = std::make_unique<btRigidBody>(ground_info);
        ground_body->setFriction(0.8f);
        ground_body->setRestitution(0.0f);
        dynamics_world->addRigidBody(
            ground_body.get(),
            btBroadphaseProxy::StaticFilter,
            btBroadphaseProxy::AllFilter);
    }

    ~SoftWorld()
    {
        for (SoftBody* body : bodies)
            dynamics_world->removeSoftBody(body->bullet_body);
        for (SoftRigidBody* body : rigid_bodies)
            dynamics_world->removeRigidBody(body->bullet_body);
        dynamics_world->removeRigidBody(ground_body.get());
    }
};

SoftBody* CreateBody(SoftWorld* world, btSoftBody* bullet_body)
{
    if (world == nullptr || bullet_body == nullptr)
        return nullptr;

    auto* body = new SoftBody{world, bullet_body};
    world->dynamics_world->addSoftBody(bullet_body);
    world->bodies.push_back(body);
    return body;
}

void RemoveBodyFromWorld(SoftBody* body)
{
    auto& bodies = body->world->bodies;
    for (auto it = bodies.begin(); it != bodies.end(); ++it)
    {
        if (*it == body)
        {
            bodies.erase(it);
            break;
        }
    }
}

void RemoveRigidBodyFromWorld(SoftRigidBody* body)
{
    auto& bodies = body->world->rigid_bodies;
    for (auto it = bodies.begin(); it != bodies.end(); ++it)
    {
        if (*it == body)
        {
            bodies.erase(it);
            break;
        }
    }
}

extern "C" {

SoftWorld* softWorldCreate(SoftVec3 gravity)
{
    return new SoftWorld(gravity);
}

void softWorldDestroy(SoftWorld* world)
{
    if (world == nullptr)
        return;

    while (!world->bodies.empty())
        softBodyDestroy(world->bodies.back());
    while (!world->rigid_bodies.empty())
        softRigidBodyDestroy(world->rigid_bodies.back());
    delete world;
}

void softWorldSetGravity(SoftWorld* world, SoftVec3 gravity)
{
    if (world == nullptr)
        return;

    world->dynamics_world->setGravity(ToBullet(gravity));
    world->world_info.m_gravity = ToBullet(gravity);
}

int softWorldStep(SoftWorld* world, float timestep_seconds)
{
    if (world == nullptr || timestep_seconds <= 0.0f)
        return 0;

    const auto start = std::chrono::steady_clock::now();
    world->dynamics_world->stepSimulation(timestep_seconds, 0, timestep_seconds);
    const auto end = std::chrono::steady_clock::now();

    world->stats.total_ms =
        std::chrono::duration<float, std::milli>(end - start).count();
    world->stats.prediction_ms = 0.0f;
    world->stats.constraints_ms = 0.0f;
    world->stats.collision_ms = 0.0f;
    world->stats.update_ms = 0.0f;
    world->stats.node_count = 0;
    world->stats.face_count = 0;
    for (const SoftBody* body : world->bodies)
    {
        world->stats.node_count += static_cast<size_t>(body->bullet_body->m_nodes.size());
        world->stats.face_count += static_cast<size_t>(body->bullet_body->m_faces.size());
    }
    return 1;
}

int softWorldGetStats(const SoftWorld* world, SoftWorldStats* out_stats)
{
    if (world == nullptr || out_stats == nullptr)
        return 0;
    *out_stats = world->stats;
    return 1;
}

SoftBody* softBodyCreateCloth(SoftWorld* world,
                              SoftVec3 origin,
                              float width,
                              float height,
                              size_t segments_x,
                              size_t segments_y,
                              float total_mass)
{
    if (world == nullptr || width <= 0.0f || height <= 0.0f ||
        segments_x < 2 || segments_y < 2 || total_mass <= 0.0f)
        return nullptr;

    const btVector3 center = ToBullet(origin);
    const btVector3 half_width(width * 0.5f, 0, 0);
    const btVector3 half_height(0, 0, height * 0.5f);
    btSoftBody* cloth = btSoftBodyHelpers::CreatePatch(
        world->world_info,
        center - half_width - half_height,
        center + half_width - half_height,
        center - half_width + half_height,
        center + half_width + half_height,
        static_cast<int>(segments_x),
        static_cast<int>(segments_y),
        1,
        true);
    if (cloth == nullptr)
        return nullptr;

    cloth->setTotalMass(total_mass, true);
    cloth->m_materials[0]->m_kLST = 0.45f;
    cloth->m_materials[0]->m_kAST = 0.25f;
    cloth->m_materials[0]->m_kVST = 0.15f;
    cloth->m_cfg.kDP = 0.01f;
    cloth->m_cfg.kDF = 0.5f;
    cloth->m_cfg.kCHR = 1.0f;
    cloth->m_cfg.kKHR = 1.0f;
    cloth->getCollisionShape()->setMargin(0.05f);
    cloth->m_cfg.collisions =
        btSoftBody::fCollision::VF_SS | btSoftBody::fCollision::SDF_RS;
    cloth->generateBendingConstraints(2, cloth->m_materials[0]);
    return CreateBody(world, cloth);
}

SoftBody* softBodyCreateSphere(SoftWorld* world,
                               SoftVec3 center,
                               float radius,
                               size_t resolution,
                               float total_mass)
{
    if (world == nullptr || radius <= 0.0f || resolution < 4 || total_mass <= 0.0f)
        return nullptr;

    btSoftBody* sphere = btSoftBodyHelpers::CreateEllipsoid(
        world->world_info, ToBullet(center), btVector3(radius, radius, radius),
        static_cast<int>(resolution));
    if (sphere == nullptr)
        return nullptr;

    sphere->setTotalMass(total_mass, true);
    sphere->m_materials[0]->m_kLST = 0.1f;
    sphere->m_materials[0]->m_kAST = 0.08f;
    sphere->m_materials[0]->m_kVST = 0.08f;
    sphere->m_cfg.kDP = 0.001f;
    sphere->m_cfg.kDF = 1.0f;
    sphere->m_cfg.kPR = 35.0f;
    sphere->m_cfg.kCHR = 0.8f;
    sphere->m_cfg.kKHR = 0.8f;
    sphere->m_cfg.kSHR = 0.8f;
    sphere->m_cfg.viterations = 2;
    sphere->m_cfg.piterations = 4;
    sphere->m_cfg.diterations = 2;
    sphere->m_cfg.citerations = 4;
    sphere->getCollisionShape()->setMargin(0.05f);
    sphere->m_cfg.collisions =
        btSoftBody::fCollision::VF_SS | btSoftBody::fCollision::SDF_RS;
    sphere->generateBendingConstraints(2, sphere->m_materials[0]);
    return CreateBody(world, sphere);
}

SoftBody* softBodyCreateCube(SoftWorld* world,
                             SoftVec3 center,
                             SoftVec3 half_extents,
                             float total_mass)
{
    if (world == nullptr || half_extents.x <= 0.0f || half_extents.y <= 0.0f ||
        half_extents.z <= 0.0f || total_mass <= 0.0f)
        return nullptr;

    const btVector3 c = ToBullet(center);
    const btVector3 h = ToBullet(half_extents);
    const btVector3 vertices[] = {
        c + btVector3(-h.x(), -h.y(), -h.z()),
        c + btVector3( h.x(), -h.y(), -h.z()),
        c + btVector3( h.x(),  h.y(), -h.z()),
        c + btVector3(-h.x(),  h.y(), -h.z()),
        c + btVector3(-h.x(), -h.y(),  h.z()),
        c + btVector3( h.x(), -h.y(),  h.z()),
        c + btVector3( h.x(),  h.y(),  h.z()),
        c + btVector3(-h.x(),  h.y(),  h.z()),
    };

    btSoftBody* cube = btSoftBodyHelpers::CreateFromConvexHull(
        world->world_info, vertices, 8, true);
    if (cube == nullptr)
        return nullptr;

    cube->setTotalMass(total_mass, true);
    cube->m_materials[0]->m_kLST = 0.08f;
    cube->m_materials[0]->m_kAST = 0.06f;
    cube->m_materials[0]->m_kVST = 0.06f;
    cube->m_cfg.kDP = 0.001f;
    cube->m_cfg.kDF = 1.0f;
    cube->m_cfg.kPR = 12.0f;
    cube->m_cfg.kCHR = 0.8f;
    cube->m_cfg.kKHR = 0.8f;
    cube->m_cfg.kSHR = 0.8f;
    cube->m_cfg.viterations = 2;
    cube->m_cfg.piterations = 4;
    cube->m_cfg.diterations = 2;
    cube->m_cfg.citerations = 4;
    cube->getCollisionShape()->setMargin(0.05f);
    cube->m_cfg.collisions =
        btSoftBody::fCollision::VF_SS | btSoftBody::fCollision::SDF_RS;
    return CreateBody(world, cube);
}

SoftRigidBody* softRigidBodyCreateSphere(SoftWorld* world,
                                         SoftVec3 center,
                                         float radius,
                                         float mass)
{
    if (world == nullptr || radius <= 0.0f || mass <= 0.0f)
        return nullptr;

    auto body = std::make_unique<SoftRigidBody>();
    body->world = world;
    body->shape = std::make_unique<btSphereShape>(radius);
    body->motion_state = std::make_unique<btDefaultMotionState>(
        btTransform(btQuaternion::getIdentity(), ToBullet(center)));

    btVector3 inertia(0, 0, 0);
    body->shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo info(
        mass, body->motion_state.get(), body->shape.get(), inertia);
    body->bullet_body = new btRigidBody(info);
    body->bullet_body->setFriction(0.7f);
    body->bullet_body->setRestitution(0.15f);
    body->bullet_body->setGravity(world->dynamics_world->getGravity());
    body->bullet_body->setCcdMotionThreshold(radius * 0.5f);
    body->bullet_body->setCcdSweptSphereRadius(radius * 0.8f);
    body->bullet_body->setActivationState(ACTIVE_TAG);
    world->dynamics_world->addRigidBody(
        body->bullet_body,
        btBroadphaseProxy::DefaultFilter,
        btBroadphaseProxy::AllFilter);

    SoftRigidBody* result = body.release();
    world->rigid_bodies.push_back(result);
    return result;
}

void softRigidBodyDestroy(SoftRigidBody* body)
{
    if (body == nullptr)
        return;
    body->world->dynamics_world->removeRigidBody(body->bullet_body);
    RemoveRigidBodyFromWorld(body);
    delete body->bullet_body;
    delete body;
}

int softRigidBodyGetPosition(const SoftRigidBody* body, SoftVec3* out_position)
{
    if (body == nullptr || out_position == nullptr)
        return 0;

    btTransform transform;
    body->bullet_body->getMotionState()->getWorldTransform(transform);
    *out_position = FromBullet(transform.getOrigin());
    return 1;
}

void softBodyDestroy(SoftBody* body)
{
    if (body == nullptr)
        return;
    body->world->dynamics_world->removeSoftBody(body->bullet_body);
    RemoveBodyFromWorld(body);
    delete body->bullet_body;
    delete body;
}

size_t softBodyGetNodeCount(const SoftBody* body)
{
    return body == nullptr ? 0 : static_cast<size_t>(body->bullet_body->m_nodes.size());
}

size_t softBodyGetFaceCount(const SoftBody* body)
{
    return body == nullptr ? 0 : static_cast<size_t>(body->bullet_body->m_faces.size());
}

size_t softBodyCopyNodePositions(const SoftBody* body,
                                 SoftVec3* destination,
                                 size_t destination_count)
{
    if (body == nullptr || destination == nullptr)
        return 0;

    const size_t count = std::min(
        destination_count, static_cast<size_t>(body->bullet_body->m_nodes.size()));
    for (size_t i = 0; i < count; ++i)
        destination[i] = FromBullet(body->bullet_body->m_nodes[static_cast<int>(i)].m_x);
    return count;
}

size_t softBodyCopyIndices(const SoftBody* body,
                           unsigned int* destination,
                           size_t destination_count)
{
    if (body == nullptr || destination == nullptr)
        return 0;

    const size_t required = static_cast<size_t>(body->bullet_body->m_faces.size()) * 3;
    const size_t count = std::min(destination_count, required);
    for (size_t i = 0; i < count / 3; ++i)
    {
        const btSoftBody::Face& face = body->bullet_body->m_faces[static_cast<int>(i)];
        const btSoftBody::Node* first_node = &body->bullet_body->m_nodes[0];
        destination[i * 3 + 0] = static_cast<unsigned int>(face.m_n[0] - first_node);
        destination[i * 3 + 1] = static_cast<unsigned int>(face.m_n[1] - first_node);
        destination[i * 3 + 2] = static_cast<unsigned int>(face.m_n[2] - first_node);
    }
    return (count / 3) * 3;
}

} // extern "C"
