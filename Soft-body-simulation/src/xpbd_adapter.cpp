
#include "physics/SoftBodyAdapter.h"
#include "soft_body/xpbd_solver.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>
#include <algorithm>
#include <iterator>
#include <utility>
#include <cstdio>

struct SoftBody;
struct SoftRigidBody;

struct SoftWorld {
    soft_body::Vec3 gravity{0.0, -9.81, 0.0};

    std::vector<SoftBody*> bodies;

    std::vector<SoftRigidBody*> rigid_bodies;

    SoftWorldStats stats{};

    std::size_t sphere_mesh_contact_count = 0;

    std::size_t soft_soft_contact_count = 0;
};

struct SoftBody {
    SoftWorld* world = nullptr;

    soft_body::XPBDSolver solver;

    std::vector<unsigned int> triangle_indices;
};



struct SoftRigidBody {
    SoftWorld* world = nullptr;

    soft_body::Vec3 position{};
    soft_body::Vec3 previous_position{};
    soft_body::Vec3 velocity{};

    double radius = 0.5;
    double mass = 1.0;
};



namespace {

soft_body::Vec3 toXPBD(SoftVec3 value) {
    return {
        static_cast<double>(value.x),
        static_cast<double>(value.y),
        static_cast<double>(value.z)
    };
}

SoftVec3 fromXPBD(const soft_body::Vec3& value) {
    return {
        static_cast<float>(value.x),
        static_cast<float>(value.y),
        static_cast<float>(value.z)
    };
}


// Sphere vs triangle-mesh contacts. Uses closest point on each triangle,
// distributes the correction to its three vertices, and pushes back on the
// dynamic rigid sphere. This is discrete collision detection (not CCD).
static soft_body::Vec3 closestPointOnTriangle(
    const soft_body::Vec3& p, const soft_body::Vec3& a,
    const soft_body::Vec3& b, const soft_body::Vec3& c,
    double& wa, double& wb, double& wc) {
    const auto ab = b-a, ac = c-a, ap = p-a;
    const auto dot=[](const soft_body::Vec3& u,const soft_body::Vec3& v){
        return u.x*v.x+u.y*v.y+u.z*v.z;
    };
    const double d1=dot(ab,ap), d2=dot(ac,ap);
    if(d1<=0.0 && d2<=0.0){wa=1;wb=0;wc=0;return a;}
    const auto bp=p-b;
    const double d3=dot(ab,bp),d4=dot(ac,bp);
    if(d3>=0.0 && d4<=d3){wa=0;wb=1;wc=0;return b;}
    const double vc=d1*d4-d3*d2;
    if(vc<=0.0 && d1>=0.0 && d3<=0.0){
        const double t=d1/(d1-d3);wa=1-t;wb=t;wc=0;return a+ab*t;
    }
    const auto cp=p-c;
    const double d5=dot(ab,cp),d6=dot(ac,cp);
    if(d6>=0.0 && d5<=d6){wa=0;wb=0;wc=1;return c;}
    const double vb=d5*d2-d1*d6;
    if(vb<=0.0 && d2>=0.0 && d6<=0.0){
        const double t=d2/(d2-d6);wa=1-t;wb=0;wc=t;return a+ac*t;
    }
    const double va=d3*d6-d5*d4;
    if(va<=0.0 && (d4-d3)>=0.0 && (d5-d6)>=0.0){
        const double t=(d4-d3)/((d4-d3)+(d5-d6));
        wa=0;wb=1-t;wc=t;return b+(c-b)*t;
    }
    const double denom=va+vb+vc;
    if(std::abs(denom)<1e-18){wa=1;wb=0;wc=0;return a;}
    wb=vb/denom;wc=vc/denom;wa=1.0-wb-wc;
    return a+ab*wb+ac*wc;
}



static bool crossedTriangleSurface(
    const soft_body::Vec3& previousCenter,
    const soft_body::Vec3& currentCenter,
    const soft_body::Vec3& a,
    const soft_body::Vec3& b,
    const soft_body::Vec3& c,
    double radius,
    soft_body::Vec3& outNormal)
{
    using soft_body::Vec3;

    constexpr double eps = 1e-12;

    const Vec3 cross = (b - a).cross(c - a);
    const double length = cross.length();

    if (length < eps)
        return false;

    const Vec3 normal = cross / length;

    const double previousDistance =
        (previousCenter - a).dot(normal);

    const double currentDistance =
        (currentCenter - a).dot(normal);

    // Lopta mora dolaziti s vanjske strane trougla.
    if (previousDistance < radius ||
        currentDistance >= radius)
        return false;

    const double denominator =
        previousDistance - currentDistance;

    if (denominator < eps)
        return false;

    // Trenutak kada povrsina sfere dostize ravan trougla.
    const double t =
        (previousDistance - radius) / denominator;

    if (t < 0.0 || t > 1.0)
        return false;

    const Vec3 centerAtContact =
        previousCenter +
        (currentCenter - previousCenter) * t;

    // Projekcija centra sfere na ravan trougla.
    const Vec3 contactPoint =
        centerAtContact - normal * radius;

    const Vec3 v0 = b - a;
    const Vec3 v1 = c - a;
    const Vec3 v2 = contactPoint - a;

    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);

    const double denom =
        d00 * d11 - d01 * d01;

    if (std::abs(denom) < eps)
        return false;

    const double v =
        (d11 * d20 - d01 * d21) / denom;

    const double w =
        (d00 * d21 - d01 * d20) / denom;

    const double u = 1.0 - v - w;

    // Kontakt mora biti unutar trougla.
    constexpr double tolerance = 1e-6;

    if (u < -tolerance ||
        v < -tolerance ||
        w < -tolerance)
        return false;

    outNormal = normal;
    return true;
}



static soft_body::Vec3 triangleNormal(
    const soft_body::Vec3& a,
    const soft_body::Vec3& b,
    const soft_body::Vec3& c)
{
    const auto n = (b - a).cross(c - a);
    const double length = n.length();

    if (length < 1e-12)
        return {0.0, 0.0, 0.0};

    return n / length;
}


static bool areDifferentBodies(
    const SoftBody* a,
    const SoftBody* b)
{
    return a != nullptr &&
           b != nullptr &&
           a != b;
}



static bool segmentIntersectsTriangle(
    const soft_body::Vec3& start,
    const soft_body::Vec3& end,
    const soft_body::Vec3& a,
    const soft_body::Vec3& b,
    const soft_body::Vec3& c)
{
    constexpr double eps = 1e-9;

    const auto direction = end - start;
    const auto edge1 = b - a;
    const auto edge2 = c - a;

    const auto h = direction.cross(edge2);
    const double determinant = edge1.dot(h);

    if (std::abs(determinant) < eps)
        return false;

    const double invDet = 1.0 / determinant;

    const auto s = start - a;
    const double u = invDet * s.dot(h);

    if (u < -eps || u > 1.0 + eps)
        return false;

    const auto q = s.cross(edge1);
    const double v = invDet * direction.dot(q);

    if (v < -eps || u + v > 1.0 + eps)
        return false;

    const double t = invDet * edge2.dot(q);

    return t >= -eps && t <= 1.0 + eps;
}



struct SoftAABB {
    soft_body::Vec3 min;
    soft_body::Vec3 max;
};

static SoftAABB computeSoftAABB(const SoftBody* body)
{
    const auto& particles = body->solver.particles;

    SoftAABB box{
        particles.front().position,
        particles.front().position
    };

    for (const auto& p : particles) {
        const auto& v = p.position;

        box.min.x = std::min(box.min.x, v.x);
        box.min.y = std::min(box.min.y, v.y);
        box.min.z = std::min(box.min.z, v.z);

        box.max.x = std::max(box.max.x, v.x);
        box.max.y = std::max(box.max.y, v.y);
        box.max.z = std::max(box.max.z, v.z);
    }

    return box;
}

static bool aabbOverlap(
    const SoftAABB& a,
    const SoftAABB& b,
    double margin = 0.02
)
{
    return
        a.min.x <= b.max.x + margin &&
        a.max.x + margin >= b.min.x &&

        a.min.y <= b.max.y + margin &&
        a.max.y + margin >= b.min.y &&

        a.min.z <= b.max.z + margin &&
        a.max.z + margin >= b.min.z;
}



static void resolveSoftSoftContacts(SoftWorld* world)
{
    constexpr double eps = 1e-9;
    constexpr double thickness = 0.02;

    world->soft_soft_contact_count = 0;

    // Provjera u oba smjera: A -> B i B -> A.
    for (SoftBody* source : world->bodies) {
        for (SoftBody* target : world->bodies) {
            if (!areDifferentBodies(source, target))
                continue;
               if (source->solver.particles.empty() ||
                    target->solver.particles.empty())
                    continue;

                const SoftAABB sourceBox = computeSoftAABB(source);
                const SoftAABB targetBox = computeSoftAABB(target);

                if (!aabbOverlap(sourceBox, targetBox, thickness))
                    continue;

            auto& particles = source->solver.particles;
            auto& mesh = target->solver.particles;
            const auto& indices = target->triangle_indices;

            for (auto& p : particles) {
                if (p.inverse_mass <= 0.0)
                    continue;

                for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
                    auto& a = mesh[indices[i]];
                    auto& b = mesh[indices[i + 1]];
                    auto& c = mesh[indices[i + 2]];

                    double wa = 0.0;
                    double wb = 0.0;
                    double wc = 0.0;

                    const auto q = closestPointOnTriangle(
                        p.position,
                        a.position, b.position, c.position,
                        wa, wb, wc
                    );

                                
            const auto delta = p.position - q;
            const double distance = delta.length();

            const auto surfaceNormal = triangleNormal(
                a.position, b.position, c.position
            );

            // Potpisana udaljenost od povrsine.
            const double signedDistance =
                delta.dot(surfaceNormal);

            // Provjera da li je cestica presla s vanjske
            // na unutrasnju stranu trougla.
            const double previousSignedDistance =
                (p.previous_position - a.position).dot(surfaceNormal);

            
            const bool crossed =
                previousSignedDistance >= 0.0 &&
                signedDistance < 0.0 &&
                segmentIntersectsTriangle(
                    p.previous_position,
                    p.position,
                    a.position,
                    b.position,
                    c.position
                );


            // Kontakt blizu povrsine, s vanjske strane.
            const bool nearSurface =
                signedDistance >= 0.0 &&
                distance < thickness;

            if (!crossed && !nearSurface)
                continue;

            // Uvijek korigujemo prema vanjskoj strani.
            const auto normal = surfaceNormal;

            const double depth = crossed
                ? thickness - signedDistance
                : thickness - distance;

            if (depth <= 0.0)
                continue;


                    const double effective =
                        p.inverse_mass +
                        wa * wa * a.inverse_mass +
                        wb * wb * b.inverse_mass +
                        wc * wc * c.inverse_mass;

                    if (effective <= eps)
                        continue;

                    const auto correction = normal * (depth / effective);

                    
                    p.position = p.position + correction * p.inverse_mass;

                    a.position = a.position - correction * (wa * a.inverse_mass);
                    b.position = b.position - correction * (wb * b.inverse_mass);
                    c.position = c.position - correction * (wc * c.inverse_mass);

                    ++world->soft_soft_contact_count;
                }
            }
        }
    }
}



static void resolveSphereMeshContacts(SoftWorld* world, double dt) {
    world->sphere_mesh_contact_count = 0;
    constexpr double eps=1e-9;
    constexpr double restitution=0.05;
    for(SoftRigidBody* sphere:world->rigid_bodies){
        if(sphere->radius<=0.0) continue;
        const double rigidInvMass=sphere->mass>0.0?1.0/sphere->mass:0.0;
        for(SoftBody* body:world->bodies){
            auto& particles=body->solver.particles;
            const auto& indices=body->triangle_indices;
            // A few passes reduce remaining penetration after neighboring
            // triangles and the rigid body have both been corrected.
            for(int iteration=0;iteration<3;++iteration){
                bool changed=false;
                for(size_t t=0;t+2<indices.size();t+=3){
                    const auto ia=indices[t],ib=indices[t+1],ic=indices[t+2];
                    if(ia>=particles.size()||ib>=particles.size()||ic>=particles.size())continue;
                    auto& a=particles[ia];auto& b=particles[ib];auto& c=particles[ic];
                    double wa=0,wb=0,wc=0;
                    const auto q=closestPointOnTriangle(sphere->position,
                        a.position,b.position,c.position,wa,wb,wc);
                                        
                    auto delta = sphere->position - q;

                    const double d2 = delta.dot(delta);
                    const double d = std::sqrt(d2);

                    soft_body::Vec3 normal{};
                    double depth = 0.0;

                    // Postojeci kontakt: sfera je blizu trougla.
                    const bool overlapping = d < sphere->radius;

                    // Provjera prelaska kroz orijentisanu povrsinu.
                    soft_body::Vec3 crossingNormal{};

                    const bool crossed = crossedTriangleSurface(
                        sphere->previous_position,
                        sphere->position,
                        a.position,
                        b.position,
                        c.position,
                        sphere->radius,
                        crossingNormal
                    );

                    if (!overlapping && !crossed)
                        continue;

                    
                    const auto surfaceNormal = triangleNormal(
                        a.position,
                        b.position,
                        c.position
                    );

                    if (surfaceNormal.length() < eps)
                        continue;

                    if (crossed) {
                        normal = crossingNormal;

                        const double signedDistance =
                            (sphere->position - q).dot(normal);

                        depth = sphere->radius - signedDistance;
                    }
                    else if (overlapping) {
                        const double signedDistance =
                            (sphere->position - q).dot(surfaceNormal);

                        // Prihvatamo kontakt s vanjske strane povrsine.
                        // Ako je centar vec presao na unutrasnju stranu,
                        // ne koristimo delta/d jer bi korekcija bila pogresna.
                        if (signedDistance < 0.0)
                            continue;

                        normal = (d > eps)
                            ? delta / d
                            : surfaceNormal;

                        depth = sphere->radius - d;
                    }
                    else {
                        continue;
                    }


                    if (depth <= 0.0)
                        continue;

                    const double invA=std::max(0.0,a.inverse_mass);
                    const double invB=std::max(0.0,b.inverse_mass);
                    const double invC=std::max(0.0,c.inverse_mass);
                    const double effective=rigidInvMass+wa*wa*invA+wb*wb*invB+wc*wc*invC;
                    if(effective<=eps)continue;
                    ++world->sphere_mesh_contact_count;
                    const auto correction=normal*(depth/effective);
                    if(rigidInvMass>0.0)sphere->position=sphere->position+correction*rigidInvMass;
                    if(invA>0.0)a.position=a.position-correction*(wa*invA);
                    if(invB>0.0)b.position=b.position-correction*(wb*invB);
                    if(invC>0.0)c.position=c.position-correction*(wc*invC);
                    
                    changed=true;
                }
                if(!changed)break;
            }
        }
        // Re-enforce the ground after mesh contact.
        if(sphere->position.y<sphere->radius){
            sphere->position.y=sphere->radius;
            if(sphere->velocity.y<0.0)sphere->velocity.y=0.0;
        }
    }
}

} // namespace

extern "C" {

SoftWorld* softWorldCreate(SoftVec3 gravity) {
    try {
        auto world = std::make_unique<SoftWorld>();
        world->gravity = toXPBD(gravity);
        return world.release();
    }
    catch (...) {
        return nullptr;
    }
}

void softWorldSetGravity(SoftWorld* world, SoftVec3 gravity) {
    if (world == nullptr)
        return;

    world->gravity = toXPBD(gravity);

    for (SoftBody* body : world->bodies) {
        body->solver.gravity = world->gravity;
    }
}

int softWorldStep(SoftWorld* world, float timestep_seconds) {
    if (world == nullptr ||
        !std::isfinite(timestep_seconds) ||
        timestep_seconds <= 0.0f) {
        return 0;
    }

    try {
        const auto start = std::chrono::steady_clock::now();

        
        const double dt = static_cast<double>(timestep_seconds);

        // 1. Reset XPBD lambda vrijednosti.
        for (SoftBody* body : world->bodies) {
            for (auto& c : body->solver.distance_constraints) {
                c.lambda = 0.0;
            }

            for (auto& c : body->solver.volume_constraints) {
                c.lambda = 0.0;
            }
        }

        // 2. Predikcija pozicija svih deformabilnih tijela.
        for (SoftBody* body : world->bodies) {
            body->solver.predict_positions(dt);
        }

        // 3. Zajednicke XPBD iteracije.
        int max_iterations = 0;

        for (SoftBody* body : world->bodies) {
            max_iterations = std::max(
                max_iterations,
                body->solver.solver_iterations
            );
        }

        for (int iteration = 0; iteration < max_iterations; ++iteration) {
            for (SoftBody* body : world->bodies) {
                if (iteration >= body->solver.solver_iterations)
                    continue;

                body->solver.solve_distance_constraints(dt);
                body->solver.solve_volume_constraints(dt);
            }

            // U narednom koraku ovdje dodajemo
            // kontaktna ogranicenja izmedju tijela.
        }

        


        

        // Jednostavna gravitacija i kolizija rigidnih sfera s podlogom y = 0.
        for (SoftRigidBody* body : world->rigid_bodies) {
            body->previous_position = body->position;
            if (body->mass == 0.0)
                continue;

            body->velocity =
                body->velocity + world->gravity * dt;

            body->position =
                body->position + body->velocity * dt;

            if (body->position.y < body->radius) {
                body->position.y = body->radius;

                if (body->velocity.y < 0.0)
                    body->velocity.y = 0.0;
            }
        }

        // Jednostavna kolizija XPBD cestica s podlogom y = 0.
        for (SoftBody* body : world->bodies) {
            for (auto& particle : body->solver.particles) {
                if (particle.position.y < 0.0) {
                    particle.position.y = 0.0;

                    if (particle.velocity.y < 0.0)
                        particle.velocity.y = 0.0;
                }
            }
        }


        const auto collision_start = std::chrono::steady_clock::now();
        resolveSphereMeshContacts(world, dt);
        resolveSoftSoftContacts(world);

            #ifdef _DEBUG
        std::fprintf(stderr, "[XPBD] Soft-soft corrections: %zu\n",
                    world->soft_soft_contact_count);
        #endif

        
        #ifdef _DEBUG
        std::fprintf(
            stderr,
            "[XPBD] Sphere-mesh contact corrections: %zu\n",
            world->sphere_mesh_contact_count
        );
        #endif

        
        // Brzina rigidne sfere nakon pozicionih korekcija.
        for (SoftRigidBody* body : world->rigid_bodies) {
            if (body->mass == 0.0) {
                body->velocity = {0.0, 0.0, 0.0};
                continue;
            }

            body->velocity =
                (body->position - body->previous_position) / dt;

            // Podloga je na y = 0.
            if (body->position.y <= body->radius + 1e-8 &&
                body->velocity.y < 0.0) {
                body->velocity.y = 0.0;
            }
        }

        const auto collision_end = std::chrono::steady_clock::now();

        // Azuriranje brzina nakon korekcije pozicija.
        for (SoftBody* body : world->bodies) {
            body->solver.update_velocities(dt);
        }

        const auto end = std::chrono::steady_clock::now();

        world->stats.total_ms =
            std::chrono::duration<float, std::milli>(
                end - start
            ).count();

        world->stats.prediction_ms = 0.0f;
        world->stats.constraints_ms = 0.0f;
        world->stats.collision_ms =
            std::chrono::duration<float, std::milli>(collision_end - collision_start).count();
        world->stats.update_ms = 0.0f;

        world->stats.node_count = 0;
        world->stats.face_count = 0;

        for (const SoftBody* body : world->bodies) {
            world->stats.node_count +=
                body->solver.particles.size();

            world->stats.face_count +=
                body->triangle_indices.size() / 3;
        }

        return 1;
    }
    catch (...) {
        return 0;
    }
}

int softWorldGetStats(
    const SoftWorld* world,
    SoftWorldStats* out_stats
) {
    if (world == nullptr || out_stats == nullptr)
        return 0;

    *out_stats = world->stats;
    return 1;
}


SoftBody* softBodyCreateCube(
    SoftWorld* world,
    SoftVec3 center,
    SoftVec3 half_extents,
    float total_mass
) {
    if (world == nullptr ||
        !std::isfinite(center.x) ||
        !std::isfinite(center.y) ||
        !std::isfinite(center.z) ||
        !std::isfinite(half_extents.x) ||
        !std::isfinite(half_extents.y) ||
        !std::isfinite(half_extents.z) ||
        !std::isfinite(total_mass) ||
        half_extents.x <= 0.0f ||
        half_extents.y <= 0.0f ||
        half_extents.z <= 0.0f ||
        total_mass <= 0.0f) {
        return nullptr;
    }

    try {
        auto body = std::make_unique<SoftBody>();
        body->world = world;
        body->solver.gravity = world->gravity;
        body->solver.solver_iterations = 10;

        const double cx = center.x;
        const double cy = center.y;
        const double cz = center.z;

        const double hx = half_extents.x;
        const double hy = half_extents.y;
        const double hz = half_extents.z;

        const soft_body::Vec3 positions[8] = {
            {cx - hx, cy - hy, cz - hz},
            {cx + hx, cy - hy, cz - hz},
            {cx + hx, cy + hy, cz - hz},
            {cx - hx, cy + hy, cz - hz},
            {cx - hx, cy - hy, cz + hz},
            {cx + hx, cy - hy, cz + hz},
            {cx + hx, cy + hy, cz + hz},
            {cx - hx, cy + hy, cz + hz}
        };

        const double inverse_mass =
            8.0 / static_cast<double>(total_mass);

        for (const auto& position : positions) {
            soft_body::Particle particle{};
            particle.position = position;
            particle.previous_position = position;
            particle.velocity = {0.0, 0.0, 0.0};
            particle.inverse_mass = inverse_mass;

            body->solver.particles.push_back(particle);
        }

        const unsigned int triangles[] = {
            0, 2, 1,  0, 3, 2,
            4, 5, 6,  4, 6, 7,
            0, 1, 5,  0, 5, 4,
            3, 7, 6,  3, 6, 2,
            0, 4, 7,  0, 7, 3,
            1, 2, 6,  1, 6, 5
        };

        body->triangle_indices.assign(
            std::begin(triangles),
            std::end(triangles)
        );

        // Svaki par čestica povezan je distance constraintom.
        // To je jednostavna, stabilna početna verzija.
        const double compliance = 1e-5;

        for (std::size_t i = 0; i < 8; ++i) {
            for (std::size_t j = i + 1; j < 8; ++j) {
                const auto difference =
                    positions[j] - positions[i];

                const double rest_length =
                    difference.length();

                body->solver.distance_constraints.push_back({
                    i,
                    j,
                    rest_length,
                    compliance,
                    0.0
                });
            }
        }

        SoftBody* result = body.get();
        world->bodies.push_back(result);
        body.release();

        return result;
    }
    catch (...) {
        return nullptr;
    }
}


SoftBody* softBodyCreateSphere(
    SoftWorld* world,
    SoftVec3 center,
    float radius,
    size_t resolution,
    float total_mass
) {
    if (world == nullptr ||
        !std::isfinite(center.x) ||
        !std::isfinite(center.y) ||
        !std::isfinite(center.z) ||
        !std::isfinite(radius) ||
        !std::isfinite(total_mass) ||
        radius <= 0.0f ||
        resolution < 4 ||
        total_mass <= 0.0f) {
        return nullptr;
    }

    try {
        auto body = std::make_unique<SoftBody>();
        body->world = world;
        body->solver.gravity = world->gravity;
        body->solver.solver_iterations = 10;

        const size_t stacks = resolution;
        const size_t slices = resolution * 2;

        // Ogranicavamo velicinu mreze za pocetnu verziju.
        if (resolution > 128)
            return nullptr;

        const double pi = std::acos(-1.0);
        const double r = static_cast<double>(radius);

        const double cx = center.x;
        const double cy = center.y;
        const double cz = center.z;

        auto add_particle = [&](double x, double y, double z) {
            soft_body::Particle particle{};

            particle.position = {x, y, z};
            particle.previous_position = particle.position;
            particle.velocity = {0.0, 0.0, 0.0};

            body->solver.particles.push_back(particle);
        };

        // Gornji pol
        add_particle(cx, cy + r, cz);

        // Prstenovi sfere
        for (size_t stack = 1; stack < stacks; ++stack) {
            const double theta =
                pi * static_cast<double>(stack) /
                static_cast<double>(stacks);

            const double y = std::cos(theta);
            const double ring_radius = std::sin(theta);

            for (size_t slice = 0; slice < slices; ++slice) {
                const double phi =
                    2.0 * pi * static_cast<double>(slice) /
                    static_cast<double>(slices);

                const double x =
                    ring_radius * std::cos(phi);

                const double z =
                    ring_radius * std::sin(phi);

                add_particle(
                    cx + r * x,
                    cy + r * y,
                    cz + r * z
                );
            }
        }

        // Donji pol
        const unsigned int bottom_index =
            static_cast<unsigned int>(
                body->solver.particles.size()
            );

        add_particle(cx, cy - r, cz);
        
        const size_t center_index =
            body->solver.particles.size();

        add_particle(cx, cy, cz);


        const size_t particle_count =
            body->solver.particles.size();

        const double inverse_mass =
            static_cast<double>(particle_count) /
            static_cast<double>(total_mass);

        for (auto& particle : body->solver.particles) {
            particle.inverse_mass = inverse_mass;
        }

        auto ring_index = [&](size_t stack, size_t slice)
            -> unsigned int {
            return static_cast<unsigned int>(
                1 + (stack - 1) * slices + (slice % slices)
            );
        };

        // Gornja kapa
        for (size_t slice = 0; slice < slices; ++slice) {
            const unsigned int a =
                ring_index(1, slice);

            const unsigned int b =
                ring_index(1, slice + 1);

            body->triangle_indices.push_back(0);
            body->triangle_indices.push_back(b);
            body->triangle_indices.push_back(a);
        }

        // Srednji dio sfere
        for (size_t stack = 1; stack < stacks - 1; ++stack) {
            for (size_t slice = 0; slice < slices; ++slice) {
                const unsigned int a =
                    ring_index(stack, slice);

                const unsigned int b =
                    ring_index(stack, slice + 1);

                const unsigned int c =
                    ring_index(stack + 1, slice);

                const unsigned int d =
                    ring_index(stack + 1, slice + 1);

                body->triangle_indices.insert(
                    body->triangle_indices.end(),
                    {a, b, c, b, d, c}
                );
            }
        }

        // Donja kapa
        for (size_t slice = 0; slice < slices; ++slice) {
            const unsigned int a =
                ring_index(stacks - 1, slice);

            const unsigned int b =
                ring_index(stacks - 1, slice + 1);

            body->triangle_indices.push_back(a);
            body->triangle_indices.push_back(b);
            body->triangle_indices.push_back(bottom_index);
        }

        // Distance constraints duz ivica trouglova
        std::vector<std::pair<unsigned int, unsigned int>> edges;

        auto add_edge = [&](unsigned int a, unsigned int b) {
            if (a > b)
                std::swap(a, b);

            edges.emplace_back(a, b);
        };

        for (size_t i = 0;
             i < body->triangle_indices.size();
             i += 3) {

            const unsigned int a =
                body->triangle_indices[i];

            const unsigned int b =
                body->triangle_indices[i + 1];

            const unsigned int c =
                body->triangle_indices[i + 2];

            add_edge(a, b);
            add_edge(b, c);
            add_edge(c, a);
        }

        std::sort(edges.begin(), edges.end());
        edges.erase(
            std::unique(edges.begin(), edges.end()),
            edges.end()
        );

        const double compliance = 1e-5;

        for (const auto& edge : edges) {
            const size_t a = edge.first;
            const size_t b = edge.second;

            const auto difference =
                body->solver.particles[b].position -
                body->solver.particles[a].position;

            body->solver.distance_constraints.push_back({
                a,
                b,
                difference.length(),
                compliance,
                0.0
            });
        }

        
const double volume_compliance = 1e-7;

for (size_t i = 0;
     i < body->triangle_indices.size();
     i += 3) {

    const size_t a = body->triangle_indices[i];
    const size_t b = body->triangle_indices[i + 1];
    const size_t c = body->triangle_indices[i + 2];
    const size_t d = center_index;

    const auto& p0 = body->solver.particles[a].position;
    const auto& p1 = body->solver.particles[b].position;
    const auto& p2 = body->solver.particles[c].position;
    const auto& p3 = body->solver.particles[d].position;

    const auto e1 = p1 - p0;
    const auto e2 = p2 - p0;
    const auto e3 = p3 - p0;

    const double rest_volume =
        e1.dot(e2.cross(e3)) / 6.0;

    body->solver.volume_constraints.push_back({
        a,
        b,
        c,
        d,
        rest_volume,
        volume_compliance,
        0.0
    });
}


        SoftBody* result = body.get();
        world->bodies.push_back(result);
        body.release();

        return result;
    }
    catch (...) {
        return nullptr;
    }
}


SoftBody* softBodyCreateCloth(
    SoftWorld* world,
    SoftVec3 origin,
    float width,
    float height,
    size_t segments_x,
    size_t segments_y,
    float total_mass
) {
    if (world == nullptr ||
        !std::isfinite(origin.x) ||
        !std::isfinite(origin.y) ||
        !std::isfinite(origin.z) ||
        !std::isfinite(width) ||
        !std::isfinite(height) ||
        !std::isfinite(total_mass) ||
        width <= 0.0f ||
        height <= 0.0f ||
        total_mass <= 0.0f ||
        segments_x < 1 ||
        segments_y < 1 ||
        segments_x > 128 ||
        segments_y > 128) {
        return nullptr;
    }

    try {
        auto body = std::make_unique<SoftBody>();

        body->world = world;
        body->solver.gravity = world->gravity;
        body->solver.solver_iterations = 10;

        const size_t columns = segments_x + 1;
        const size_t rows = segments_y + 1;
        const size_t particle_count = columns * rows;

        const double inverse_mass =
            static_cast<double>(particle_count) /
            static_cast<double>(total_mass);

        const double dx =
            static_cast<double>(width) /
            static_cast<double>(segments_x);

        const double dy =
            static_cast<double>(height) /
            static_cast<double>(segments_y);

        // Platno je u XY ravni.
        // Origin predstavlja njegov donji lijevi ugao.
        for (size_t y = 0; y < rows; ++y) {
            for (size_t x = 0; x < columns; ++x) {
                soft_body::Particle particle{};

                particle.position = {
                    static_cast<double>(origin.x) + x * dx,
                    static_cast<double>(origin.y) + y * dy,
                    static_cast<double>(origin.z)
                };

                particle.previous_position = particle.position;
                particle.velocity = {0.0, 0.0, 0.0};
                particle.inverse_mass = inverse_mass;

                body->solver.particles.push_back(particle);
            }
        }

        auto index = [&](size_t x, size_t y) -> size_t {
            return y * columns + x;
        };

        // Svaka celija se dijeli na dva trougla.
        for (size_t y = 0; y < segments_y; ++y) {
            for (size_t x = 0; x < segments_x; ++x) {
                const unsigned int a =
                    static_cast<unsigned int>(index(x, y));

                const unsigned int b =
                    static_cast<unsigned int>(index(x + 1, y));

                const unsigned int c =
                    static_cast<unsigned int>(index(x, y + 1));

                const unsigned int d =
                    static_cast<unsigned int>(index(x + 1, y + 1));

                body->triangle_indices.insert(
                    body->triangle_indices.end(),
                    {a, b, c, b, d, c}
                );
            }
        }

        const double compliance = 1e-5;

        auto add_constraint = [&](size_t a, size_t b) {
            const auto difference =
                body->solver.particles[b].position -
                body->solver.particles[a].position;

            body->solver.distance_constraints.push_back({
                a,
                b,
                difference.length(),
                compliance,
                0.0
            });
        };

        // Horizontalne i vertikalne veze.
        for (size_t y = 0; y < rows; ++y) {
            for (size_t x = 0; x < columns; ++x) {
                if (x + 1 < columns) {
                    add_constraint(
                        index(x, y),
                        index(x + 1, y)
                    );
                }

                if (y + 1 < rows) {
                    add_constraint(
                        index(x, y),
                        index(x, y + 1)
                    );
                }
            }
        }

        // Dijagonalne veze protiv smicanja.
        for (size_t y = 0; y < segments_y; ++y) {
            for (size_t x = 0; x < segments_x; ++x) {
                add_constraint(
                    index(x, y),
                    index(x + 1, y + 1)
                );

                add_constraint(
                    index(x + 1, y),
                    index(x, y + 1)
                );
            }
        }

        // Veze izmedju cvorova udaljenih dvije celije.
        // Pomazu da se platno manje preklapa i savija.
        for (size_t y = 0; y < rows; ++y) {
            for (size_t x = 0; x < columns; ++x) {
                if (x + 2 < columns) {
                    add_constraint(
                        index(x, y),
                        index(x + 2, y)
                    );
                }

                if (y + 2 < rows) {
                    add_constraint(
                        index(x, y),
                        index(x, y + 2)
                    );
                }
            }
        }

        SoftBody* result = body.get();
        world->bodies.push_back(result);
        body.release();

        return result;
    }
    catch (...) {
        return nullptr;
    }
}



SoftRigidBody* softRigidBodyCreateSphere(
    SoftWorld* world,
    SoftVec3 center,
    float radius,
    float mass
) {
    if (!world ||
        !std::isfinite(center.x) ||
        !std::isfinite(center.y) ||
        !std::isfinite(center.z) ||
        !std::isfinite(radius) ||
        !std::isfinite(mass) ||
        radius <= 0.0f ||
        mass < 0.0f) {
        return nullptr;
    }

    try {
        auto body = std::make_unique<SoftRigidBody>();
        body->world = world;
        body->position = toXPBD(center);
        body->previous_position = body->position;
        body->radius = radius;
        body->mass = mass;

        SoftRigidBody* result = body.get();
        world->rigid_bodies.push_back(result);
        body.release();

        return result;
    }
    catch (...) {
        return nullptr;
    }
}

void softRigidBodyDestroy(SoftRigidBody* body) {
    if (!body)
        return;

    if (body->world) {
        auto& bodies = body->world->rigid_bodies;
        auto it = std::find(bodies.begin(), bodies.end(), body);

        if (it != bodies.end())
            bodies.erase(it);
    }

    delete body;
}

int softRigidBodyGetPosition(
    const SoftRigidBody* body,
    SoftVec3* out_position
) {
    if (!body || !out_position)
        return 0;

    *out_position = fromXPBD(body->position);
    return 1;
}



void softBodyDestroy(SoftBody* body) {
    if (body == nullptr)
        return;

    if (body->world != nullptr) {
        auto& bodies = body->world->bodies;

        for (auto it = bodies.begin();
             it != bodies.end(); ++it) {
            if (*it == body) {
                bodies.erase(it);
                break;
            }
        }
    }

    delete body;
}

void softWorldDestroy(SoftWorld* world) {
    if (world == nullptr)
        return;

    while (!world->bodies.empty()) {
    softBodyDestroy(world->bodies.back());
}

    while (!world->rigid_bodies.empty()) {
        softRigidBodyDestroy(world->rigid_bodies.back());
    }

    delete world;
}

size_t softBodyGetNodeCount(const SoftBody* body) {
    if (body == nullptr)
        return 0;

    return body->solver.particles.size();
}

size_t softBodyGetFaceCount(const SoftBody* body) {
    if (body == nullptr)
        return 0;

    return body->triangle_indices.size() / 3;
}

size_t softBodyCopyNodePositions(
    const SoftBody* body,
    SoftVec3* destination,
    size_t destination_count
) {
    if (body == nullptr || destination == nullptr)
        return 0;

    const size_t count = std::min(
        destination_count,
        body->solver.particles.size()
    );

    for (size_t i = 0; i < count; ++i) {
        destination[i] = fromXPBD(
            body->solver.particles[i].position
        );
    }

    return count;
}

size_t softBodyCopyIndices(
    const SoftBody* body,
    unsigned int* destination,
    size_t destination_count
) {
    if (body == nullptr || destination == nullptr)
        return 0;

    const size_t available =
        body->triangle_indices.size();

    const size_t count =
        std::min(destination_count, available);

    const size_t complete_count =
        (count / 3) * 3;

    for (size_t i = 0; i < complete_count; ++i) {
        destination[i] =
            body->triangle_indices[i];
    }

    return complete_count;
}


} // extern "C"
