
#include "soft_body/soft_body_api.h"
#include "soft_body/xpbd_solver.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <new>
#include <vector>

using soft_body::XPBDSolver;
using soft_body::Particle;
using soft_body::DistanceConstraint;
using soft_body::VolumeConstraint;

struct SoftBodyHandle {
    XPBDSolver solver;
};

namespace {

bool valid_number(double value) {
    return std::isfinite(value);
}

bool valid_particle_index(
    const SoftBodyHandle* body,
    int index
) {
    return body != nullptr &&
           index >= 0 &&
           static_cast<std::size_t>(index) <
               body->solver.particles.size();
}

} // namespace

extern "C" {

SoftBodyHandle* soft_body_create(void) {
    try {
        return new SoftBodyHandle{};
    }
    catch (...) {
        return nullptr;
    }
}

void soft_body_destroy(SoftBodyHandle* body) {
    delete body;
}

int soft_body_add_particle(
    SoftBodyHandle* body,
    double x,
    double y,
    double z,
    double inverse_mass
) {
    if (!body ||
        !valid_number(x) ||
        !valid_number(y) ||
        !valid_number(z) ||
        !valid_number(inverse_mass) ||
        inverse_mass < 0.0) {
        return -1;
    }

    try {
        auto& particles = body->solver.particles;

        if (particles.size() >=
            static_cast<std::size_t>(
                std::numeric_limits<int>::max())) {
            return -1;
        }

        const int index =
            static_cast<int>(particles.size());

        Particle particle{};
        particle.position = {x, y, z};
        particle.previous_position = {x, y, z};
        particle.velocity = {0.0, 0.0, 0.0};
        particle.inverse_mass = inverse_mass;

        particles.push_back(particle);

        return index;
    }
    catch (...) {
        return -1;
    }
}

int soft_body_add_distance_constraint(
    SoftBodyHandle* body,
    int particle_a,
    int particle_b,
    double rest_length,
    double compliance
) {
    if (!valid_particle_index(body, particle_a) ||
        !valid_particle_index(body, particle_b) ||
        particle_a == particle_b ||
        !valid_number(rest_length) ||
        !valid_number(compliance) ||
        rest_length <= 0.0 ||
        compliance < 0.0) {
        return 0;
    }

    try {
        body->solver.distance_constraints.push_back({
            static_cast<std::size_t>(particle_a),
            static_cast<std::size_t>(particle_b),
            rest_length,
            compliance,
            0.0
        });

        return 1;
    }
    catch (...) {
        return 0;
    }
}

int soft_body_add_volume_constraint(
    SoftBodyHandle* body,
    int particle_a,
    int particle_b,
    int particle_c,
    int particle_d,
    double rest_volume,
    double compliance
) {
    if (!valid_particle_index(body, particle_a) ||
        !valid_particle_index(body, particle_b) ||
        !valid_particle_index(body, particle_c) ||
        !valid_particle_index(body, particle_d) ||
        particle_a == particle_b ||
        particle_a == particle_c ||
        particle_a == particle_d ||
        particle_b == particle_c ||
        particle_b == particle_d ||
        particle_c == particle_d ||
        !valid_number(rest_volume) ||
        !valid_number(compliance) ||
        std::abs(rest_volume) <= 1e-12 ||
        compliance < 0.0) {
        return 0;
    }

    try {
        body->solver.volume_constraints.push_back({
            static_cast<std::size_t>(particle_a),
            static_cast<std::size_t>(particle_b),
            static_cast<std::size_t>(particle_c),
            static_cast<std::size_t>(particle_d),
            rest_volume,
            compliance,
            0.0
        });

        return 1;
    }
    catch (...) {
        return 0;
    }
}

void soft_body_set_gravity(
    SoftBodyHandle* body,
    double x,
    double y,
    double z
) {
    if (!body ||
        !valid_number(x) ||
        !valid_number(y) ||
        !valid_number(z)) {
        return;
    }

    body->solver.gravity = {x, y, z};
}

void soft_body_set_iterations(
    SoftBodyHandle* body,
    int iterations
) {
    if (!body || iterations <= 0) {
        return;
    }

    body->solver.solver_iterations = iterations;
}

int soft_body_step(
    SoftBodyHandle* body,
    double dt
) {
    if (!body || !valid_number(dt) || dt <= 0.0) {
        return 0;
    }

    try {
        body->solver.step(dt);
        return 1;
    }
    catch (...) {
        return 0;
    }
}

int soft_body_get_particle_count(
    const SoftBodyHandle* body
) {
    if (!body) {
        return -1;
    }

    if (body->solver.particles.size() >
        static_cast<std::size_t>(
            std::numeric_limits<int>::max())) {
        return -1;
    }

    return static_cast<int>(
        body->solver.particles.size()
    );
}

int soft_body_get_particle_position(
    const SoftBodyHandle* body,
    int index,
    double* x,
    double* y,
    double* z
) {
    if (!valid_particle_index(body, index) ||
        !x || !y || !z) {
        return 0;
    }

    const auto& position =
        body->solver.particles[
            static_cast<std::size_t>(index)
        ].position;

    *x = position.x;
    *y = position.y;
    *z = position.z;

    return 1;
}

} // extern "C"
