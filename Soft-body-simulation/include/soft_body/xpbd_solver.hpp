
#pragma once

#include <vector>
#include <cstddef>
#include "soft_body/particle.hpp"

namespace soft_body {

struct DistanceConstraint {
    std::size_t particle_a;
    std::size_t particle_b;

    double rest_length;
    double compliance = 0.0;
    double lambda = 0.0;
};



struct VolumeConstraint {
    std::size_t particle_a;
    std::size_t particle_b;
    std::size_t particle_c;
    std::size_t particle_d;

    double rest_volume;
    double compliance = 0.0;
    double lambda = 0.0;
};


class XPBDSolver {
public:
    std::vector<Particle> particles;
    std::vector<DistanceConstraint> distance_constraints;
    std::vector<VolumeConstraint> volume_constraints;

    Vec3 gravity{0.0, -9.81, 0.0};

    int solver_iterations = 10;

    void step(double dt);

    // Exposed for adapters that need to insert collision handling between
    // prediction, constraint solving, and velocity integration.
    void predict_positions(double dt);
    void solve_distance_constraints(double dt);
    void update_velocities(double dt);
    void solve_volume_constraints(double dt);
};

} // namespace soft_body
