
#include "soft_body/xpbd_solver.hpp"

#include <cmath>
#include <stdexcept>

namespace soft_body {

void XPBDSolver::predict_positions(double dt) {
    for (auto& p : particles) {
        p.previous_position = p.position;

        if (p.inverse_mass == 0.0)
            continue;

        p.velocity = p.velocity + gravity * dt;
        p.position = p.position + p.velocity * dt;
    }
}

void XPBDSolver::solve_distance_constraints(double dt) {
    constexpr double epsilon = 1e-12;

    for (auto& c : distance_constraints) {
        if (c.particle_a >= particles.size() ||
            c.particle_b >= particles.size()) {
            throw std::out_of_range("Invalid particle index");
        }

        Particle& a = particles[c.particle_a];
        Particle& b = particles[c.particle_b];

        Vec3 delta = a.position - b.position;
        double length = delta.length();

        if (length < epsilon)
            continue;

        double C = length - c.rest_length;

        Vec3 gradient = delta / length;

        double w = a.inverse_mass + b.inverse_mass;

        double alpha = c.compliance / (dt * dt);

        double denominator = w + alpha;

        if (denominator < epsilon)
            continue;

        double delta_lambda =
            (-C - alpha * c.lambda) / denominator;

        c.lambda += delta_lambda;

        a.position = a.position
                   + gradient * (a.inverse_mass * delta_lambda);

        b.position = b.position
                   - gradient * (b.inverse_mass * delta_lambda);
    }
}



void XPBDSolver::solve_volume_constraints(double dt) {
    constexpr double epsilon = 1e-12;

    for (auto& c : volume_constraints) {
        if (c.particle_a >= particles.size() ||
            c.particle_b >= particles.size() ||
            c.particle_c >= particles.size() ||
            c.particle_d >= particles.size()) {
            throw std::out_of_range("Invalid particle index");
        }

        Particle& p0 = particles[c.particle_a];
        Particle& p1 = particles[c.particle_b];
        Particle& p2 = particles[c.particle_c];
        Particle& p3 = particles[c.particle_d];

        Vec3 e1 = p1.position - p0.position;
        Vec3 e2 = p2.position - p0.position;
        Vec3 e3 = p3.position - p0.position;

        double volume = e1.dot(e2.cross(e3)) / 6.0;
        double C = volume - c.rest_volume;

        Vec3 g1 = e2.cross(e3) / 6.0;
        Vec3 g2 = e3.cross(e1) / 6.0;
        Vec3 g3 = e1.cross(e2) / 6.0;
        Vec3 g0 = (g1 + g2 + g3) * (-1.0);

        double w0 = p0.inverse_mass;
        double w1 = p1.inverse_mass;
        double w2 = p2.inverse_mass;
        double w3 = p3.inverse_mass;

        double sum =
            w0 * g0.dot(g0) +
            w1 * g1.dot(g1) +
            w2 * g2.dot(g2) +
            w3 * g3.dot(g3);

        double alpha = c.compliance / (dt * dt);
        double denominator = sum + alpha;

        if (denominator < epsilon)
            continue;

        double delta_lambda =
            (-C - alpha * c.lambda) / denominator;

        c.lambda += delta_lambda;

        p0.position = p0.position + g0 * (w0 * delta_lambda);
        p1.position = p1.position + g1 * (w1 * delta_lambda);
        p2.position = p2.position + g2 * (w2 * delta_lambda);
        p3.position = p3.position + g3 * (w3 * delta_lambda);
    }
}


void XPBDSolver::update_velocities(double dt) {
    for (auto& p : particles) {
        if (p.inverse_mass == 0.0) {
            p.velocity = {0.0, 0.0, 0.0};
            continue;
        }

        p.velocity =
            (p.position - p.previous_position) / dt;
    }
}


void XPBDSolver::step(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::invalid_argument(
            "Time step must be positive and finite"
        );
    }

    for (auto& c : distance_constraints) {
        c.lambda = 0.0;
    }

    for (auto& c : volume_constraints) {
        c.lambda = 0.0;
    }

    predict_positions(dt);

    for (int i = 0; i < solver_iterations; ++i) {
        solve_distance_constraints(dt);
        solve_volume_constraints(dt);
    }

    update_velocities(dt);
}


} // namespace soft_body
