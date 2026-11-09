
#include "soft_body/xpbd_solver.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>

using namespace soft_body;

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_gravity() {
    XPBDSolver solver;

    solver.particles.push_back({
        {0.0, 10.0, 0.0},
        {0.0, 10.0, 0.0},
        {0.0, 0.0, 0.0},
        1.0
    });

    const double dt = 0.1;
    solver.step(dt);

    const double expected_y = 10.0 - 9.81 * dt * dt;
    const double actual_y = solver.particles[0].position.y;

    check(std::abs(actual_y - expected_y) < 1e-9,
          "Gravity integration failed");

    std::cout << "[PASS] Gravity integration\n";
}

void test_distance_constraint() {
    XPBDSolver solver;
    solver.gravity = {0.0, 0.0, 0.0};

    solver.particles.push_back({
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        1.0
    });

    solver.particles.push_back({
        {2.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        1.0
    });

    solver.distance_constraints.push_back({
        0, 1, 1.0, 0.0, 0.0
    });

    solver.step(0.01);

    const double distance =
        (solver.particles[0].position -
         solver.particles[1].position).length();

    check(std::abs(distance - 1.0) < 1e-9,
          "Distance constraint failed");

    std::cout << "[PASS] Distance constraint\n";
}

void test_fixed_particle() {
    XPBDSolver solver;

    solver.particles.push_back({
        {0.0, 5.0, 0.0},
        {0.0, 5.0, 0.0},
        {0.0, 0.0, 0.0},
        0.0
    });

    solver.step(0.1);

    check(std::abs(solver.particles[0].position.y - 5.0)
              < 1e-9,
          "Fixed particle moved");

    std::cout << "[PASS] Fixed particle\n";
}

void test_numerical_stability() {
    XPBDSolver solver;

    solver.particles.push_back({
        {0.0, 10.0, 0.0},
        {0.0, 10.0, 0.0},
        {0.0, 0.0, 0.0},
        1.0
    });

    for (int i = 0; i < 1000; ++i) {
        solver.step(0.01);

        const auto& p = solver.particles[0];

        check(std::isfinite(p.position.x) &&
              std::isfinite(p.position.y) &&
              std::isfinite(p.position.z) &&
              std::isfinite(p.velocity.x) &&
              std::isfinite(p.velocity.y) &&
              std::isfinite(p.velocity.z),
              "Numerical stability failed");
    }

    std::cout << "[PASS] Numerical stability\n";
}


void test_volume_constraint() {
    XPBDSolver solver;
    solver.gravity = {0.0, 0.0, 0.0};

    solver.particles.push_back({
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        0.0
    });

    solver.particles.push_back({
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        0.0
    });

    solver.particles.push_back({
        {0.0, 1.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0},
        0.0
    });

    solver.particles.push_back({
        {0.0, 0.0, 2.0},
        {0.0, 0.0, 2.0},
        {0.0, 0.0, 0.0},
        1.0
    });

    const double rest_volume = 1.0 / 6.0;

    solver.volume_constraints.push_back({
        0, 1, 2, 3, rest_volume, 0.0, 0.0
    });

    solver.step(0.01);

    const Vec3& a = solver.particles[0].position;
    const Vec3& b = solver.particles[1].position;
    const Vec3& c = solver.particles[2].position;
    const Vec3& d = solver.particles[3].position;

    const double volume =
        (b - a).dot((c - a).cross(d - a)) / 6.0;

    check(std::abs(volume - rest_volume) < 1e-9,
          "Volume constraint failed");

    std::cout << "[PASS] Volume constraint\n";
}


void test_combined_constraints() {
    XPBDSolver solver;
    solver.gravity = {0.0, 0.0, 0.0};
    solver.solver_iterations = 50;

    // Tri fiksirane cestice i jedan pomjeren vrh.
    solver.particles = {
        {{0,0,0}, {0,0,0}, {0,0,0}, 0.0},
        {{1,0,0}, {1,0,0}, {0,0,0}, 0.0},
        {{0,1,0}, {0,1,0}, {0,0,0}, 0.0},
        {{0,0,1.5}, {0,0,1.5}, {0,0,0}, 1.0}
    };

    const double sqrt2 = std::sqrt(2.0);

    solver.distance_constraints = {
        {0,1,1.0,0.0,0.0},
        {0,2,1.0,0.0,0.0},
        {0,3,1.0,0.0,0.0},
        {1,2,sqrt2,0.0,0.0},
        {1,3,sqrt2,0.0,0.0},
        {2,3,sqrt2,0.0,0.0}
    };

    solver.volume_constraints.push_back({
        0,1,2,3,1.0/6.0,0.0,0.0
    });

    solver.step(0.01);

    const auto& p = solver.particles;
    double volume =
        (p[1].position - p[0].position)
        .dot((p[2].position - p[0].position)
        .cross(p[3].position - p[0].position)) / 6.0;

    check(std::abs(volume - 1.0/6.0) < 1e-5,
          "Combined volume constraint failed");

    check((p[3].position - p[0].position).length()
              > 0.0,
          "Invalid tetrahedron geometry");

    check(std::abs(
        (p[3].position - p[0].position).length()
        - 1.0) < 1e-4,
        "Combined distance constraint failed");

    std::cout << "[PASS] Combined constraints\n";
}

void test_compliance() {
    XPBDSolver rigid;
    XPBDSolver soft;

    rigid.gravity = {0,0,0};
    soft.gravity = {0,0,0};

    Particle a{{0,0,0},{0,0,0},{0,0,0},1.0};
    Particle b{{2,0,0},{2,0,0},{0,0,0},1.0};

    rigid.particles = {a,b};
    soft.particles = {a,b};

    rigid.distance_constraints.push_back({
        0,1,1.0,0.0,0.0
    });

    soft.distance_constraints.push_back({
        0,1,1.0,0.01,0.0
    });

    rigid.step(0.01);
    soft.step(0.01);

    double rigid_distance =
        (rigid.particles[0].position -
         rigid.particles[1].position).length();

    double soft_distance =
        (soft.particles[0].position -
         soft.particles[1].position).length();

    check(rigid_distance < soft_distance,
          "Compliance behavior failed");

    check(soft_distance < 2.0,
          "Soft constraint did not correct positions");

    std::cout << "[PASS] XPBD compliance\n";
}

void test_center_of_mass() {
    XPBDSolver solver;
    solver.gravity = {0,0,0};

    solver.particles = {
        {{0,0,0},{0,0,0},{0,0,0},1.0},
        {{2,0,0},{2,0,0},{0,0,0},1.0}
    };

    double before =
        (solver.particles[0].position.x +
         solver.particles[1].position.x) / 2.0;

    solver.distance_constraints.push_back({
        0,1,1.0,0.0,0.0
    });

    solver.step(0.01);

    double after =
        (solver.particles[0].position.x +
         solver.particles[1].position.x) / 2.0;

    check(std::abs(before - after) < 1e-9,
          "Center of mass changed");

    std::cout << "[PASS] Center of mass\n";
}

void test_multiple_steps() {
    XPBDSolver solver;
    solver.gravity = {0,0,0};

    solver.particles = {
        {{0,0,0},{0,0,0},{0,0,0},0.0},
        {{2,0,0},{2,0,0},{0,0,0},1.0}
    };

    solver.distance_constraints.push_back({
        0,1,1.0,0.0,0.0
    });

    for (int i = 0; i < 500; ++i) {
        solver.step(0.01);

        const auto& p = solver.particles[1];

        check(std::isfinite(p.position.x) &&
              std::isfinite(p.position.y) &&
              std::isfinite(p.position.z) &&
              std::isfinite(p.velocity.x) &&
              std::isfinite(p.velocity.y) &&
              std::isfinite(p.velocity.z),
              "Multiple steps produced invalid values");
    }

    double distance =
        (solver.particles[0].position -
         solver.particles[1].position).length();

    check(std::abs(distance - 1.0) < 1e-6,
          "Constraint drift over multiple steps");

    std::cout << "[PASS] Multiple simulation steps\n";
}

void test_invalid_timestep() {
    XPBDSolver solver;

    for (double dt : {0.0, -0.01,
                      std::nan(""),
                      std::numeric_limits<double>::infinity()}) {
        bool caught = false;

        try {
            solver.step(dt);
        }
        catch (const std::invalid_argument&) {
            caught = true;
        }

        check(caught, "Invalid timestep accepted");
    }

    std::cout << "[PASS] Invalid timestep\n";
}

void test_invalid_particle_index() {
    XPBDSolver solver;
    solver.gravity = {0,0,0};

    solver.particles.push_back({
        {0,0,0},{0,0,0},{0,0,0},1.0
    });

    solver.distance_constraints.push_back({
        0,99,1.0,0.0,0.0
    });

    bool caught = false;

    try {
        solver.step(0.01);
    }
    catch (const std::out_of_range&) {
        caught = true;
    }

    check(caught, "Invalid particle index accepted");

    std::cout << "[PASS] Invalid particle index\n";
}

void test_fixed_constraint_particles() {
    XPBDSolver solver;
    solver.gravity = {0,0,0};

    solver.particles = {
        {{0,0,0},{0,0,0},{0,0,0},0.0},
        {{2,0,0},{2,0,0},{0,0,0},1.0}
    };

    solver.distance_constraints.push_back({
        0,1,1.0,0.0,0.0
    });

    solver.step(0.01);

    check(solver.particles[0].position.x == 0.0,
          "Fixed particle moved under constraint");

    check(std::abs(
        solver.particles[1].position.x - 1.0) < 1e-9,
        "Movable particle was not corrected");

    std::cout << "[PASS] Fixed constraint particles\n";
}




int main() {
    try {
        std::cout << "=== XPBD SERIAL SMOKE TEST ===\n";

        test_gravity();
        test_distance_constraint();
        test_volume_constraint();
        test_fixed_particle();
        test_numerical_stability();

        test_combined_constraints();
        test_compliance();
        test_center_of_mass();
        test_multiple_steps();
        test_invalid_timestep();
        test_invalid_particle_index();
        test_fixed_constraint_particles();

        std::cout << "\nAll 12 tests passed!\n";
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[FAIL] " << e.what() << '\n';
        return 1;
    }
}

