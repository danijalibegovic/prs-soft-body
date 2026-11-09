
#include "soft_body/soft_body_api.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

int main() {
    SoftBodyHandle* body = nullptr;

    try {
        std::cout << "=== XPBD DLL SMOKE TEST ===\n";

        body = soft_body_create();
        check(body != nullptr, "Failed to create simulation");

        std::cout << "[PASS] Create simulation\n";

        soft_body_set_gravity(body, 0.0, 0.0, 0.0);
        soft_body_set_iterations(body, 20);

        int a = soft_body_add_particle(
            body, 0.0, 0.0, 0.0, 0.0
        );

        int b = soft_body_add_particle(
            body, 2.0, 0.0, 0.0, 1.0
        );

        check(a == 0 && b == 1,
              "Failed to add particles");

        check(soft_body_get_particle_count(body) == 2,
              "Incorrect particle count");

        std::cout << "[PASS] Add particles\n";

        check(
            soft_body_add_distance_constraint(
                body, a, b, 1.0, 0.0
            ) == 1,
            "Failed to add distance constraint"
        );

        std::cout << "[PASS] Add distance constraint\n";

        check(soft_body_step(body, 0.01) == 1,
              "Simulation step failed");

        std::cout << "[PASS] Simulation step\n";

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        check(
            soft_body_get_particle_position(
                body, b, &x, &y, &z
            ) == 1,
            "Failed to read particle position"
        );

        check(std::abs(x - 1.0) < 1e-9,
              "Incorrect XPBD position");

        std::cout << "[PASS] Read corrected position\n";

        check(
            soft_body_add_distance_constraint(
                body, 0, 99, 1.0, 0.0
            ) == 0,
            "Invalid index was accepted"
        );

        check(soft_body_step(body, -0.01) == 0,
              "Invalid timestep was accepted");

        std::cout << "[PASS] Invalid input handling\n";

        
        SoftBodyHandle* volume_body = soft_body_create();

        check(volume_body != nullptr,
            "Failed to create volume simulation");

        soft_body_set_gravity(volume_body, 0.0, 0.0, 0.0);

        int p0 = soft_body_add_particle(
            volume_body, 0.0, 0.0, 0.0, 0.0);

        int p1 = soft_body_add_particle(
            volume_body, 1.0, 0.0, 0.0, 0.0);

        int p2 = soft_body_add_particle(
            volume_body, 0.0, 1.0, 0.0, 0.0);

        int p3 = soft_body_add_particle(
            volume_body, 0.0, 0.0, 2.0, 1.0);

        check(p0 == 0 && p1 == 1 &&
            p2 == 2 && p3 == 3,
            "Failed to add volume particles");

        check(
            soft_body_add_volume_constraint(
                volume_body,
                p0, p1, p2, p3,
                1.0 / 6.0,
                0.0
            ) == 1,
            "Failed to add volume constraint"
        );

        check(soft_body_step(volume_body, 0.01) == 1,
            "Volume simulation step failed");

        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;

        check(
            soft_body_get_particle_position(
                volume_body, p3, &vx, &vy, &vz
            ) == 1,
            "Failed to read volume particle position"
        );

        check(std::abs(vz - 1.0) < 1e-9,
            "Volume constraint produced incorrect position");

        soft_body_destroy(volume_body);

        std::cout << "[PASS] DLL volume constraint\n";


        soft_body_destroy(body);
        body = nullptr;

        std::cout << "[PASS] Destroy simulation\n";
        std::cout << "\nAll 8 DLL tests passed!\n";

        return 0;
    }
    catch (const std::exception& e) {
        soft_body_destroy(body);

        std::cerr << "[FAIL] " << e.what() << '\n';
        return 1;
    }
}
