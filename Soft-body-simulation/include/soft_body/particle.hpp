
#pragma once

#include "soft_body/vec3.hpp"

namespace soft_body {

struct Particle {
    // Trenutna pozicija cestice
    Vec3 position;

    // Pozicija prije simulacijskog koraka
    Vec3 previous_position;

    // Brzina cestice
    Vec3 velocity;

    // Inverzna masa: 1 / masa
    // Vrijednost 0 oznacava fiksiranu cesticu
    double inverse_mass = 1.0;
};

} // namespace soft_body
