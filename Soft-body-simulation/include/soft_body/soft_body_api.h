
#pragma once

#include <stddef.h>

#ifdef _WIN32
    #ifdef SOFT_BODY_DLL_EXPORTS
        #define SOFT_BODY_API __declspec(dllexport)
    #else
        #define SOFT_BODY_API __declspec(dllimport)
    #endif
#else
    #define SOFT_BODY_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SoftBodyHandle SoftBodyHandle;

// Kreiranje i unistavanje simulacije
SOFT_BODY_API SoftBodyHandle* soft_body_create(void);
SOFT_BODY_API void soft_body_destroy(SoftBodyHandle* body);

// Dodavanje cestica
// Vraca indeks cestice ili -1 ako operacija nije uspjela.
SOFT_BODY_API int soft_body_add_particle(
    SoftBodyHandle* body,
    double x,
    double y,
    double z,
    double inverse_mass
);

// Dodavanje distance constrainta
SOFT_BODY_API int soft_body_add_distance_constraint(
    SoftBodyHandle* body,
    int particle_a,
    int particle_b,
    double rest_length,
    double compliance
);

// Dodavanje volume constrainta
SOFT_BODY_API int soft_body_add_volume_constraint(
    SoftBodyHandle* body,
    int particle_a,
    int particle_b,
    int particle_c,
    int particle_d,
    double rest_volume,
    double compliance
);

// Postavke simulacije
SOFT_BODY_API void soft_body_set_gravity(
    SoftBodyHandle* body,
    double x,
    double y,
    double z
);

SOFT_BODY_API void soft_body_set_iterations(
    SoftBodyHandle* body,
    int iterations
);

// Izvrsavanje jednog simulacijskog koraka
// Vraca 1 za uspjeh, 0 za gresku.
SOFT_BODY_API int soft_body_step(
    SoftBodyHandle* body,
    double dt
);

// Citanje rezultata simulacije
SOFT_BODY_API int soft_body_get_particle_count(
    const SoftBodyHandle* body
);

SOFT_BODY_API int soft_body_get_particle_position(
    const SoftBodyHandle* body,
    int index,
    double* x,
    double* y,
    double* z
);

#ifdef __cplusplus
}
#endif
