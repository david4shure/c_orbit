#include "./propagation.h"
#include "corbit_math.h"
#include "../../src/utils/logger.h"

// overly simple method, TODO add actual numerical integration
PhysicalState apply_force_to(PhysicalState rv, DVector3 force, double delta_t) {
    // Validate mass
    if (rv.mass <= 0.0) {
        Debug("Error: Mass must be greater than zero.\n");
        return rv; // Exit early
    }

    // Calculate acceleration: a = F / m
    DVector3 acceleration;
    acceleration.x = force.x / rv.mass;
    acceleration.y = force.y / rv.mass;
    acceleration.z = force.z / rv.mass;

    // Update velocity: v_new = v_current + a * delta_t
    rv.v.x += acceleration.x * delta_t;
    rv.v.y += acceleration.y * delta_t;
    rv.v.z += acceleration.z * delta_t;

    // Update position: p_new = p_current + v_new * delta_t
    rv.r.x += rv.v.x * delta_t;
    rv.r.y += rv.v.y * delta_t;
    rv.r.z += rv.v.z * delta_t;

    return rv;
}

