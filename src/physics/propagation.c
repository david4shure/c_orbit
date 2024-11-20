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
    Debug("Applying the following force, (%.4f, %.4f, %.4f)\n", force.x, force.y, force.z);
    DVector3 acceleration;
    acceleration.x = force.x / rv.mass;
    acceleration.y = force.y / rv.mass;
    acceleration.z = force.z / rv.mass;

    Debug("Applying the following acceleration, (%.4f, %.4f, %.4f)\n",
          acceleration.x, acceleration.y, acceleration.z);

    // Update velocity: v_new = v_current + a * delta_t
    rv.v.x += acceleration.x * delta_t;
    rv.v.y += acceleration.y * delta_t;
    rv.v.z += acceleration.z * delta_t;

    // Update position: p_new = p_current + v_new * delta_t
    rv.r.x += rv.v.x * delta_t;
    rv.r.y += rv.v.y * delta_t;
    rv.r.z += rv.v.z * delta_t;

    Debug("Updated state: Velocity = (%.4f, %.4f, %.4f), Position = (%.4f, %.4f, %.4f)\n",
          rv.v.x, rv.v.y, rv.v.z, rv.r.x, rv.r.y, rv.r.z);

    return rv;
}

