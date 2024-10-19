#include "constants.h"
#include "kepler.h"
#include "propagation.h"
#include "../utils/darray.h"
#include "../utils/logger.h"
#include "assert.h"
#include "raymath.h"
#include <stdint.h>
#include <time.h>
#include <stdbool.h>

// This function propagates an orbit and figures
// out how to do so based on its orbit type (ellipse,parabola,hyperbola etc)
// NOTE: This function returns a dynamic array that is malloc'ed!!
// It should be freed accordingly
void* propagate_orbit(PhysicalState rv,float t,float M_naught, float t_naught) {
    OrbitalElements oe = orb_elems_from_rv(rv,M_naught,t_naught);

    if (oe.eccentricity < 1.0) {
        return propagate_orbit_ellipse(oe,rv,t);
    } else if (oe.eccentricity >= 1.0) {
        return propagate_orbit_non_ellipse(oe,rv,t);
    } else {
        Fatal("Parabola???");
    }
}

void* propagate_orbit_non_ellipse(OrbitalElements oe, PhysicalState rv, float t) {
    float r_at_sphere_of_influence = calculate_sphere_of_influence_r(EARTH_SEMIMAJOR_AXIS_KM, oe.mass_of_parent, oe.mass_of_grandparent);

    // Go in the negative direction until we hit SOI
    bool hit_soi = false;
    float time_increment = 1.0;

    void* darr = darray_init(1000,sizeof(Vector3));

    PhysicalState rv_init = rv;

    // Propagate backwards until we hit SOI
    float time = t;
    while (!hit_soi) {
        rv = rv_from_r0v0(rv, time);

        if (Vector3Length(rv.r) >= r_at_sphere_of_influence) {
            Debug("Broke on backward propagation, soi = %.2f, len(r) = %.2f, t = %.2f\n",r_at_sphere_of_influence,rv.r,time);
            break;
        }

        darr = darray_insert_at(darr,(void*)&rv,0);
        time -= time_increment;
    }

    hit_soi = false;

    // Propagate forwards until we hit SOI
    time = t;
    rv = rv_init; 
    while (!hit_soi) {
        rv = rv_from_r0v0(rv, time);

        if (Vector3Length(rv.r) >= r_at_sphere_of_influence) {
            Debug("Broke on forward propagation, soi = %.2f, len(r) = %.2f, t = %.2f\n",r_at_sphere_of_influence,rv.r,time);
            break;
        }

        darr = darray_push(darr, (void*)&rv);
        time += time_increment;
    }

    Warn("Length of darr in propagator = %d\n",darray_length(darr));

    return darr;
}

void* propagate_orbit_ellipse(OrbitalElements oe, PhysicalState rv, float t) {
    float time_increment = 1.0;

    void* darr = darray_init(1000,sizeof(Vector3));

    PhysicalState rv_init = rv;

    // Propagate backwards until we hit SOI
    float time = t;
    for (int t = time; t < 10000.0; t += time_increment) {
        rv = rv_from_r0v0(rv, time);

        darr = darray_push(darr,(void*)&rv);
    }

    return darr;
}

// True anomaly goes from 0 -> 2 PI, I want it to go from -PI to +PI
float normalize_true_anomaly(float true_anomaly) {
    // -180 -> 0
    // 0 -> 180
    if (true_anomaly > PI) {
        return PI - true_anomaly;
    } else {
        return true_anomaly;
    }
}
