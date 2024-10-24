#include "propagation.h"
#include "../utils/darray.h"
#include "../utils/logger.h"
#include "assert.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "constants.h"
#include "corbit_math.h"
#include "kepler.h"

const double LOW_ECC_DISTANCE_THRESHOLD = 1000.0; // 5K KM 
const double HIGH_ECC_DISTANCE_THRESHOLD = 10000.0; // 100k KM

double clampd(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double delta_t_from_velocity(double velocityMagnitude, double scalingFactor) {
    return scalingFactor / (velocityMagnitude);  // Time step inversely proportional to velocity cube
}

double delta_t_from_dist_from_periapsis(double distance_from_periapsis, double scalingFactor, double min, double max) {
    return clampd(scalingFactor / (distance_from_periapsis),min,max);  // Time step inversely proportional to velocity cube }
}

// This function propagates an orbit and figures
// out how to do so based on its orbit type (ellipse,parabola,hyperbola etc)
// NOTE: This function returns a dynamic array that is malloc'ed!!
// It should be freed accordingly
void* propagate_orbit(PhysicalState rv,float t,float M_naught, float t_naught, float max_render_distance) {
    OrbitalElements oe = orb_elems_from_rv(rv,M_naught,t_naught);

    if (oe.eccentricity < 1.0) { // Ellipse
        return propagate_orbit_ellipse(oe,rv,t,max_render_distance);
    } else { // Parabolas / Hyperbolas
        return propagate_orbit_non_ellipse(oe,rv,t,max_render_distance);
    }
}

void* propagate_orbit_non_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_render_distance) {
    // Go in the negative direction until we hit SOI
    PhysicalState init_rv = rv;
    bool hit_max_dist = false;
    double scalingFactor = 500.0;
    void* darr = darray_init(1000,sizeof(DVector3));
    float delta_t = delta_t_from_velocity(fabs(DVector3Length(init_rv.v)), scalingFactor);

    PhysicalState rv_init = rv;

    darr = darray_push(darr, (void*)&rv_init.r);
    // Propagate backwards until we hit SOI
    float time = t;
    while (!hit_max_dist) {
        delta_t = delta_t_from_velocity(DVector3Length(rv.v), scalingFactor);
        rv = rv_from_r0v0(rv, -delta_t);
        float distance = fabs(DVector3Length(rv.r));
        hit_max_dist = distance > max_render_distance;
        darr = darray_insert_at(darr,(void*)&rv.r,0);
        time -= delta_t;
    }

    hit_max_dist = false;

    // Propagate forwards until we hit SOI
    time = t;
    rv = rv_init; 
    darr = darray_push(darr, (void*)&rv_init.r);
    delta_t = delta_t_from_velocity(DVector3Length(init_rv.v), scalingFactor);

    while (!hit_max_dist) {
        delta_t = delta_t_from_velocity(DVector3Length(rv.v), scalingFactor);
        rv = rv_from_r0v0(rv, delta_t);
        float distance = fabs(DVector3Length(rv.r));
        hit_max_dist = distance > max_render_distance;
        darr = darray_push(darr, (void*)&rv.r);
        time += delta_t;
    }

    Debug("done forward propagating\n");
    Warn("Length of darr in propagator = %d\n",darray_length(darr));

    return darr;
}

void* propagate_orbit_ellipse(OrbitalElements oe, PhysicalState rv, float t, float z_far) {
    void* darr = darray_init(1000, sizeof(DVector3));

    float delta_t;
    PhysicalState rv_prev = rv;
    PhysicalState init_rv = rv;
    darr = darray_push(darr, (void*)&rv.r);  // Store initial point
    double distances[3] = {0.0,0.0,0.0};
    int num_happenings = 0;

    // Sample 3 points, two behind, one current
    // Check if the distance between the point and the start of where we propagated closed in.
    // ie [10km away, 5km away, 10km away] <- we know we iterated past where we started propagating if we see something like this
    while (true) {
        // Handles distances
        // Propagate to the next point
        rv = rv_from_r0v0(rv_prev, delta_t);

        // Check distance between points
        double dist = DVector3Distance(rv_prev.r, rv.r);

        // If distance too large, halve delta_t and recalculate
        while (dist > LOW_ECC_DISTANCE_THRESHOLD) {
            delta_t /= 2;
            rv = rv_from_r0v0(rv_prev, delta_t);
            dist = DVector3Distance(rv_prev.r, rv.r);
        }

        // If distance is acceptable, store the point
        darr = darray_push(darr, (void*)&rv.r);

        // Reset delta_t to constant for the next iteration
        delta_t = 5000;

        // Have we gone past the physical position where we started propagaing from?
        if (distances[0] > distances[1] && distances[2] > distances[1]) {
            num_happenings++;
            break;
        }

        // Shift "distance from initial r point" items to the left
        distances[0] = distances[1];
        distances[1] = distances[2];
        distances[2] = DVector3Distance(init_rv.r, rv.r);

        // Update previous state
        rv_prev = rv;
    }

    // Final point
    darr = darray_push(darr, (void*)&rv_prev.r);

    // Return or use the points as needed
    // darray_free(darr);
    return darr;
}

