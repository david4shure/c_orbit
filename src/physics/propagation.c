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

const double LOW_ECC_DISTANCE_THRESHOLD = 5000.0; // 5K KM 
const double HIGH_ECC_DISTANCE_THRESHOLD = 31000.0; // 20k KM
const int MAX_POINTS = 100000;

double clampd(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double delta_t_from_velocity(double velocityMagnitude, double scalingFactor) {
    return scalingFactor / (velocityMagnitude * velocityMagnitude);  // Time step inversely proportional to velocity cube
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
    void* darr = darray_init(10000,sizeof(DVector3));
    float delta_t = delta_t_from_velocity(fabs(DVector3Length(init_rv.v)), scalingFactor);

    PhysicalState rv_init = rv;

    darr = darray_push(darr, (void*)&rv_init.r);
    // Propagate backwards until we hit SOI
    float time = t;
    int num_points = 0;

    while (!hit_max_dist) {
        delta_t = delta_t_from_velocity(DVector3Length(rv.v), scalingFactor);
        rv = rv_from_r0v0(rv, -delta_t);
        float distance = fabs(DVector3Length(rv.r));
        hit_max_dist = distance > max_render_distance;
        darr = darray_insert_at(darr,(void*)&rv.r,0);
        time -= delta_t;
        num_points++;

        if (num_points > MAX_POINTS) {
            Warn("Exceeded max renderable points, max=%d, num_points=%d\n", MAX_POINTS, num_points);
            darray_free(darr);
            darr = darray_init(10, sizeof(PointBundle));
            break;
        }
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
        num_points++;
        if (num_points > MAX_POINTS) {
            Warn("Exceeded max renderable points, max=%d, num_points=%d\n", MAX_POINTS, num_points);
            darray_free(darr);
            darr = darray_init(10, sizeof(PointBundle));
            break;
        }
    }

    return darr;
}

double calculate_dynamic_delta_t(double r, double r_p, double r_a, double delta_t_min, double delta_t_max) {
    // Normalize the radial distance to the range [0, 1]
    double normalized_distance = (r - r_p) / (r_a - r_p);

    // Calculate dynamic delta_t based on normalized distance
    double delta_t = delta_t_min + (delta_t_max - delta_t_min) * normalized_distance;

    // Ensure delta_t stays within bounds
    if (delta_t > delta_t_max) delta_t = delta_t_max;
    if (delta_t < delta_t_min) delta_t = delta_t_min;

    return delta_t;
}

void* propagate_orbit_ellipse(OrbitalElements oe, PhysicalState rv, float t, float z_far) {
    double delta_t_min;
    double delta_t_max;

    // TODO: Improve this logic, this does not scale. I need to move on to other things 
    // but I will come back to this.
    if (oe.eccentricity <= 0.7) {
        delta_t_min = 1000.0;
        delta_t_max = 1000.0;
    } else if (oe.eccentricity <= 0.9) {
        // For moderate eccentricity, gently scale from 1000 to an intermediate value.
        double scale = (oe.eccentricity - 0.7) / (0.9 - 0.7);
        delta_t_min = 100.0 - scale * (100.0 - 500.0);   // Scale to a bit lower than 1000, like 500.
        delta_t_max = 1000.0 + scale * (50000.0 - 1000.0); // Scale up to 50000.
    } else {
        // For very high eccentricity, scale from intermediate values to the extremes.
        double scale = (oe.eccentricity - 0.9) / (0.999999 - 0.9);
        delta_t_min = 7000.0;
        delta_t_max = 2000000.0;
    }

    Debug("delta_t_min = %.2f\n",delta_t_min);
    Debug("delta_t_max = %.2f\n",delta_t_max);

    double total_true_anomaly = 0.0;
    double true_anomaly = oe.true_anomaly;
    double prev_true_anomaly = oe.true_anomaly;

    PhysicalState prev_rv = rv;
    void* darr = darray_init(10000, sizeof(PointBundle));
    float delta_t = 5000.0;
    float time = t;

    bool done = false;
    int num_points = 0;

    while (!done) {
        prev_rv = rv;
        prev_true_anomaly = true_anomaly;
        OrbitalElements elems = orb_elems_from_rv(rv, 0.0, 0.0);
        true_anomaly = elems.true_anomaly;

        double delta_true_anomaly = true_anomaly - prev_true_anomaly;
        if (delta_true_anomaly > M_PI) delta_true_anomaly -= 2 * M_PI;
        if (delta_true_anomaly < -M_PI) delta_true_anomaly += 2 * M_PI;
        total_true_anomaly += delta_true_anomaly;

        double r = DVector3Length(rv.r);
        double v = DVector3Length(rv.v);

        double r_p = oe.periapsis_distance;
        double r_a = oe.apoapsis_distance;
        delta_t = calculate_dynamic_delta_t(r, r_p, r_a, delta_t_min, delta_t_max);

        rv = rv_from_r0v0(rv, delta_t);

        PointBundle bun = { rv.r, time, oe.period + time };
        darr = darray_push(darr, (void*)&bun);
        num_points++;
        time += delta_t;

        // Stop after completing the orbit or exceeding allowed points
        if (total_true_anomaly >= 2 * D_PI - 1e-5) {  // Threshold to handle numerical drift
            done = true;
        }

        if (num_points > MAX_POINTS) {
            Warn("Exceeded max renderable points, max=%d, num_points=%d\n", MAX_POINTS, num_points);
            darray_free(darr);
            darr = darray_init(10, sizeof(PointBundle));
            break;
        }
    }

    return darr;
}

