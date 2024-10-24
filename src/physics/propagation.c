#include "propagation.h"
#include "../utils/darray.h"
#include "../utils/logger.h"
#include "assert.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "constants.h"
#include "kepler.h"

const double THRESHOLD = 10000.0;

double clampd(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double compute_delta_t(double velocityMagnitude, double scalingFactor, double max, double min) {
    Debug("delta_t = %.2f\n",scalingFactor/velocityMagnitude);
    float eta = 10000.0;
    return clampd(scalingFactor / (velocityMagnitude),min,max);  // Time step inversely proportional to velocity cube }
}
/* double compute_delta_t(OrbitalElements oe, double k) { */
/*     // Calculate the radial distance r using semi-major axis (a), eccentricity (e), and true anomaly (nu) */
/*     double r = (oe.semimajor_axis * (1 - oe.eccentricity * oe.eccentricity)) / */ 
/*                (1 + oe.eccentricity * cos(oe.true_anomaly)); */

/*     // Use the Vis-Viva equation to compute the orbital velocity */
/*     double v = sqrt(oe.grav_param * ((2.0 / r) - (1.0 / oe.semimajor_axis))); */

/*     // Introduce clamping for extreme values */
/*     double MAX_RADIUS = 100000.0;   // Upper bound for distance */
/*     double MIN_VELOCITY = 0.1;      // Lower bound for velocity */

/*     if (r > MAX_RADIUS) { */
/*         r = MAX_RADIUS; */
/*     } */
/*     if (v < MIN_VELOCITY) { */
/*         v = MIN_VELOCITY; */
/*     } */

/*     // Compute adaptive deltaT based on r and v */
/*     double delta_t = k * (r / v); */

/*     // Log r, v, and deltaT to debug */
/*     /1* Debug("r = %.2f, v = %.2f, deltaT = %.2f\n", r, v, delta_t); *1/ */

/*     // Clamp deltaT if necessary */
/*     if (delta_t > 50000.0) {  // Example clamping */
/*         delta_t = 50000.0; */
/*         /1* Debug("Clamped deltaT to 1000.0\n"); *1/ */
/*     } */

/*     return delta_t; */
/* } */

// This function propagates an orbit and figures
// out how to do so based on its orbit type (ellipse,parabola,hyperbola etc)
// NOTE: This function returns a dynamic array that is malloc'ed!!
// It should be freed accordingly
void* propagate_orbit(PhysicalState rv,float t,float M_naught, float t_naught, float max_render_distance) {
    OrbitalElements oe = orb_elems_from_rv(rv,M_naught,t_naught);

    if (oe.eccentricity < 0.9) { // Ellipse
        return propagate_orbit_low_ecc_ellipse(oe,rv,t, max_render_distance);
    } else if (oe.eccentricity < 1.0) {
        return propagate_orbit_high_ecc_ellipse(oe,rv,t, max_render_distance);
    } else { // Parabolas / Hyperbolas
        return propagate_orbit_non_ellipse(oe,rv,t,max_render_distance);
    }
}

void* propagate_orbit_non_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_render_distance) {
    // Go in the negative direction until we hit SOI
    PhysicalState init_rv = rv;
    bool hit_max_dist = false;
    double scalingFactor = 500000.0;
    double min = 10000;
    double max = 10000000;
    void* darr = darray_init(1000,sizeof(DVector3));
    float delta_t = compute_delta_t(fabs(DVector3Length(init_rv.v)), scalingFactor, min,max);

    PhysicalState rv_init = rv;

    darr = darray_push(darr, (void*)&rv_init.r);
    // Propagate backwards until we hit SOI
    float time = t;
    while (!hit_max_dist) {
        delta_t = compute_delta_t(fabs(DVector3Length(rv.v)), scalingFactor,min,max);
        rv = rv_from_r0v0(rv, -delta_t);
        float distance = fabs(DVector3Length(rv.r));
        hit_max_dist = distance > max_render_distance;
        darr = darray_insert_at(darr,(void*)&rv.r,0);
        time -= delta_t;
    }
    Debug("done back propagating\n");

    hit_max_dist = false;

    // Propagate forwards until we hit SOI
    time = t;
    rv = rv_init; 
    darr = darray_push(darr, (void*)&rv_init.r);
    delta_t = compute_delta_t(fabs(DVector3Length(init_rv.v)), scalingFactor,min,max);

    while (!hit_max_dist) {
        delta_t = compute_delta_t(fabs(DVector3Length(rv.v)), scalingFactor,min,max);
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

void* propagate_orbit_high_ecc_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_render_distance) {
    PhysicalState init_rv = rv;
    void* darr = darray_init(1000,sizeof(DVector3));
    Debug("arr len = %d\n",darray_length(darr));
    float total = 0.0;
    double scalingFactor = 80000.0;
    double min = 10000;
    double max = 10000000;

    double constant_delta_t = compute_delta_t(DVector3Length(rv.v), scalingFactor, min, max);
    float deltaT = constant_delta_t;
    PhysicalState rv_init = rv;
    darr = darray_push(darr, (void*)&rv.r); // Store the initial position

    while (total <= oe.period + oe.period / 4.0) {
        float deltaT = constant_delta_t;  // Reset to constant delta_t
        PhysicalState prev_rv = rv;       // Save the previous state

        // Propagate using the current deltaT
        rv = rv_from_r0v0(rv, deltaT);

        // Calculate distance between points
        double dist = DVector3Distance(prev_rv.r, rv.r);

        // If distance is too large, use bisection approach to refine deltaT
        while (dist > THRESHOLD) {
            deltaT /= 2.0;  // Halve deltaT
            rv = rv_from_r0v0(prev_rv, deltaT);  // Recalculate the position with the halved deltaT
            dist = DVector3Distance(prev_rv.r, rv.r);  // Recheck the distance
        }

        // Update total time and save the propagated point
        total += deltaT;
        darr = darray_push(darr, (void*)&rv.r);  // Save the new position
    }

    Debug("r=(%.2f,%.2f,%.2f)\n", rv.r.x, rv.r.y, rv.r.z);
    Debug("v=(%.2f,%.2f,%.2f)\n", rv.v.x, rv.v.y, rv.v.z);
    Debug("Finished propagating orbit, total = %.2f, period = %.2f\n", total, oe.period);

    darr = darray_push(darr, (void*)&init_rv.r);  // Optionally add the initial point again at the end

    return darr;
}


/* void* propagate_orbit_high_ecc_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_render_distance) { */
/*     PhysicalState init_rv = rv; */
/*     void* darr = darray_init(1000,sizeof(DVector3)); */
/*     Debug("arr len = %d\n",darray_length(darr)); */
/*     float total = 0.0; */
/*     double scalingFactor = 80000.0; */

/*     double min = 10000; */
/*     double max = 10000000; */
    
/*     PhysicalState rv_init = rv; */

/*     double speed = DVector3Length(rv.v); */ 
/*     float deltaT = compute_delta_t(DVector3Length(rv.v),scalingFactor,min,max); */

/*     while(total <= oe.period + oe.period/4.0) { */
/*         float deltaT = compute_delta_t(DVector3Length(rv.v),scalingFactor,min,max); */
/*         total += deltaT; */

/*         rv = rv_from_r0v0(rv, deltaT); */
/*         OrbitalElements oe = orb_elems_from_rv(rv, 0.0, 0.0); */

/*         darr = darray_push(darr,(void*)&rv.r); */
/*     } */

/*     Debug("r=(%.2f,%.2f,%.2f)\n",rv.r.x,rv.r.y,rv.r.z); */
/*     Debug("v=(%.2f,%.2f,%.2f)\n",rv.v.x,rv.v.y,rv.v.z); */
/*     Debug("Finished propagating orbit, total = %.2f, period = %.2f\n",total,oe.period); */

/*     darr = darray_push(darr,(void*)&init_rv); */

/*     return darr; */
/* } */

/* void* propagate_orbit_low_ecc_ellipse(OrbitalElements oe, PhysicalState rv, float t, float z_far) { */
/*     PhysicalState init_rv = rv; */
/*     void* darr = darray_init(1000,sizeof(DVector3)); */
/*     Debug("arr len = %d\n",darray_length(darr)); */
/*     float total = 0.0; */
/*     double scalingFactor = 80.0; */
/*     double min = 10; */
/*     double max = 1000; */

/*     PhysicalState rv_init = rv; */

/*     double speed = DVector3Length(rv.v); */ 
/*     /1* float deltaT = compute_delta_t(DVector3Length(rv.v),scalingFactor,min,max); *1/ */
/*     float deltaT = 500.0; */

/*     while(total < oe.period) { */
/*         /1* float deltaT = compute_delta_t(DVector3Length(rv.v),scalingFactor,min,max); *1/ */
/*         float deltaT = 500.0; */
/*         total += deltaT; */

/*         rv = rv_from_r0v0(rv, deltaT); */
/*         OrbitalElements oe = orb_elems_from_rv(rv, 0.0, 0.0); */

/*         darr = darray_push(darr,(void*)&rv.r); */
/*     } */

/*     Debug("r=(%.2f,%.2f,%.2f)\n",rv.r.x,rv.r.y,rv.r.z); */
/*     Debug("v=(%.2f,%.2f,%.2f)\n",rv.v.x,rv.v.y,rv.v.z); */
/*     Debug("Finished propagating orbit, total = %.2f, period = %.2f\n",total,oe.period); */

/*     darr = darray_push(darr,(void*)&init_rv); */

/*     return darr; */
/* } */


void* propagate_orbit_low_ecc_ellipse(OrbitalElements oe, PhysicalState rv, float t, float z_far) {
    void* darr = darray_init(1000, sizeof(DVector3));

    float delta_t = 50000.0;  // Start with a constant delta t
    PhysicalState rv_prev = rv;
    darr = darray_push(darr, (void*)&rv.r);  // Store initial point

    while (t < z_far) {
        // Propagate to the next point
        rv = rv_from_r0v0(rv_prev, delta_t);

        // Check distance between points
        double dist = DVector3Distance(rv_prev.r, rv.r);

        // If distance too large, halve delta_t and recalculate
        while (dist > THRESHOLD) {
            delta_t /= 2;
            rv = rv_from_r0v0(rv_prev, delta_t);
            dist = DVector3Distance(rv_prev.r, rv.r);
        }

        // If distance is acceptable, store the point
        darr = darray_push(darr, (void*)&rv.r);

        // Reset delta_t to constant for the next iteration
        delta_t = 5000;

        // Update previous state
        rv_prev = rv;

        // Increment time
        t += delta_t;
    }

    // Final point
    darr = darray_push(darr, (void*)&rv_prev.r);

    // Return or use the points as needed
    // darray_free(darr);
    return darr;
}


// True anomaly goes from 0 -> 2 PI, I want it to go from -PI to +PI
float normalize_true_anomaly(float true_anomaly) {
    // -180 -> 0
    // 0 -> 180
    if (true_anomaly > D_PI) {
        return D_PI - true_anomaly;
    } else {
        return true_anomaly;
    }
}
