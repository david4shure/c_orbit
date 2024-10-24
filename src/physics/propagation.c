#include "kepler.h"
#include "propagation.h"
#include "../utils/darray.h"
#include "../utils/logger.h"
#include "assert.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "constants.h"

double _computeDeltaT(double velocityMagnitude, double scalingFactor) {
    return scalingFactor / (velocityMagnitude);  // Time step inversely proportional to velocity cube
}

double compute_delta_t(OrbitalElements oe, double k) {
    // Calculate the radial distance r using semi-major axis (a), eccentricity (e), and true anomaly (nu)
    double r = (oe.semimajor_axis * (1 - oe.eccentricity * oe.eccentricity)) / 
               (1 + oe.eccentricity * cos(oe.true_anomaly));

    // Use the Vis-Viva equation to compute the orbital velocity
    double v = sqrt(oe.grav_param * ((2.0 / r) - (1.0 / oe.semimajor_axis)));

    // Introduce clamping for extreme values
    double MAX_RADIUS = 100000.0;   // Upper bound for distance
    double MIN_VELOCITY = 0.1;      // Lower bound for velocity

    if (r > MAX_RADIUS) {
        r = MAX_RADIUS;
    }
    if (v < MIN_VELOCITY) {
        v = MIN_VELOCITY;
    }

    // Compute adaptive deltaT based on r and v
    double delta_t = k * (r / v);

    // Log r, v, and deltaT to debug
    /* Debug("r = %.2f, v = %.2f, deltaT = %.2f\n", r, v, delta_t); */

    // Clamp deltaT if necessary
    if (delta_t > 1000.0) {  // Example clamping
        delta_t = 1000.0;
        /* Debug("Clamped deltaT to 1000.0\n"); */
    }

    return delta_t;
}

// This function propagates an orbit and figures
// out how to do so based on its orbit type (ellipse,parabola,hyperbola etc)
// NOTE: This function returns a dynamic array that is malloc'ed!!
// It should be freed accordingly
void* propagate_orbit(PhysicalState rv,float t,float M_naught, float t_naught, float max_render_distance) {
    OrbitalElements oe = orb_elems_from_rv(rv,M_naught,t_naught);

    if (oe.eccentricity < 1.0) { // Ellipse
        return propagate_orbit_ellipse(oe,rv,t);
    } else { // Parabolas / Hyperbolas
        return propagate_orbit_non_ellipse(oe,rv,t,max_render_distance);
    }
}

void* propagate_orbit_non_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_render_distance) {
    // Go in the negative direction until we hit SOI
    PhysicalState init_rv = rv;
    bool hit_max_dist = false;
    double scalingFactor = 20000.0;
    void* darr = darray_init(1000,sizeof(DVector3));
    float delta_t = _computeDeltaT(fabs(DVector3Length(init_rv.v)), scalingFactor);

    PhysicalState rv_init = rv;

    darr = darray_push(darr, (void*)&rv_init.r);
    // Propagate backwards until we hit SOI
    float time = t;
    while (!hit_max_dist) {
        delta_t = _computeDeltaT(fabs(DVector3Length(rv.v)), scalingFactor);
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
    delta_t = _computeDeltaT(fabs(DVector3Length(init_rv.v)), scalingFactor);

    while (!hit_max_dist) {
        delta_t = _computeDeltaT(fabs(DVector3Length(rv.v)), scalingFactor);
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

void* propagate_orbit_ellipse(OrbitalElements oe, PhysicalState rv, float t) {
    PhysicalState init_rv = rv;
    void* darr = darray_init(1000,sizeof(DVector3));
    Debug("arr len = %d\n",darray_length(darr));
    float total = 0.0;
    double scalingFactor = 500000.0;

    PhysicalState rv_init = rv;

    double speed = DVector3Length(rv.v); 
    float deltaT = compute_delta_t(oe, 0.2);

    while(total < oe.period) {
        /* float deltaT = compute_delta_t(oe, 0.2); */
        deltaT = 50000;
        total += deltaT;

        /* Debug("deltaT=%.2f\n",deltaT); */
        
        rv = rv_from_r0v0(rv, deltaT);
        OrbitalElements oe = orb_elems_from_rv(rv, 0.0, 0.0);

        darr = darray_push(darr,(void*)&rv.r);
    }

    Debug("r=(%.2f,%.2f,%.2f)\n",rv.r.x,rv.r.y,rv.r.z);
    Debug("v=(%.2f,%.2f,%.2f)\n",rv.v.x,rv.v.y,rv.v.z);
    Debug("Finished propagating orbit, total = %.2f, period = %.2f\n",total,oe.period);

    darr = darray_push(darr,(void*)&init_rv);

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
