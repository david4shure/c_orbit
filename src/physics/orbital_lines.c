#include "orbital_lines.h"
#include "../utils/darray.h"
#include "../utils/logger.h"
#include "assert.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "corbit_math.h"
#include "kepler.h"

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
darray compute_orbital_lines(PhysicalState rv,float t,float M_naught, float t_naught, float max_render_distance) {
    OrbitalElements oe = orb_elems_from_rv(rv,M_naught,t_naught);

    if (oe.eccentricity < 1.0) { // Ellipse
        return compute_orbital_lines_ellipse(oe,rv,t,max_render_distance);
    } else { // Parabolas / Hyperbolas
        return compute_orbital_lines_non_ellipse(oe,rv,t,max_render_distance);
    }
}

darray compute_orbital_lines_non_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_render_distance) {
    // Go in the negative direction until we hit max_render_distance
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

darray compute_orbital_lines_ellipse(OrbitalElements oe, PhysicalState rv, float t, float z_far) {
    Info("Starting propagate_orbit_ellipse: ecc=%.6f, true_anomaly=%.2f", 
         oe.eccentricity, oe.true_anomaly * 180.0 / M_PI);

    // Loop through true anomaly by step size
    darray arr = darray_init(1850, sizeof(PointBundle));

    // Scale delta T by distance to apoapsis
    float delta_t;

    // True anomaly [0,2PI]
    // loop through true anomaly
    // True anomaly is 0 at periapsis, and PI at apoapsis
    double r_p = oe.periapsis_distance;
    double r_a = oe.apoapsis_distance;
    
    for (float time = 0.0; time< oe.period; time += delta_t) {
        // Make delta_t larger at apoapsis
        // Make delta_t smaller at periapsis

        DVector3 position = solve_kepler_ellipse_inertial(oe, 0.0, 0.0, time);

        // Calculate distance
        double r = DVector3Length(position);

        // Shallow range of min/max values
        double max_circular = oe.period/1000;
        double min_circular = oe.period/1000;

        // Extreme range of min/max values
        double min_elliptical = oe.period/10000000;
        double max_elliptical = oe.period/500;


        // Linear interpolation between them based on eccentricity :)
        double min = min_elliptical + (1-oe.eccentricity) * (min_circular - min_elliptical);
        double max = max_circular + (1-oe.eccentricity) * (max_circular - max_elliptical);

        // Linear interpolate between max/min from 0..1 fraction determined
        // where 0 is distance at periapsis, and 1 is distance at apoapsis
        delta_t = min + (r-r_p)/(r_a-r_p) * (max-min);

        // Construct point bundle
        PointBundle bundle = (PointBundle){
            .point = position,
            .time_at_point = time,
            .period = oe.period,
        };

        // Push point to the darray
        arr = darray_push(arr,(void*)&bundle);
    }

    // Connect the geometry so it forms a completed ellipse
    void* element = darray_get(arr,0);
    arr = darray_push(arr,(void*)element);

    return arr;
}
