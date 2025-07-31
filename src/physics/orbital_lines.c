#include "orbital_lines.h"
#include <stdio.h>
#include "../utils/darray.h"
#include "../utils/logger.h"
#include "assert.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "corbit_math.h"
#include "kepler.h"

const int MAX_POINTS = 20000;



double delta_t_from_velocity(double velocityMagnitude, double scalingFactor) {
    return scalingFactor / (velocityMagnitude * velocityMagnitude);  // Time step inversely proportional to velocity cube
}

// This function propagates an orbit and figures
// out how to do so based on its orbit type (ellipse,parabola,hyperbola etc)
// NOTE: This function returns a dynamic array that is malloc'ed!!
// It should be freed accordingly
darray compute_orbital_lines(PhysicalState rv, double grav_param, float t, float max_render_distance) {
    printf("DEBUG: compute_orbital_lines called with grav_param: %f, t: %f\n", grav_param, t);
    printf("DEBUG: About to compute orbital elements\n");
    ClassicalOrbitalElements oe = rv_to_classical_elements(rv,grav_param);
    printf("DEBUG: Orbital elements computed, eccentricity: %f\n", oe.eccentricity);
    
    if (oe.eccentricity < 1.0) { // Ellipse
        printf("DEBUG: Calling compute_orbital_lines_ellipse\n");
        return compute_orbital_lines_ellipse(oe,rv);
    } else { // Parabolas / Hyperbolas
        printf("DEBUG: Calling compute_orbital_lines_non_ellipse\n");
        return compute_orbital_lines_non_ellipse(oe,rv,grav_param,t,max_render_distance);
    }
}

darray compute_orbital_lines_non_ellipse(ClassicalOrbitalElements oe, PhysicalState rv, double grav_param, float t, float max_render_distance) {
    // Go in the negative direction until we hit max_render_distance
    PhysicalState init_rv = rv;
    bool hit_max_dist = false;
    double scalingFactor = 15000.0;
    void* darr = darray_init(10000,sizeof(DVector3));
    float delta_t = delta_t_from_velocity(fabs(DVector3Length(init_rv.v)), scalingFactor);
    PhysicalState rv_init = rv;
    darr = darray_push(darr, (void*)&rv_init.r);
    // Propagate backwards until we hit SOI
    float time = t;
    int num_points = 0;
    while (!hit_max_dist) {
        delta_t = delta_t_from_velocity(DVector3Length(rv.v), scalingFactor);
        rv = rv_from_r0v0(rv, grav_param, -delta_t);
        float distance = fabs(DVector3Length(rv.r));
        hit_max_dist = distance > max_render_distance;
        DVector3 position = rv.r;
        darr = darray_insert_at(darr,(void*)&position,0);
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
        rv = rv_from_r0v0(rv, grav_param, delta_t);
        float distance = fabs(DVector3Length(rv.r));
        hit_max_dist = distance > max_render_distance;
        DVector3 position = rv.r;
        darr = darray_push(darr, (void*)&position);
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

darray compute_orbital_lines_ellipse(ClassicalOrbitalElements oe, PhysicalState rv) {
    // Safety check for invalid orbital elements
    if (isnan(oe.semimajor_axis) || isnan(oe.eccentricity) || isnan(oe.period) ||
        isinf(oe.semimajor_axis) || isinf(oe.eccentricity) || isinf(oe.period) ||
        oe.period <= 0.0 || oe.semimajor_axis <= 0.0) {
        // Return empty array for invalid elements
        return darray_init(10, sizeof(PointBundle));
    }
    
    // Loop through true anomaly by step size
    darray arr = darray_init(500, sizeof(PointBundle));

    // Scale delta T by distance to apoapsis
    float delta_t = 0.0;
    float time = 0.0;

    // True anomaly [0,2PI]
    // loop through true anomaly
    // True anomaly is 0 at periapsis, and PI at apoapsis
    double r_p = oe.periapsis_distance;
    double r_a = oe.apoapsis_distance;
    
    while (true) {
        
        // Make delta_t larger at apoapsis
        // Make delta_t smaller at periapsis
        DVector3 position = solve_kepler_ellipse_inertial(oe, 0.0, 0.0, time);

        // Calculate distance
        double r = DVector3Length(position);

        if(isnan(r)) {
            break;
        }

        if(r==0.0) {
            break;
        }

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
        if (oe.eccentricity < 0.1) {
            // Handle degenerate case, e.g., circular orbit
            delta_t = min;  // or an appropriate fallback
        } else {
            delta_t = min + (r-r_p)/(r_a-r_p) * (max-min);
        }

        // Construct point bundle
        PointBundle bundle = (PointBundle){
            .point = position,
            .time_at_point = time,
            .period = oe.period,
        };



        // Push point to the darray (store the object directly, not a pointer)
        arr = darray_push(arr,(void*)&bundle);

        time += delta_t;

        if (time > oe.period) {
            break;
        }
    }

    // Connect the geometry so it forms a completed ellipse
    void* element = darray_get(arr,0);
    arr = darray_push(arr,(void*)element);

    return arr;
}
