#include "kepler.h"
#include "propagation.h"
#include "../utils/darray.h"
#include "../utils/logger.h"
#include "assert.h"
#include "raymath.h"

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
    float r_at_sphere_of_influence = calculate_sphere_of_influence_r(149597870.7, oe.mass_of_parent, oe.mass_of_grandparent);

    // Compute the desired true anomaly at the sphere of influence
    float desired_true_anomaly = acos((oe.semilatus_rectum / (r_at_sphere_of_influence * oe.eccentricity)) - (1.0 / oe.eccentricity));
    float hyperbolic_anomaly = true_anom_to_hyperbolic_anom(desired_true_anomaly, oe.eccentricity);
    float mean_anomaly_at_sof = hyperbolic_anom_to_mean_anom(hyperbolic_anomaly, oe.eccentricity);

    // Return the time value at that mean anomaly
    float time_till_sof = ((mean_anomaly_at_sof - oe.mean_anomaly) / oe.mean_motion) + t;

    // Figure out how far out in time to calculate orbit
    float t_start = t; // TODO remove this, its always t?
    float t_end   = t + fabs(time_till_sof) * 2;

    // Todo consider passing this argument in optionally
    float t_increment = 1.0; // seconds
    float num_lines_to_draw = (t_end - t_start)/t_increment;

    Debug("t_start = %.2f, t_end = %.2f\n",t_start,t_end);
    Debug("Number of lines to draw = %d\n",num_lines_to_draw);

    void* darr = darray_init(num_lines_to_draw, sizeof(Vector3));

    int points = 0;
    float t_loop = t_start;
    while (points < num_lines_to_draw) {
        // Propagate the orbit forward by t_increment seconds
        rv = rv_from_r0v0(rv, t_loop);
    
        t_loop += t_increment;

        darr = darray_push(darr,(void*)&rv);
        points++;
    }

    return darr;

}

void* propagate_orbit_ellipse(OrbitalElements oe, PhysicalState rv, float t) {

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
