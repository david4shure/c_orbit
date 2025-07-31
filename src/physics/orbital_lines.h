#pragma once

#include "corbit_math.h"
#ifndef ORBITALLINES_H
#define ORBITALLINES_H 

#include "kepler.h"
#include "../utils/darray.h"

// Propagates an orbit in the general sense, starting at time = t
darray compute_orbital_lines(PhysicalState rv, double grav_param, float t,float max_distance);

// Propagates only ellipse orbits
darray compute_orbital_lines_ellipse(ClassicalOrbitalElements oe, PhysicalState rv);

// Propagates only parabolic or hyperbolic orbits
darray compute_orbital_lines_non_ellipse(ClassicalOrbitalElements oe, PhysicalState rv, double grav_param, float t, float max_distance);

typedef struct __attribute__((aligned(8))) {
    DVector3 point;
    float time_at_point;
    float period;
} PointBundle;

#endif
