#pragma once

#ifndef PROPAGATION_H
#define PROPAGATION_H

#include "kepler.h"

// Propagates an orbit in the general sense, starting at time = t
void* propagate_orbit(PhysicalState rv,float t,float M_naught, float t_naught, float max_distance);

// Propagates only ellipse orbits
void* propagate_orbit_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_distance);

// Propagates only parabolic or hyperbolic orbits
void* propagate_orbit_non_ellipse(OrbitalElements oe, PhysicalState rv, float t, float max_distance);

typedef struct {
    DVector3 point;
    float time_at_point;
    float period;
} PointBundle;

#endif
