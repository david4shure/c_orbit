#pragma once

#ifndef PROPAGATION_H
#define PROPAGATION_H

#include "kepler.h"

// Propagates an orbit in the general sense, starting at time = t
void* propagate_orbit(PhysicalState rv,float t,float M_naught, float t_naught);

// Propagates only ellipse orbits
void* propagate_orbit_ellipse(OrbitalElements oe, PhysicalState rv, float t);

// Propagates only parabolic or hyperbolic orbits
void* propagate_orbit_non_ellipse(OrbitalElements oe, PhysicalState rv, float t);

#endif
