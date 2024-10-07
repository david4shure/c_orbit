#pragma once

#include <stdio.h>
#include <raymath.h>

// We do everything in radians here
typedef struct OrbitalElements {
    float a; // semi-major axis
    float e; // eccentricity
    float M; // Mean anomaly (Rads)
    float E; // Eccentric Anomaly (Rads)
    float θ; // True anomaly (Rads)
    float ω; // Argument of periapsis (Rads)
    float Ω; // Longitude of the ascending node (Rads)
    float i; // Inclination (Rads)
    float T; // Orbital Period
    float μ; // Specific gravitational parameter for central body
} OrbitalElements;

typedef struct OrbitalState {
    Vector3 r; // Position in inertial frame 
    Vector3 v; // Velocity in inertial frame
} OrbitalState;

// Mean anomaly as a function of M_naught (Mean anomaly at epoch, t time, t_naught time of M_naught, T orbital period)
float mean_anom(float M_naught, float t, float t_naught, float T); 

// Solves for the eccentric anomaly from the following eq.
float solve_ecc_anom_newton(float e, float M);

float ecc_anom_to_true_anom(float e, float E);

float distance(float e, float a, float E);

// Solves for the true anomaly from eccentric anomaly
float kepler_E_newt(float e, float M, int max_iters);

// Gets orbital elements in struct form from R (position) V (velcoity) vectors.
OrbitalElements orb_elems_from_rv(OrbitalState rv, float μ);

// Gets R (position), V (velocity) vectors from OrbitalElements
OrbitalState rv_from_orb_elems(OrbitalElements elems);
