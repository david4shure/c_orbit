#pragma once

#define RENDER_COORDS_TO_PHYSICAL_COORDS 100
#define PHYSICS_COORDS_TO_RENDER_COORDS 0.01

#include <stdio.h>
#include <raymath.h>

// We do everything in radians here
typedef struct OrbitalElements {
    float semimajor_axis; // semi-major axis
    float eccentricity; // eccentricity
    float mean_anomaly; // Mean anomaly (Rads)
    float eccentric_anomaly; // Eccentric Anomaly (Rads)
    float true_anomaly; // True anomaly (Rads)
    float arg_of_periapsis; // Argument of periapsis (Rads)
    float long_of_asc_node; // Longitude of the ascending node (Rads)
    float inclination; // Inclination (Rads)
    float period; // Orbital Period
    float grav_param; // Specific gravitational parameter for central body
} OrbitalElements;

typedef struct PhysicalState {
    Vector3 r; // Position in inertial frame 
    Vector3 v; // Velocity in inertial frame
} PhysicalState;

// Mean anomaly as a function of M_naught (Mean anomaly at epoch, t time, t_naught time of M_naught, T orbital period)
float mean_anom(float M_naught, float t, float t_naught, float T); 

// Computes eccentric anomaly from true anomaly
float true_anom_to_ecc_anom(float true_anomaly, float eccentricity);

// Computes true anomaly from eccentric anomaly
float ecc_anom_to_true_anom(float eccentricity, float eccentric_anomaly);

// Eccentric anomaly to mean anomaly
float ecc_anom_to_mean_anom(float eccentricity, float eccentric_anomaly);

// Solves for the eccentric anomaly from the following eq.
float solve_kepler_eq_ellipse(float e, float M, int max_iters);

// StumpC helper function
float stump_c(float z);

// StumpS helper function
float stump_s(float z);

// Computes the distance of body from center point
float distance_sphere_coords(float e, float a, float E);

// Gets orbital elements in struct form from R (position) V (velcoity) vectors.
OrbitalElements orb_elems_from_rv(PhysicalState rv, float μ);

// Gets R (position), V (velocity) vectors from OrbitalElements
PhysicalState rv_from_orb_elems_ellipse(OrbitalElements elems);

// Converts a Vec3 in World render coords to physical coords
Vector3 vector_from_world_to_physical(Vector3 vec);

// Converts a Vec3 in physical coords World render coords
Vector3 vector_from_physical_to_world(Vector3 vec);

void print_orbital_elements(OrbitalElements e);
