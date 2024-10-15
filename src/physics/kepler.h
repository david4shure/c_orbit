#pragma once

#include <raymath.h>

// We do everything in radians here
typedef struct OrbitalElements {
    float semimajor_axis; // semi-major axis
    float semilatus_rectum; // p, semilatus rectum
    float eccentricity; // eccentricity
    float mean_anomaly; // Mean anomaly (Rads)
    float eccentric_anomaly; // Eccentric Anomaly (Rads)
    float hyperbolic_anomaly; // Hyperbolic Anomaly (Rads)
    float true_anomaly; // True anomaly (Rads)
    float arg_of_periapsis; // Argument of periapsis (Rads)
    float long_of_asc_node; // Longitude of the ascending node (Rads)
    float inclination; // Inclination (Rads)
    float period; // Orbital Period
    float grav_param; // Specific gravitational parameter for central body
    float ang_momentum; // Angular momentum (km^2/s)
    Vector3 ang_momentum_vec; // Angular momentum vector
} OrbitalElements;

typedef struct PhysicalState {
    Vector3 r; // Position in inertial frame 
    Vector3 v; // Velocity in inertial frame
} PhysicalState;

typedef struct LagrangeCoefs {
    float f; // f
    float g; // g
} LagrangeCoefs;

// Time derivatives of f and g, f_dot, g_dot
typedef struct LagrangeTimeDerivs {
    float f_dot;
    float g_dot;
} LagrangeTimeDerivs;

// Mean anomaly as a function of M_naught (Mean anomaly at epoch, t time, t_naught time of M_naught, T orbital period)
float mean_anom(float M_naught, float t, float t_naught, float T); 

// Computes eccentric anomaly from true anomaly
float true_anom_to_ecc_anom(float true_anomaly, float eccentricity);

// Computes mean anomaly given hyperbolic anomaly
float hyperbolic_anom_to_mean_anom(float hyperbolic_anom, float eccentricity);

// Converts true anomaly to hyperbolic anomaly
float true_anom_to_hyperbolic_anom(float true_anomaly, float eccentricity);

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

// Solves for universal anomaly (algorithm 3.3, curtis D.5)
// Units: km^0.5
float solve_universal_anomaly(float dt, float r0, float v0, float a, float grav_param);

// Computes the lagrange coeficients for given universal anomaly, t, etc.
LagrangeCoefs compute_lagrange_f_g(float univ_anomaly, float t, float r0, float a, float grav_param);

// Computes time derivatives of the lagrange f and g coeficients
LagrangeTimeDerivs compute_lagrange_fdot_gdot(float univ_anomaly, float t, float r0, float a, float grav_param);

// Computes the position R, and velocity V given an initial position vector R0 and velocity vector V0, after time t
PhysicalState rv_from_r0v0(Vector3 r0, Vector3 v0, float t, float grav_param);

// Solves keplers equation for the carteesian coords in inertial frame
// with proper rotations applied (inclination, arg periapsis, long asc, etc)
Vector3 solve_kepler_ellipse_inertial(OrbitalElements elems, float M_naught, float t_naught, float t);

// Solves keplers equation for the carteesian coords in perifocal frame 
// Vector2 because only x and y are relevant for perifocal frame
Vector2 solve_kepler_ellipse_perifocal(OrbitalElements elems, float M_naught, float t_naught, float t); 

// Gets R (position), V (velocity) vectors from OrbitalElements
PhysicalState rv_from_orb_elems(OrbitalElements elems);

// Converts a Vec3 in World render coords to physical coords
Vector3 vector_from_world_to_physical(Vector3 vec);

// Converts a Vec3 in physical coords World render coords
Vector3 vector_from_physical_to_world(Vector3 vec);

// Prints orbital elements duh
void print_orbital_elements(OrbitalElements e);

// Applies necessary transforms to convert perifocal coords to inertial coords
Vector3 perifocal_coords_to_inertial_coords(Vector2 pq,float long_of_asc_node,float arg_of_periapsis, float inclination);
