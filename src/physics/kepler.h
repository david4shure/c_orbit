#pragma once

#include <raymath.h>

// We do everything in radians here
typedef struct OrbitalElements {
    // Physical parameters of an orbit
    float mass_of_parent; // Mass of the central body KG e.g. Earth
    float mass_of_grandparent; // Mass of the central body's central body KG e.g. Sun
    float grav_param; // Specific gravitational parameter for central body IE G * M
    
    // Geometrical parameters of an orbit
    float semimajor_axis; // semi-major axis KMs
    float semilatus_rectum; // p, semilatus rectum KMs
    float eccentricity; // eccentricity (unitless, ratio)
    float mean_anomaly; // Mean anomaly (Rads)
    float mean_anomaly_at_epoch; // Mean anomaly at epoch (Rads)
    float time_at_epoch; // time in seconds at epoch
    float eccentric_anomaly; // Eccentric Anomaly (Rads)
    float hyperbolic_anomaly; // Hyperbolic Anomaly (Rads)
    float true_anomaly; // True anomaly (Rads)
    float arg_of_periapsis; // Argument of periapsis (Rads)
    float long_of_asc_node; // Longitude of the ascending node (Rads)
    float inclination; // Inclination (Rads)
    float period; // Orbital Period
    float ang_momentum; // Angular momentum (km^2/s)
    Vector3 ang_momentum_vec; // Angular momentum vector
    float mean_motion; // Mean motion = sqrt(grav_param/a^3)
} OrbitalElements;

typedef struct PhysicalState {
    Vector3 r; // Position in inertial frame 
    Vector3 v; // Velocity in inertial frame
    float mass_of_parent; // Mass of the central body KG e.g. Earth
    float mass_of_grandparent; // Mass of the central body's central body KG e.g. Sun
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

// Contains time info about what the time at periapsis will be
typedef struct TimeOfPassage {
    float time_at_point; // seconds
    float duration_until_point; // seconds
    float current_time; // seconds
} TimeOfPassage;

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

// Computes mean anomaly from a true anomaly
float true_anom_to_mean_anom(float true_anomaly, float eccentricity);

// Solves for the eccentric anomaly from the following eq.
float solve_kepler_eq_ellipse(float e, float M, int max_iters);

// StumpC helper function
float stump_c(float z);

// StumpS helper function
float stump_s(float z);

// Computes the distance of body from center point
float distance_sphere_coords(float e, float a, float E);

// Computes the time until the desired_true_anomaly given then orbital elements and current time
TimeOfPassage compute_time_until(OrbitalElements oe, float desired_true_anomaly, float t);

// Gives the distance at which the sphere of influence begins/ends for the inner body
float calculate_sphere_of_influence_r(float a, float mass_of_body, float mass_of_satellite);

// Compute the time till periapsis for an orbit
TimeOfPassage compute_time_of_passage(OrbitalElements oe, float grav_param, float t);

// Gets orbital elements in struct form from R (position) V (velcoity) vectors.
OrbitalElements orb_elems_from_rv(PhysicalState rv, float mean_anomaly_at_epoch, float time_at_epoch);

// Solves for universal anomaly (algorithm 3.3, curtis D.5)
// Units: km^0.5
float solve_universal_anomaly(float dt, float r0, float v0, float a, float grav_param);

// Computes the lagrange coeficients for given universal anomaly, t, etc.
LagrangeCoefs compute_lagrange_f_g(float univ_anomaly, float t, float r0, float a, float grav_param);

// Computes time derivatives of the lagrange f and g coeficients
LagrangeTimeDerivs compute_lagrange_fdot_gdot(float univ_anomaly, float t, float r0, float a, float grav_param);

// Computes the position R, and velocity V given an initial position vector R0 and velocity vector V0, after time t
PhysicalState rv_from_r0v0(PhysicalState rv, float t);

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

// Prints physical state duh
void print_physical_state(PhysicalState rv);

// Applies necessary transforms to convert perifocal coords to inertial coords
Vector3 perifocal_coords_to_inertial_coords(Vector2 pq,float long_of_asc_node,float arg_of_periapsis, float inclination);
