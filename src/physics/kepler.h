#pragma once

#ifndef KEPLER_H
#define KEPLER_H

#include "corbit_math.h"

// All units are in Kilometers, Seconds, Newtons, KM/s, etc.
typedef struct PhysicalParameters {
    double radius;
    double mass;
    double grav_param;
} PhysicalParameters;

// We do everything in radians here
typedef struct ClassicalOrbitalElements {
    // Physical parameters of an orbit
    double mass_of_parent; // Mass of the central body KG e.g. Earth
    double mass_of_grandparent; // Mass of the central body's central body KG e.g. Sun
    double grav_param; // Specific gravitational parameter for central body IE G * M
    
    // Geometrical parameters of an orbit
    double semimajor_axis; // semi-major axis KMs
    double semilatus_rectum; // p, semilatus rectum KMs
    double eccentricity; // eccentricity (unitless, ratio)
    double mean_anomaly; // Mean anomaly (Rads)
    double mean_anomaly_at_epoch; // Mean anomaly at epoch (Rads)
    double time_at_epoch; // time in seconds at epoch
    double eccentric_anomaly; // Eccentric Anomaly (Rads)
    double hyperbolic_anomaly; // Hyperbolic Anomaly (Rads)
    double true_anomaly; // True anomaly (Rads)
    double arg_of_periapsis; // Argument of periapsis (Rads)
    double long_of_asc_node; // Longitude of the ascending node (Rads)
    double inclination; // Inclination (Rads)
    double period; // Orbital Period
    double ang_momentum; // Angular momentum (km^2/s)
    DVector3 ang_momentum_vec; // Angular momentum vector
    double mean_motion; // Mean motion = sqrt(grav_param/a^3)
    double periapsis_distance; // distance at periapsis
    double apoapsis_distance; // distance at apoapsis
} ClassicalOrbitalElements;

typedef struct PhysicalState {
    DVector3 r; // Position in inertial frame 
    DVector3 v; // Velocity in inertial frame
    double mass_of_parent; // Mass of the central body KG e.g. Earth
    double mass_of_grandparent; // Mass of the central body's central body KG e.g. Sun
} PhysicalState;

typedef struct LagrangeCoefs {
    double f;
    double g;
} LagrangeCoefs;

// Time derivatives of f and g, f_dot, g_dot
typedef struct LagrangeTimeDerivs {
    double f_dot;
    double g_dot;
} LagrangeTimeDerivs;

// Contains time info about what the time at periapsis will be
typedef struct TimeOfPassage {
    double time_at_point; // seconds
    double duration_until_point; // seconds
    double current_time; // seconds
} TimeOfPassage;

// Mean anomaly as a function of M_naught (Mean anomaly at epoch, t time, t_naught time of M_naught, T orbital period)
double mean_anom(double M_naught, double t, double t_naught, double T); 

// Computes eccentric anomaly from true anomaly
double true_anom_to_ecc_anom(double true_anomaly, double eccentricity);

// Computes mean anomaly given hyperbolic anomaly
double hyperbolic_anom_to_mean_anom(double hyperbolic_anom, double eccentricity);

// Converts true anomaly to hyperbolic anomaly
double true_anom_to_hyperbolic_anom(double true_anomaly, double eccentricity);

// Computes true anomaly from eccentric anomaly
double ecc_anom_to_true_anom(double eccentricity, double eccentric_anomaly);

// Eccentric anomaly to mean anomaly
double ecc_anom_to_mean_anom(double eccentricity, double eccentric_anomaly);

// Computes mean anomaly from a true anomaly
double true_anom_to_mean_anom(double true_anomaly, double eccentricity);

// Solves for the eccentric anomaly from the following eq.
double solve_kepler_eq_ellipse(double e, double M, int max_iters);

// StumpC helper function
double stump_c(double z);

// StumpS helper function
double stump_s(double z);

// Computes the distance of body from center point
double distance_sphere_coords(double e, double a, double E);

// Computes the time until the desired_true_anomaly given then orbital elements and current time
TimeOfPassage compute_time_until(ClassicalOrbitalElements oe, double desired_true_anomaly, double t);

// Gives the distance at which the sphere of influence begins/ends for the inner body
double calculate_sphere_of_influence_r(double a, double mass_of_body, double mass_of_satellite);

// Compute the time till periapsis for an orbit
TimeOfPassage compute_time_of_passage(ClassicalOrbitalElements oe, double grav_param, double t);

// Gets orbital elements in struct form from R (position) V (velcoity) vectors.
ClassicalOrbitalElements rv_to_classical_elements(PhysicalState rv);

// Gets R (position), V (velocity) vectors from ClassicalOrbitalElements
PhysicalState classical_elements_to_rv(ClassicalOrbitalElements elems);

// Solves for universal anomaly (algorithm 3.3, curtis D.5)
// Units: km^0.5
double solve_universal_anomaly(double dt, double r0, double v0, double a, double grav_param);

// Computes the lagrange coeficients for given universal anomaly, t, etc.
LagrangeCoefs compute_lagrange_f_g(double univ_anomaly, double t, double r0, double a, double grav_param);

// Computes time derivatives of the lagrange f and g coeficients
LagrangeTimeDerivs compute_lagrange_fdot_gdot(double univ_anomaly, double t, double r0, double a, double grav_param);

// Computes the position R, and velocity V given an initial position vector R0 and velocity vector V0, after time t
PhysicalState rv_from_r0v0(PhysicalState rv, double t);

// Solves keplers equation for the carteesian coords in inertial frame
// with proper rotations applied (inclination, arg periapsis, long asc, etc)
DVector3 solve_kepler_ellipse_inertial(ClassicalOrbitalElements elems, double M_naught, double t_naught, double t);

// Solves keplers equation for the carteesian coords in perifocal frame 
// DVector2 because only x and y are relevant for perifocal frame
DVector3 solve_kepler_ellipse_perifocal(ClassicalOrbitalElements elems, double M_naught, double t_naught, double t); 


// Converts a Vec3 in World render coords to physical coords
DVector2 vector2_from_world_to_physical(DVector2 vec);

// Converts a Vec3 in physical coords World render coords
DVector2 vector2_from_physical_to_world(DVector2 vec);

// Converts a Vec3 in World render coords to physical coords
DVector3 vector_from_world_to_physical(DVector3 vec);

// Converts a Vec3 in physical coords World render coords
DVector3 vector_from_physical_to_world(DVector3 vec);

// Prints orbital elements duh
void print_orbital_elements(ClassicalOrbitalElements e);

// Prints physical state duh
void print_physical_state(PhysicalState rv);

// Applies necessary transforms to convert perifocal coords to inertial coords
DVector3 perifocal_coords_to_inertial_coords(DVector3 pq,double long_of_asc_node,double arg_of_periapsis, double inclination);

// Applies transform to convert from inertial coords back to perifocal coords
DVector3 inertial_coords_to_perifocal_coords(DVector3 eci, double long_of_asc_node, double arg_of_periapsis, double inclination);

// Computes periapsis distance for an orbit
double periapsis_distance(ClassicalOrbitalElements oe);

#endif
