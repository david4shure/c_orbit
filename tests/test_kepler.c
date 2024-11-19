#include "../src/physics/kepler.h"
#include "constants.h"
#include "test_utils.h"
#include "../src/utils/logger.h"
#include "assert.h"

void test_orb_elems_from_rv() {
    DVector3 moon_position = {384400.0, 0.0, 0.0};
    DVector3 moon_velocity = {0.0, 1.022, 0.0};
    
    PhysicalState RV = {
        .r = moon_position,
        .v = moon_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG,
    };

    float M_naught = 2.35585;
    float t_naught = 0.0;

    OrbitalElements oe = orb_elems_from_rv(RV,0.0,0.0);

    ASSERT_NEAR(0.007274,oe.eccentricity,1e-6);
    ASSERT_NEAR(387216.80439,oe.semimajor_axis,1e-6);
    ASSERT_NEAR(0.0,oe.inclination,1e-6);
    ASSERT_NEAR(0.000197,oe.arg_of_periapsis,1e-6);
    ASSERT_NEAR(0.0,oe.long_of_asc_node,1e-6);
    ASSERT_NEAR(392856.781250,oe.ang_momentum,1e-6);
}

void test_rv_from_orb_elems_circular_equatorial() {
    DVector3 position = {384400.0, 0.0, 0.0};
    DVector3 velocity = {0.0, 1.022, 0.0};

    PhysicalState initial_state = {
        .r = position,
        .v = velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG
    };

    OrbitalElements oe = orb_elems_from_rv(initial_state, 0.0, 0.0);
    PhysicalState final_state = rv_from_orb_elems(oe);

    ASSERT_EQ_DVECTOR3(initial_state.r, final_state.r, 1e-3);
    ASSERT_EQ_DVECTOR3(initial_state.v, final_state.v, 1e-3);

    Debug("Initial Position: (%.6f, %.6f, %.6f)\n", initial_state.r.x, initial_state.r.y, initial_state.r.z);
    Debug("Final Position: (%.6f, %.6f, %.6f)\n", final_state.r.x, final_state.r.y, final_state.r.z);
    Debug("Initial Velocity: (%.6f, %.6f, %.6f)\n", initial_state.v.x, initial_state.v.y, initial_state.v.z);
    Debug("Final Velocity: (%.6f, %.6f, %.6f)\n", final_state.v.x, final_state.v.y, final_state.v.z);
}

void test_rv_from_orb_elems_circular_inclined() {
    DVector3 position = {384400.0, 0.0, 0.1};
    DVector3 velocity = {0.0, 1.022, 0.1};

    PhysicalState initial_state = {
        .r = position,
        .v = velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG
    };

    OrbitalElements oe = orb_elems_from_rv(initial_state, 0.0, 0.0);
    PhysicalState final_state = rv_from_orb_elems(oe);

    ASSERT_EQ_DVECTOR3(initial_state.r, final_state.r, 1e-3);
    ASSERT_EQ_DVECTOR3(initial_state.v, final_state.v, 1e-3);

    Debug("Initial Position: (%.6f, %.6f, %.6f)\n", initial_state.r.x, initial_state.r.y, initial_state.r.z);
    Debug("Final Position: (%.6f, %.6f, %.6f)\n", final_state.r.x, final_state.r.y, final_state.r.z);
    Debug("Initial Velocity: (%.6f, %.6f, %.6f)\n", initial_state.v.x, initial_state.v.y, initial_state.v.z);
    Debug("Final Velocity: (%.6f, %.6f, %.6f)\n", final_state.v.x, final_state.v.y, final_state.v.z);
}

void test_intermediate_consistency() {
    // Example position and velocity vectors
    DVector3 initial_position = {384400.0, 0.1, 0.3};
    DVector3 initial_velocity = {0.0, 1.022, -0.1};

    PhysicalState initial_state = {
        .r = initial_position,
        .v = initial_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG
    };

    // Step 1: Convert RV to Orbital Elements
    OrbitalElements oe = orb_elems_from_rv(initial_state, 0.0, 0.0);

    // Log orbital elements
    Debug("Orbital Elements Derived:\n");
    Debug("  Semi-major Axis: %.10f\n", oe.semimajor_axis);
    Debug("  Eccentricity: %.10f\n", oe.eccentricity);
    Debug("  Inclination: %.10f\n", oe.inclination);
    Debug("  RAAN: %.10f\n", oe.long_of_asc_node);
    Debug("  Argument of Periapsis: %.10f\n", oe.arg_of_periapsis);
    Debug("  True Anomaly: %.10f\n", oe.true_anomaly);

    // Step 2: Convert back to RV
    PhysicalState reconstructed_state = rv_from_orb_elems(oe);

    // Log initial and reconstructed states
    Warn("Initial State:\n");
    Warn("  Position=(%.6f, %.6f, %.6f), Velocity=(%.6f, %.6f, %.6f)\n",
         initial_state.r.x, initial_state.r.y, initial_state.r.z,
         initial_state.v.x, initial_state.v.y, initial_state.v.z);

    Warn("Reconstructed State:\n");
    Warn("  Position=(%.6f, %.6f, %.6f), Velocity=(%.6f, %.6f, %.6f)\n",
         reconstructed_state.r.x, reconstructed_state.r.y, reconstructed_state.r.z,
         reconstructed_state.v.x, reconstructed_state.v.y, reconstructed_state.v.z);

    // Step 3: Assert positions and velocities match
    ASSERT_EQ_DVECTOR3(initial_state.r, reconstructed_state.r, 1e-3);
    ASSERT_EQ_DVECTOR3(initial_state.v, reconstructed_state.v, 1e-3);
}

// Tolerance for floating-point comparisons
#define TOLERANCE 1e-6

void test_perifocal_to_inertial_and_back() {
    // Define test orbital parameters
    double long_of_asc_node = 0.1; // Ω (RAAN) in radians
    double arg_of_periapsis = 0.2; // ω (Argument of Periapsis) in radians
    double inclination = 0.3;      // i (Inclination) in radians

    // Define a position vector in the perifocal frame
    DVector3 perifocal_position = {.x = 7000.0, .y = 0.0, .z = 0.0};
    // Print results
    printf("Original Perifocal Position: (%.6f, %.6f, %.6f)\n", perifocal_position.x, perifocal_position.y, perifocal_position.z);

    // Convert from perifocal to inertial
    DVector3 inertial_position = perifocal_coords_to_inertial_coords(perifocal_position, long_of_asc_node, arg_of_periapsis, inclination);
    printf("Inertial Position: (%.6f, %.6f, %.6f)\n", inertial_position.x, inertial_position.y, inertial_position.z);

    // Convert back from inertial to perifocal
    DVector3 reconstructed_perifocal_position = inertial_coords_to_perifocal_coords(inertial_position, long_of_asc_node, arg_of_periapsis, inclination);
    printf("Reconstructed Perifocal Position: (%.6f, %.6f, %.6f)\n", reconstructed_perifocal_position.x, reconstructed_perifocal_position.y, reconstructed_perifocal_position.z);


    // Check if the reconstructed perifocal position matches the original
    if (fabs(reconstructed_perifocal_position.x - perifocal_position.x) < TOLERANCE &&
        fabs(reconstructed_perifocal_position.y - perifocal_position.y) < TOLERANCE &&
        fabs(reconstructed_perifocal_position.z - perifocal_position.z) < TOLERANCE) {
        printf("Test Passed: Round-trip conversion is consistent.\n");
    } else {
        printf("Test Failed: Round-trip conversion is inconsistent.\n");
        assert(false);
    }
}

