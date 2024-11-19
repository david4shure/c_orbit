#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../physics/constants.h"

int main() {
    DVector3 initial_position = {384400.0, 0.1, 0.3};
    DVector3 initial_velocity = {0.0, 1.022, -0.1};

    PhysicalState initial_state = {
        .r = initial_position,
        .v = initial_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG
    };

    OrbitalElements oe = orb_elems_from_rv(initial_state, 0.0, 0.0);

    Debug("Orbital Elements Derived:\n");
    Debug("  Semi-major Axis: %.10f\n", oe.semimajor_axis);
    Debug("  Eccentricity: %.10f\n", oe.eccentricity);
    Debug("  Inclination: %.10f\n", oe.inclination);
    Debug("  RAAN: %.10f\n", oe.long_of_asc_node);
    Debug("  Argument of Periapsis: %.10f\n", oe.arg_of_periapsis);
    Debug("  True Anomaly: %.10f\n", oe.true_anomaly);

    PhysicalState reconstructed_state = rv_from_orb_elems(oe);

    Warn("Initial State:\n");
    Warn("  Position=(%.6f, %.6f, %.6f), Velocity=(%.6f, %.6f, %.6f)\n",
         initial_state.r.x, initial_state.r.y, initial_state.r.z,
         initial_state.v.x, initial_state.v.y, initial_state.v.z);

    Warn("Reconstructed State:\n");
    Warn("  Position=(%.6f, %.6f, %.6f), Velocity=(%.6f, %.6f, %.6f)\n",
         reconstructed_state.r.x, reconstructed_state.r.y, reconstructed_state.r.z,
         reconstructed_state.v.x, reconstructed_state.v.y, reconstructed_state.v.z);

    ASSERT_EQ_DVECTOR3(initial_state.r, reconstructed_state.r, 1e-3);
    ASSERT_EQ_DVECTOR3(initial_state.v, reconstructed_state.v, 1e-3);

    return 0;
}

