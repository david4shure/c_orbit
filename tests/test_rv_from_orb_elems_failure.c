#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../src/physics/constants.h"

int main() {
    DVector3 position = {384053.94007, 16366.87890, 0.00000};
    DVector3 velocity = {-0.04337, 1.02517, 0.00000};

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

    return 0;
}

