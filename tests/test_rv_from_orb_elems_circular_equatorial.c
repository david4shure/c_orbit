#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../src/physics/constants.h"

int main() {
    DVector3 position = {384400.0, 0.0, 0.0};
    DVector3 velocity = {0.0, 1.022, 0.0};

    PhysicalState initial_state = {
        .r = position,
        .v = velocity,
    };

    ClassicalOrbitalElements oe = rv_to_classical_elements(initial_state,EARTH_MASS_KG*G);
    PhysicalState final_state = classical_elements_to_rv(oe,EARTH_MASS_KG*G);

    ASSERT_EQ_DVECTOR3(initial_state.r, final_state.r, 1e-3);
    ASSERT_EQ_DVECTOR3(initial_state.v, final_state.v, 1e-3);

    Debug("Initial Position: (%.6f, %.6f, %.6f)\n", initial_state.r.x, initial_state.r.y, initial_state.r.z);
    Debug("Final Position: (%.6f, %.6f, %.6f)\n", final_state.r.x, final_state.r.y, final_state.r.z);
    Debug("Initial Velocity: (%.6f, %.6f, %.6f)\n", initial_state.v.x, initial_state.v.y, initial_state.v.z);
    Debug("Final Velocity: (%.6f, %.6f, %.6f)\n", final_state.v.x, final_state.v.y, final_state.v.z);

    return 0;
}

