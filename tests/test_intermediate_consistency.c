#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../physics/constants.h"

int main() {
    InitializeLogger(DEBUG, true);
    DVector3 initial_position = {384400.0, 0.1, 0.3};
    DVector3 initial_velocity = {0.0, 1.022, -0.1};

    PhysicalState initial_state = {
        .r = initial_position,
        .v = initial_velocity,
    };

    ClassicalOrbitalElements oe = rv_to_classical_elements(initial_state,EARTH_MASS_KG*G);

    print_orbital_elements("earth",oe);

    PhysicalState reconstructed_state = classical_elements_to_rv(oe,EARTH_MASS_KG*G);

    print_physical_state(reconstructed_state);
    print_physical_state(initial_state);

    ASSERT_EQ_DVECTOR3(initial_state.r, reconstructed_state.r, 1e-3);
    ASSERT_EQ_DVECTOR3(initial_state.v, reconstructed_state.v, 1e-3);

    return 0;
}

