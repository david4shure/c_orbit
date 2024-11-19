#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../physics/constants.h"

int main() {
    InitializeLogger(DEBUG, true);
    DVector3 initial_position = {-6045,-3490,2500};
    DVector3 initial_velocity = {-3.457,6.618,2.533};

    PhysicalState initial_state = {
        .r = initial_position,
        .v = initial_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG
    };

    OrbitalElements oe = orb_elems_from_rv(initial_state, 0.0, 0.0);

    print_orbital_elements(oe);

    printf("oe.ang_mom = %.5f\n",oe.ang_momentum);
    ASSERT_NEAR(0.171212,oe.eccentricity,1e-3);
    ASSERT_NEAR(255.279*D_DEG2RAD,oe.long_of_asc_node,1e-3);
    ASSERT_NEAR(153.249*D_DEG2RAD,oe.inclination,1e-3);
    ASSERT_NEAR(20.0683*D_DEG2RAD, oe.arg_of_periapsis,1e-3);
    ASSERT_NEAR(28.4456*D_DEG2RAD, oe.true_anomaly,1e-3);
    ASSERT_NEAR(8788.1, oe.semimajor_axis,1e1);

    return 0;
}

