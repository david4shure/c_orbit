#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../src/physics/constants.h"

int main() {
    DVector3 moon_position = {384400.0, 0.0, 0.0};
    DVector3 moon_velocity = {0.0, 1.022, 0.0};

    PhysicalState RV = {
        .r = moon_position,
        .v = moon_velocity,
        .mass_of_parent = EARTH_MASS_KG,
        .mass_of_grandparent = SUN_MASS_KG,
    };

    OrbitalElements oe = orb_elems_from_rv(RV, 0.0, 0.0);

    print_orbital_elements(oe);

    ASSERT_NEAR(0.007274, oe.eccentricity, 1e-6);
    ASSERT_NEAR(387216.80439, oe.semimajor_axis, 1e-6);
    ASSERT_NEAR(0.0, oe.inclination, 1e-6);
    ASSERT_NEAR(0.000197, oe.arg_of_periapsis, 1e-6);
    ASSERT_NEAR(0.0, oe.long_of_asc_node, 1e-6);
    ASSERT_NEAR(392856.781250, oe.ang_momentum, 1e-6);

    return 0;
}
