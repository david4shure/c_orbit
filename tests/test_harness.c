#include <assert.h>
#include "../src/physics/kepler.h"
#include "constants.h"
#include "logger.h"
#include "stdio.h"
#include "test_utils.h"
#include "test_kepler.h"


int main() {
    InitializeLogger(DEBUG, true);
    test_perifocal_to_inertial_and_back();
    test_rv_from_orb_elems_circular_equatorial();
    test_rv_from_orb_elems_circular_inclined();
    test_intermediate_consistency();
    return 0;
}
