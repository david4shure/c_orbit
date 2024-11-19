#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../src/physics/constants.h"

#define TOLERANCE 1e-6

int main() {
    double long_of_asc_node = 0.1; // Ω (RAAN) in radians
    double arg_of_periapsis = 0.2; // ω (Argument of Periapsis) in radians
    double inclination = 0.3;      // i (Inclination) in radians

    DVector3 perifocal_position = {.x = 7000.0, .y = 0.0, .z = 0.0};

    printf("Original Perifocal Position: (%.6f, %.6f, %.6f)\n", perifocal_position.x, perifocal_position.y, perifocal_position.z);

    DVector3 inertial_position = perifocal_coords_to_inertial_coords(perifocal_position, long_of_asc_node, arg_of_periapsis, inclination);
    printf("Inertial Position: (%.6f, %.6f, %.6f)\n", inertial_position.x, inertial_position.y, inertial_position.z);

    DVector3 reconstructed_perifocal_position = inertial_coords_to_perifocal_coords(inertial_position, long_of_asc_node, arg_of_periapsis, inclination);
    printf("Reconstructed Perifocal Position: (%.6f, %.6f, %.6f)\n", reconstructed_perifocal_position.x, reconstructed_perifocal_position.y, reconstructed_perifocal_position.z);

    ASSERT_EQ_DVECTOR3(perifocal_position, reconstructed_perifocal_position, 1e-3);
}

