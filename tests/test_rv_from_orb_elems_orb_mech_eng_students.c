#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/utils/logger.h"
#include "../physics/constants.h"

int main() {
    InitializeLogger(DEBUG, true);

    // Input data
    double mu = EARTH_MASS_KG * G; // km^3/s^2
    double h = 80000.0;                        // km^2/s
    double e = 1.4;                            // Eccentricity
    double RA_deg = 40.0;                      // Right Ascension of Ascending Node (degrees)
    double incl_deg = 30.0;                    // Inclination (degrees)
    double w_deg = 60.0;                       // Argument of Perigee (degrees)
    double TA_deg = 30.0;                      // True Anomaly (degrees)

    // Convert angles to radians
    double RA_rad = RA_deg * D_DEG2RAD;
    double incl_rad = incl_deg * D_DEG2RAD;
    double w_rad = w_deg * D_DEG2RAD;
    double TA_rad = TA_deg * D_DEG2RAD;

    // Create Orbital Elements structure
    ClassicalOrbitalElements coe = {
        .ang_momentum = h,
        .eccentricity = e,
        .long_of_asc_node = RA_rad,
        .inclination = incl_rad,
        .arg_of_periapsis = w_rad,
        .true_anomaly = TA_rad,
    };

    // Compute state vector (position and velocity)
    PhysicalState sv = classical_elements_to_rv(coe,EARTH_MASS_KG*G);
    print_physical_state(sv);

    // Log inputs and outputs
    printf("---------------------------------------------------\n");
    printf("Example Test Case\n");
    printf("Gravitational parameter (km^3/s^2) = %.1f\n", mu);
    printf("Angular momentum (km^2/s) = %.1f\n", h);
    printf("Eccentricity = %.2f\n", e);
    printf("Right ascension (deg) = %.1f\n", RA_deg);
    printf("Inclination (deg) = %.1f\n", incl_deg);
    printf("Argument of perigee (deg) = %.1f\n", w_deg);
    printf("True anomaly (deg) = %.1f\n", TA_deg);
    printf("\nState vector:\n");
    printf("r (km) = [%.2f, %.2f, %.2f]\n", sv.r.x, sv.r.y, sv.r.z);
    printf("v (km/s) = [%.2f, %.2f, %.2f]\n", sv.v.x, sv.v.y, sv.v.z);
    printf("---------------------------------------------------\n");

    // Assertions (replace values with expected results)
    ASSERT_NEAR(-4039.9, sv.r.x, 1e-1);
    ASSERT_NEAR(4814.56, sv.r.y, 1e-1);
    ASSERT_NEAR(3628.62, sv.r.z, 1e-1);
    ASSERT_NEAR(-10.386, sv.v.x, 1e-2);
    ASSERT_NEAR(-4.77192, sv.v.y, 1e-2);
    ASSERT_NEAR(1.74388, sv.v.z, 1e-2);

    return 0;
}

