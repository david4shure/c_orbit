#include "tree.h"
#include "../physics/constants.h"
#include <string.h>

OrbitalTreeNode* load_earth_moon_system() {
    PhysicalParameters earth_physical_parameters = (PhysicalParameters){
        .mass=EARTH_MASS_KG,
        .radius=EARTH_RADIUS_KM,
        .grav_param=EARTH_MASS_KG*G,
        .sphere_of_influence = calculate_sphere_of_influence_r(EARTH_SEMIMAJOR_AXIS_KM, EARTH_MASS_KG, SUN_MASS_KG),
    };

    OrbitalTreeNode* earth = malloc(sizeof(OrbitalTreeNode)); 
    memset(earth, 0, sizeof(OrbitalTreeNode)); // Zero out all fields


    darray earth_children = darray_init(10, sizeof(OrbitalTreeNode));

    earth->physical_params = earth_physical_parameters;
    earth->orbital_elements = (ClassicalOrbitalElements){}; // Earth is not itself orbiting another body in our simulation
    earth->physical_state = (PhysicalState){
        .r = (DVector3){0,0,0},
        .v = (DVector3){0,0,0},
    }; // Earth itself is fixed at (0,0,0)
    earth->body_name = "Earth";
    earth->parent = NULL;
    earth->children = earth_children;

    PhysicalParameters moon_physical_parameters = (PhysicalParameters){
        .mass = MOON_MASS_KG,
        .radius = MOON_RADIUS_KM,
        .grav_param = MOON_MASS_KG * G,
        .sphere_of_influence = calculate_sphere_of_influence_r(MOON_SEMIMAJOR_AXIS_KM, MOON_MASS_KG, EARTH_MASS_KG),
    };

    PhysicalState moon_physical_state = (PhysicalState){
        .r = (DVector3){-3.955179399127587E+05,-5.322604944038965E+04,1.063540351362642E+04},
        .v = (DVector3){1.646009855804641E-01,-9.678650138048399E-01,-8.717381215592590E-02},
    };

    OrbitalTreeNode* moon = malloc(sizeof(OrbitalTreeNode));

    moon->physical_params = moon_physical_parameters;
    moon->orbital_elements = (ClassicalOrbitalElements){}; // Empty, using state vector to populate this
    moon->physical_state = moon_physical_state;
    moon->use_state_vector = true; // use our state vector for orbital determination
    moon->is_state_vector_initialized = true;
    moon->parent = earth;
    moon->body_name = "Moon";
    moon->children = darray_init(1,sizeof(OrbitalTreeNode*));
    earth->children = darray_push(earth_children, (void*)moon);

    return earth;
}
