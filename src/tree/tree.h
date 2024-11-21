#pragma once

#ifndef ORBITTREE_H
#define ORBITTREE_H

#include "../physics/kepler.h"
#include "../utils/darray.h"
#include "stdbool.h"

typedef struct OrbitalTreeNode {
    // Physical Parameters & Information about body, IE mass, mu, physical radius, etc
    PhysicalParameters physical_params;
    // Classic Keplerian Orbital Elements
    ClassicalOrbitalElements orbital_elements;
    // Position / Velocity Vectors
    PhysicalState physical_state;
    // Positions of ascending and descending nodes
    OrbitalNodes asc_desc;
    bool is_state_vector_initialized;
    // Do we render it's initial position from its state vector? 
    // if not, we default to the ClassicalOrbitalElements
    bool use_state_vector;
    // Name (i.e. "Moon")
    char* body_name;
    // Parent node
    struct OrbitalTreeNode* parent;
    // Children (darray<OrbitalTreeNode*> type) MALLOC
    darray children;
    // Orbital lines (darray<PointBundle> type) MALLOC
    darray orbital_lines;
} OrbitalTreeNode;

// Mallocs things FYI
OrbitalTreeNode* load_earth_moon_system();

#endif
