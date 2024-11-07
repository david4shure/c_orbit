#pragma once

#ifndef ORBITTREE_H
#define ORBITTREE_H

#include "../physics/kepler.h"
#include "../utils/darray.h"

typedef struct OrbitalTreeNode {
    // Physical Parameters & Information about body, IE mass, mu, physical radius, etc
    PhysicalParameters* physical_params;
    // Keplerian Orbital Elements
    OrbitalElements* elements;
    // Position / Velocity Vectors
    PhysicalState* state;
    // Name (i.e. "Moon")
    char* body_name;
    // Parent node
    struct OrbitalTreeNode* parent;
    // Children (darray<OrbitalTreeNode> type)
    darray children;
} OrbitalTreeNode;

OrbitalTreeNode* load_orbital_tree_from_file(char* filename) {
    
}

#endif
