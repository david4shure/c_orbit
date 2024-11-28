#pragma once

#ifndef ORBITTREE_H
#define ORBITTREE_H

#include "../physics/kepler.h"
#include "../utils/darray.h"
#include "stdbool.h"
#include "../physics/time.h"
#include "raylib.h"

typedef struct OrbitalTreeNode {
    // Physical Parameters & Information about body, IE mass, mu, physical radius, etc
    PhysicalParameters physical_params;
    // Classic Keplerian Orbital Elements
    ClassicalOrbitalElements orbital_elements;
    // Position / Velocity Vectors
    PhysicalState physical_state;
    // Position / Velocity Vectors
    PhysicalState global_physical_state;
    // Positions of ascending and descending nodes
    OrbitalNodes asc_desc;
    bool is_state_vector_initialized;
    // Do we render it's initial position from its state vector? 
    // if not, we default to the ClassicalOrbitalElements
    bool use_state_vector;
    // Whether or not to render / calculate sphere of influence
    bool draw_sphere_of_influence;
    // Name (i.e. "Moon")
    char* body_name;
    // Parent node
    struct OrbitalTreeNode* parent;
    // Children (darray<OrbitalTreeNode**> type) MALLOC
    darray children;
    // Color of body sphere
    Color body_color;
    // Color of orbital lines
    Color line_color;
    // Orbital lines (darray<PointBundle> type) MALLOC
    darray orbital_lines;
} OrbitalTreeNode;

// Mallocs things FYI
OrbitalTreeNode* load_earth_moon_system();

// Returns vector of orbital tree nodes in pre order
darray dfs_orbital_tree_nodes(OrbitalTreeNode* n, darray list);

// Updates the state of our Orbital Tree recursively
void update_orbital_tree_recursive(OrbitalTreeNode* root, OrbitalTreeNode* node, PhysicsTimeClock* clock);

// Post order traversal of orbital tree to find the current SOI for a given node
OrbitalTreeNode* get_sphere_of_influence_for_node(darray bodies, OrbitalTreeNode* root, OrbitalTreeNode* node);

// Reconstructs the tree based on sphere of influence etc.
void restructure_orbital_tree_recursive(OrbitalTreeNode* root, OrbitalTreeNode* tree);

// Does tree have node beneath it?
bool subtree_has_node(OrbitalTreeNode* tree, OrbitalTreeNode* node);

// Construct a list of nodes that is a path from root to node_to_find
darray get_path_to(OrbitalTreeNode* root, OrbitalTreeNode* node_to_find, darray arr_of_nodes);

// Gets a Physical State in 3d space corresponding to the current node's global Physical State (position+velocity)
PhysicalState get_global_state_for_node(OrbitalTreeNode* root, OrbitalTreeNode* node);

// Gets a position in 3d space corresponding to the current nodes global position
DVector3 get_offset_position_for_node(OrbitalTreeNode* root, OrbitalTreeNode* node);

PhysicalState global_physical_state(OrbitalTreeNode* root, OrbitalTreeNode* node);

#endif
