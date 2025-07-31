#include "tree.h"
#include "../physics/constants.h"
#include <string.h>
#include <stdio.h>
#include "../utils/logger.h"
#include "../physics/time.h"
#include "../physics/orbital_lines.h"
#include "raylib.h"

// MALLOCS!
OrbitalTreeNode* load_earth_moon_system() {
    PhysicalParameters earth_physical_parameters = (PhysicalParameters){
        .mass=EARTH_MASS_KG,
        .radius=EARTH_RADIUS_KM,
        .grav_param=EARTH_MASS_KG*G,
        .sphere_of_influence = calculate_sphere_of_influence_r(EARTH_SEMIMAJOR_AXIS_KM, EARTH_MASS_KG, SUN_MASS_KG),
    };

    OrbitalTreeNode* earth = malloc(sizeof(OrbitalTreeNode)); 

    earth->physical_params = earth_physical_parameters;
    earth->orbital_elements = (ClassicalOrbitalElements){}; // Earth is not itself orbiting another body in our simulation
    earth->physical_state = (PhysicalState){
        .r = (DVector3){0,0,0},
        .v = (DVector3){0,0,0},
    }; // Earth itself is fixed at (0,0,0)
    earth->body_name = "Earth";
    earth->parent = NULL;
    earth->draw_sphere_of_influence = true;
    earth->body_color = SKYBLUE;
    earth->orbital_lines = NULL; // Initialize orbital lines to NULL
    earth->children = darray_init(10,sizeof(OrbitalTreeNode**));

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
    moon->draw_sphere_of_influence = true;
    moon->parent = earth;
    moon->body_name = "Moon";
    moon->children = darray_init(1,sizeof(OrbitalTreeNode**));
    moon->body_color = (Color){200, 200, 255, 255}; // Light blue
    moon->line_color = (Color){100, 200, 255, 180}; // Bright blue with reduced opacity
    moon->orbital_lines = NULL; // Initialize orbital lines to NULL
    earth->children = darray_push(earth->children, &moon);

    PhysicalParameters satellite_physical_parameters = (PhysicalParameters){
        .mass = 1000, // KG
        .radius = MOON_RADIUS_KM/10.0,
        .grav_param = 1000 * G,
        .sphere_of_influence = calculate_sphere_of_influence_r(SATELLITE_SEMIMAJOR_AXIS, SATELLITE_MASS_KG, MOON_MASS_KG),
    };

    PhysicalState satellite_physical_state = (PhysicalState){
        .r = (DVector3){-0.4855179399127587E+05,0.1322604944038965E+04,0.0063540351362642E+04},
        .v = (DVector3){2.33646009855804641E-01,-2.6678650138048399E-01,-0.117381215592590E-02},
        .mass = satellite_physical_parameters.mass,
    };

    OrbitalTreeNode* satellite = malloc(sizeof(OrbitalTreeNode));

    satellite->physical_params = satellite_physical_parameters;
    satellite->orbital_elements = (ClassicalOrbitalElements){}; // Empty, using state vector to populate this
    satellite->physical_state = satellite_physical_state;
    satellite->use_state_vector = true; // use our state vector for orbital determination
    satellite->is_state_vector_initialized = true;
    satellite->draw_sphere_of_influence = false;
    satellite->parent = moon;
    satellite->body_name = "Satellite";
    satellite->children = darray_init(1,sizeof(OrbitalTreeNode**));
    satellite->body_color = (Color){255, 150, 0, 255}; // Bright orange
    satellite->line_color = (Color){255, 200, 0, 180}; // Golden yellow with reduced opacity
    satellite->orbital_lines = NULL; // Initialize orbital lines to NULL
    moon->children = darray_push(moon->children, &satellite);

    return earth;
}

// It is assumed that for every OrbitalTreeNode, its coordinates are relative to its 
// parents
void update_orbital_tree_recursive(OrbitalTreeNode* root, OrbitalTreeNode* node, PhysicsTimeClock* clock) {
    printf("DEBUG: update_orbital_tree_recursive called for node: %s\n", node->body_name);
    bool is_root_node = node->parent == NULL;

    if (!is_root_node) {
        printf("DEBUG: Processing non-root node: %s\n", node->body_name);
        PhysicalState parent_state = node->parent->physical_state;
        PhysicalState current_state = node->physical_state;

        printf("DEBUG: About to propagate orbital state\n");
        current_state = rv_from_r0v0(current_state, node->parent->physical_params.grav_param, clock->delta_seconds);

        printf("DEBUG: About to compute orbital elements\n");
        node->orbital_elements = rv_to_classical_elements(current_state, node->parent->physical_params.grav_param);

        node->physical_state = current_state;

        if (node->orbital_lines != NULL) {
            printf("DEBUG: Freeing old orbital lines\n");
            darray_free(node->orbital_lines);
            node->orbital_lines = NULL;
        }

        printf("DEBUG: About to compute orbital lines for node: %s\n", node->body_name);
        node->orbital_lines = compute_orbital_lines(
            current_state,
            node->parent->physical_params.grav_param,
            clock->clock_seconds,
            node->parent->physical_params.sphere_of_influence * 5
        );
        printf("DEBUG: Orbital lines computed for node: %s, count: %d\n", node->body_name, 
               node->orbital_lines ? darray_length(node->orbital_lines) : -1);

        printf("DEBUG: About to compute nodes\n");
        node->asc_desc = compute_nodes(node->orbital_elements);
        printf("DEBUG: Nodes computed for node: %s\n", node->body_name);
    }

    for (int i = 0; i < darray_length(node->children) && node->children != NULL; i++) {
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(node->children, i);
        update_orbital_tree_recursive(root,(*child), clock);
    }
}


// Returns list of OrbitalTreeNode in pre order
darray dfs_orbital_tree_nodes(OrbitalTreeNode* tree, darray list) {
    // Store a pointer to a pointer to a OrbitalTreeNode
    // REASON: Darray copies the underlying data into its memory
    // segment that we pass in, we want a list of pointers.

    list = darray_push(list,&tree);

    bool has_children = tree->children != NULL && darray_length(tree->children) > 0;

    if (has_children) {
        for (int i = 0; i < darray_length(tree->children); i++) {
            OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(tree->children,i);
            list = dfs_orbital_tree_nodes(*child,list);
        }
    }

    return list;
}

// Post order traversal of orbital tree to find the current SOI for a given node
OrbitalTreeNode* get_sphere_of_influence_for_node(darray bodies, OrbitalTreeNode* root, OrbitalTreeNode* node) {
    // Assumptions: children of parents have their SOIs within their parent's SOI <- lol jk, would be easier if it was true
    // Assumptions: tree is the root of the orbital tree
    // Iterate through the tree with the following rules in mind

    if(node == NULL || node->parent == NULL) {
        return NULL;
    }

    OrbitalTreeNode* soi_node = node->parent;
    double closest_distance = 100000000000000000000.0;

    if (darray_length(bodies) > 0) {
        // Dont modify the parent of the root node
        if (node->parent == NULL) {
            return NULL;
        }

        for (int j = 0; j < darray_length(bodies); j++) {
            OrbitalTreeNode** other_node = (OrbitalTreeNode**)darray_get(bodies,j);

            if (node != *other_node) {
                DVector3 this_center = get_offset_position_for_node(root,node);
                DVector3 this_position = DVector3Add(this_center,node->physical_state.r);

                DVector3 that_center = get_offset_position_for_node(root,(*other_node));
                DVector3 that_position = DVector3Add(that_center,(*other_node)->physical_state.r);

                double distance_between_nodes = DVector3Distance(this_position,that_position);

                // are we in the soi AND does the compared node indicate that we are using a sphere of influence?
                bool is_in_soi = distance_between_nodes < (*other_node)->physical_params.sphere_of_influence;

                // distance_between_nodes serves as a tiebreaker in the case
                // where we are in two SOIs
                if (is_in_soi == true && distance_between_nodes < closest_distance) {
                    soi_node = *other_node;
                    closest_distance = distance_between_nodes;
                }
            }
        }
    }
    
    // Did something change?
    if(node->parent != soi_node) {
        // Convert coordinates to new body?
        Error("CHANGING SPHERE OF INFLUENCES\n");
    }

    return soi_node;
}

int index_of_node_in_tree(darray bodies, OrbitalTreeNode** node) {
    if (bodies == NULL || node == NULL) {
        return -1;
    }

    for (int i = 0; i < darray_length(bodies); i++) {
        OrbitalTreeNode** other = (OrbitalTreeNode**)darray_get(bodies,i);

        if ((*other)->body_name == (*node)->body_name) {
            return i;
        }
    }

    return -1;
}

// Reconstructs the tree based on sphere of influence etc.
void restructure_orbital_tree_recursive(OrbitalTreeNode* root, OrbitalTreeNode* tree) {
    // Iterate through all OrbitalTreeNodes in tree
    // For each out, iterate through the rest

    darray bodies = darray_init(10,sizeof(OrbitalTreeNode**));
    bodies = dfs_orbital_tree_nodes(tree,bodies);

    for (int j = 0; j < darray_length(bodies); j++) {
        OrbitalTreeNode** node = (OrbitalTreeNode**)darray_get(bodies,j);
        OrbitalTreeNode* soi = get_sphere_of_influence_for_node(bodies, root, *node);

        // Nothing to do, root node
        if(soi == NULL) {
            continue;
        }

        // Modify the tree structure
        OrbitalTreeNode* parent = (*node)->parent;

        if (parent == NULL) {
            continue;
        }

        /* // Remove node from it's parent's children */
        int index_of_this_child = index_of_node_in_tree(parent->children, node);

        if (index_of_this_child == -1) {
            Warn("Index of child is -1\n");
            continue;
        }

        if (soi == (*node)->parent) {
            continue;
        }

        Warn("%s is now in %s's sphere of influence... updating tree.",(*node)->body_name, soi->body_name);

        // Update nodes current position/velocity based on parents frame
        PhysicalState global_offset = get_global_state_for_node(root, *node);

        // Global state from there
        PhysicalState global_state;

        // Compute global r and v
        global_state.r = DVector3Add(global_offset.r,(*node)->physical_state.r);
        global_state.v = DVector3Add(global_offset.v,(*node)->physical_state.v);

        print_physical_state(global_state);

        // Now compute global offset of new parent 
        PhysicalState new_parent_global_offset = get_global_state_for_node(root, soi);

        // Compute global state of soi node
        PhysicalState new_parent_global_state;

        // Compute global r and v of parent
        new_parent_global_state.r = DVector3Add(new_parent_global_offset.r,soi->physical_state.r);
        new_parent_global_state.v = DVector3Add(new_parent_global_offset.v,soi->physical_state.v);

        print_physical_state(new_parent_global_state);

        // Now assign the new position of the current node as the 
        // difference between its global state and the soi nodes global state
        (*node)->physical_state.r = DVector3Subtract(global_state.r,new_parent_global_state.r);
        (*node)->physical_state.v = DVector3Subtract(global_state.v,new_parent_global_state.v);

        print_physical_state((*node)->physical_state);

        // Pop the child from its old parents array
        parent->children = darray_pop_at(parent->children, index_of_this_child);

        // Push the current node to the SOI's children
        soi->children = darray_push(soi->children,node);

        // Register the soi as the current node's parent
        (*node)->parent = soi;
    }

    darray_free(bodies);
}

bool subtree_has_node(OrbitalTreeNode* tree, OrbitalTreeNode* node) {
    // Handle edge cases
    if (tree == NULL || node == NULL) {
        return false;
    }

    // Base case: we found the node
    if (tree == node) {
        return true;
    }

    // If the node has no children, return false
    if (tree->children == NULL || darray_length(tree->children) == 0) {
        return false;
    }

    // Recursive case: search through the children
    int num_children = darray_length(tree->children);
    for (int i = 0; i < num_children; i++) {
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(tree->children, i);
        if (subtree_has_node(*child, node)) {
            return true;
        }
    }

    // If not found in any child, return false
    return false;
}

// DFS: Returns a list of nodes that represents the path from root to node_to_find
darray get_path_to(OrbitalTreeNode* root, OrbitalTreeNode* node_to_find, darray arr_of_nodes) {
    if (root == NULL) {
        return arr_of_nodes;
    }

    if (root == node_to_find) {
        return darray_push(arr_of_nodes,&root);
    }

    arr_of_nodes = darray_push(arr_of_nodes, &root);

    for (int i = 0; root->children != NULL && i < darray_length(root->children); i++) {
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(root->children,i);

        if (subtree_has_node(*child, node_to_find)) {
            arr_of_nodes = get_path_to(*child, node_to_find,arr_of_nodes);
        }
    }

    return arr_of_nodes;
}

PhysicalState get_global_state_for_node(OrbitalTreeNode* root, OrbitalTreeNode* node) {
    darray path = darray_init(5,sizeof(OrbitalTreeNode**));
    path = get_path_to(root, node, path);

    for (int i = 0; i < darray_length(path); i++) {
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(path,i);
    }

    PhysicalState state = root->physical_state;

    if (node == root) {
        return (PhysicalState){
            state.r,
            state.v,
        };
    }

    for (int i = 0; i < darray_length(path); i++) {
        OrbitalTreeNode** item = (OrbitalTreeNode**)darray_get(path,i);
        if ((*item) != node) {
            state.r = DVector3Add(state.r,(*item)->physical_state.r);
            state.v = DVector3Add(state.v,(*item)->physical_state.v);
        }
    }

    darray_free(path);

    return state;
}

DVector3 get_offset_position_for_node(OrbitalTreeNode* root, OrbitalTreeNode* node) {
    PhysicalState s = get_global_state_for_node(root, node);
    return s.r;
}

PhysicalState global_physical_state(OrbitalTreeNode* root, OrbitalTreeNode* node) {
    // global offset 
    PhysicalState offset = get_global_state_for_node(root, node);

    // Compute global state of soi node
    PhysicalState state;

    // Compute global r and v of parent
    state.r = DVector3Add(offset.r,node->physical_state.r);
    state.v = DVector3Add(offset.v,node->physical_state.v);

    return state;
}
