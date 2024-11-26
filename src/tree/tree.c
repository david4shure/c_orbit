#include "tree.h"
#include "../physics/constants.h"
#include <string.h>
#include "../utils/logger.h"
#include "../physics/time.h"
#include "../physics/orbital_lines.h"
#include "raylib.h"

OrbitalTreeNode* load_earth_moon_system() {
    PhysicalParameters earth_physical_parameters = (PhysicalParameters){
        .mass=EARTH_MASS_KG,
        .radius=EARTH_RADIUS_KM,
        .grav_param=EARTH_MASS_KG*G,
        .sphere_of_influence = calculate_sphere_of_influence_r(EARTH_SEMIMAJOR_AXIS_KM, EARTH_MASS_KG, SUN_MASS_KG),
    };

    OrbitalTreeNode* earth = malloc(sizeof(OrbitalTreeNode)); 
    memset(earth, 0, sizeof(OrbitalTreeNode)); // Zero out all fields


    darray earth_children = darray_init(10, sizeof(OrbitalTreeNode**));

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
    moon->draw_sphere_of_influence = true;
    moon->parent = earth;
    moon->body_name = "Moon";
    moon->children = darray_init(1,sizeof(OrbitalTreeNode**));
    moon->body_color = LIGHTGRAY;
    moon->line_color = LIGHTGRAY;
    earth->children = darray_push(earth_children, &moon);

    PhysicalParameters satellite_physical_parameters = (PhysicalParameters){
        .mass = 1000, // KG
        .radius = MOON_RADIUS_KM/10.0,
        .grav_param = 1000 * G,
        .sphere_of_influence = calculate_sphere_of_influence_r(SATELLITE_SEMIMAJOR_AXIS, SATELLITE_MASS_KG, MOON_MASS_KG),
    };

    PhysicalState satellite_physical_state = (PhysicalState){
        .r = (DVector3){-3.955179399127587E+05,-5.322604944038965E+04,1.463540351362642E+04},
        .v = (DVector3){1.646009855804641E-01,-8.678650138048399E-01,-2.717381215592590E-02},
    };

    OrbitalTreeNode* satellite = malloc(sizeof(OrbitalTreeNode));

    satellite->physical_params = satellite_physical_parameters;
    satellite->orbital_elements = (ClassicalOrbitalElements){}; // Empty, using state vector to populate this
    satellite->physical_state = satellite_physical_state;
    satellite->use_state_vector = true; // use our state vector for orbital determination
    satellite->is_state_vector_initialized = true;
    satellite->draw_sphere_of_influence = false;
    satellite->parent = earth;
    satellite->body_name = "Satellite";
    satellite->children = darray_init(1,sizeof(OrbitalTreeNode**));
    satellite->body_color = ORANGE;
    satellite->line_color = ORANGE;
    moon->children = darray_push(moon->children, &satellite);

    return earth;
}

// It is assumed that for every OrbitalTreeNode, its coordinates are relative to its 
// parents
void update_orbital_tree_recursive(OrbitalTreeNode* root, OrbitalTreeNode* node, PhysicsTimeClock clock) {
    bool is_root_node = node->parent == NULL;

    if (!is_root_node) {
        // Step 1: Compute relative state
        PhysicalState parent_state = node->parent->physical_state;

        PhysicalState current_state = node->physical_state;

        // Step 2: Propagate relative state using parent parameters
        current_state = rv_from_r0v0(current_state, node->parent->physical_params.grav_param, clock.delta_seconds);

        // Step 3: Update orbital elements
        node->orbital_elements = rv_to_classical_elements(current_state, node->parent->physical_params.grav_param);

        // Step 5: Compute orbital lines in the parent-relative frame
        if (node->orbital_lines != NULL) {
            darray_free(node->orbital_lines);
            node->orbital_lines = NULL;
        }

        node->orbital_lines = compute_orbital_lines(
            current_state,
            node->parent->physical_params.grav_param,
            clock.clock_seconds,
            node->parent->physical_params.sphere_of_influence * 5
        );
    }

    // Step 7: Update children recursively
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
    Warn("traversing %s\n",tree->body_name);
    Debug("tree = %p\n",tree);

    Debug("pushing to array\n");
    list = darray_push(list,&tree);

    bool has_children = tree->children != NULL && darray_length(tree->children) > 0;

    Debug("has children = %d\n",has_children);

    if (has_children) {
        Debug("iterating over children of %s\n",tree->body_name);
        Debug("len(children) = %d\n",darray_length(tree->children));
        for (int i = 0; i < darray_length(tree->children); i++) {
            OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(tree->children,i);
            list = dfs_orbital_tree_nodes(*child,list);
        }
    }

    return list;
}

// Post order traversal of orbital tree to find the current SOI for a given node
OrbitalTreeNode* get_sphere_of_influence_for_node(darray bodies, OrbitalTreeNode* node) {
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
            OrbitalTreeNode** other_node= (OrbitalTreeNode**)darray_get(bodies,j);

            Debug("Checking if %s is in %s's soi\n",node->body_name,(*other_node)->body_name);

            if (node != *other_node) {
                double distance_between_nodes = DVector3Distance(node->physical_state.r,(*other_node)->physical_state.r);

                // are we in the soi AND does the compared node indicate that we are using a sphere of influence?
                bool is_in_soi = distance_between_nodes < (*other_node)->physical_params.sphere_of_influence;

                Debug("distance between %s and %s = %.3f, soi = %.3f, in_soi?=%d\n",node->body_name,(*other_node)->body_name, distance_between_nodes,(*other_node)->physical_params.sphere_of_influence,is_in_soi);
                Debug("is true=%d\n",is_in_soi);

                // distance_between_nodes serves as a tiebreaker in the case
                // where we are in two SOIs
                if (is_in_soi == true && distance_between_nodes < closest_distance) {
                    Debug("trying to assign for %s and other %s\n",node->body_name,(*other_node)->body_name);
                    soi_node = *other_node;
                    closest_distance = distance_between_nodes;
                }
            }
        }
    }
    
    Debug("returning ... %s for %s\n",soi_node->body_name, node->body_name);

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
void restructure_orbital_tree_recursive(OrbitalTreeNode* tree) {
    // Iterate through all OrbitalTreeNodes in tree
    // For each out, iterate through the rest

    darray bodies = darray_init(10,sizeof(OrbitalTreeNode**));
    bodies = dfs_orbital_tree_nodes(tree,bodies);

    for (int j = 0; j < darray_length(bodies); j++) {
        OrbitalTreeNode** node = (OrbitalTreeNode**)darray_get(bodies,j);
        Warn("Computing SOI for.. %s\n",(*node)->body_name);
        OrbitalTreeNode* soi = get_sphere_of_influence_for_node(bodies, *node);
        Warn("SOI = %p\n",soi);

        // Nothing to do, root node
        if(soi == NULL) {
            Debug("Skipping root node...\n");
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

        // Pop the child from its old parents array
        parent->children = darray_pop_at(parent->children, index_of_this_child);

        // Push the current node to the SOI's children
        soi->children = darray_push(soi->children,node);

        // Register the soi as the current node's parent
        (*node)->parent = soi;

        Info("SOI of %s is %s\n",(*node)->body_name,soi->body_name);
    }

    darray_free(bodies);
}

bool subtree_has_node(OrbitalTreeNode* tree, OrbitalTreeNode* node) {
    // Handle edge cases
    if (tree == NULL || node == NULL) {
        return false;
    }

    // Recursive base case, we found our node
    if (tree == node) {
        return true;
    }
    
    bool has_node = false;

    for (int i = 0; tree->children != NULL && i < darray_length(tree->children); i++) {
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(tree->children,i);
        has_node = subtree_has_node(*child, node);

        if (has_node) {
            break;
        }
    }

    return has_node;
}

// Returns a list of nodes that represents the path from root to node_to_find
darray get_path_to(OrbitalTreeNode* root, OrbitalTreeNode* node_to_find, darray arr_of_nodes) {
    // Base case: if root is NULL, return the current path (empty or partially constructed)
    if (root == NULL) {
        return arr_of_nodes;
    }
    
    // Add the current node to the path
    arr_of_nodes = darray_push(arr_of_nodes, &root);

    // If the current node is the one we are looking for, return the path
    if (root == node_to_find) {
        return arr_of_nodes;
    }

    // Recursively check each child
    for (int i = 0; root->children != NULL && i < darray_length(root->children); i++) {
        OrbitalTreeNode** child = (OrbitalTreeNode**)darray_get(root->children, i);

        // Call get_path_to recursively on the child
        darray result = get_path_to(*child, node_to_find, arr_of_nodes);
        
        // If the result contains the node_to_find, return it
        if (darray_length(result) > darray_length(arr_of_nodes)) {
            return result;
        }
    }

    // If the node_to_find is not in this subtree, backtrack by removing the current node
    darray_pop(arr_of_nodes);
    return arr_of_nodes;
}

DVector3 get_global_offset_for_node(OrbitalTreeNode* root, OrbitalTreeNode* node) {
    Error("START for root=%s,node=%s\n",root->body_name,node->body_name);
    darray path = darray_init(5,sizeof(OrbitalTreeNode**));
    path = get_path_to(root, node, path);

    DVector3 center = {0,0,0};

    if (node == root) {
        Error("END, 0 0 0\n");
        return center;
    }

    for (int i = 0; i < darray_length(node->children); i++) {
        OrbitalTreeNode** item = (OrbitalTreeNode**)darray_get(node->children,i);
        Error("offsetting for %s\n",(*item)->body_name);
        center = DVector3Add(center,(*item)->physical_state.r);
    }

    darray_free(path);

    Error("END global position of %s = (%.3f,%3.f,%.3f)\n",node->body_name,center.x,center.y,center.z);

    return center;
}
