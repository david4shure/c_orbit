#include "test_utils.h"
#include "../src/physics/kepler.h"
#include "../src/physics/constants.h"
#include "time.h"
#include "tree.h"

int main() {
    InitializeLogger(DEBUG, true);
    Debug("Loading eaerth moon system\n");
    OrbitalTreeNode* tree = load_earth_moon_system();


    OrbitalTreeNode* earth = tree;
    OrbitalTreeNode* moon;
    OrbitalTreeNode* satellite;

    darray children = earth->children;

    Debug("setting moon variable\n");
    moon = *(OrbitalTreeNode**)darray_get(earth->children,0);

    Debug("setting satellite variable\n");
    satellite = *(OrbitalTreeNode**)darray_get(earth->children,1);

    Debug("testing first assertion...\n");
    ASSERT_EQ(subtree_has_node(earth, satellite),true);
    Debug("testing second assertion...\n");
    ASSERT_EQ(subtree_has_node(moon, satellite),false);

    // restructure the tree, satellite will get parented under the moon
    restructure_orbital_tree_recursive(tree,tree);

    moon = *(OrbitalTreeNode**)darray_get(earth->children,0);

    satellite = *(OrbitalTreeNode**)darray_get(moon->children,0);

    ASSERT_EQ(subtree_has_node(earth, satellite),true);
    ASSERT_EQ(subtree_has_node(moon, satellite),true);

    darray path = darray_init(3,sizeof(OrbitalTreeNode**));

    path = get_path_to(earth,satellite,path);

    OrbitalTreeNode** first_item = (OrbitalTreeNode**)darray_get(path,0);
    OrbitalTreeNode** second_item = (OrbitalTreeNode**)darray_get(path,1);
    OrbitalTreeNode** third_item = (OrbitalTreeNode**)darray_get(path,2);

    ASSERT_EQ(darray_length(path),3);

    Debug("first = %p\n",(*first_item));
    Debug("second = %p\n",(*second_item));
    Debug("third = %p\n",(*third_item));
    Log("%s\n",(*first_item)->body_name);
    Log("%s\n",(*second_item)->body_name);
    Log("%s\n",(*third_item)->body_name);

    ASSERT_EQ_STR((*first_item)->body_name, "Earth");
    ASSERT_EQ_STR((*second_item)->body_name, "Moon");
    ASSERT_EQ_STR((*third_item)->body_name, "Satellite");
}
