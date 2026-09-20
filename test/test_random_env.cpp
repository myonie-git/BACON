#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "aabb.h"
#include "types.h"

int main(){
    using S = double;
    std::vector<CollisionObject<S>*> collision_objects = randomCollisionObjects<S>(1);
    CollisionEnv<S> env;
    env.InitTree(collision_objects);
    env.dtree.print();
    return 0;
}
