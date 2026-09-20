#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>

int main(){
    using S = double;
    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

    CollisionEnv<S> env;
    env.InitTree(collision_objects);
    // env.dtree.print();

    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; // Replace with your URDF file path
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }
    
    std::ofstream outfile(BACON_SOURCE_DIR "/data/test_aabb_ram_gen_results.txt");

    std::vector<CollisionObject<S>> collision_geometry_;
    for(int i = 0; i < model.collisionGeometries.size(); i++){
        const auto& collisionGeomPtr = model.collisionGeometries[i];
        //需要打印的信息：AABB_RADIUS
        //需要打印的信息：该AABB对应的TF编号（应该是joint)
        //需要打印的信息：AABB_CENTER
        // geom.aabb.center()
        // aabb_radius

        model.collisionGeometries[i]->printAABB_toram(outfile);
    }
    
    outfile.close();

}