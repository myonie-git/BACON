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

    //build the env
    using S = double;
    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

    CollisionEnv<S> env;
    env.InitTree(collision_objects);
    // env.dtree.print();
    
    //read the robot model
    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; // Replace with your URDF file path
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    //set joint value
    std::map<std::string, double> jointAngles = {
        {"prbt_joint_1", 0.0},
        {"prbt_joint_2", 0.0},
        {"prbt_joint_3", 0.0},
        {"prbt_joint_4", 0.0},
        {"prbt_joint_5", 0.0},
        {"prbt_joint_6", 0.0}
    };
    
    std::string rootLink = "prbt_base_link";
    model.setJointAngles(jointAngles);
    model.calculateWorldCoordinates(rootLink);

    //创建一个由COLLISIONOBJECT组成的向量，表示机器人的碰撞物
    bool result = false;
    std::vector<CollisionObject<S>> collision_geometry_;

    //TODO把Model转换成Collision_geometry
    
    std::cout << model.getCollisionNum() << std::endl;
    for (const auto& collisionGeomPtr : model.collisionGeometries) {
        CollisionObject<S> collisionObject(*collisionGeomPtr);
        collision_geometry_.push_back(std::move(collisionObject));
    }

    for(int i = 0; i < collision_geometry_.size() && !result; i++){
        result = env.collide(&collision_geometry_[i], nullptr);
    }

    return 0;
}