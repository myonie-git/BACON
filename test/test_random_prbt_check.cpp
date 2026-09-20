#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>
#include <filesystem>
#include <iomanip>


namespace fs = std::filesystem;

int main(){
    
    
    using S = double;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> random_joint(-3.14, 3.14);

    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

    CollisionEnv<S> env;
    env.InitTree(collision_objects);

    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; 

    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

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

    std::ofstream resultFile(BACON_SOURCE_DIR "/exp/exp4/prbt/collision_results.csv");
    if (!resultFile.is_open()) {
        std::cerr << "Failed to open result file." << std::endl;
        return -1;
    }
    resultFile << "Test";
    for (int i = 0; i < model.collisionGeometries.size(); i++) {
        resultFile << ",CollisionObject_" << i;
    }
    resultFile << std::endl;


    for(int test = 0; test < 100000; test++){
        
        double joint1 = random_joint(gen);
        double joint2 = random_joint(gen);
        double joint3 = random_joint(gen);
        double joint4 = random_joint(gen);
        double joint5 = random_joint(gen);
        double joint6 = random_joint(gen); 
    
        jointAngles["prbt_joint_1"] = joint1;
        jointAngles["prbt_joint_2"] = joint2;
        jointAngles["prbt_joint_3"] = joint3;
        jointAngles["prbt_joint_4"] = joint4;
        jointAngles["prbt_joint_5"] = joint5;
        jointAngles["prbt_joint_6"] = joint6;

        model.setJointAngles(jointAngles);
        model.calculateWorldCoordinates(rootLink);

        std::vector<CollisionObject<S>> collision_geometry_;
        for (int i = 0; i < model.collisionGeometries.size(); i++) {
            const auto& collisionGeomPtr = model.collisionGeometries[i];
            CollisionObject<S> collisionObject(*collisionGeomPtr);
            collision_geometry_.push_back(std::move(collisionObject));
        }

        std::vector<bool> result(collision_geometry_.size());
        for(int i = 0; i < collision_geometry_.size(); i++){
            result[i] = env.collide(&collision_geometry_[i], nullptr);
        }

        resultFile << test; 
        for (bool r : result) {
            resultFile << "," << r; 
        }
        resultFile << std::endl; 

    }

    return 0;
}