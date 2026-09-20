#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "aabb.h"
#include "types.h"

using S = double;

int main() {
    const std::string jointFilePath = BACON_SOURCE_DIR "/data/joint_file.txt";

    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);
    
    CollisionEnv<S> env;
    env.InitTree(collision_objects);

    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; // Replace with your URDF file path
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    std::ifstream jointfile(jointFilePath);
    if (!jointfile.is_open()) {
        std::cerr << "Failed to open joint file: " << jointFilePath << std::endl;
        return 1;
    }

    std::map<std::string, double> jointAngles;

    std::string line;
    while (std::getline(jointfile, line)) {
        std::istringstream iss(line);
        iss >> jointAngles["prbt_joint_1"]
            >> jointAngles["prbt_joint_2"]
            >> jointAngles["prbt_joint_3"]
            >> jointAngles["prbt_joint_4"]
            >> jointAngles["prbt_joint_5"]
            >> jointAngles["prbt_joint_6"];

        std::string rootLink = "prbt_base_link";
        model.setJointAngles(jointAngles);
        model.calculateWorldCoordinates(rootLink);

        std::vector<CollisionObject<S>> collision_geometry_;
        for (int i = 0; i < model.collisionGeometries.size(); i++) {
            const auto& collisionGeomPtr = model.collisionGeometries[i];
            CollisionObject<S> collisionObject(*collisionGeomPtr);
            collision_geometry_.push_back(std::move(collisionObject));
        }

        bool result = false;
        for (int i = 0; i < collision_geometry_.size(); i++) {
            result = env.collide(&collision_geometry_[i], nullptr);
            std::cout << "Collision check result for joint configuration: " << result << std::endl;
        }
    }

    jointfile.close();
    return 0;
}
