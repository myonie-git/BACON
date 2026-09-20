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

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> random_joint(-3.14, 3.14);

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

    std::map<std::string, double> jointAngles = {
        {"prbt_joint_1", 0.0},
        {"prbt_joint_2", 0.0},
        {"prbt_joint_3", 0.0},
        {"prbt_joint_4", 0.0},
        {"prbt_joint_5", 0.0},
        {"prbt_joint_6", 0.0}
    };

    std::ofstream outfile(BACON_SOURCE_DIR "/data/top_test_results.txt");
    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }


    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    for(int i = 0; i < 10000; i++){
        //随机生成六个角度
        
        jointAngles["prbt_joint_1"] = random_joint(gen);
        jointAngles["prbt_joint_2"] = random_joint(gen);
        jointAngles["prbt_joint_3"] = random_joint(gen);
        jointAngles["prbt_joint_4"] = random_joint(gen);
        jointAngles["prbt_joint_5"] = random_joint(gen);
        jointAngles["prbt_joint_6"] = random_joint(gen);
        
        std::string rootLink = "prbt_base_link";
        model.setJointAngles(jointAngles);
        model.calculateWorldCoordinates(rootLink);

        outfile << std::hex << std::setfill('0');
        outfile << "0x" << std::setw(8) << to_fixed_point(jointAngles["prbt_joint_1"]) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(jointAngles["prbt_joint_2"]) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(jointAngles["prbt_joint_3"]) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(jointAngles["prbt_joint_4"]) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(jointAngles["prbt_joint_5"]) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(jointAngles["prbt_joint_6"]) << " ";

        std::vector<CollisionObject<S>> collision_geometry_;
        for (int i = 0; i < model.collisionGeometries.size(); i++) {
            const auto& collisionGeomPtr = model.collisionGeometries[i];
            CollisionObject<S> collisionObject(*collisionGeomPtr);
            collision_geometry_.push_back(std::move(collisionObject));
        }

        bool result = false;
        for(int i = 0; i < collision_geometry_.size() && !result; i++){
            result = env.collide(&collision_geometry_[i], nullptr);
        }
        outfile << result << std::endl;
    }

    outfile.close();
}