#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>


using S = double;

S from_fixed_point(int32_t value) {
    return static_cast<S>(value) / (1 << 16);
}

int main() {
    // 打开生成的测试结果文件
    std::ifstream infile(BACON_SOURCE_DIR "/data/top_test_results.txt");
    if (!infile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

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

    int line_count = 0;
    int passed_tests = 0, failed_tests = 0;
    std::string line;

    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string hex_value;
        std::map<std::string, S> joint_angles;
        bool result, collision_detected  = false; 

        ss >> hex_value; jointAngles["prbt_joint_1"] = from_fixed_point(static_cast<int32_t>(std::stoul(hex_value, nullptr, 16)));
        ss >> hex_value; jointAngles["prbt_joint_2"] = from_fixed_point(static_cast<int32_t>(std::stoul(hex_value, nullptr, 16)));
        ss >> hex_value; jointAngles["prbt_joint_3"] = from_fixed_point(static_cast<int32_t>(std::stoul(hex_value, nullptr, 16)));
        ss >> hex_value; jointAngles["prbt_joint_4"] = from_fixed_point(static_cast<int32_t>(std::stoul(hex_value, nullptr, 16)));
        ss >> hex_value; jointAngles["prbt_joint_5"] = from_fixed_point(static_cast<int32_t>(std::stoul(hex_value, nullptr, 16)));
        ss >> hex_value; jointAngles["prbt_joint_6"] = from_fixed_point(static_cast<int32_t>(std::stoul(hex_value, nullptr, 16)));


        std::string rootLink = "prbt_base_link";
        model.setJointAngles(jointAngles);
        model.calculateWorldCoordinates(rootLink);
        
        std::vector<CollisionObject<S>> collision_geometry_;
        for (int i = 0; i < model.collisionGeometries.size(); i++) {
            const auto& collisionGeomPtr = model.collisionGeometries[i];
            CollisionObject<S> collisionObject(*collisionGeomPtr);
            collision_geometry_.push_back(std::move(collisionObject));
        }

        for (int i = 0; i < collision_geometry_.size() && !collision_detected; i++) {
            collision_detected = env.collide(&collision_geometry_[i], nullptr);
            if(collision_detected == true){
                std::cout << "Collision AT " << i << std::endl; 
            }
        }

        ss >> result;

        if (collision_detected == result) {
            ++passed_tests;
        } else {
            std::cerr << "Test failed at line " << line_count + 1 << ": collision result mismatch." << std::endl;
            ++failed_tests;
        }

        ++line_count;
    }

    infile.close();

    // 输出测试结果
    std::cout << "Total tests: " << line_count << std::endl;
    std::cout << "Passed tests: " << passed_tests << std::endl;
    std::cout << "Failed tests: " << failed_tests << std::endl;

    return 0;
}