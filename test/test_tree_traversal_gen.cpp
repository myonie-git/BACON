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

int main(){

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

    std::ofstream outfile(BACON_SOURCE_DIR "/data/tree_traversal_test_results.txt");
    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    std::ofstream jointfile(BACON_SOURCE_DIR "/data/joint_file.txt");
    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    auto write_obb = [&](const OBB<S>& obb) {
        outfile << std::hex << std::setfill('0');
        // 写入旋转矩阵元素
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                outfile << "0x" << std::setw(8) << to_fixed_point(obb.axis(i, j)) << " ";
            }
        }
        // 写入中心点
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.z()) << " ";
        // 写入尺寸
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.z()) << " ";
    };
    
    auto write_aabb = [&](const AABB<S>& aabb) {
        outfile << std::hex << std::setfill('0');
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.min_.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.min_.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.min_.z()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.max_.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.max_.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.max_.z()) << " ";
    };

    for(int i = 0; i < 100; i++){
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

        jointfile << jointAngles["prbt_joint_1"] << " ";
        jointfile << jointAngles["prbt_joint_2"] << " ";
        jointfile << jointAngles["prbt_joint_3"] << " ";
        jointfile << jointAngles["prbt_joint_4"] << " ";
        jointfile << jointAngles["prbt_joint_5"] << " ";
        jointfile << jointAngles["prbt_joint_6"] << " ";
        jointfile << std::endl;

        std::vector<CollisionObject<S>> collision_geometry_;
        // 循环将当前角度下的每个机械臂的collision生成为CollisionObject,用于后续的碰撞检测
        for (int i = 0; i < model.collisionGeometries.size(); i++) {
            const auto& collisionGeomPtr = model.collisionGeometries[i];
            CollisionObject<S> collisionObject(*collisionGeomPtr);
            collision_geometry_.push_back(std::move(collisionObject));
        }

        bool result = false;
        for(int i = 0; i < collision_geometry_.size(); i++){
            result = env.collide(&collision_geometry_[i], nullptr);
            write_aabb(collision_geometry_[i].aabb);
            write_obb(collision_geometry_[i].obb);
            outfile << result << std::endl;
        }
    }
    
    jointfile.close();
    outfile.close();
    std::cout << "Random tree traversal test data and results have been written to tree_traversal_test_results.txt" << std::endl;
}