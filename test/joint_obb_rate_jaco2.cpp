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
#include <getopt.h>

namespace fs = std::filesystem;
int main(int argc, char *argv[]) {
    using S = double;
    std::string directory = BACON_SOURCE_DIR "/env/32";  
    int filenum = 100; 

    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {0, 0, 0, 0} 
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                directory = optarg;
                break;
        }
    }
    
    // 读取机器人模型
    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/j2n6s200_standalone.urdf"; 
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }


    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {  // 只处理常规文件
            std::string filename = entry.path().string();
            // std::cout << "Processing file: " << filename << std::endl;

            // 读取每个文件中的碰撞对象
            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            // 构建并打印环境
            CollisionEnv<S> env;
            env.InitTree(collision_objects);

            

            // 关节角度的名称和初始值
            std::map<std::string, double> jointAngles = {
                {"j2n6s200_joint_1", 0.0},
                {"j2n6s200_joint_2", 0.0},
                {"j2n6s200_joint_3", 0.0},
                {"j2n6s200_joint_4", 0.0},
                {"j2n6s200_joint_5", 0.0},
                {"j2n6s200_joint_6", 0.0},
                {"j2n6s200_joint_finger_1", 0.0},
                {"j2n6s200_joint_finger_2", 0.0},
                {"j2n6s200_joint_finger_tip_1", 0.0},
                {"j2n6s200_joint_finger_tip_2", 0.0}
            };

            // 定义角度范围和步长
            double start_angle = -1.57;
            double end_angle = 1.57;
            double step_size = 1.0;

            // 执行碰撞检测逻辑
            for (double joint1 = start_angle; joint1 <= end_angle; joint1 += step_size) {
                for (double joint2 = start_angle; joint2 <= end_angle; joint2 += step_size) {
                    for (double joint3 = start_angle; joint3 <= end_angle; joint3 += step_size) {
                        for (double joint4 = start_angle; joint4 <= end_angle; joint4 += step_size) {
                            for (double joint5 = start_angle; joint5 <= end_angle; joint5 += step_size) {
                                for (double joint6 = start_angle; joint6 <= end_angle; joint6 += step_size) {
                                    
                                    jointAngles["j2n6s200_joint_1"] = joint1;
                                    jointAngles["j2n6s200_joint_2"] = joint2;
                                    jointAngles["j2n6s200_joint_3"] = joint3;
                                    jointAngles["j2n6s200_joint_4"] = joint4;
                                    jointAngles["j2n6s200_joint_5"] = joint5;
                                    jointAngles["j2n6s200_joint_6"] = joint6;

                                    std::string rootLink = "root";
                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink);

                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }

                                    // 碰撞检测
                                    bool result = false;
                                    for (int i = 0; i < collision_geometry_.size() && !result; i++) {
                                        result = env.collide(&collision_geometry_[i], nullptr);
                                        // if (result == true) {
                                        //     std::cout << "Collision AT " << i << std::endl;
                                        // }
                                    }

                                    // std::cout << "Joint Angles: " << joint1 << ", " << joint2 << ", " << joint3 << ", "
                                    //           << joint4 << ", " << joint5 << ", " << joint6
                                    //           << " - Collision Detection: " << (result ? "Env Collision" : "No Collision") << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return 0;
}
