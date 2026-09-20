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

namespace fs = std::filesystem;
int main() {
    using S = double;
    const std::string directory = BACON_SOURCE_DIR "/env/32";  
    int filenum = 100; 
    

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {  // 只处理常规文件
            std::string filename = entry.path().string();
            // std::cout << "Processing file: " << filename << std::endl;

            // 读取每个文件中的碰撞对象
            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            // 构建并打印环境
            CollisionEnv<S> env;
            env.InitTree(collision_objects);

            // 读取机器人模型
            URDFModel model;
            std::string urdfFilePath = BACON_SOURCE_DIR "/go1.urdf"; 
            if (!model.loadURDF(urdfFilePath)) {
                std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
                assert(0);
                return -1;
            }

            // 关节角度的名称和初始值
            std::map<std::string, double> jointAngles = {
                {"FL_calf_joint", 0.0},
                {"FL_hip_joint", 0.0},
                {"FL_thigh_joint", 0.0},
                {"FR_calf_joint", 0.0},
                {"FR_hip_joint", 0.0},
                {"FR_thigh_joint", 0.0},
                {"RL_calf_joint", 0.0},
                {"RL_hip_joint", 0.0},
                {"RL_thigh_joint", 0.0},
                {"RR_calf_joint", 0.0},
                {"RR_hip_joint", 0.0},
                {"RR_thigh_joint", 0.0}
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
                                    
                                    jointAngles["FL_calf_joint"] = joint1;
                                    jointAngles["FR_hip_joint"] = joint2;
                                    jointAngles["RL_calf_joint"] = joint3;
                                    jointAngles["RR_calf_joint"] = joint4;
                                    jointAngles["FL_thigh_joint"] = joint5;
                                    jointAngles["FR_hip_joint"] = joint6;

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

                                    std::cout << "Joint Angles: " << joint1 << ", " << joint2 << ", " << joint3 << ", "
                                              << joint4 << ", " << joint5 << ", " << joint6
                                              << " - Collision Detection: " << (result ? "Env Collision" : "No Collision") << std::endl;
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
