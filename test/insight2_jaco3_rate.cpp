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
    std::string directory = BACON_SOURCE_DIR "/env/48/";  
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
    std::string urdfFilePath = BACON_SOURCE_DIR "/j2n6s300_standalone.urdf"; 
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }
    
    // 碰撞统计变量
    std::vector<int> collision_counts;  // 用于统计每个几何体的碰撞次数
    bool counts_initialized = false;   // 标记是否已初始化计数器
    
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
                {"j2n6s300_joint_1", 0.0},
                {"j2n6s300_joint_2", 0.0},
                {"j2n6s300_joint_3", 0.0},
                {"j2n6s300_joint_4", 0.0},
                {"j2n6s300_joint_5", 0.0},
                {"j2n6s300_joint_6", 0.0},
                {"j2n6s300_joint_finger_1", 0.0},
                {"j2n6s300_joint_finger_2", 0.0},
                {"j2n6s300_joint_finger_3", 0.0},
                {"j2n6s300_joint_finger_tip_1", 0.0},
                {"j2n6s300_joint_finger_tip_2", 0.0},
                {"j2n6s300_joint_finger_tip_3", 0.0}
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
                                    
                                    jointAngles["j2n6s300_joint_1"] = joint1;
                                    jointAngles["j2n6s300_joint_2"] = joint2;
                                    jointAngles["j2n6s300_joint_3"] = joint3;
                                    jointAngles["j2n6s300_joint_4"] = joint4;
                                    jointAngles["j2n6s300_joint_5"] = joint5;
                                    jointAngles["j2n6s300_joint_6"] = joint6;

                                    std::string rootLink = "root";
                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink);

                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }

                                    // 初始化碰撞计数器（仅在第一次时）
                                    if (!counts_initialized) {
                                        collision_counts.resize(collision_geometry_.size(), 0);
                                        counts_initialized = true;
                                    }

                                    // 碰撞检测 - 检查所有几何体并统计碰撞次数
                                    bool overall_collision = false;
                                    for (int i = 0; i < collision_geometry_.size(); i++) {
                                        bool result = env.collide(&collision_geometry_[i], nullptr);
                                        if (result) {
                                            collision_counts[i]++;  // 增加该几何体的碰撞计数
                                            overall_collision = true;
                                            // std::cout << "Collision detected at geometry " << i << std::endl;
                                        }
                                    }

                                    // std::cout << "Joint Angles: " << joint1 << ", " << joint2 << ", " << joint3 << ", "
                                    //           << joint4 << ", " << joint5 << ", " << joint6
                                    //           << " - Overall Collision: " << (overall_collision ? "Yes" : "No") << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 输出碰撞统计结果
    std::cout << "\n=== Collision Statistics ===" << std::endl;
    std::cout << "Geometry Index | Collision Count" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    for (int i = 0; i < collision_counts.size(); i++) {
        std::cout << "     " << i << "         |      " << collision_counts[i] << std::endl;
    }
    
    // 计算并输出总体统计
    int total_collisions = 0;
    int max_collisions = 0;
    int min_collisions = (collision_counts.empty() ? 0 : collision_counts[0]);
    for (int count : collision_counts) {
        total_collisions += count;
        max_collisions = std::max(max_collisions, count);
        min_collisions = std::min(min_collisions, count);
    }
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Total collision events: " << total_collisions << std::endl;
    std::cout << "Max collisions for single geometry: " << max_collisions << std::endl;
    std::cout << "Min collisions for single geometry: " << min_collisions << std::endl;
    if (!collision_counts.empty()) {
        std::cout << "Average collisions per geometry: " << (double)total_collisions / collision_counts.size() << std::endl;
    }

    return 0;
}