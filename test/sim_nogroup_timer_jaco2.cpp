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
#include <chrono> 

namespace fs = std::filesystem;

struct GroupInfo{
    std::vector<int> indices;
};


// Define groups
std::vector<GroupInfo> groups = {
    {{0, 1, 2, 3}},
    {{4, 5, 6, 7}},
    {{8, 9, 10}},
};

#define NUM_TREE_TRAVERSAL 3
#define NUM_TF 1
#define NUM_PARA 10
#define CLK_FREQ 40000000

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

std::string urdfFilePath = BACON_SOURCE_DIR "/j2n6s200_standalone.urdf"; 
std::string rootLink = "root";

int main(int argc, char *argv[]) {
    using S = double;
    // std::string directory = BACON_SOURCE_DIR "/env/48-bak";
    std::string directory = BACON_SOURCE_DIR "/env/48";
    
    int filenum = 100; 
    
    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {0, 0, 0, 0}
    };

    int timer = 0;

    int opt;
    while ((opt = getopt_long(argc, argv, "d:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                directory = optarg;
                break;
        }
    }
    
    // Define constants for timing
    // const int AABB_CHECK_TIME = 1;
    // const int OBB_CHECK_TIME = 6;
    
    // 读取机器人模型
    URDFModel model;
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }


    // 遍历目录中的每个文件
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {  // 只处理常规文件
            std::string filename = entry.path().string();
            // std::cout << "Processing file: " << filename << std::endl;

            // 读取每个文件中的碰撞对象
            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);
            CollisionEnv<S> env;
            env.InitTree(collision_objects);

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
                                    
                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink, timer);
                                    
                                    // Prepare collision geometries
                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }
                                    timer += 6 * ((model.collisionGeometries.size() + NUM_TF - 1) / NUM_TF);

                                    // Initialize group available times
                                    std::vector<int> group_available_time(groups.size(), 0);
                                    
                                    for(size_t g = 0; g < groups.size(); ++g){
                                        int test_timer = 0;
                                        bool result = false;
                                        int local_aabb_timer = 0;
                                        for (int idx : groups[g].indices) {
                                            if (result) break;
                                            result = env.collide(&collision_geometry_[idx], nullptr, test_timer);
                                        }
                                        // std::cout << "test_timer" << test_timer <<  std::endl;
                                        group_available_time[g] = test_timer;
                                    }

                                    // std::cout << "timer0 " << timer << std::endl;
                                    timer = timer + *std::max_element(group_available_time.begin(), group_available_time.end());
                                }
                            }
                        }
                    }
                }
            }
        }
    }  
    
    // std::cout << "timer : " << timer << std::endl;
    double time_in_seconds = static_cast<double>(timer) / CLK_FREQ;
    std::cout << time_in_seconds << std::endl;

    return 0;
}
