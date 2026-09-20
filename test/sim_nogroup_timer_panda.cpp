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

#define NUM_TREE_TRAVERSAL 9
#define NUM_TF 9
#define CLK_FREQ 40000000

// 关节角度的名称和初始值
std::map<std::string, double> jointAngles = {
    {"panda_joint1", 0.0},
    {"panda_joint2", 0.0},
    {"panda_joint3", 0.0},
    {"panda_joint4", 0.0},
    {"panda_joint5", 0.0},
    {"panda_joint6", 0.0},
    {"panda_joint7", 0.0}
};

std::string rootLink = "panda_link0";
std::string urdfFilePath = BACON_SOURCE_DIR "/panda_description.urdf"; 

// Define groups
std::vector<GroupInfo> groups = {
    {{0, 1, 2, 3, 4, 7}},
    {{5, 6, 8, 9, 10, 11}},
    {{12, 13, 14, 15, 16, 17}},
    {{18, 19, 20, 21, 22, 23}},
    {{24, 25, 26, 27, 28, 29}},
    {{30, 31, 32, 33, 34, 35}},
    {{36, 37, 38, 39, 40, 41}},
    {{42, 43, 44, 45, 46, 47}},
    {{48, 49, 50, 51, 52}}
};


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
                                    
                                    jointAngles["panda_joint1"] = joint1;
                                    jointAngles["panda_joint2"] = joint2;
                                    jointAngles["panda_joint3"] = joint3;
                                    jointAngles["panda_joint4"] = joint4;
                                    jointAngles["panda_joint5"] = joint5;
                                    jointAngles["panda_joint6"] = joint6;
                                    
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
                                    
                                    // std::cout << "timer1 " << timer << std::endl;
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
