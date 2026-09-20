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

#define NUM_TREE_TRAVERSAL 2
#define NUM_TF 1
#define CLK_FREQ 40000000

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    using S = double;
    // std::string directory = BACON_SOURCE_DIR "/env/48-bak";
    std::string directory = BACON_SOURCE_DIR "/env/48";
    std::string urdfFilePath = BACON_SOURCE_DIR "/fanuc_description.urdf"; 
    std::string rootLink = "base_link";
    
    // 关节角度的名称和初始值
    std::map<std::string, double> jointAngles = {
        {"joint_1", 0.0},
        {"joint_2", 0.0},
        {"joint_3", 0.0},
        {"joint_4", 0.0},
        {"joint_5", 0.0},
        {"joint_6", 0.0}
    };

    
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

            // 构建并打印环境
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
                                    
                                    jointAngles["joint_1"] = joint1;
                                    jointAngles["joint_2"] = joint2;
                                    jointAngles["joint_3"] = joint3;
                                    jointAngles["joint_4"] = joint4;
                                    jointAngles["joint_5"] = joint5;
                                    jointAngles["joint_6"] = joint6;
                                    
                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink, timer);
                                    
                                    int tmp_timer = 0;
                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                        tmp_timer += 6;
                                    }
                                    timer += 6 * ((model.collisionGeometries.size() + NUM_TF - 1) / NUM_TF);

                                    std::vector<int> device_available_time(NUM_TREE_TRAVERSAL, timer);
                                    bool result = false;
                                    for(int i = 0; i < collision_geometry_.size() && !result; i++){
                                        int test_timer = 0;
                                        int min_device = 0;
                                        int earliest_time = device_available_time[0];
                                        for(int j = 1; j < NUM_TREE_TRAVERSAL; j++){
                                            if(device_available_time[j] < earliest_time){
                                                earliest_time = device_available_time[j];
                                                min_device = j;
                                            }
                                        }
                                        result = env.collide(&collision_geometry_[i], nullptr, test_timer);
                                        // std::cout << "test_timer " << test_timer << std::endl;
                                        // std::cout << "min_device " << min_device << std::endl;
                                        device_available_time[min_device] = std::max(earliest_time, timer) + test_timer;
                                    }
                                    timer = *std::max_element(device_available_time.begin(), device_available_time.end());
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
