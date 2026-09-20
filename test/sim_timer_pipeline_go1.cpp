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
#include <algorithm>
#include <map>

#define NUM_TREE_TRAVERSAL 5
#define NUM_TF 3
#define NUM_PARA 8
#define CLK_FREQ 40000000


namespace fs = std::filesystem;

struct Job {
    int job_id;
    int stage1_start_time;
    int stage1_end_time;
    int stage2_start_time;
    int stage2_end_time;
    int stage3_start_time;
    int stage3_end_time;
    bool collision_result;
};

int main(int argc, char *argv[]) {
    using S = double;
    std::string directory = BACON_SOURCE_DIR "/env/48";
    std::string urdfFilePath = BACON_SOURCE_DIR "/go1.urdf"; 
    std::string rootLink = "base";
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

    URDFModel model;
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

            // 定义角度范围和步长
            double start_angle = -1.57;
            double end_angle = 1.57;
            double step_size = 1.0;

            int current_time = 0;
            std::vector<int> device_available_time(NUM_TREE_TRAVERSAL, 0);
            std::vector<Job> jobs;

            // int prev_stage1_end_time = 0;
            // int prev_stage2_end_time = 0;
            // int prev_stage3_end_time = 0;

            Job prev_job = {0, 0, 0, 0, 0, 0, 0, false};
            int job_id = 0;

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
                                    
                                    Job job;
                                    job.job_id = job_id ++;
                                    
                                    int stage1_duration = 0;
                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink, stage1_duration);
                                    job.stage1_start_time = prev_job.stage1_end_time; //std::max(current_time, prev_job.stage1_end_time);
                                    job.stage1_end_time = job.stage1_start_time + stage1_duration;
                                            
                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }
                                    int stage2_duration = 0;
                                    stage2_duration += 6 * ((model.collisionGeometries.size() + NUM_TF - 1) / NUM_TF);
                                    job.stage2_start_time = std::max(job.stage1_end_time, prev_job.stage2_end_time);
                                    job.stage2_end_time = job.stage2_start_time + stage2_duration;


                                    int stage3_duration = 0;
                                    std::vector<int> device_available_time(NUM_TREE_TRAVERSAL, stage3_duration);
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
                                        device_available_time[min_device] = std::max(earliest_time, stage3_duration) + test_timer;
                                    }
                                    stage3_duration = *std::max_element(device_available_time.begin(), device_available_time.end());
                                    job.stage3_start_time = std::max(job.stage2_end_time, prev_job.stage3_end_time);
                                    job.stage3_end_time = job.stage3_start_time + stage3_duration;
                                    current_time = std::max(current_time, job.stage3_end_time);

                                    jobs.push_back(job);
                                    prev_job = job;
                                }
                            }
                        }
                    }
                }
            }
            
            
            timer += current_time;

        }
    }  
    
    double time_in_seconds = static_cast<double>(timer) / CLK_FREQ / NUM_PARA;
    std::cout << time_in_seconds << std::endl;

    return 0;
}
