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
    // {{0, 1, 2, 3, 4}},
    // {{5, 6, 7, 8, 9}},
    // {{10, 11, 12, 13, 14}},
    // {{15, 16}},
    {{0}},
    {{1,2,3}},
    {{4,5}},
    {{6,7,8}},
    {{9,10}},
    {{11,12,13,14,15}},
    {{16}}
};

// 关节角度的名称和初始值
std::map<std::string, double> jointAngles = {
    {"prbt_joint_1", 0.0},
    {"prbt_joint_2", 0.0},
    {"prbt_joint_3", 0.0},
    {"prbt_joint_4", 0.0},
    {"prbt_joint_5", 0.0},
    {"prbt_joint_6", 0.0}
};

#define CLK_FREQ 40000000
#define NUM_TREE_TRAVERSAL 2
#define NUM_TF 3
#define NUM_PARA 6
#define NUM_TREE_PARA 17
#define NUM_PARA_PARA 4
#define NUM_TREE_SERIAL 1
#define NUM_PARA_SERIAL 12

std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; 
std::string rootLink = "prbt_base_link";

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
    // std::string directory = BACON_SOURCE_DIR "/env/48-bak";
    std::string directory = BACON_SOURCE_DIR "/env/48";
    
    int filenum = 100; 
    bool pipelined = false;
    bool grouped = false;
    bool serial = false;
    bool parallelism = false;
    bool only_software = false;
    
    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {"throughput", no_argument, 0, 't'},
        {"group", no_argument, 0, 'g'},
        {"parallisim", no_argument, 0, 'p'},
        {"serial", no_argument, 0, 's'},
        {"onlysoftware", no_argument, 0, 'o'},
        {0, 0, 0, 0}
    };

    int timer = 0;
    int pipelined_timer = 0;

    int opt;
    while ((opt = getopt_long(argc, argv, "d:t:g:p:s", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                directory = optarg;
                break;
            case 't':
                pipelined = true;
                break;
            case 'g':
                grouped = true;
                break;
            case 'p':
                parallelism = true;
                break;
            case 's':
                serial = true;
                break;
            case 'o':
                only_software = true;
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

    auto start_time = std::chrono::high_resolution_clock::now();

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
            
            int current_time = 0;
            std::vector<Job> jobs;

            Job prev_job = {0, 0, 0, 0, 0, 0, 0, false};
            int job_id = 0;

            // 执行碰撞检测逻辑
            for (double joint1 = start_angle; joint1 <= end_angle; joint1 += step_size) {
                for (double joint2 = start_angle; joint2 <= end_angle; joint2 += step_size) {
                    for (double joint3 = start_angle; joint3 <= end_angle; joint3 += step_size) {
                        for (double joint4 = start_angle; joint4 <= end_angle; joint4 += step_size) {
                            for (double joint5 = start_angle; joint5 <= end_angle; joint5 += step_size) {
                                for (double joint6 = start_angle; joint6 <= end_angle; joint6 += step_size) {
                                    
                                    jointAngles["prbt_joint_1"] = joint1;
                                    jointAngles["prbt_joint_2"] = joint2;
                                    jointAngles["prbt_joint_3"] = joint3;
                                    jointAngles["prbt_joint_4"] = joint4;
                                    jointAngles["prbt_joint_5"] = joint5;
                                    jointAngles["prbt_joint_6"] = joint6;

                                    Job job;
                                    job.job_id = job_id ++;

                                    int stage1_duration = 0;
                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink, stage1_duration);
                                    timer += stage1_duration;
                                    job.stage1_start_time = prev_job.stage1_end_time; 
                                    job.stage1_end_time = job.stage1_start_time + stage1_duration;
                                    
                                    // Prepare collision geometries
                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }
                                    int stage2_duration = 0;
                                    timer += 6 * ((model.collisionGeometries.size() + NUM_TF - 1) / NUM_TF);
                                    stage2_duration += 6 * ((model.collisionGeometries.size() + NUM_TF - 1) / NUM_TF);
                                    job.stage2_start_time = std::max(job.stage1_end_time, prev_job.stage2_end_time);
                                    job.stage2_end_time = job.stage2_start_time + stage2_duration;

                                    // Initialize group available times
                                    
                                    int stage3_duration = 0;
                                    std::vector<int> group_available_time;
                                    if(grouped){
                                        group_available_time.resize(groups.size(), 0);
                                        for(size_t g = 0; g < groups.size(); ++g){
                                            int test_timer = 0;
                                            bool result = false;
                                            AABB<S> tmp_aabb;
                                            for (int idx : groups[g].indices) {
                                                tmp_aabb += collision_geometry_[idx].aabb;
                                            }
                                            int local_aabb_timer = 0;
                                            bool aabb_sum_result = env.collide_aabb(&tmp_aabb, nullptr, test_timer);
                                            if(aabb_sum_result) {
                                                for (int idx : groups[g].indices) {
                                                    if (result) break;
                                                    result = env.collide(&collision_geometry_[idx], nullptr, test_timer);
                                                }
                                            }
                                            // std::cout << "test_timer" << test_timer <<  std::endl;
                                            group_available_time[g] = test_timer;
                                        }
                                    }
                                    else if(parallelism){
                                        group_available_time.resize(NUM_TREE_PARA, 0);
                                        bool result = false;
                                        for(int i = 0; i < collision_geometry_.size() && !result; i++){
                                            int test_timer = 0;
                                            int min_device = 0;
                                            int earliest_time = group_available_time[0];
                                            for(int j = 1; j < NUM_TREE_PARA; j++){
                                                if(group_available_time[j] < earliest_time){
                                                    earliest_time = group_available_time[j];
                                                    min_device = j;
                                                }
                                            }
                                            result = env.collide(&collision_geometry_[i], nullptr, test_timer);
                                            // std::cout << "test_timer " << test_timer << std::endl;
                                            // std::cout << "min_device " << min_device << std::endl;
                                            group_available_time[min_device] = std::max(earliest_time, stage3_duration) + test_timer;
                                        }
                                    }
                                    else if(serial){
                                        group_available_time.resize(NUM_TREE_SERIAL, 0);
                                        bool result = false;
                                        for(int i = 0; i < collision_geometry_.size() && !result; i++){
                                            int test_timer = 0;
                                            int min_device = 0;
                                            int earliest_time = group_available_time[0];
                                            for(int j = 1; j < NUM_TREE_SERIAL; j++){
                                                if(group_available_time[j] < earliest_time){
                                                    earliest_time = group_available_time[j];
                                                    min_device = j;
                                                }
                                            }
                                            result = env.collide(&collision_geometry_[i], nullptr, test_timer);
                                            // std::cout << "test_timer " << test_timer << std::endl;
                                            // std::cout << "min_device " << min_device << std::endl;
                                            group_available_time[min_device] = std::max(earliest_time, stage3_duration) + test_timer;
                                        }
                                    }
                                    else{
                                        group_available_time.resize(groups.size(), 0);
                                        for(size_t g = 0; g < groups.size(); ++g){
                                            int test_timer = 0;
                                            bool result = false;
                                            for (int idx : groups[g].indices) {
                                                if (result) break;
                                                result = env.collide(&collision_geometry_[idx], nullptr, test_timer);
                                            }
                                            group_available_time[g] = test_timer;
                                            // std::cout << "test_timer : " << stage3_duration <<  std::endl;
                                        }
                                        // std::cout << std::endl;
                                    }
                                    stage3_duration = *std::max_element(group_available_time.begin(), group_available_time.end());
                                    timer += stage3_duration;
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

            pipelined_timer += current_time;

        }
    }  
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;

    
    // std::cout << "timer : " << timer << std::endl;
    if(only_software){
        std::cout << elapsed_seconds.count() << std::endl;
    }
    else if(!pipelined){
        double time_in_seconds = static_cast<double>(timer) / CLK_FREQ;
        std::cout << time_in_seconds << std::endl;
    }
    else{
        if(grouped){
            double time_in_seconds = static_cast<double>(pipelined_timer) / CLK_FREQ / NUM_PARA;
            std::cout << time_in_seconds << std::endl;
        }
        else if(parallelism){
            double time_in_seconds = static_cast<double>(pipelined_timer) / CLK_FREQ / NUM_PARA_PARA;
            std::cout << time_in_seconds << std::endl;
        }
        else if(serial){
            double time_in_seconds = static_cast<double>(pipelined_timer) / CLK_FREQ / NUM_PARA_SERIAL;
            std::cout << time_in_seconds << std::endl;
        }
        else{
            double time_in_seconds = static_cast<double>(pipelined_timer) / CLK_FREQ / NUM_PARA;
            std::cout << time_in_seconds << std::endl;
        }
    }

    return 0;
}
