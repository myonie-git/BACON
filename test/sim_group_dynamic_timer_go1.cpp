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


// // Define groups
// std::vector<GroupInfo> groups = {
//     {{0, 1, 2, 3, 4, 5}},
//     {{6, 7, 8, 9, 10, 11}},
//     {{12, 13, 14, 15, 16, 17}},
//     {{18, 19, 20, 21, 22, 23}},
//     {{24, 25, 26, 27, 28}},
// };
std::vector<GroupInfo> groups = {
    {{0, 3, 2, 1, 5, 4}},
    {{8, 6, 7, 11, 9, 10}},
    {{16, 13, 12, 17, 15, 14}},
    {{22, 18, 19, 20, 23, 21}},
    {{27, 28, 24, 25, 28, 26}},
};

#define CLK_FREQ 40000000
#define NUM_TREE_TRAVERSAL 2
#define NUM_TF 3
#define NUM_PARA 8
#define NUM_TREE_PARA 29
#define NUM_PARA_PARA 3
#define NUM_TREE_SERIAL 1
#define NUM_PARA_SERIAL 10

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

std::string urdfFilePath = BACON_SOURCE_DIR "/go1.urdf"; 
std::string rootLink = "base";

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
    std::string directory = BACON_SOURCE_DIR "/env/8";
    
    int filenum = 100; 
    bool pipelined = false;
    bool grouped = false;
    bool serial = false;
    bool parallelism = false;
    bool only_software = false;
    
    int n_random_objects = 128;
    int n_random_units = 1;
    int ping_pong_buffer_size = 4;

    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {"throughput", no_argument, 0, 't'},
        {"group", no_argument, 0, 'g'},
        {"parallisim", no_argument, 0, 'p'},
        {"serial", no_argument, 0, 's'},
        {"onlysoftware", no_argument, 0, 'o'},
        {"random-objects", required_argument, 0, 'r'},
        {"units", required_argument, 0, 'u'},
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
            case 'r':
                n_random_objects = std::stoi(optarg);
                break;
            case 'u':
                n_random_units = std::max(1, std::stoi(optarg));
                break;
        }
    }
    
    // Define constants for timing
    // const int AABB_CHECK_TIME = 1;
    // const int OBB_CHECK_TIME = 6;
    
    URDFModel model;
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();

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
                                    
                                    // A bvh检索部分
                                    int stage3_duration = 0;
                                    int bvh_collision = 0;
                                    std::vector<int> group_result;
                                    std::vector<int> group_available_time;
                                    if(grouped){
                                        group_available_time.resize(groups.size(), 0);
                                        group_result.resize(groups.size(), 0);
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
                                                    result = env.collide(&collision_geometry_[idx], nullptr, test_timer);
                                                    if (result) {
                                                        bvh_collision = 1;
                                                        group_result[g] = 1;
                                                        break;
                                                    }
                                                }
                                            }
                                            // std::cout << "test_timer" << test_timer <<  std::endl;
                                            group_available_time[g] = test_timer;
                                        }
                                    }
                                    if(bvh_collision == 0){
                                        stage3_duration = *std::max_element(group_available_time.begin(), group_available_time.end());
                                        // printf("min_time_nonecolli : %d\n", stage3_duration);
                                    } else{
                                        int min_time = INT_MAX;
                                        for(size_t g = 0; g < groups.size(); ++g) {
                                            if(group_result[g] != 0 && group_available_time[g] < min_time) {
                                                min_time = group_available_time[g];
                                            }
                                        }
                                        stage3_duration = min_time;
                                        // printf("min_time_colli : %d\n", stage3_duration);
                                    }

                                    //(B) 随机部分的并行
                                    int stage3_duration_random = 0;
                                    int dynamic_result = 0;
                                    if(n_random_objects > 0){
                                        
                                        std::vector<CollisionObject<S>*> collision_objects = randomCollisionObjects<S>(n_random_objects);
                                        
                                        std::vector<int> random_unit_available(n_random_units * groups.size(), 0);
                                        std::vector<int> random_unit_result(n_random_units * groups.size(), 0);
                                        int rand_result = 0;
                                        int ping_pong_step = n_random_units;
                                        for(int g = 0; g < groups.size(); ++g){ // 每个组的数据
                                            for(int i = 0; i < n_random_objects; i += ping_pong_buffer_size){ //一共的random number的数量,每轮检测一个ping-pong buffer
                                                for(int j = 0; j < n_random_units; j++){ //每轮ping-pong buffer会被n个随机单元所检测
                                                    for(int k = j; k < ping_pong_buffer_size; k+= ping_pong_step){ //每个并行单元在该ping-pong
                                                        int idx_obj = i + k;
                                                        if (idx_obj >= n_random_objects) break; 
                                                        int device_index = g * n_random_units + j;
                                                        int test_timer = 0;
                                                        bool result = false;
                                                        AABB<S> tmp_aabb;
                                                        for(int idx : groups[g].indices){
                                                            tmp_aabb += collision_geometry_[idx].aabb;
                                                        }
                                                        int local_aabb_time = 0;
                                                        test_timer += 2;
                                                        bool aabb_sum_result = collision_objects[k+i]->aabb.overlap(tmp_aabb, test_timer);
                                                        if(aabb_sum_result){
                                                            test_timer += 2;
                                                            for(int idx : groups[g].indices){
                                                                bool obb_result = collision_objects[k+i]->aabb.overlap(collision_geometry_[idx].aabb, test_timer);
                                                                if(obb_result){
                                                                    // printf("obb_result\n");
                                                                    result = collision_objects[k+i]->obb.overlap(collision_geometry_[idx].obb, test_timer);
                                                                }
                                                                if(result){
                                                                    dynamic_result = 1;
                                                                    random_unit_result[device_index] = 1;
                                                                    break;
                                                                }
                                                            }
                                                        }
                                                        // printf("test_timer:%d\n",test_timer);
                                                        random_unit_available[device_index] += test_timer;
                                                    }
                                                }
                                                //本轮ping-pong buffer检测结束
                                                if(dynamic_result == 1){
                                                    break;
                                                }
                                            }
                                            if(dynamic_result == 1){
                                                break;
                                            }
                                        }
                                        if(dynamic_result == 0){
                                            stage3_duration_random = *std::max_element(random_unit_available.begin(), random_unit_available.end());
                                        
                                            printf("stage3_duration_random1: %d\n", stage3_duration_random);
                                        }
                                        else{
                                            int min_time = INT_MAX;
                                            for(size_t g = 0; g < n_random_units * groups.size(); ++g) {
                                                if(random_unit_result[g] != 0 && random_unit_available[g] < min_time) {
                                                    min_time = random_unit_available[g];
                                                }
                                            }
                                            stage3_duration_random = min_time;
                                            printf("stage3_duration_random2: %d\n", stage3_duration_random);
                                        }
                                    }
                                    
                                    printf("stage3_duration: %d\n", stage3_duration);
                                    
                                    //
                                    int stage3_final_duration;
                                    if(dynamic_result && bvh_collision){
                                        stage3_final_duration = std::min(stage3_duration_random, stage3_duration);
                                        printf("stage3_final_duration1: %d\n", stage3_final_duration);
                                    }
                                    else if(dynamic_result && !bvh_collision){
                                        stage3_final_duration = stage3_duration_random;
                                        printf("stage3_final_duration2: %d\n", stage3_final_duration);
                                    }
                                    else if(bvh_collision && !dynamic_result){
                                        stage3_final_duration = stage3_duration;
                                        printf("stage3_final_duration3: %d\n", stage3_final_duration);
                                    }
                                    else{
                                        stage3_final_duration = std::max(stage3_duration_random, stage3_duration);
                                        printf("stage3_final_duration4: %d\n", stage3_final_duration);
                                    }



                                    timer += stage3_final_duration;
                                    job.stage3_start_time = std::max(job.stage2_end_time, prev_job.stage3_end_time);
                                    job.stage3_end_time = job.stage3_start_time + stage3_final_duration;
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
