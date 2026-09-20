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
#include <map>

namespace fs = std::filesystem;

// 分组信息结构体
struct GroupInfo {
    std::vector<int> indices;
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
#define NUM_TREE_PARA 17
#define NUM_PARA_PARA 4
#define NUM_TREE_SERIAL 1
#define NUM_PARA_SERIAL 12

std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; 
std::string rootLink = "prbt_base_link";

// 任务结构体
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

// 辅助函数：根据指定的分组数量生成分组
std::vector<GroupInfo> generateGroups(int num_groups, int total_elements) {
    std::vector<GroupInfo> groups(num_groups);
    int base_size = total_elements / num_groups;
    int remainder = total_elements % num_groups;
    int current_index = 0;

    for(int g = 0; g < num_groups; ++g){
        int current_group_size = base_size + (g < remainder ? 1 : 0);
        for(int i = 0; i < current_group_size; ++i){
            groups[g].indices.push_back(current_index++);
        }
    }

    return groups;
}

int main(int argc, char *argv[]) {
    using S = double;
    std::string directory = BACON_SOURCE_DIR "/env/48";
    
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

    // 定义分组数量和 NUM_PARA 的配对
    std::vector<std::pair<int, int>> group_para_pairs = {
        {17, 4},
        {16, 4},
        {15, 4},
        {14, 4},
        {13, 5},
        {12, 5},
        {11, 5},
        {10, 5},
        {9, 6},
        {8, 6},
        {7, 6},
        {6, 7},
        {5, 7},
        {4, 8},
        {3, 8},
        {2, 9},
        {1, 10}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:t:g:p:s:o", long_options, nullptr)) != -1) {
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
            default:
                // 处理未知选项
                break;
        }
    }
    
    // 读取机器人模型
    URDFModel model;
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "无法加载 URDF 文件: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    // 获取总的碰撞几何体数量
    int total_collision_geometries = model.collisionGeometries.size();

    // 遍历每个 (group_size, NUM_PARA) 配对
    for(const auto& pair : group_para_pairs){
        int num_groups = pair.first;
        int NUM_PARA = pair.second;

        // 生成分组
        std::vector<GroupInfo> groups = generateGroups(num_groups, total_collision_geometries);

        // 打印当前的分组结果
        // std::cout << "当前分组 (Number of Groups: " << num_groups << "):" << std::endl;
        // for(size_t g = 0; g < groups.size(); ++g){
        //     std::cout << "  Group " << g+1 << ": { ";
        //     for(auto it = groups[g].indices.begin(); it != groups[g].indices.end(); ++it){
        //         std::cout << *it;
        //         if(it + 1 != groups[g].indices.end()) std::cout << ", ";
        //     }
        //     std::cout << " }" << std::endl;
        // }

        int timer = 0;
        int pipelined_timer = 0;

        auto start_time = std::chrono::high_resolution_clock::now();

        // 遍历目录中的每个文件
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (entry.is_regular_file()) {  // 只处理常规文件
                std::string filename = entry.path().string();
                // std::cout << "处理文件: " << filename << std::endl;

                // 从文件中读取碰撞对象
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
                                        
                                        // 准备碰撞几何体
                                        std::vector<CollisionObject<S>> collision_geometry_;
                                        for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                            const auto& collisionGeomPtr = model.collisionGeometries[i];
                                            CollisionObject<S> collisionObject(*collisionGeomPtr);
                                            collision_geometry_.push_back(std::move(collisionObject));
                                        }
                                        int stage2_duration = 0;
                                        stage2_duration += 6 * ((model.collisionGeometries.size() + NUM_TF - 1) / NUM_TF);
                                        timer += 6 * ((model.collisionGeometries.size() + NUM_TF - 1) / NUM_TF);
                                        job.stage2_start_time = std::max(job.stage1_end_time, prev_job.stage2_end_time);
                                        job.stage2_end_time = job.stage2_start_time + stage2_duration;

                                        // 初始化分组可用时间
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
                                                bool aabb_sum_result = env.collide_aabb(&tmp_aabb, nullptr, test_timer);
                                                if(aabb_sum_result) {
                                                    for (int idx : groups[g].indices) {
                                                        if (result) break;
                                                        result = env.collide(&collision_geometry_[idx], nullptr, test_timer);
                                                    }
                                                }
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
                                            }
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

                        pipelined_timer += current_time;

                    }
                }
            }  
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;

        // Output the result for the current pair
        // std::cout << "Group Size: " << num_groups  << ", NUM_PARA: " << NUM_PARA << ", ";

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
                std::cout <<  time_in_seconds << std::endl;
            }
            else if(parallelism){
                double time_in_seconds = static_cast<double>(pipelined_timer) / CLK_FREQ / NUM_PARA_PARA;
                std::cout <<  time_in_seconds << std::endl;
            }
            else if(serial){
                double time_in_seconds = static_cast<double>(pipelined_timer) / CLK_FREQ / NUM_PARA_SERIAL;
                std::cout <<  time_in_seconds << std::endl;
            }
            else{
                double time_in_seconds = static_cast<double>(pipelined_timer) / CLK_FREQ / NUM_PARA;
                std::cout <<  time_in_seconds << std::endl;
            }
        }
    }

    return 0;
}
