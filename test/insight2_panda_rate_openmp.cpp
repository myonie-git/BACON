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
#include <omp.h>
#include <cstdlib>  // for atoi
#include <iomanip>  // for CSV formatting

using namespace std;
namespace fs = std::filesystem;

// Function to write collision results to CSV
void writeCollisionResultsToCSV(const std::string& csvFilename, 
                                const std::string& filename, 
                                const std::vector<int>& collision_counts) {
    std::ofstream csvFile;
    bool fileExists = fs::exists(csvFilename);
    
    csvFile.open(csvFilename, std::ios::app);  // Append mode
    
    if (!csvFile.is_open()) {
        std::cerr << "Error: Could not open CSV file " << csvFilename << std::endl;
        return;
    }
    
    // Calculate statistics
    int total_collisions = 0;
    int max_collisions = 0;
    int min_collisions = (collision_counts.empty() ? 0 : collision_counts[0]);
    for (int count : collision_counts) {
        total_collisions += count;
        max_collisions = std::max(max_collisions, count);
        min_collisions = std::min(min_collisions, count);
    }
    double avg_collisions = collision_counts.empty() ? 0.0 : (double)total_collisions / collision_counts.size();
    
    // Write header if file doesn't exist
    if (!fileExists) {
        csvFile << "Filename";
        for (size_t i = 0; i < collision_counts.size(); i++) {
            csvFile << ",Geometry_" << i;
        }
        csvFile << ",Total_Collisions,Max_Collisions,Min_Collisions,Avg_Collisions";
        // Add ratio columns for each geometry
        for (size_t i = 0; i < collision_counts.size(); i++) {
            csvFile << ",Geom_" << i << "_Ratio";
        }
        csvFile << std::endl;
    }
    
    // Write data row
    csvFile << fs::path(filename).filename().string();  // Just filename, not full path
    for (int count : collision_counts) {
        csvFile << "," << count;
    }
    csvFile << "," << total_collisions 
            << "," << max_collisions 
            << "," << min_collisions 
            << "," << std::fixed << std::setprecision(2) << avg_collisions;
    
    // Add ratio values for each geometry (Geom / Total Collision)
    for (int count : collision_counts) {
        double ratio = (total_collisions > 0) ? (double)count / total_collisions : 0.0;
        csvFile << "," << std::fixed << std::setprecision(4) << ratio;
    }
    csvFile << std::endl;
    
    csvFile.close();
}

int main(int argc, char *argv[]) {
    using S = double;
    std::string directory = BACON_SOURCE_DIR "/env/48/";  
    int filenum = 100; 
    int thread_count = 1;
    std::string csv_output = "panda_collision_results.csv";  // Default CSV output file

    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {"threads", required_argument, 0, 't'},
        {"output", required_argument, 0, 'o'},
        {0, 0, 0, 0} 
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:t:o:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                directory = optarg;
                break;
            case 't':
                thread_count = std::atoi(optarg);
                break;
            case 'o':
                csv_output = optarg;
                break;
            default:
                break;
        }
    }

    // 读取机器人模型
    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/panda_description.urdf"; 
    std::string rootLink = "panda_link0";
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }
    
    // Remove existing CSV file to start fresh
    if (fs::exists(csv_output)) {
        fs::remove(csv_output);
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    int file_counter = 0;
    
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {  // 只处理常规文件
            std::string filename = entry.path().string();
            file_counter++;
            std::cout << "Processing file " << file_counter << ": " << fs::path(filename).filename().string() << std::endl;

            // 读取每个文件中的碰撞对象
            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            // 构建并打印环境
            CollisionEnv<S> env;
            env.InitTree(collision_objects);

            // 定义角度范围和步长
            double start_angle = -1.57;
            double end_angle = 1.57;
            double step_size = 1.0;
            int steps = static_cast<int>((end_angle - start_angle) / step_size) + 1;

            // 为当前文件初始化碰撞计数器
            std::vector<int> file_collision_counts;
            // 临时创建一个几何体向量来获取大小
            URDFModel temp_model = model;
            std::map<std::string, double> temp_jointAngles = {
                {"panda_joint1", 0.0},
                {"panda_joint2", 0.0},
                {"panda_joint3", 0.0},
                {"panda_joint4", 0.0},
                {"panda_joint5", 0.0},
                {"panda_joint6", 0.0},
                {"panda_joint7", 0.0}
            };
            temp_model.setJointAngles(temp_jointAngles);
            temp_model.calculateWorldCoordinates(rootLink);
            file_collision_counts.resize(temp_model.collisionGeometries.size(), 0);

            // 执行并行碰撞检测逻辑
            #pragma omp parallel for collapse(6) schedule(dynamic) num_threads(thread_count)
            for (int i1 = 0; i1 < steps; i1++) {
                for (int i2 = 0; i2 < steps; i2++) {
                    for (int i3 = 0; i3 < steps; i3++) {
                        for (int i4 = 0; i4 < steps; i4++) {
                            for (int i5 = 0; i5 < steps; i5++) {
                                for (int i6 = 0; i6 < steps; i6++) {
                                    // for (int i7 = 0; i7 < steps; i7++) {
                                        // 创建线程本地的模型副本
                                        // URDFModel local_model = model;
                                        
                                        std::map<std::string, double> jointAngles = {
                                            {"panda_joint1", start_angle + i1 * step_size},
                                            {"panda_joint2", start_angle + i2 * step_size},
                                            {"panda_joint3", start_angle + i3 * step_size},
                                            {"panda_joint4", start_angle + i4 * step_size},
                                            {"panda_joint5", start_angle + i5 * step_size},
                                            {"panda_joint6", start_angle + i6 * step_size},
                                            {"panda_joint7", 0.0}
                                        };

                                        // std::cout << "Joint Angles: " << jointAngles["panda_joint7"] << std::endl;

                                        model.setJointAngles(jointAngles);
                                        model.calculateWorldCoordinates(rootLink);

                                        std::vector<CollisionObject<S>> collision_geometry_;
                                        for (size_t j = 0; j < model.collisionGeometries.size(); j++) {
                                            const auto& collisionGeomPtr = model.collisionGeometries[j];
                                            CollisionObject<S> collisionObject(*collisionGeomPtr);
                                            collision_geometry_.push_back(std::move(collisionObject));
                                        }

                                        // 碰撞检测 - 检查所有几何体并统计碰撞次数（线程安全）
                                        bool overall_collision = false;
                                        for (size_t j = 0; j < collision_geometry_.size(); j++) {
                                            bool result = env.collide(&collision_geometry_[j], nullptr);
                                            if (result) {
                                                #pragma omp atomic
                                                file_collision_counts[j]++;  // 原子操作确保线程安全
                                                overall_collision = true;
                                                // std::cout << "Collision detected at geometry " << j << std::endl;
                                            }
                                        }
                                    
                                }
                            }
                        }
                    }
                }
            }
            
            // 将当前文件的碰撞统计结果写入CSV
            writeCollisionResultsToCSV(csv_output, filename, file_collision_counts);
            
            // 输出当前文件的碰撞统计结果
            std::cout << "\n=== Collision Statistics for " << fs::path(filename).filename().string() << " ===" << std::endl;
            std::cout << "Geometry Index | Collision Count" << std::endl;
            std::cout << "--------------------------------" << std::endl;
            for (size_t i = 0; i < file_collision_counts.size(); i++) {
                std::cout << "     " << i << "         |      " << file_collision_counts[i] << std::endl;
            }
            
            // 计算并输出当前文件的总体统计
            int total_collisions = 0;
            int max_collisions = 0;
            int min_collisions = (file_collision_counts.empty() ? 0 : file_collision_counts[0]);
            for (int count : file_collision_counts) {
                total_collisions += count;
                max_collisions = std::max(max_collisions, count);
                min_collisions = std::min(min_collisions, count);
            }
            
            std::cout << "\n=== Summary for " << fs::path(filename).filename().string() << " ===" << std::endl;
            std::cout << "Total collision events: " << total_collisions << std::endl;
            std::cout << "Max collisions for single geometry: " << max_collisions << std::endl;
            std::cout << "Min collisions for single geometry: " << min_collisions << std::endl;
            if (!file_collision_counts.empty()) {
                std::cout << "Average collisions per geometry: " << (double)total_collisions / file_collision_counts.size() << std::endl;
            }
            std::cout << "Results saved to CSV file: " << csv_output << std::endl;
            std::cout << std::endl;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    std::cout << "=== Overall Execution Summary ===" << std::endl;
    std::cout << "Total execution time: " << elapsed_seconds.count() << " seconds" << std::endl;
    std::cout << "Processed " << file_counter << " files" << std::endl;
    std::cout << "Used " << thread_count << " threads" << std::endl;
    std::cout << "Results saved to: " << csv_output << std::endl;

    return 0;
}