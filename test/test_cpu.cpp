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
// #include <cuda_runtime.h>
#include <chrono>

namespace fs = std::filesystem;
using S = double;

int main() {
    
    const std::string directory = BACON_SOURCE_DIR "/env/box";  
    int filenum = 100; 
    int count = 0;
    int aabb_count = 0;
    int obb_count = 0;
    
    std::chrono::duration<double, std::micro> total_aabb_time(0);
    std::chrono::duration<double, std::micro> total_obb_time(0);
    
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) { 
            std::string filename = entry.path().string();

            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            URDFModel model;
            std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; 
            if (!model.loadURDF(urdfFilePath)) {
                std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
                assert(0);
                return -1;
            }

            std::map<std::string, double> jointAngles = {
                {"prbt_joint_1", 0.0},
                {"prbt_joint_2", 0.0},
                {"prbt_joint_3", 0.0},
                {"prbt_joint_4", 0.0},
                {"prbt_joint_5", 0.0},
                {"prbt_joint_6", 0.0}
            };

            double start_angle = -1.57;
            double end_angle = 1.57;
            double step_size = 1.0;

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

                                    std::string rootLink = "prbt_base_link";
                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink);

                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }

                                    bool result = false;
                                    for (int i = 0; i < collision_geometry_.size() && !result; i++) { // num_i = 6-53
                                        for(int j = 0; j < collision_objects.size() && !result; j++){ // num_j = 0-48
                                            
                                            // // auto start_aabb = std::chrono::high_resolution_clock::now();
                                            bool aabb_overlap = collision_objects[j]->aabb.overlap(collision_geometry_[i].aabb);
                                            aabb_count++;
                                            // // auto end_aabb = std::chrono::high_resolution_clock::now();
                                            // // total_aabb_time += end_aabb - start_aabb;

                                            if(aabb_overlap){
                                            //     // 记录 obb.overlap 的开始时间
                                            //     // auto start_obb = std::chrono::high_resolution_clock::now();
                                            bool obb_overlap = collision_objects[j]->obb.overlap(collision_geometry_[i].obb);
                                                obb_count++;
                                            //     // auto end_obb = std::chrono::high_resolution_clock::now();
                                            //     // total_obb_time += end_obb - start_obb;
                                                if(obb_overlap){
                                                    result = true;
                                                }
                                            //     if(obb_overlap){
                                            //         // count += 1;
                                            //         result = true;
                                                }
                                            // }
                                        }
                                    }
                                    
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    std::cout << "aabb_count : " << aabb_count << std::endl;
    std::cout << "obb_count : " << obb_count << std::endl;
    
    return 0;
}
