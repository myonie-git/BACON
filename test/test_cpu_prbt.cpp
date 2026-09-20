#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include "aabb.h"
#include <assert.h>
#include <filesystem>

namespace fs = std::filesystem;
using S = double;

struct GroupInfo{
    std::vector<int> indices;
    bool collision_result;
    int aabb_count;
    int obb_count;

    GroupInfo(const std::vector<int>& idx) 
        : indices(idx), collision_result(false), aabb_count(0), obb_count(0) {}
};

int main() {
    
    const std::string directory = BACON_SOURCE_DIR "/env/48-bak";  
    int filenum = 100; 
    int obb_count = 0;
    int aabb_count = 0;
    
    std::vector<GroupInfo> groups = {
        // {{0}},
        // {{1, 2, 3}},
        // {{4, 5}},
        // {{6, 7, 8}},
        // {{9, 10}},
        // {{11, 12, 13, 14}},
        // {{15, 16}}
        {{0, 1, 2, 3,6}},
        {{4, 5, 7,8,9}},
        {{10,11,12}},
        {{13, 14,15, 16}},
    };

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

                                    // std::vector<CollisionObject<S>*> collision_geometry_;
                                    // for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                    //     const auto& collisionGeomPtr = model.collisionGeometries[i];
                                    //     collision_geometry_.push_back(new CollisionObject<S>(*collisionGeomPtr));
                                    //     // CollisionObject<S> collisionObject(*collisionGeomPtr);
                                    //     // collision_geometry_.push_back(std::move(collisionObject));
                                    // }
                                    
                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr);
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }
                                    bool obb_happened = false;
                                    bool result = false;
                                    int temp_aabb_count = 0;
                                    for (int i = 0; i < collision_objects.size() && !result; i++) { //num_i = 6-53
                                        //todo: 把可以更进一步把collision_geometry_和result打包成向量    
                                        
                                        for(auto& group : groups){
                                            if(result) break;
                                            bool aabb_sum_result = 0;
                                            AABB<S> tmp_aabb; 
                                            for(int idx : group.indices){
                                                tmp_aabb += collision_geometry_[idx].aabb;
                                            }
                                            temp_aabb_count ++;
                                            aabb_count++;
                                            aabb_sum_result = collision_objects[i]->aabb.overlap(tmp_aabb);
                                            if(aabb_sum_result){
                                                //逐个检测
                                                for(int idx : group.indices){
                                                    if(result) break;
                                                    bool aabb_result;
                                                    aabb_count++;
                                                    temp_aabb_count ++;
                                                    aabb_result = collision_objects[i]->aabb.overlap(collision_geometry_[idx].aabb);
                                                    if(aabb_result){
                                                        result = collision_objects[i]->obb.overlap(collision_geometry_[idx].obb);
                                                        obb_count++;
                                                        obb_happened = true;
                                                    }
                                                }
                                            }
                                        }
                                        if(obb_happened){
                                            aabb_count -= temp_aabb_count;
                                        }
                                    }

                                    // std::cout << "Joint Angles: " << joint1 << ", " << joint2 << ", " << joint3 << ", "
                                    //         << joint4 << ", " << joint5 << ", " << joint6
                                    //         << " - Collision Detection: " << (result ? "Env Collision" : "No Collision") << std::endl;
                                    
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