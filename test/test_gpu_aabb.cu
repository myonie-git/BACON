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
#include <cuda_runtime.h>
#include <chrono>

namespace fs = std::filesystem;
using S = double;

struct AABB_GPU{
    S min_[3];
    S max_[3];
};

__device__ bool aabb_overlap_gpu(const AABB_GPU& aabb1, const AABB_GPU& aabb2) {
    for (int i = 0; i < 3; ++i) {
        if (aabb1.min_[i] > aabb2.max_[i])
            return false;
        if (aabb1.max_[i] < aabb2.min_[i])
            return false;
    }
    return true;
}

__global__ void overlap_kernel(const AABB_GPU* collision_geometry, int num_i, const AABB_GPU* collision_objects, int num_j, bool* results){
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = num_i * num_j;
    if (idx < total) {
        int i = idx / num_j;
        int j = idx % num_j;

        bool overlap = aabb_overlap_gpu(collision_geometry[i], collision_objects[j]);
        results[idx] = overlap ? 1 : 0;
    }
}

void checkcollisionOnGPU(const std::vector<CollisionObject<S>*>& collision1, const std::vector<CollisionObject<S>*>& collision2, bool* result, 
                         AABB_GPU* d_collision1_aabb, AABB_GPU* d_collision2_aabb, bool* d_results, size_t num_i, size_t num_j) {

    size_t total = num_i * num_j;

    // Prepare host vectors of AABBs
    std::vector<AABB_GPU> h_collision1_aabb(num_i);

    // Copy data from collision1 to h_collision1_aabb
    
    for(size_t i = 0; i < num_i; i++){
        const auto& aabb = collision1[i]->aabb;
        for (int dim = 0; dim < 3; dim++) {
            h_collision1_aabb[i].min_[dim] = aabb.min_[dim];
            h_collision1_aabb[i].max_[dim] = aabb.max_[dim];
        }
    }

    // Copy h_collision1_aabb to device
    cudaMemcpy(d_collision1_aabb, h_collision1_aabb.data(), num_i * sizeof(AABB_GPU), cudaMemcpyHostToDevice);

    // Launch kernel
    int numThreads = 32;
    int numBlocks = (total + numThreads - 1) / numThreads;

    overlap_kernel<<<numBlocks, numThreads>>>(d_collision1_aabb, num_i, d_collision2_aabb, num_j, d_results);

    cudaDeviceSynchronize();

    // Copy results back
    std::vector<int> h_results(total);
    cudaMemcpy(h_results.data(), d_results, total * sizeof(int), cudaMemcpyDeviceToHost);

    *result = false;
    for (size_t idx = 0; idx < total; idx++) {
        if (h_results[idx]) {
            *result = true;
            break;
        }
    }
}

int main() {
    const std::string directory = BACON_SOURCE_DIR "/env/48";  
    int filenum = 100; 

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

            size_t num_j = collision_objects.size();

            // Prepare host vector for collision_objects
            std::vector<AABB_GPU> h_collision2_aabb(num_j);

            for (size_t i = 0; i < num_j; i++) {
                const auto& aabb = collision_objects[i]->aabb;
                for (int dim = 0; dim < 3; dim++) {
                    h_collision2_aabb[i].min_[dim] = aabb.min_[dim];
                    h_collision2_aabb[i].max_[dim] = aabb.max_[dim];
                }
            }

            // Allocate device memory for collision_objects
            AABB_GPU* d_collision2_aabb;
            cudaMalloc((void**)&d_collision2_aabb, num_j * sizeof(AABB_GPU));
            cudaMemcpy(d_collision2_aabb, h_collision2_aabb.data(), num_j * sizeof(AABB_GPU), cudaMemcpyHostToDevice);

            // Assume collision_geometry_ size is constant
            size_t num_i = model.collisionGeometries.size();
            AABB_GPU* d_collision1_aabb;
            cudaMalloc((void**)&d_collision1_aabb, num_i * sizeof(AABB_GPU));

            size_t total = num_i * num_j;
            bool* d_results;
            cudaMalloc((void**)&d_results, total * sizeof(bool));

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

                                    std::vector<CollisionObject<S>*> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        collision_geometry_.push_back(new CollisionObject<S>(*collisionGeomPtr));
                                    }

                                    bool result = false;

                                    checkcollisionOnGPU(collision_geometry_, collision_objects, &result, d_collision1_aabb, d_collision2_aabb, d_results, num_i, num_j);

                                    for(auto obj : collision_geometry_) {
                                        delete obj;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Free device memory after processing the file
            cudaFree(d_collision1_aabb);
            cudaFree(d_collision2_aabb);
            cudaFree(d_results);

            // Clean up collision_objects
            for(auto obj : collision_objects) {
                delete obj;
            }
        }
    }

    return 0;
}
