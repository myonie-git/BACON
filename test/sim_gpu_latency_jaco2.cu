#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>
#include <filesystem>
#include <cuda_runtime.h>
#include <chrono>
#include <getopt.h>

namespace fs = std::filesystem;
using S = double;

struct OBB_GPU{
    S axis[9];
    S To[3];
    S extent[3];
};

template <typename S>
__device__ bool obbDisjoint_gpu(const S* B, const S* T, const S* a, const S* b) {
    S t_val, s_val;
    const S reps = 1e-6;

    S Bf[9];
    for (int i = 0; i < 9; i++) {
        Bf[i] = fabs(B[i]) + reps;
    }

    for (int i = 0; i < 3; i++) {
        t_val = fabs(T[i]);
        S sum = 0.0;
        for (int j = 0; j < 3; j++) {
            sum += Bf[j * 3 + i] * b[j];
        }
        if (t_val > (a[i] + sum))
            return true;
    }

    for (int i = 0; i < 3; i++) {
        S s = 0.0;
        for (int j = 0; j < 3; j++)
            s += B[i * 3 + j] * T[j];
        t_val = fabs(s);
        S sum = 0.0;
        for (int j = 0; j < 3; j++)
            sum += Bf[i * 3 + j] * a[j];
        if (t_val > (b[i] + sum))
            return true;
    }

    for (int i = 0; i < 3; i++) { 
        for (int j = 0; j < 3; j++) { 
            S s = T[(i + 2) % 3] * B[j * 3 + (i + 1) % 3] - T[(i + 1) % 3] * B[j * 3 + (i + 2) % 3];
            t_val = fabs(s);
            S threshold = a[(i + 1) % 3] * Bf[j * 3 + (i + 2) % 3] + a[(i + 2) % 3] * Bf[j * 3 + (i + 1) % 3]
                        + b[(j + 1) % 3] * Bf[(j + 2) % 3 * 3 + i] + b[(j + 2) % 3] * Bf[(j + 1) % 3 * 3 + i];
            if (t_val > threshold)
                return true;
        }
    }

    return false;
}

__device__ bool overlap_gpu(const OBB_GPU& obb1, const OBB_GPU& obb2) {

    const S* axis_ptr = obb1.axis;
    const S* other_axis_ptr = obb2.axis;

    const S* to_ptr = obb1.To;
    const S* other_to_ptr = obb2.To;

    const S* extent_ptr = obb1.extent;
    const S* other_extent_ptr = obb2.extent;

    S t[3];
    for (int i = 0; i < 3; i++) {
        t[i] = other_to_ptr[i] - to_ptr[i];
    }

    S T[3];
    for (int i = 0; i < 3; i++) {
        T[i] = axis_ptr[i * 3 + 0] * t[0] +
               axis_ptr[i * 3 + 1] * t[1] +
               axis_ptr[i * 3 + 2] * t[2];
    }

    S R0_trans[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R0_trans[i * 3 + j] = axis_ptr[j * 3 + i];
        }
    }

    S R[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i * 3 + j] = 0.0;
            for (int k = 0; k < 3; k++) {
                R[i * 3 + j] += R0_trans[i * 3 + k] * other_axis_ptr[k * 3 + j];
            }
        }
    }

    return !obbDisjoint_gpu(R, T, extent_ptr, other_extent_ptr);
}

__global__ void overlap_kernel(const OBB_GPU* collision_geometry, int num_i, const OBB_GPU* collision_objects, int num_j, bool* results){
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = num_i * num_j;
    if (idx < total) {
        int i = idx / num_j;
        int j = idx % num_j;

        bool overlap = overlap_gpu(collision_geometry[i], collision_objects[j]);
        results[idx] = overlap ? 1 : 0;
    }
}

__global__ void warmup_kernel() {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    float a = 1.0f;
    float b = 2.0f;
    float c = a + b;
    if (c > 0.0f) {
        
    }
}

void checkcollisionOnGPU(const std::vector<CollisionObject<S>*>& collision1, const std::vector<CollisionObject<S>*>& collision2, bool* result, 
                         OBB_GPU* d_collision1_obb, OBB_GPU* d_collision2_obb, bool* d_results, size_t num_i, size_t num_j) {

    size_t total = num_i * num_j;

    // Prepare host vectors of OBBs
    std::vector<OBB_GPU> h_collision1_obb(num_i);

    // Copy data from collision1 to h_collision1_obb
    for(size_t i = 0; i < num_i; i++){
        const auto& obb = collision1[i]->obb;
        // Flatten the axis matrix into the array
        for (int row = 0; row < 3; row++) {
            h_collision1_obb[i].To[row] = obb.To[row];
            h_collision1_obb[i].extent[row] = obb.extent[row];
            for (int col = 0; col < 3; col++) {
                h_collision1_obb[i].axis[row * 3 + col] = obb.axis(row, col);
            }
        }
    }

    // Copy h_collision1_obb to device
    cudaMemcpy(d_collision1_obb, h_collision1_obb.data(), num_i * sizeof(OBB_GPU), cudaMemcpyHostToDevice);

    // Launch kernel
    int numThreads = 32;
    int numBlocks = (total + numThreads - 1) / numThreads;

    overlap_kernel<<<numBlocks, numThreads>>>(d_collision1_obb, num_i, d_collision2_obb, num_j, d_results);

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

int main(int argc, char *argv[]) {
    
    std::string directory = BACON_SOURCE_DIR "/env/48";  
    int filenum = 100; 

    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {0, 0, 0, 0} 
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                directory = std::string(optarg);
                break;
        }
    }

    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/urdf_gpu/j2n6s200_standalone.urdf";
    std::string rootLink = "root";
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    
    int warmupThreads = 1;
    int warmupBlocks = 1;
    warmup_kernel<<<warmupBlocks, warmupThreads>>>();
    cudaDeviceSynchronize();

    auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) { 
            std::string filename = entry.path().string();

            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            std::map<std::string, double> jointAngles = {
                {"j2n6s200_joint_1", 0.0},
                {"j2n6s200_joint_2", 0.0},
                {"j2n6s200_joint_3", 0.0},
                {"j2n6s200_joint_4", 0.0},
                {"j2n6s200_joint_5", 0.0},
                {"j2n6s200_joint_6", 0.0},
                {"j2n6s200_joint_finger_1", 0.0},
                {"j2n6s200_joint_finger_2", 0.0},
                {"j2n6s200_joint_finger_tip_1", 0.0},
                {"j2n6s200_joint_finger_tip_2", 0.0}
            };

            double start_angle = -1.57;
            double end_angle = 1.57;
            double step_size = 1.0;

            size_t num_j = collision_objects.size();

            // Prepare host vector for collision_objects
            std::vector<OBB_GPU> h_collision2_obb(num_j);

            for (size_t i = 0; i < num_j; i++) {
                const auto& obb = collision_objects[i]->obb;
                for (int row = 0; row < 3; row++) {
                    h_collision2_obb[i].To[row] = obb.To[row];
                    h_collision2_obb[i].extent[row] = obb.extent[row];
                    for (int col = 0; col < 3; col++) {
                        h_collision2_obb[i].axis[row * 3 + col] = obb.axis(row, col);
                    }
                }
            }

            OBB_GPU* d_collision2_obb;
            cudaMalloc((void**)&d_collision2_obb, num_j * sizeof(OBB_GPU));
            cudaMemcpy(d_collision2_obb, h_collision2_obb.data(), num_j * sizeof(OBB_GPU), cudaMemcpyHostToDevice);

            // Assume collision_geometry_ size is constant
            size_t num_i = model.collisionGeometries.size();
            OBB_GPU* d_collision1_obb;
            cudaMalloc((void**)&d_collision1_obb, num_i * sizeof(OBB_GPU));

            size_t total = num_i * num_j;
            bool* d_results;
            cudaMalloc((void**)&d_results, total * sizeof(bool));

            for (double joint1 = start_angle; joint1 <= end_angle; joint1 += step_size) {
                for (double joint2 = start_angle; joint2 <= end_angle; joint2 += step_size) {
                    for (double joint3 = start_angle; joint3 <= end_angle; joint3 += step_size) {
                        for (double joint4 = start_angle; joint4 <= end_angle; joint4 += step_size) {
                            for (double joint5 = start_angle; joint5 <= end_angle; joint5 += step_size) {
                                for (double joint6 = start_angle; joint6 <= end_angle; joint6 += step_size) {
                                    jointAngles["j2n6s200_joint_1"] = joint1;
                                    jointAngles["j2n6s200_joint_2"] = joint2;
                                    jointAngles["j2n6s200_joint_3"] = joint3;
                                    jointAngles["j2n6s200_joint_4"] = joint4;
                                    jointAngles["j2n6s200_joint_5"] = joint5;
                                    jointAngles["j2n6s200_joint_6"] = joint6;


                                    model.setJointAngles(jointAngles);
                                    model.calculateWorldCoordinates(rootLink);

                                    std::vector<CollisionObject<S>*> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        collision_geometry_.push_back(new CollisionObject<S>(*collisionGeomPtr));
                                    }

                                    bool result = false;

                                    checkcollisionOnGPU(collision_geometry_, collision_objects, &result, d_collision1_obb, d_collision2_obb, d_results, num_i, num_j);

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
            cudaFree(d_collision1_obb);
            cudaFree(d_collision2_obb);
            cudaFree(d_results);

            // Clean up collision_objects
            for(auto obj : collision_objects) {
                delete obj;
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    std::cout << elapsed_seconds.count() << std::endl;
    
    return 0;
}