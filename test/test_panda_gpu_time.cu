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

struct OBB_GPU {
    S axis[9];
    S To[3];
    S extent[3];
};

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

__global__ void overlap_kernel(const OBB_GPU* collision_geometry, int num_i, const OBB_GPU* collision_objects, int num_j, int* results, int batch_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = num_i * num_j * batch_size;
    if (idx < total) {
        int config_idx = idx / (num_i * num_j);
        int rem = idx % (num_i * num_j);
        int i = rem / num_j;
        int j = rem % num_j;

        // Access the correct collision geometry for this configuration
        const OBB_GPU& obb1 = collision_geometry[config_idx * num_i + i];
        const OBB_GPU& obb2 = collision_objects[j];

        bool overlap = overlap_gpu(obb1, obb2);
        if (overlap) {
            results[config_idx] = 1;
        }
    }
}

void checkcollisionOnGPU(const OBB_GPU* h_collision1_obb, size_t num_i, const OBB_GPU* d_collision2_obb, size_t num_j, int* h_results, int* d_results, size_t batch_size) {
    size_t total = num_i * num_j * batch_size;

    // Allocate device memory for collision geometry
    OBB_GPU* d_collision1_obb;
    cudaMalloc((void**)&d_collision1_obb, num_i * batch_size * sizeof(OBB_GPU));

    // Copy h_collision1_obb to device
    cudaMemcpy(d_collision1_obb, h_collision1_obb, num_i * batch_size * sizeof(OBB_GPU), cudaMemcpyHostToDevice);

    // Initialize results on device
    cudaMemset(d_results, 0, batch_size * sizeof(int));

    // Launch kernel
    int numThreads = 32;
    int numBlocks = (total + numThreads - 1) / numThreads;

    overlap_kernel<<<numBlocks, numThreads>>>(d_collision1_obb, num_i, d_collision2_obb, num_j, d_results, batch_size);

    cudaDeviceSynchronize();

    // Copy results back
    cudaMemcpy(h_results, d_results, batch_size * sizeof(int), cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_collision1_obb);
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
    std::string urdfFilePath = BACON_SOURCE_DIR "/urdf_gpu/panda_description.urdf";
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    std::chrono::duration<double, std::micro> total_obb_time(0);

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().string();

            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            URDFModel model;
            std::string urdfFilePath = BACON_SOURCE_DIR "/urdf_gpu/panda_description.urdf";
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

            // Allocate device memory for collision_objects
            OBB_GPU* d_collision2_obb;
            cudaMalloc((void**)&d_collision2_obb, num_j * sizeof(OBB_GPU));
            cudaMemcpy(d_collision2_obb, h_collision2_obb.data(), num_j * sizeof(OBB_GPU), cudaMemcpyHostToDevice);

            // Assume collision_geometry_ size is constant
            size_t num_i = model.collisionGeometries.size();

            // Prepare all joint angle configurations
            int num_steps = static_cast<int>((end_angle - start_angle) / step_size) + 1;
            std::vector<std::array<double, 6>> joint_angle_configs;

            for (int idx1 = 0; idx1 < num_steps; ++idx1) {
                for (int idx2 = 0; idx2 < num_steps; ++idx2) {
                    for (int idx3 = 0; idx3 < num_steps; ++idx3) {
                        for (int idx4 = 0; idx4 < num_steps; ++idx4) {
                            for (int idx5 = 0; idx5 < num_steps; ++idx5) {
                                for (int idx6 = 0; idx6 < num_steps; ++idx6) {
                                    std::array<double, 6> joint_angles = {
                                        start_angle + idx1 * step_size,
                                        start_angle + idx2 * step_size,
                                        start_angle + idx3 * step_size,
                                        start_angle + idx4 * step_size,
                                        start_angle + idx5 * step_size,
                                        start_angle + idx6 * step_size
                                    };
                                    joint_angle_configs.push_back(joint_angles);
                                }
                            }
                        }
                    }
                }
            }

            size_t total_configs = joint_angle_configs.size();

            // Allocate device memory for results
            size_t batch_size = 4096; // Adjust based on GPU memory
            int* d_results;
            cudaMalloc((void**)&d_results, batch_size * sizeof(int));

            // Allocate host memory for results
            std::vector<int> h_results(batch_size);

            // Process configurations in batches
            for (size_t batch_start = 0; batch_start < total_configs; batch_start += batch_size) {
                size_t current_batch_size = std::min(batch_size, total_configs - batch_start);

                // Prepare host vectors of OBBs for current batch
                std::vector<OBB_GPU> h_collision1_obb(num_i * current_batch_size);

                // For each configuration in the batch
                for (size_t config_idx = 0; config_idx < current_batch_size; ++config_idx) {
                    // Set joint angles for this configuration
                    const auto& joint_angles = joint_angle_configs[batch_start + config_idx];
                    jointAngles["prbt_joint_1"] = joint_angles[0];
                    jointAngles["prbt_joint_2"] = joint_angles[1];
                    jointAngles["prbt_joint_3"] = joint_angles[2];
                    jointAngles["prbt_joint_4"] = joint_angles[3];
                    jointAngles["prbt_joint_5"] = joint_angles[4];
                    jointAngles["prbt_joint_6"] = joint_angles[5];

                    model.setJointAngles(jointAngles);
                    model.calculateWorldCoordinates("prbt_base_link");

                    std::vector<CollisionObject<S>*> collision_geometry_;
                    for (size_t i = 0; i < model.collisionGeometries.size(); ++i) {
                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                        collision_geometry_.push_back(new CollisionObject<S>(*collisionGeomPtr));
                    }

                    // Copy data to h_collision1_obb
                    for (size_t i = 0; i < num_i; ++i) {
                        const auto& obb = collision_geometry_[i]->obb;
                        size_t index = config_idx * num_i + i;
                        for (int row = 0; row < 3; row++) {
                            h_collision1_obb[index].To[row] = obb.To[row];
                            h_collision1_obb[index].extent[row] = obb.extent[row];
                            for (int col = 0; col < 3; col++) {
                                h_collision1_obb[index].axis[row * 3 + col] = obb.axis(row, col);
                            }
                        }
                    }

                    // Clean up collision_geometry_
                    for (auto obj : collision_geometry_) {
                        delete obj;
                    }
                }

                // Perform collision detection for the batch
                checkcollisionOnGPU(h_collision1_obb.data(), num_i, d_collision2_obb, num_j, h_results.data(), d_results, current_batch_size);

                // Process results
                // for (size_t config_idx = 0; config_idx < current_batch_size; ++config_idx) {
                //     if (h_results[config_idx]) {
                //         // Collision detected for this configuration
                //         // Handle accordingly (e.g., log, store, or process the collision)
                //         // std::cout << "Collision detected for configuration index: " << (batch_start + config_idx) << std::endl;
                //     }
                // }
            }

            // Free device memory after processing the file
            cudaFree(d_collision2_obb);
            cudaFree(d_results);

            // Clean up collision_objects
            for (auto obj : collision_objects) {
                delete obj;
            }
        }
    }

    return 0;
}
