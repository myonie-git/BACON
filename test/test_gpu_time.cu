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
#include <map>

namespace fs = std::filesystem;
using S = double;

// 错误检查宏
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error in " << __FILE__ << ":" << __LINE__ << " : " \
                      << cudaGetErrorString(err) << std::endl; \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

// 用于记录每次checkcollisionOnGPU调用的时间
struct CollisionTimings {
    double type_conversion_time_us;                  // 微秒
    double gpu_operations_time_ms;                   // 毫秒
    double malloc_d_collision1_time_us;              // 微秒
    double malloc_d_collision2_time_us;              // 微秒
    double malloc_d_results_time_us;                 // 微秒
    double copy_H2D_time_us;                         // 微秒
    double copy_D2H_time_us;                         // 微秒
    double free_h_results_time_us;                   // 微秒
    double free_d_collision1_time_us;                // 微秒
    double free_d_collision2_time_us;                // 微秒
    double free_d_results_time_us;                   // 微秒
    double malloc_host_collision_geometry_time_us;   // 微秒
    double malloc_h_results_time_us;                 // 微秒
    double gpu_kernel_time_us;
};

// OBB结构体在GPU上的表示
struct OBB_GPU{
    S axis[9];
    S To[3];
    S extent[3];
};

// 判断两个OBB是否不相交的GPU设备函数
template <typename T>
__device__ bool obbDisjoint_gpu(const T* B, const T* T_vec, const T* a, const T* b) {
    T t_val, s_val;
    const T reps = 1e-6;

    T Bf[9];
    for (int i = 0; i < 9; i++) {
        Bf[i] = fabs(B[i]) + reps;
    }

    for (int i = 0; i < 3; i++) {
        t_val = fabs(T_vec[i]);
        T sum = 0.0;
        for (int j = 0; j < 3; j++) {
            sum += Bf[j * 3 + i] * b[j];
        }
        if (t_val > (a[i] + sum))
            return true;
    }

    for (int i = 0; i < 3; i++) {
        T s = 0.0;
        for (int j = 0; j < 3; j++)
            s += B[i * 3 + j] * T_vec[j];
        t_val = fabs(s);
        T sum = 0.0;
        for (int j = 0; j < 3; j++)
            sum += Bf[i * 3 + j] * a[j];
        if (t_val > (b[i] + sum))
            return true;
    }

    for (int i = 0; i < 3; i++) { // A 的轴
        for (int j = 0; j < 3; j++) { // B 的轴
            T s = T_vec[(i + 2) % 3] * B[j * 3 + (i + 1) % 3] - T_vec[(i + 1) % 3] * B[j * 3 + (i + 2) % 3];
            t_val = fabs(s);
            T threshold = a[(i + 1) % 3] * Bf[j * 3 + (i + 2) % 3] + a[(i + 2) % 3] * Bf[j * 3 + (i + 1) % 3]
                        + b[(j + 1) % 3] * Bf[(j + 2) % 3 * 3 + i] + b[(j + 2) % 3] * Bf[(j + 1) % 3 * 3 + i];
            if (t_val > threshold)
                return true;
        }
    }

    return false;
}

// 判断两个OBB是否重叠的GPU设备函数
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

// GPU核函数，用于检查所有OBB对是否重叠
__global__ void overlap_kernel(const OBB_GPU* collision_geometry, int num_i, const OBB_GPU* collision_objects, int num_j, bool* results){
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = num_i * num_j;
    if (idx < total) {
        int i = idx / num_j;
        int j = idx % num_j;

        bool overlap = overlap_gpu(collision_geometry[i], collision_objects[j]);
        results[idx] = overlap;
    }
}

// 在GPU上检查碰撞的函数，并测量各个部分的运行时间
void checkcollisionOnGPU(const std::vector<CollisionObject<S>*>& collision1, 
                         const std::vector<CollisionObject<S>*>& collision2, 
                         bool* result,
                         CollisionTimings* timings){

    size_t num_i = collision1.size();
    size_t num_j = collision2.size();
    size_t total = num_i * num_j;

    // 开始类型转换计时
    auto start_conversion = std::chrono::high_resolution_clock::now();

    // 开始 std::vector 构造计时
    auto start_vector = std::chrono::high_resolution_clock::now();
    std::vector<OBB_GPU> h_collision1_obb(num_i);
    std::vector<OBB_GPU> h_collision2_obb(num_j);
    auto end_vector = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> vector_duration = end_vector - start_vector;
    // 将 std::vector 构造时间纳入类型转换时间
    timings->type_conversion_time_us = vector_duration.count();

    // 将碰撞对象转换为OBB_GPU格式
    for(size_t i = 0; i < num_i; i++){
        const auto& obb = collision1[i]->obb;

        // 将Eigen矩阵转换为普通数组
        Eigen::Map<Eigen::Matrix<S, 3, 3, Eigen::RowMajor>> axis_mat(const_cast<S*>(obb.axis.data()));
        Eigen::Map<Eigen::Matrix<S, 3, 1>>(h_collision1_obb[i].To) = obb.To;
        Eigen::Map<Eigen::Matrix<S, 3, 1>>(h_collision1_obb[i].extent) = obb.extent;

        // 将轴矩阵展开为数组
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                h_collision1_obb[i].axis[row * 3 + col] = obb.axis(row, col);
            }
        }
    }

    for (size_t i = 0; i < num_j; i++) {
        const auto& obb = collision2[i]->obb;
        Eigen::Map<Eigen::Matrix<S, 3, 3, Eigen::RowMajor>> axis_mat(const_cast<S*>(obb.axis.data()));
        Eigen::Map<Eigen::Matrix<S, 3, 1>>(h_collision2_obb[i].To) = obb.To;
        Eigen::Map<Eigen::Matrix<S, 3, 1>>(h_collision2_obb[i].extent) = obb.extent;
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                h_collision2_obb[i].axis[row * 3 + col] = obb.axis(row, col);
            }
        }
    }

    // 结束类型转换计时
    auto end_conversion = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> conversion_time = end_conversion - start_conversion;
    timings->type_conversion_time_us += conversion_time.count();

    // 设备内存指针
    OBB_GPU* d_collision1_obb;
    OBB_GPU* d_collision2_obb;
    bool* d_results;
    bool* h_results;

    // 开始主函数中通过 new 分配 h_results 的计时
    auto start_malloc_h_results = std::chrono::high_resolution_clock::now();
    h_results = new bool[total];
    auto end_malloc_h_results = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> malloc_h_results_duration = end_malloc_h_results - start_malloc_h_results;
    timings->malloc_h_results_time_us = malloc_h_results_duration.count();

    // 创建CUDA事件用于计时
    cudaEvent_t startTotal, stopTotal;
    CUDA_CHECK(cudaEventCreate(&startTotal));
    CUDA_CHECK(cudaEventCreate(&stopTotal));
    CUDA_CHECK(cudaEventRecord(startTotal, 0));

    // 分配设备内存并测量时间
    auto start_malloc_d1 = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaMalloc((void**)&d_collision1_obb, num_i * sizeof(OBB_GPU)));
    auto end_malloc_d1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> malloc_d1_duration = end_malloc_d1 - start_malloc_d1;
    timings->malloc_d_collision1_time_us = malloc_d1_duration.count();

    auto start_malloc_d2 = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaMalloc((void**)&d_collision2_obb, num_j * sizeof(OBB_GPU)));
    auto end_malloc_d2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> malloc_d2_duration = end_malloc_d2 - start_malloc_d2;
    timings->malloc_d_collision2_time_us = malloc_d2_duration.count();

    auto start_malloc_dresults = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaMalloc((void**)&d_results, total * sizeof(bool)));
    auto end_malloc_dresults = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> malloc_dresults_duration = end_malloc_dresults - start_malloc_dresults;
    timings->malloc_d_results_time_us = malloc_dresults_duration.count();

    // 记录Host到Device的内存拷贝时间
    auto start_copy_H2D = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaMemcpy(d_collision1_obb, h_collision1_obb.data(), num_i * sizeof(OBB_GPU), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_collision2_obb, h_collision2_obb.data(), num_j * sizeof(OBB_GPU), cudaMemcpyHostToDevice));
    auto end_copy_H2D = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> copy_H2D_duration = end_copy_H2D - start_copy_H2D;
    timings->copy_H2D_time_us = copy_H2D_duration.count();

    // 设置CUDA核函数的执行配置
    int numThreads = 256;
    int numBlocks = (total + numThreads - 1) / numThreads;

    // 记录核函数执行时间
    cudaEvent_t startKernel, stopKernel;
    CUDA_CHECK(cudaEventCreate(&startKernel));
    CUDA_CHECK(cudaEventCreate(&stopKernel));
    CUDA_CHECK(cudaEventRecord(startKernel, 0));

    // 启动核函数
    overlap_kernel<<<numBlocks, numThreads>>>(d_collision1_obb, num_i, d_collision2_obb, num_j, d_results);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cudaEventRecord(stopKernel, 0));
    CUDA_CHECK(cudaEventSynchronize(stopKernel));

    float timeKernel = 0.0f;
    CUDA_CHECK(cudaEventElapsedTime(&timeKernel, startKernel, stopKernel));

    // 记录Device到Host的内存拷贝时间
    auto start_copy_D2H = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaMemcpy(h_results, d_results, total * sizeof(bool), cudaMemcpyDeviceToHost));
    auto end_copy_D2H = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> copy_D2H_duration = end_copy_D2H - start_copy_D2H;
    timings->copy_D2H_time_us = copy_D2H_duration.count();

    // 记录总的GPU操作时间
    CUDA_CHECK(cudaEventRecord(stopTotal, 0));
    CUDA_CHECK(cudaEventSynchronize(stopTotal));

    float timeTotal = 0.0f;
    CUDA_CHECK(cudaEventElapsedTime(&timeTotal, startTotal, stopTotal));

    // 输出各部分的时间
    std::cout << "GPU操作时间统计：" << std::endl;
    std::cout << "  Host到Device的内存拷贝时间: " << timings->copy_H2D_time_us << " 微秒" << std::endl;
    std::cout << "  核函数执行时间: " << timeKernel << " 毫秒" << std::endl;
    std::cout << "  Device到Host的内存拷贝时间: " << timings->copy_D2H_time_us << " 微秒" << std::endl;
    std::cout << "  总GPU操作时间: " << timeTotal << " 毫秒" << std::endl;

    timings->gpu_kernel_time_us = timeKernel;
    timings->gpu_operations_time_ms = timeTotal;

    // 记录资源释放时间
    auto start_free = std::chrono::high_resolution_clock::now();

    // 释放资源并测量时间
    auto start_free_h_results = std::chrono::high_resolution_clock::now();
    delete[] h_results;
    auto end_free_h_results = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> free_h_results_duration = end_free_h_results - start_free_h_results;
    timings->free_h_results_time_us = free_h_results_duration.count();

    auto start_free_dresults = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaFree(d_results));
    auto end_free_dresults = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> free_dresults_duration = end_free_dresults - start_free_dresults;
    timings->free_d_results_time_us = free_dresults_duration.count();

    auto start_free_d1 = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaFree(d_collision1_obb));
    auto end_free_d1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> free_d1_duration = end_free_d1 - start_free_d1;
    timings->free_d_collision1_time_us = free_d1_duration.count();

    auto start_free_d2 = std::chrono::high_resolution_clock::now();
    CUDA_CHECK(cudaFree(d_collision2_obb));
    auto end_free_d2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> free_d2_duration = end_free_d2 - start_free_d2;
    timings->free_d_collision2_time_us = free_d2_duration.count();

    auto end_free = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> free_total_duration = end_free - start_free;
    // 可以选择不使用这个总时间，因为已经分别测量了各个部分
    // double total_free_time_us = free_total_duration.count();
    std::cout << "资源创建时间统计：" << std::endl;
    std::cout << "  总的GPU操作时间: " << timings->gpu_operations_time_ms << " 毫秒" << std::endl;
    std::cout << "    cudaMalloc(d_collision1_obb) 时间: " << timings->malloc_d_collision1_time_us << " 微秒" << std::endl;
    std::cout << "    cudaMalloc(d_collision2_obb) 时间: " << timings->malloc_d_collision2_time_us << " 微秒" << std::endl;
    std::cout << "    cudaMalloc(d_results) 时间: " << timings->malloc_d_results_time_us << " 微秒" << std::endl;
    std::cout << "    Host到Device的内存拷贝时间: " << timings->copy_H2D_time_us << " 微秒" << std::endl;
    std::cout << "    Device到Host的内存拷贝时间: " << timings->copy_D2H_time_us << " 微秒" << std::endl;
    std::cout << "    核函数时间: " << timings->gpu_kernel_time_us << " 微秒" << std::endl;

    std::cout << "资源释放时间统计：" << std::endl;
    std::cout << "  delete[] h_results 时间: " << timings->free_h_results_time_us << " 微秒" << std::endl;
    std::cout << "  cudaFree(d_collision1_obb) 时间: " << timings->free_d_collision1_time_us << " 微秒" << std::endl;
    std::cout << "  cudaFree(d_collision2_obb) 时间: " << timings->free_d_collision2_time_us << " 微秒" << std::endl;
    std::cout << "  cudaFree(d_results) 时间: " << timings->free_d_results_time_us << " 微秒" << std::endl;

    // 检查是否存在重叠
    *result = false;
    for (size_t idx = 0; idx < total; idx++) {
        if (h_results[idx]) {
            *result = true;
            break;
        }
    }

    // 释放CUDA事件
    CUDA_CHECK(cudaEventDestroy(startTotal));
    CUDA_CHECK(cudaEventDestroy(stopTotal));
    CUDA_CHECK(cudaEventDestroy(startKernel));
    CUDA_CHECK(cudaEventDestroy(stopKernel));
}

// 主函数
int main() {
    
    const std::string directory = BACON_SOURCE_DIR "/env/48";  
    int filenum = 100; 
    
    // 总的时间统计
    double total_type_conversion_time_us = 0.0;
    double total_gpu_operations_time_ms = 0.0;
    double total_malloc_d_collision1_time_us = 0.0;
    double total_malloc_d_collision2_time_us = 0.0;
    double total_malloc_d_results_time_us = 0.0;
    double total_copy_H2D_time_us = 0.0;
    double total_copy_D2H_time_us = 0.0;
    double total_malloc_host_collision_geometry_time_us = 0.0;
    double total_malloc_h_results_time_us = 0.0;
    double total_free_h_results_time_us = 0.0;
    double total_free_d_collision1_time_us = 0.0;
    double total_free_d_collision2_time_us = 0.0;
    double total_free_d_results_time_us = 0.0;
    double total_gpu_kernel_time_us = 0.0;

    // 用于统计总的OBB检测时间
    std::chrono::duration<double, std::micro> total_obb_time(0);

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) { 
            std::string filename = entry.path().string();

            // 开始记录主函数中通过 new 读取碰撞对象的时间
            auto start_malloc_host_collision_geometry = std::chrono::high_resolution_clock::now();

            // 读取碰撞对象
            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            auto end_malloc_host_collision_geometry = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::micro> malloc_host_collision_geometry_duration = end_malloc_host_collision_geometry - start_malloc_host_collision_geometry;
            total_malloc_host_collision_geometry_time_us += malloc_host_collision_geometry_duration.count();

            // 加载URDF模型
            URDFModel model;
            std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; 
            if (!model.loadURDF(urdfFilePath)) {
                std::cerr << "无法加载URDF文件: " << urdfFilePath << std::endl;
                assert(0);
                return -1;
            }

            // 设置关节角度
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

            // 迭代所有可能的关节角度组合
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

                                    // 开始记录主函数中通过 new 分配 collision_geometry_ 的时间
                                    auto start_malloc_collision_geometry = std::chrono::high_resolution_clock::now();

                                    // 生成碰撞几何体
                                    std::vector<CollisionObject<S>*> collision_geometry_;
                                    for (int i = 0; i < model.collisionGeometries.size(); i++) {
                                        const auto& collisionGeomPtr = model.collisionGeometries[i];
                                        collision_geometry_.push_back(new CollisionObject<S>(*collisionGeomPtr));
                                    }

                                    auto end_malloc_collision_geometry = std::chrono::high_resolution_clock::now();
                                    std::chrono::duration<double, std::micro> malloc_collision_geometry_duration = end_malloc_collision_geometry - start_malloc_collision_geometry;
                                    // 注意：此处不应再次累加到总时间中，因为 `checkcollisionOnGPU` 已经处理
                                    // total_malloc_host_collision_geometry_time_us += malloc_collision_geometry_duration.count();

                                    bool result = false;
                                    
                                    // 记录OBB检测的总时间
                                    auto start_obb = std::chrono::high_resolution_clock::now();

                                    // 创建一个CollisionTimings实例用于记录时间
                                    CollisionTimings current_timings = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                                    // 调用GPU碰撞检测
                                    checkcollisionOnGPU(collision_geometry_, collision_objects, &result, &current_timings);

                                    auto end_obb = std::chrono::high_resolution_clock::now();
                                    std::chrono::duration<double, std::micro> obb_duration = end_obb - start_obb;
                                    total_obb_time += obb_duration;

                                    // 累计类型转换时间和GPU操作时间
                                    total_type_conversion_time_us += current_timings.type_conversion_time_us;
                                    total_gpu_operations_time_ms += current_timings.gpu_operations_time_ms;
                                    total_malloc_d_collision1_time_us += current_timings.malloc_d_collision1_time_us;
                                    total_malloc_d_collision2_time_us += current_timings.malloc_d_collision2_time_us;
                                    total_malloc_d_results_time_us += current_timings.malloc_d_results_time_us;
                                    total_copy_H2D_time_us += current_timings.copy_H2D_time_us;
                                    total_copy_D2H_time_us += current_timings.copy_D2H_time_us;
                                    total_malloc_h_results_time_us += current_timings.malloc_h_results_time_us;
                                    total_free_h_results_time_us += current_timings.free_h_results_time_us;
                                    total_free_d_collision1_time_us += current_timings.free_d_collision1_time_us;
                                    total_free_d_collision2_time_us += current_timings.free_d_collision2_time_us;
                                    total_free_d_results_time_us += current_timings.free_d_results_time_us;
                                    total_gpu_kernel_time_us += current_timings.gpu_kernel_time_us;
                                    
                                    // 释放生成的碰撞几何体
                                    for(auto obj : collision_geometry_) {
                                        delete obj;
                                    }

                                    // 可选：输出关节角度和碰撞检测结果
                                    /*
                                    std::cout << "关节角度: " << joint1 << ", " << joint2 << ", " << joint3 << ", "
                                              << joint4 << ", " << joint5 << ", " << joint6
                                              << " - 碰撞检测: " << (result ? "环境碰撞" : "无碰撞") << std::endl;
                                    */
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 输出总的时间统计
    std::cout << "\n总的时间统计：" << std::endl;
    std::cout << "  总的类型转换时间: " << total_type_conversion_time_us << " 微秒" << std::endl;
    std::cout << "  总的GPU操作时间: " << total_gpu_operations_time_ms << " 毫秒" << std::endl;
    std::cout << "    总的cudaMalloc(d_collision1_obb) 时间: " << total_malloc_d_collision1_time_us << " 微秒" << std::endl;
    std::cout << "    总的cudaMalloc(d_collision2_obb) 时间: " << total_malloc_d_collision2_time_us << " 微秒" << std::endl;
    std::cout << "    总的cudaMalloc(d_results) 时间: " << total_malloc_d_results_time_us << " 微秒" << std::endl;
    std::cout << "    总的Host到Device的内存拷贝时间: " << total_copy_H2D_time_us << " 微秒" << std::endl;
    std::cout << "    总的Device到Host的内存拷贝时间: " << total_copy_D2H_time_us << " 微秒" << std::endl;
    std::cout << "    总的核函数时间: " << total_gpu_kernel_time_us << " 微秒" << std::endl;
    std::cout << "  总的内存分配时间:" << std::endl;
    std::cout << "    总的Host Collision Geometry 分配时间: " << total_malloc_host_collision_geometry_time_us << " 微秒" << std::endl;
    std::cout << "    总的h_results 分配时间: " << total_malloc_h_results_time_us << " 微秒" << std::endl;
    std::cout << "  总的资源释放时间:" << std::endl;
    std::cout << "    总的delete[] h_results 时间: " << total_free_h_results_time_us << " 微秒" << std::endl;
    std::cout << "    总的cudaFree(d_collision1_obb) 时间: " << total_free_d_collision1_time_us << " 微秒" << std::endl;
    std::cout << "    总的cudaFree(d_collision2_obb) 时间: " << total_free_d_collision2_time_us << " 微秒" << std::endl;
    std::cout << "    总的cudaFree(d_results) 时间: " << total_free_d_results_time_us << " 微秒" << std::endl;
    std::cout << "  总的OBB重叠检测时间: " << total_obb_time.count() << " 微秒" << std::endl;

    return 0;

}