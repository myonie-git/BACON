#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>
#include <Eigen/Geometry>
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>
#include <filesystem>
#include <map>

namespace fs = std::filesystem;
using S = double;

struct GroupInfo {
    std::vector<int> indices;   // 存储组内元素的索引
};

// 修复函数，确保每组的元素数量不超过k
void repair_individual(std::vector<int>& individual, int k) {
    // 统计每个组中元素的索引
    std::map<int, std::vector<int>> group_to_indices;
    for (int i = 0; i < individual.size(); ++i) {
        group_to_indices[individual[i]].push_back(i);
    }

    int next_group_num = group_to_indices.size();
    for (auto& pair : group_to_indices) {
        int group_num = pair.first;
        std::vector<int>& indices = pair.second;
        if (indices.size() > k) {
            // 将超出k个元素的组拆分成多个组
            int num_new_groups = (indices.size() + k - 1) / k;
            for (int i = 0; i < num_new_groups; ++i) {
                int start_idx = i * k;
                int end_idx = std::min((i + 1) * k, (int)indices.size());
                int new_group_num = (i == 0) ? group_num : next_group_num++;
                for (int idx = start_idx; idx < end_idx; ++idx) {
                    individual[indices[idx]] = new_group_num;
                }
            }
        }
    }
}

double simulate(const std::vector<GroupInfo>& group_info) {
    const std::string directory = BACON_SOURCE_DIR "/env/48-bak";  
    int aabb_count = 0;
    int obb_count = 0;
    
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

                                    bool obb_happened = false;
                                    bool result = false;
                                    int temp_aabb_count = 0;
                                    for (int i = 0; i < collision_objects.size() && !result; i++) {
                                        for(auto& group : group_info){
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
                                                for(int idx : group.indices){
                                                    if(result) break;
                                                    bool aabb_result;
                                                    aabb_count++;
                                                    temp_aabb_count ++;
                                                    aabb_result = collision_objects[i]->aabb.overlap(collision_geometry_[idx].aabb);
                                                    if(aabb_result){
                                                        result = collision_objects[i]->obb.overlap(collision_geometry_[idx].obb);
                                                        obb_count++;
                                                    }
                                                }
                                            }
                                        }
                                        
                                        if(obb_happened){
                                            aabb_count -= temp_aabb_count;
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

    return aabb_count;
}

// 将 groups 转换为 GroupInfo 结构体的向量
std::vector<GroupInfo> convert_to_group_info_vector(const std::vector<int>& groups) {
    // 先找出所有的唯一组号
    std::vector<int> unique_groups(groups);
    std::sort(unique_groups.begin(), unique_groups.end());
    unique_groups.erase(std::unique(unique_groups.begin(), unique_groups.end()), unique_groups.end());

    // 创建 GroupInfo 向量，大小为唯一组号的数量
    std::vector<GroupInfo> group_info(unique_groups.size());

    // 遍历所有元素，将每个元素分配到相应的组
    for (size_t i = 0; i < groups.size(); ++i) {
        int group = groups[i];
        // 找到组号对应的位置
        auto it = std::find(unique_groups.begin(), unique_groups.end(), group);
        int group_index = std::distance(unique_groups.begin(), it);

        // 将当前元素的索引添加到相应的组
        group_info[group_index].indices.push_back(i);
    }

    return group_info;
}

double complex_function(const std::vector<int>& groups) {
    // 将 groups 转换为 GroupInfo 结构体的向量
    std::vector<GroupInfo> group_info = convert_to_group_info_vector(groups);

    // 调用 simulate 函数处理 group_info
    double fitness = simulate(group_info);

    return fitness;
}

int main() {
    std::srand(static_cast<unsigned>(std::time(0))); // 设置随机种子

    const int population_size = 100;  // 种群大小
    const int num_generations = 100;  // 迭代次数
    const double mutation_rate = 0.05; // 变异率

    const int n = 17; // 元素数量
    const int k = 5;  // 每组最多的元素数量

    std::vector<std::vector<int>> population(population_size, std::vector<int>(n));

    // 初始化种群，确保每组元素数量不超过k
    for (int i = 0; i < population_size; ++i) {
        // 初始化组号，从0开始
        int group_num = 0;
        int count_in_group = 0;

        // 打乱元素的顺序
        std::vector<int> indices(n);
        std::iota(indices.begin(), indices.end(), 0);
        std::random_shuffle(indices.begin(), indices.end());

        for (int idx : indices) {
            population[i][idx] = group_num;
            count_in_group++;
            if (count_in_group >= k) {
                group_num++;
                count_in_group = 0;
            }
        }
    }
    // 遗传算法主循环
    for (int generation = 0; generation < num_generations; ++generation) {
        std::vector<double> fitness_values(population_size);

        // 计算适应度值
        for (int i = 0; i < population_size; ++i) {
            double aabb_count = complex_function(population[i]);

            // 适应度值为 aabb_count 的倒数，值越小越好
            fitness_values[i] = 1.0 / (aabb_count + 1e-6); // 防止除以零
        }

        // 归一化适应度值
        double fitness_sum = std::accumulate(fitness_values.begin(), fitness_values.end(), 0.0);
        if (fitness_sum > 0) {
            for (auto& f : fitness_values) {
                f /= fitness_sum;
            }
        } else {
            // 所有适应度值为零时，赋予均等概率
            double equal_probability = 1.0 / population_size;
            for (auto& f : fitness_values) {
                f = equal_probability;
            }
        }

        // 选择操作（轮盘赌选择）
        std::discrete_distribution<int> selection_dist(fitness_values.begin(), fitness_values.end());
        std::random_device rd;
        std::mt19937 gen(rd());

        // 创建新种群
        std::vector<std::vector<int>> new_population(population_size, std::vector<int>(n));
        for (int i = 0; i < population_size; ++i) {
            int selected_index = selection_dist(gen);
            new_population[i] = population[selected_index];
        }

        // 交叉操作（单点交叉），并确保新个体满足每组不超过k个元素
        for (int i = 0; i < population_size; i += 2) {
            if (i + 1 < population_size) {
                int crossover_point = std::rand() % (n - 1) + 1;
                // 生成子代
                std::vector<int> offspring1 = new_population[i];
                std::vector<int> offspring2 = new_population[i + 1];

                // 进行交叉
                std::swap_ranges(offspring1.begin(), offspring1.begin() + crossover_point, offspring2.begin());

                // 修复子代，确保每组不超过k个元素
                repair_individual(offspring1, k);
                repair_individual(offspring2, k);

                // 替换到新种群中
                new_population[i] = offspring1;
                new_population[i + 1] = offspring2;
            }
        }

        // 变异操作，并确保新个体满足每组不超过k个元素
        for (auto& individual : new_population) {
            if (static_cast<double>(std::rand()) / RAND_MAX < mutation_rate) {
                int mutate_point = std::rand() % n;
                int current_max_group = *std::max_element(individual.begin(), individual.end());
                // 随机选择一个新的组号
                int new_group = std::rand() % (current_max_group + 2); // +2以可能产生新的组
                individual[mutate_point] = new_group;

                // 修复个体，确保每组不超过k个元素
                repair_individual(individual, k);
            }
        }

        // 更新种群
        population = new_population;

        // 每隔10代打印最佳适应度值和对应的 aabb_count
        if (generation % 10 == 0) {
            // 找到最佳适应度值及其索引
            auto best_fitness_it = std::max_element(fitness_values.begin(), fitness_values.end());
            double best_fitness = *best_fitness_it;
            int best_index = std::distance(fitness_values.begin(), best_fitness_it);
            double best_aabb_count = 1.0 / best_fitness - 1e-6;

            std::cout << "第 " << generation << " 代: 最佳适应度值 = " << best_fitness
                      << ", 对应的 aabb_count = " << best_aabb_count << std::endl;
        }
    }

    // 输出最优解
    // 找到最佳个体
    double best_fitness = 0.0;
    int best_index = 0;
    for (int i = 0; i < population_size; ++i) {
        double aabb_count = complex_function(population[i]);
        double fitness = 1.0 / (aabb_count + 1e-6);
        if (fitness > best_fitness) {
            best_fitness = fitness;
            best_index = i;
        }
    }

    const std::vector<int>& best_individual = population[best_index];
    double best_aabb_count = complex_function(best_individual);

    // 获取唯一的组号
    std::vector<int> unique_groups(best_individual);
    std::sort(unique_groups.begin(), unique_groups.end());
    unique_groups.erase(std::unique(unique_groups.begin(), unique_groups.end()), unique_groups.end());

    // 输出最优分组方案
    std::cout << "\n最优的分组方案：" << std::endl;
    for (int g : best_individual) {
        std::cout << g << " ";
    }
    std::cout << "\n对应的适应度值：" << best_fitness << std::endl;
    std::cout << "对应的 aabb_count：" << best_aabb_count << std::endl;

    // 输出每组中的元素编号
    std::cout << "\n每组中的元素编号：" << std::endl;
    for (int group : unique_groups) {
        std::cout << "组 " << group << " 中的元素编号: ";
        for (size_t i = 0; i < best_individual.size(); ++i) {
            if (best_individual[i] == group) {
                std::cout << i << " ";
            }
        }
        std::cout << std::endl;
    }

    return 0;
}
