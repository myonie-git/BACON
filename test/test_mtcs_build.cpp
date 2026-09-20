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
#include <memory>
#include <limits>
#include <chrono>

namespace fs = std::filesystem;
using S = double;

struct GroupInfo {
    std::vector<int> indices;   // 存储组内元素的索引
};

// ------------------ 碰撞检测相关函数 ------------------
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


// ------------------ MCTS 相关实现 ------------------

// 节点类
struct Node {
    std::vector<int> groups;  // 当前的分组方案
    int element_index;        // 当前分配到第几个元素
    int num_groups;           // 当前   的组数
    std::vector<std::shared_ptr<Node>> children; // 子节点
    double total_reward;      // 总的奖励值（适应度值）
    int visit_count;          // 被访问的次数
    Node* parent;             // 父节点

    Node(const std::vector<int>& groups_, int element_index_, int num_groups_, Node* parent_)
        : groups(groups_), element_index(element_index_), num_groups(num_groups_), total_reward(0.0), visit_count(0), parent(parent_) {}
};

// UCT 公式中的探索参数
const double exploration_constant = std::sqrt(2.0);

// 从根节点开始，进行一次蒙特卡洛树搜索
void MCTS(std::shared_ptr<Node>& root, int n, int k, int max_iterations, int time_limit_ms) {
    auto start_time = std::chrono::steady_clock::now();
    int iterations = 0;

    while (iterations < max_iterations) {
        // 检查时间限制
        auto current_time = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (elapsed_ms >= time_limit_ms) {
            break;
        }

        // 1. 选择
        Node* node = root.get();
        while (node->element_index < n && !node->children.empty()) {
            // 使用 UCT 选择子节点
            double best_value = -std::numeric_limits<double>::infinity();
            Node* best_child = nullptr;
            for (auto& child : node->children) {
                double uct_value = (child->visit_count == 0) ? std::numeric_limits<double>::infinity()
                    : child->total_reward / child->visit_count +
                    exploration_constant * std::sqrt(std::log(node->visit_count) / child->visit_count);
                if (uct_value > best_value) {
                    best_value = uct_value;
                    best_child = child.get();
                }
            }
            node = best_child;
        }

        // 2. 扩展
        if (node->element_index < n) {
            // 生成可能的动作，即将下一个元素分配到一个未满的组或新建一个组
            std::vector<int> possible_groups;

            std::map<int, int> group_sizes;
            for (int i = 0; i < node->element_index; ++i) {
                group_sizes[node->groups[i]]++; //统计每个Group的大小
            }

            // 可以分配到已有的未满的组
            for (const auto& pair : group_sizes) {
                if (pair.second < k) {
                    possible_groups.push_back(pair.first); //统计没有满的组的编号
                }
            }
            
            int next_group_id = node->num_groups;
            possible_groups.push_back(next_group_id); // 新建一个组

            // 对每个可能的动作创建一个子节点
            for (int group_id : possible_groups) {
                std::vector<int> new_groups = node->groups;
                new_groups[node->element_index] = group_id; //将该Group分配给不同组
                int new_num_groups = node->num_groups; 
                if (group_id == next_group_id) {
                    new_num_groups++;
                }
                auto child = std::make_shared<Node>(new_groups, node->element_index + 1, new_num_groups, node);
                node->children.push_back(child);
            }

            // 随机选择一个子节点
            std::uniform_int_distribution<int> dist(0, node->children.size() - 1);
            std::random_device rd;
            std::mt19937 gen(rd());
            node = node->children[dist(gen)].get(); //随机选择一个node进行模拟
        }

        // 3. 模拟
        // 从当前状态开始，随机完成剩余元素的分配
        std::vector<int> simulation_groups = node->groups;
        int simulation_element_index = node->element_index;
        int simulation_num_groups = node->num_groups;
        std::map<int, int> simulation_group_sizes;
        for (int i = 0; i < simulation_element_index; ++i) {
            simulation_group_sizes[simulation_groups[i]]++;
        }

        std::random_device rd;
        std::mt19937 gen(rd());

        for (int i = simulation_element_index; i < n; ++i) {
            // 获取可用的组号
            std::vector<int> possible_groups;
            for (const auto& pair : simulation_group_sizes) {
                if (pair.second < k) {
                    possible_groups.push_back(pair.first);
                }
            }
            // 可以新建一个组
            int next_group_id = simulation_num_groups;
            possible_groups.push_back(next_group_id);

            // 随机选择一个组
            std::uniform_int_distribution<int> dist(0, possible_groups.size() - 1);
            int selected_group = possible_groups[dist(gen)];
            simulation_groups[i] = selected_group;
            simulation_group_sizes[selected_group]++;
            if (selected_group == next_group_id) {
                simulation_num_groups++;
            }
        }

        // 计算适应度值
        double aabb_count = complex_function(simulation_groups);
        // double scale_factor = 200000.0;
        // double reward = std::exp(-aabb_count / scale_factor);
        double reward = 1.0 / (aabb_count + 1e-6); // 适应度值越大越好
        // double reward = -std::log(aabb_count + 1.0);
        // double reward = std::exp(-aabb_count / scale_factor);

        // 4. 回溯
        while (node != nullptr) {
            node->visit_count++;
            node->total_reward += reward;
            node = node->parent;
        }

        iterations++;
    }
}

int main() {
    std::srand(static_cast<unsigned>(std::time(0))); // 设置随机种子

    const int n = 17; // 元素数量
    const int k = 5;  // 每组最多的元素数量

    // 初始化根节点
    std::vector<int> initial_groups(n, -1); // -1 表示尚未分配
    auto root = std::make_shared<Node>(initial_groups, 0, 0, nullptr);

    // 设定蒙特卡洛树搜索的参数
    int max_iterations = 100000;    // 最大迭代次数
    int time_limit_ms = 50000;      // 时间限制（毫秒）

    // 开始蒙特卡洛树搜索
    MCTS(root, n, k, max_iterations, time_limit_ms);

    // 从根节点开始，选择访问次数最多的子节点，构建最优的分组方案
    Node* node = root.get();
    while (node->element_index < n && !node->children.empty()) {
        Node* best_child = nullptr;
        int max_visits = -1;
        for (auto& child : node->children) {
            if (child->visit_count > max_visits) {
                max_visits = child->visit_count;
                best_child = child.get();
            }
        }
        if (best_child == nullptr) {
            break;
        }
        node = best_child;
    }

    // 如果未分配完所有元素，随机完成剩余的分配
    std::vector<int> best_groups = node->groups;
    int element_index = node->element_index;
    int num_groups = node->num_groups;
    std::map<int, int> group_sizes;
    for (int i = 0; i < element_index; ++i) {
        group_sizes[best_groups[i]]++;
    }

    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = element_index; i < n; ++i) {
        // 获取可用的组号
        std::vector<int> possible_groups;
        for (const auto& pair : group_sizes) {
            if (pair.second < k) {
                possible_groups.push_back(pair.first);
            }
        }
        // 可以新建一个组
        int next_group_id = num_groups;
        possible_groups.push_back(next_group_id);

        // 随机选择一个组
        std::uniform_int_distribution<int> dist(0, possible_groups.size() - 1);
        int selected_group =  possible_groups[dist(gen)];
        best_groups[i] = selected_group;
        group_sizes[selected_group]++;
        if (selected_group == next_group_id) {
            num_groups++;
        }
    }

    // 计算最优分组方案的适应度值和 aabb_count
    double best_aabb_count = complex_function(best_groups);
    double best_fitness = 1.0 / (best_aabb_count + 1e-6);

    // 获取唯一的组号
    std::vector<int> unique_groups = best_groups;
    std::sort(unique_groups.begin(), unique_groups.end());
    unique_groups.erase(std::unique(unique_groups.begin(), unique_groups.end()), unique_groups.end());

    // 输出最优分组方案
    std::cout << "\n最优的分组方案：" << std::endl;
    for (int g : best_groups) {
        std::cout << g << " ";
    }
    std::cout << "\n对应的适应度值：" << best_fitness << std::endl;
    std::cout << "对应的 aabb_count：" << best_aabb_count << std::endl;

    // 输出每组中的元素编号
    std::cout << "\n每组中的元素编号：" << std::endl;
    for (int group : unique_groups) {
        std::cout << "组 " << group << " 中的元素编号: ";
        for (size_t i = 0; i < best_groups.size(); ++i) {
            if (best_groups[i] == group) {
                std::cout << i << " ";
            }
        }
        std::cout << std::endl;
    }

    return 0;
}
