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
#include <getopt.h>
#include <map>
#include <memory>
#include <limits>
#include <chrono>
#include <omp.h>
#include <cstdlib>

namespace fs = std::filesystem;
using S = double;

struct GroupInfo {
    std::vector<int> indices;   // 存储组内元素的索引
};

// 设定蒙特卡洛树搜索的参数
int max_iterations = 100000;    // 每个元素的最大迭代次数
int time_limit_ms = 5000000;      // 每个元素的时间限制（毫秒）
int simulate_time = 0;
int thread_count = 4;            // 采样并行度（默认 4）
std::string env_directory = BACON_SOURCE_DIR "/env/48-bak";

// 机器人模型与采样关节
URDFModel model;
std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf";
std::string rootLink = "prbt_base_link";
const int k = 5;  // 每组最多的元素数量
std::map<std::string, double> jointAngles = {
    {"prbt_joint_1", 0.0},
    {"prbt_joint_2", 0.0},
    {"prbt_joint_3", 0.0},
    {"prbt_joint_4", 0.0},
    {"prbt_joint_5", 0.0},
    {"prbt_joint_6", 0.0}
};


// ------------------ 碰撞检测相关函数 ------------------
// 基于“linkName -> model.links[linkName] 的位姿”构造碰撞几何（避免拷贝 URDFModel 后 parentLink 指针失效的问题）
static inline void build_collision_geometry_from_model(
    const URDFModel& m,
    std::vector<CollisionObject<S>>& out)
{
    out.clear();
    out.reserve(m.collisionGeometries.size());
    for (const auto& collisionGeomPtr : m.collisionGeometries) {
        if (!collisionGeomPtr || !collisionGeomPtr->parentLink) continue;
        const std::string& link_name = collisionGeomPtr->parentLink->name;
        auto link_it = m.links.find(link_name);
        if (link_it == m.links.end()) continue;

        const Link& link = link_it->second;

        OBB<S> obb = collisionGeomPtr->obb;
        Transform3<S> tf = Transform3<S>::Identity();
        tf.translation() = link.position.cast<S>();
        tf.linear() = link.rotation.toRotationMatrix().cast<S>();

        obb.To = tf * obb.center();
        obb.axis = tf.linear() * obb.axis;

        const Matrix3<S> abs_axis = obb.axis.cwiseAbs();
        const Vector3<S> radius = abs_axis * obb.extent;
        const AABB<S> aabb(obb.To - radius, obb.To + radius);

        out.emplace_back(aabb, obb);
    }
}

double simulate(const std::vector<GroupInfo>& group_info) {
    long long aabb_count = 0;
    long long obb_count = 0;
    
    for (const auto& entry : fs::directory_iterator(env_directory)) {
        if (entry.is_regular_file()) { 
            std::string filename = entry.path().string();

            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            const double start_angle = -1.57;
            const double end_angle = 1.57;
            const double step_size = 1.0;
            const int steps = static_cast<int>((end_angle - start_angle) / step_size) + 1;

            #pragma omp parallel num_threads(thread_count)
            {
                URDFModel local_model = model; // 每线程拷贝一份，避免 model/jointAngles 数据竞争
                std::map<std::string, double> local_jointAngles = jointAngles;
                std::vector<CollisionObject<S>> collision_geometry_;
                collision_geometry_.reserve(local_model.collisionGeometries.size());

                long long aabb_local = 0;
                long long obb_local = 0;

                #pragma omp for collapse(6) schedule(dynamic)
                for (int i1 = 0; i1 < steps; i1++) {
                    for (int i2 = 0; i2 < steps; i2++) {
                        for (int i3 = 0; i3 < steps; i3++) {
                            for (int i4 = 0; i4 < steps; i4++) {
                                for (int i5 = 0; i5 < steps; i5++) {
                                    for (int i6 = 0; i6 < steps; i6++) {
                                        const double joint1 = start_angle + i1 * step_size;
                                        const double joint2 = start_angle + i2 * step_size;
                                        const double joint3 = start_angle + i3 * step_size;
                                        const double joint4 = start_angle + i4 * step_size;
                                        const double joint5 = start_angle + i5 * step_size;
                                        const double joint6 = start_angle + i6 * step_size;

                                        local_jointAngles["prbt_joint_1"] = joint1;
                                        local_jointAngles["prbt_joint_2"] = joint2;
                                        local_jointAngles["prbt_joint_3"] = joint3;
                                        local_jointAngles["prbt_joint_4"] = joint4;
                                        local_jointAngles["prbt_joint_5"] = joint5;
                                        local_jointAngles["prbt_joint_6"] = joint6;

                                        local_model.setJointAngles(local_jointAngles);
                                        local_model.calculateWorldCoordinates(rootLink);
                                        build_collision_geometry_from_model(local_model, collision_geometry_);

                                        bool obb_happened = false;
                                        bool result = false;
                                        int temp_aabb_count = 0;

                                        for (size_t obj_i = 0; obj_i < collision_objects.size() && !result; obj_i++) {
                                            for (const auto& group : group_info) {
                                                if (result) break;
                                                bool aabb_sum_result = false;
                                                AABB<S> tmp_aabb;
                                                for (int idx : group.indices) {
                                                    tmp_aabb += collision_geometry_[idx].aabb;
                                                }

                                                temp_aabb_count++;
                                                aabb_local++;
                                                aabb_sum_result = collision_objects[obj_i]->aabb.overlap(tmp_aabb);
                                                if (aabb_sum_result) {
                                                    for (int idx : group.indices) {
                                                        if (result) break;
                                                        aabb_local++;
                                                        temp_aabb_count++;
                                                        const bool aabb_result =
                                                            collision_objects[obj_i]->aabb.overlap(collision_geometry_[idx].aabb);
                                                        if (aabb_result) {
                                                            result = collision_objects[obj_i]->obb.overlap(collision_geometry_[idx].obb);
                                                            obb_local++;
                                                        }
                                                    }
                                                }
                                            }

                                            if (obb_happened) {
                                                aabb_local -= temp_aabb_count;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                #pragma omp atomic
                aabb_count += aabb_local;
                #pragma omp atomic
                obb_count += obb_local;
            } // omp parallel
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
    simulate_time++;
    std::cout << "Simulate " << simulate_time << ": aabb_count = " << fitness << std::endl;
    return fitness;
}


// ------------------ MCTS 相关实现 ------------------

// 节点类
struct Node {
    std::vector<int> groups;  // 当前的分组方案
    int element_index;        // 当前分配到第几个元素
    int num_groups;           // 当前的组数
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

    int current_element_index = root->element_index;

    while (iterations < max_iterations) {
        // 检查时间限制
        auto current_time = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (elapsed_ms >= time_limit_ms) {
            break;
        }

        // 1. 选择
        Node* node = root.get();
        // 我们只需要为当前元素分配组，所以深度为 1
        while (node->element_index == current_element_index && !node->children.empty()) {
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
        if (node->element_index == current_element_index) {
            // 生成可能的动作，即将当前元素分配到一个未满的组或新建一个组
            std::vector<int> possible_groups;
            std::map<int, int> group_sizes;
            for (int i = 0; i < node->groups.size(); ++i) {
                if (node->groups[i] >= 0) {
                    group_sizes[node->groups[i]]++;
                }
            }

            // 可以分配到已有的未满的组
            for (const auto& pair : group_sizes) {
                if (pair.second < k) {
                    possible_groups.push_back(pair.first);
                }
            }
            int next_group_id = node->num_groups;
            possible_groups.push_back(next_group_id); // 新建一个组

            // 对每个可能的动作创建一个子节点
            for (int group_id : possible_groups) {
                std::vector<int> new_groups = node->groups;
                new_groups[node->element_index] = group_id;
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
            node = node->children[dist(gen)].get();
        }

        // 3. 模拟
        // 从当前状态开始，随机完成剩余元素的分配
        std::vector<int> simulation_groups = node->groups;
        int simulation_element_index = node->element_index;
        int simulation_num_groups = node->num_groups;
        std::map<int, int> simulation_group_sizes;
        for (int i = 0; i < simulation_groups.size(); ++i) {
            if (simulation_groups[i] >= 0) {
                simulation_group_sizes[simulation_groups[i]]++;
            }
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
        double reward = 1.0 / (aabb_count + 1e-6); // 适应度值越大越好

        // 4. 回溯
        while (node != nullptr) {
            node->visit_count++;
            node->total_reward += reward;
            node = node->parent;
        }

        iterations++;
    }
}

int main(int argc, char *argv[]) {
    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {"threads", required_argument, 0, 't'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:t:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                env_directory = optarg;
                break;
            case 't':
                thread_count = std::atoi(optarg);
                break;
            default:
                break;
        }
    }
    if (thread_count <= 0) thread_count = 4;

    std::srand(static_cast<unsigned>(std::time(0))); // 设置随机种子
    auto program_start_time = std::chrono::steady_clock::now();

    // 加载模型（只加载一次）
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    const int n = static_cast<int>(model.collisionGeometries.size()); // 元素数量

    // 初始化分组方案
    std::vector<int> groups(n, -1); // -1 表示尚未分配
    int num_groups = 0;

    // 逐个元素进行分配
    for (int element_index = 0; element_index < n; ++element_index) {
        // 创建当前状态的根节点
        auto root = std::make_shared<Node>(groups, element_index, num_groups, nullptr);


        // 开始蒙特卡洛树搜索，为当前元素选择最优的组
        MCTS(root, n, k, max_iterations, time_limit_ms);

        // 从根节点的子节点中选择访问次数最多的，即为当前元素选择的组
        Node* best_child = nullptr;
        int max_visits = -1;
        for (auto& child : root->children) {
            if (child->visit_count > max_visits) {
                max_visits = child->visit_count;
                best_child = child.get();
            }
        }
        if (best_child == nullptr) {
            std::cerr << "未找到最佳子节点，元素索引：" << element_index << std::endl;
            continue;
        }

        // 更新分组方案
        groups[element_index] = best_child->groups[element_index];

        // 如果创建了新组，更新组数
        if (best_child->groups[element_index] >= num_groups) {
            num_groups = best_child->groups[element_index] + 1;
        }

        // 输出当前元素的分配结果
        std::cout << "元素 " << element_index << " 被分配到组 " << groups[element_index] << std::endl;
    }

    // 计算最优分组方案的适应度值和 aabb_count
    double best_aabb_count = complex_function(groups);
    double best_fitness = 1.0 / (best_aabb_count + 1e-6);

    // 获取唯一的组号
    std::vector<int> unique_groups = groups;
    std::sort(unique_groups.begin(), unique_groups.end());
    unique_groups.erase(std::unique(unique_groups.begin(), unique_groups.end()), unique_groups.end());

    // 输出最优分组方案
    std::cout << "\n最终的分组方案：" << std::endl;
    for (int g : groups) {
        std::cout << g << " ";
    }
    std::cout << "\n对应的适应度值：" << best_fitness << std::endl;
    std::cout << "对应的 aabb_count：" << best_aabb_count << std::endl;

    // 输出每组中的元素编号
    std::cout << "\n每组中的元素编号：" << std::endl;
    for (int group : unique_groups) {
        std::cout << "组 " << group << " 中的元素编号: ";
        for (size_t i = 0; i < groups.size(); ++i) {
            if (groups[i] == group) {
                std::cout << i << " ";
            }
        }
        std::cout << std::endl;
    }

    auto program_end_time = std::chrono::steady_clock::now();
    auto program_elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(program_end_time - program_start_time).count();
    std::cout << "\n程序运行时间: " << program_elapsed_ms << " ms" << std::endl;
    std::cout << "\n Simulate Time: " << simulate_time << std::endl;
    return 0;
}