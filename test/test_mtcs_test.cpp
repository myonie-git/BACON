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

// ------------------ MCTS Implementation ------------------

// Node class
struct Node {
    std::vector<int> groups;  // Current grouping scheme
    int element_index;        // Index of the element to assign next
    int num_groups;           // Current number of groups
    std::vector<std::shared_ptr<Node>> children; // Child nodes
    double total_reward;      // Total reward (fitness value)
    int visit_count;          // Number of visits
    Node* parent;             // Parent node

    Node(const std::vector<int>& groups_, int element_index_, int num_groups_, Node* parent_)
        : groups(groups_), element_index(element_index_), num_groups(num_groups_), total_reward(0.0), visit_count(0), parent(parent_) {}
};

// UCT exploration constant
const double exploration_constant = std::sqrt(2.0);

// MCTS function
void MCTS(std::shared_ptr<Node>& root, int n, int k, int max_iterations, int time_limit_ms, const std::vector<bool>& assigned_elements) {
    auto start_time = std::chrono::steady_clock::now();
    int iterations = 0;
    std::random_device rd;
    std::mt19937 gen(rd());

    while (iterations < max_iterations) {
        // Check time limit
        auto current_time = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (elapsed_ms >= time_limit_ms) {
            break;
        }

        // 1. Selection
        Node* node = root.get();
        while (node->element_index < n) {
            if (node->children.empty()) {
                // Node is not expanded yet
                break;
            } else {
                // Use UCT to select the best child
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
        }

        // 2. Expansion
        if (node->element_index < n) {
            // Skip assigned elements
            while (node->element_index < n && assigned_elements[node->element_index]) {
                node->element_index++;
            }
            if (node->element_index >= n) {
                // All elements are assigned
                continue;
            }

            // Generate possible actions
            std::vector<int> possible_groups;
            std::map<int, int> group_sizes;
            for (int i = 0; i < n; ++i) {
                if (node->groups[i] != -1) {
                    group_sizes[node->groups[i]]++;
                }
            }

            // Existing groups that are not full
            for (const auto& pair : group_sizes) {
                if (pair.second < k) {
                    possible_groups.push_back(pair.first);
                }
            }

            // Option to create a new group
            int next_group_id = node->num_groups;
            possible_groups.push_back(next_group_id);

            // Expand all possible actions
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
    
            // Randomly select one child to proceed
            std::uniform_int_distribution<int> dist(0, node->children.size() - 1);
            node = node->children[dist(gen)].get();
        }

        // 3. Simulation
        // Assign remaining unassigned elements to a temporary group
        std::vector<int> simulation_groups = node->groups;
        int simulation_num_groups = node->num_groups;
        int temp_group_id = simulation_num_groups; // Temporary group ID for unassigned elements

        for (int i = 0; i < n; ++i) {
            if (simulation_groups[i] == -1) {
                simulation_groups[i] = temp_group_id;
            }
        }
        simulation_num_groups++;

        // Compute fitness
        double aabb_count = complex_function(simulation_groups);
        double reward = 1.0 / (aabb_count + 1e-6);

        // 4. Backpropagation
        Node* backprop_node = node;
        while (backprop_node != nullptr) {
            backprop_node->visit_count++;
            backprop_node->total_reward += reward;
            backprop_node = backprop_node->parent;
        }

        iterations++;
    }
}

int main() {
    std::srand(static_cast<unsigned>(std::time(0))); // Set random seed

    const int n = 17; // Number of elements
    const int k = 5;  // Maximum elements per group

    // Keep track of assigned elements
    std::vector<bool> assigned_elements(n, false);
    std::vector<int> final_groups(n, -1); // -1 indicates unassigned
    int total_assigned = 0;

    int group_id = 0;

    // Loop until all elements are assigned
    while (total_assigned < n) {
        // Initialize root node for current MCTS call
        
        for (const int &group : final_groups) {
            std::cout << group << " ";
        }
        std::cout << std::endl;


        std::vector<int> initial_groups = final_groups; // Start from current assignment
        auto root = std::make_shared<Node>(initial_groups, 0, group_id, nullptr);

        // Set MCTS parameters
        int max_iterations = 10000;    // Maximum iterations for each MCTS call
        int time_limit_ms = 5000;      // Time limit in milliseconds for each MCTS call

        // Run MCTS
        MCTS(root, n, k, max_iterations, time_limit_ms, assigned_elements);

        // Reconstruct the best grouping from the tree
        Node* node = root.get();
        while (node->element_index <= n && !node->children.empty()) {
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

        // Assign elements from the best grouping
        std::vector<int> best_groups = node->groups;
        std::vector<int> new_assigned_elements;
        for (int i = 0; i < n; ++i) {
            if (!assigned_elements[i] && best_groups[i] != -1) {
                final_groups[i] = group_id;
                assigned_elements[i] = true;
                new_assigned_elements.push_back(i);
                total_assigned++;
                if (new_assigned_elements.size() >= k) {
                    break; // Assigned enough elements in this group
                }
            }
        }

        // If no new elements were assigned, assign one unassigned element to a new group
        if (new_assigned_elements.empty()) {
            for (int i = 0; i < n; ++i) {
                if (!assigned_elements[i]) {
                    final_groups[i] = group_id;
                    assigned_elements[i] = true;
                    total_assigned++;
                    new_assigned_elements.push_back(i);
                    break;
                }
            }
        }

        // Output the current group assignment
        std::cout << "Group " << group_id << " assigned elements: ";
        for (int idx : new_assigned_elements) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;

        // Move to next group ID
        group_id++;
    }

    // Compute fitness and aabb_count
    double best_aabb_count = complex_function(final_groups);
    double best_fitness = 1.0 / (best_aabb_count + 1e-6);

    // Get unique group IDs
    std::vector<int> unique_groups = final_groups;
    std::sort(unique_groups.begin(), unique_groups.end());
    unique_groups.erase(std::unique(unique_groups.begin(), unique_groups.end()), unique_groups.end());

    // Output the final grouping
    std::cout << "\nFinal grouping scheme:" << std::endl;
    for (int g : final_groups) {
        std::cout << g << " ";
    }
    std::cout << "\nCorresponding fitness value: " << best_fitness << std::endl;
    std::cout << "Corresponding aabb_count: " << best_aabb_count << std::endl;

    // Output elements in each group
    std::cout << "\nElements in each group:" << std::endl;
    for (int group : unique_groups) {
        std::cout << "Group " << group << " contains elements: ";
        for (size_t i = 0; i < final_groups.size(); ++i) {
            if (final_groups[i] == group) {
                std::cout << i << " ";
            }
        }
        std::cout << std::endl;
    }

    return 0;
}