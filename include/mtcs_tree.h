#ifndef MCTREE_H
#define MCTREE_H

#include <vector>
#include <limits>
#include <random>
#include "mtcs_node.h"
#include "robot_node_base.h"
#include "aabb.h"
#include "collision_object.h"


template <typename BV>
class MCTree {
public:
    using S = typename BV::S;
    typedef RobotNodeBase<AABB<double>> NodeType;
    typedef MCTreeNode<AABB<double>> TreeNode;

    TreeNode* root; 
    int iterations;
    double exploration_constant;
    std::mt19937 gen;
    
    MCTree(NodeType* root_node, int iter = 1000, double exploration = std::sqrt(2.0)) : iterations(iter), exploration_constant(exploration), gen(std::random_device{}()) {
        root = new TreeNode(root_node);
    }

    ~MCTree(){
        delete root;
    }

    /**
     * @brief 运行 MCTS 算法，返回优化后的 BVH 树根节点指针
     * @param transformed_geometries 机器人在不同配置下的变换后碰撞几何体
     * @param env 碰撞环境
     * @return 优化后的 BVH 树根节点指针
     */
    NodeType* runMCTS(const std::vector<std::vector<CollisionObject<S>>>& transformed_geometries, const CollisionEnv<S>& env);

    TreeNode* selection(TreeNode* node);
    TreeNode* expansion(TreeNode* node, const std::vector<CollisionObject<S>*>& available_objects, int max_children);
    double simulation(TreeNode* node, const std::vector<std::vector<CollisionObject<S>>>& transformed_geometries, const CollisionEnv<S>& env);
    void backpropagation(TreeNode* node, double result);

};


template class MCTree<AABB<double>>;

#endif