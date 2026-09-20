#include "MCTree.h"
#include <cassert>
#include <algorithm>

template <typename BV>
typename MCTree<BV>::TreeNode* MCTree<BV>::selection(TreeNode* node) {
    while(!node->isLeaf()){
        node = node->selectChild();
    }
    return node;
}

// 扩展步骤
template <typename BV>
typename MCTree<BV>::TreeNode* MCTree<BV>::expansion(TreeNode* node, const std::vector<CollisionObject<S>*>& available_objects, int max_children) {
    // 假设我们有一些方式来决定如何分割碰撞几何体，这里简化为随机选择子节点
    // 实际应用中，您需要根据具体策略进行分割

    // 生成子节点数量
    std::uniform_int_distribution<> dis_num_children(2, max_children);
    int num_children = dis_num_children(gen);

    // 随机选择子节点的碰撞几何体
    // 这里简化为均匀分割，实际应用中应根据 AABB 分割策略
    std::vector<CollisionObject<S>*> subset = available_objects; // 简化处理

    // 创建子节点
    for(int i=0; i<num_children; ++i){
        // 随机选择一个对象作为叶节点
        std::uniform_int_distribution<> dis_obj(0, subset.size()-1);
        int idx = dis_obj(gen);
        CollisionObject<S>* obj = subset[idx];
        subset.erase(subset.begin() + idx);

        // 创建子 BVHNode
        NodeType* child_bvh = new NodeType(*obj->getAABB());
        child_bvh->data = obj;

        // 创建子 MCTreeNode
        TreeNode* child_node = new TreeNode(child_bvh, node);
        node->addChild(child_node);
    }

    node->updateBoundingBox();

    // 返回其中一个新添加的子节点
    if(!node->children.empty()){
        return node->children.back();
    }
    return node;
}

// 模拟步骤
template <typename BV>
double MCTree<BV>::simulation(TreeNode* node, const std::vector<std::vector<CollisionObject<S>>>& transformed_geometries, const CollisionEnv<S>& env) {
    // 简化模拟为统计碰撞检查次数
    double collision_checks = 0.0;

    for(const auto& config_geometries : transformed_geometries){
        collision_checks += simulateCollisionDetection(node->bvh_node, config_geometries, env);
    }

    return collision_checks;
}

// 回传步骤
template <typename BV>
void MCTree<BV>::backpropagation(TreeNode* node, double result){
    while(node != nullptr){
        node->visits += 1;
        node->value += result;
        node = node->parent;
    }
}

// 运行 MCTS
template <typename BV>
NodeType* MCTree<BV>::runMCTS(const std::vector<std::vector<CollisionObject<S>>>& transformed_geometries, const CollisionEnv<S>& env){
    for(int i=0; i<iterations; ++i){
        // 选择
        TreeNode* selected_node = selection(root);

        // 扩展
        // 收集可用的碰撞几何体
        std::vector<CollisionObject<S>*> available_objects;
        if(selected_node->isLeaf()){
            if(selected_node->bvh_node->data != nullptr){
                available_objects.push_back(selected_node->bvh_node->data);
            }
        }

        TreeNode* expanded_node = expansion(selected_node, available_objects, 4);

        // 模拟
        double simulation_result = simulation(expanded_node, transformed_geometries, env);

        // 回传
        backpropagation(expanded_node, simulation_result);
    }

    // 选择最佳子节点作为优化结果
    double best_value = -std::numeric_limits<double>::infinity();
    TreeNode* best_node = nullptr;

    for(auto child : root->children){
        double average_value = child->visits > 0 ? child->value / child->visits : 0.0;
        if(average_value > best_value){
            best_value = average_value;
            best_node = child;
        }
    }

    if(best_node != nullptr){
        return best_node->bvh_node;
    }

    return root->bvh_node;
}

template class MCTree<AABB<double>>;