#include "mtcs_node.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <random>

template <typename BV>
bool MCTreeNode<BV>::isLeaf() const {
    return robot_node_->isLeaf();
}

template <typename BV>
bool MCTreeNode<BV>::isInternal() const {
    return bvh_node->isInternal();
}

MCTreeNode<BV>* MCTreeNode<BV>::selectChild() {
    assert(!children.empty() && "No children to select from.");

    double best_score = -std::numeric_limits<double>::infinity();
    MCTreeNode<BV>* best_child = nullptr;

    for(auto child : children){
        if(child->visits == 0){
            return child; // 立即选择未访问的子节点
        }
        double exploitation = child->value / child->visits;
        double exploration = std::sqrt(2.0 * std::log(visits) / child->visits);
        double score = exploitation + exploration;

        if(score > best_score){
            best_score = score;
            best_child = child;
        }
    }

    return best_child;
}

template <typename BV>
void MCTreeNode<BV>::addChild(MCTreeNode<BV>* child) {
    children.push_back(child);
}