#ifndef MTCS_TREE_H
#define MTCS_TREE_H

#include <vector>
#include <map>
#include <iostream>


#include "collision_object.h"
#include "aabb.h"
#include "obb.h"
#include "types.h"
#include "robot_node_base.h"

template <typename BV>
class MCTreeNode{
public:
    using S = typename BV::S;

    typedef RobotNodeBase<AABB<double>> RobotNodeType;

    RobotNodeType* robot_node_;
    std::vector<MCTree
    MCTreeNode<S>* parent_;
    int visits_;
    double value_;
    
    MCTreeNode() : robot_node_(nullptr), parent(nullptr), visits(0), value(0.0)
    MCTreeNode(RobotNodeType* robot_node, MCTreeNode<BV>* parent = nullptr) : robot_node_(robot_node), parent_(parent), visits_(0), value_(0.0) {}
    ~MCTreeNode(){
        for(auto child : children_){
            delete child;
        }
        delete robot_node_;
    }

    RobotNodeType* getBVHNode() const;
    MCTreeNode<BV>* getParent() const;

    bool isLeaf() const;
    bool isInternal() const;
    MCTreeNode<BV>* selectChild();
    void addChild(MCTreeNode<BV>* child);

};

template class MCTreeNode<AABB<double>>;

#endif