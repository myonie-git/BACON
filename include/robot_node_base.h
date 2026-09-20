#ifndef ROBOT_NODE_BASE_H
#define ROBOT_NODE_BASE_H

#include "collision_object.h"

template<typename BV>
class RobotNodeBase{
public:
    using S = double;
    BV bv;
    RobotNodeBase<BV>* parent;

    std::vector<RobotNodeBase<BV>*> children; // 子节点指针列表
    CollisionObject<S>* data;

    RobotNodeBase();
    RobotNodeBase(const BV& bv_);

    bool isLeaf() const;
    bool isInternal() const;
    void updateBoundingBox();
};

#endif