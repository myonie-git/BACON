#include "robot_node_base.h"
#include "aabb.h"
#include "types.h"
#include "obb.h"

template<typename BV>
RobotNodeBase<BV>::RobotNodeBase() : parent(nullptr), data(nullptr) {
}

template<typename BV>
RobotNodeBase<BV>::RobotNodeBase(const BV& bv_) : bv(bv_), parent(nullptr), data(nullptr) {
}

template<typename BV>
bool RobotNodeBase<BV>::isLeaf() const {
    return children.empty();
}

template<typename BV>
bool RobotNodeBase<BV>::isInternal() const {
    return !isLeaf();
}

template<typename BV>
void RobotNodeBase<BV>::updateBoundingBox() {
    if (isLeaf()) {
        assert(data != nullptr && "Leaf node must have a collision object.");
        bv = data->getAABB();
    } else {
        assert(!children.empty() && "Internal node must have at least one child.");
        bv = children[0]->bv;
        for (size_t i = 1; i < children.size(); ++i) {
            bv = bv + children[i]->bv;
        }
    }
}

template class RobotNodeBase<AABB<double>>;