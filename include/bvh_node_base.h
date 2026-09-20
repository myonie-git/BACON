#ifndef BVH_NODE_BASE_H
#define BVH_NODE_BASE_H

#include "collision_object.h"

template<typename BV>
class NodeBase{
public:

    using S = double;

    BV bv;
    NodeBase<BV>* parent;

    // union
    // {
        /// @brief for leaf node, children nodes
    NodeBase<BV>* children[2];
    CollisionObject<S>* data; 
        // void* data; //记录其AABB形状
    // };

    bool isLeaf() const;
    bool isInternal() const;

    NodeBase();
    NodeBase(const BV& bv_);
};

#endif