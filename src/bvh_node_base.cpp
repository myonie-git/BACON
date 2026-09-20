#include "bvh_node_base.h"
#include "aabb.h"
#include "types.h"
#include "obb.h"

// 构造函数定义
template<typename BV>
NodeBase<BV>::NodeBase(const BV& bv_) : bv(bv_), parent(nullptr) {
    children[0] = nullptr;
    children[1] = nullptr;
    data = nullptr;
}

// 构造函数定义
template<typename BV>
NodeBase<BV>::NodeBase() : parent(nullptr) {
    children[0] = nullptr;
    children[1] = nullptr;
    data = nullptr;
}

template <typename BV>
bool NodeBase<BV>::isLeaf() const
{
  return (children[1] == nullptr);
}

template <typename BV>
bool NodeBase<BV>::isInternal() const
{
  return !isLeaf();
}

template class NodeBase<AABB<double>>;