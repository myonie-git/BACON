#ifndef BVH_TREE_H
#define BVH_TREE_H

#include <vector>
#include <map>
#include <iostream>


#include "collision_object.h"
#include "aabb.h"
#include "obb.h"
#include "types.h"
#include "bvh_node_base.h"

template <typename BV>
class BvhTree{
public:

    using S = typename BV::S;

    typedef NodeBase<AABB<double>> NodeType;
    BvhTree();
    ~BvhTree();
    void init(std::vector<NodeType*>& leaves); //初始化bvh树
    NodeType* getRoot() const;
    NodeType*& getRoot();
    NodeType* createNode(NodeType* parent, const BV& bv, CollisionObject<double>* data);
    void clear();
    size_t size() const;
    void print(NodeType* root, int depth);
    void print(){
        print(root_node, 0);
    }

    int bu_threshold;
    size_t n_leaves;
    NodeType* root_node;

    void recurseDeleteNode(NodeType* Node);
    void deleteNode(NodeType* node);

    typedef typename std::vector<NodeType*>::iterator NodeVecIterator;
    NodeType* topdown(const NodeVecIterator lbeg, const NodeVecIterator lend);
    void bottomup(const NodeVecIterator lbeg, const NodeVecIterator lend);
    
};


#endif