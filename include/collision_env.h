#ifndef COLLISION_ENV_H
#define COLLISION_ENV_H

#include "aabb.h"
#include "obb.h"
#include "types.h"
#include "bvh_node_base.h"
#include "collision_object.h"
#include "bvh_tree.h"

#include <vector>
#include <unordered_map>

//定义环境状态
template <typename S>
class CollisionEnv{

public:
    using AABBNode = NodeBase<AABB<S>>;

    CollisionEnv();

    // void InitTree(std::vector<NodeType*>& leaves); //输入一系列的碰撞物,以建树
    void InitTree(const std::vector<CollisionObject<S>*>& other_objs);
    void clear();
    bool collide(CollisionObject<S>* obj, void* cdata) const;
    bool collide(CollisionObject<S>* obj, void* cdata, int &timer) const;
    bool collide_obbcount(CollisionObject<S>* obj, void* cdata, int &aabb_count, int &obb_count) const;
    bool collide_gpu(CollisionObject<S>* obj, void* cdata) const;
    bool collide_aabb(AABB<S>* obj, void* cdata) const;
    bool collide_aabb(AABB<S>* obj, void* cdata, int &timer) const;
    bool collide_aabb_obbcount(AABB<S>* obj, void* cdata, int &aabb_count) const;
    
    size_t size() const;

    BvhTree<AABB<S>>dtree;
    std::unordered_map<CollisionObject<S>*, AABBNode*> table;
};

#endif