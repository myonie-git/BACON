#include "collision_env.h"
#include "collision_object.h"
#include <vector>

template <typename S>
CollisionEnv<S>::CollisionEnv(){

}

template <typename S>
void CollisionEnv<S>::InitTree(const std::vector<CollisionObject<S>*>& other_objs){
    if(dtree.size() > 0){
        assert(0);
    }
    else
    {
        std::vector<AABBNode*> leaves(other_objs.size());
        table.rehash(other_objs.size());
        for(size_t i = 0, size = other_objs.size(); i < size; ++i)
        {
            AABBNode* node = new AABBNode; // node will be managed by the dtree
            node->bv = other_objs[i]->getAABB();
            node->parent = nullptr;
            node->children[1] = nullptr;
            node->data = other_objs[i];
            table[other_objs[i]] = node;
            leaves[i] = node;
        }
        dtree.init(leaves);
    }
}

template <typename S>
void CollisionEnv<S>::clear(){
    dtree.clear();
    table.clear();
}

template <typename S>
bool collisionRecurse_obbcount(typename CollisionEnv<S>::AABBNode* root,  CollisionObject<S>* query, void* cdata, int &aabb_count, int &obb_count){
    // timer += 2;  
    if(root->isLeaf()){
        if(!root->bv.overlap_obbcount(query->getAABB(), aabb_count)) return false;
        if(!root->data->obb.overlap_obbcount(query->getOBB(), obb_count)) return false;
        return true;
    }

    if(!root->bv.overlap_obbcount(query->getAABB(), aabb_count)) return false;
    
    if(collisionRecurse_obbcount(root->children[0], query, cdata, aabb_count, obb_count))
        return true; 

    if(collisionRecurse_obbcount(root->children[1], query, cdata, aabb_count, obb_count))
        return true;

    return false;
}


template <typename S>
bool collisionRecurse(typename CollisionEnv<S>::AABBNode* root,  CollisionObject<S>* query, void* cdata, int &timer){
    // timer += 2;  
    if(root->isLeaf()){
        if(!root->bv.overlap(query->getAABB(), timer)) return false;
        if(!root->data->obb.overlap(query->getOBB(), timer)) return false;
        return true;
    }

    if(!root->bv.overlap(query->getAABB(), timer)) return false;
    
    if(collisionRecurse(root->children[0], query, cdata, timer))
        return true; 

    if(collisionRecurse(root->children[1], query, cdata, timer))
        return true;

    return false;
}

template <typename S>
bool collisionRecurse_aabb(typename CollisionEnv<S>::AABBNode* root,  AABB<S>* query, void* cdata, int &timer){
    if(root->isLeaf()){
        if(!root->bv.overlap(*query, timer)) return false;
        return true;
    }
    if(!root->bv.overlap(*query, timer)) return false;
    if(collisionRecurse_aabb(root->children[0], query, cdata, timer))
        return true; 
    if(collisionRecurse_aabb(root->children[1], query, cdata, timer))
        return true;
    return false;
}

template <typename S>
bool collisionRecurse_aabb_obbcount(typename CollisionEnv<S>::AABBNode* root,  AABB<S>* query, void* cdata, int &aabb_count){
    if(root->isLeaf()){
        if(!root->bv.overlap_obbcount(*query, aabb_count)) return false;
        return true;
    }
    if(!root->bv.overlap_obbcount(*query, aabb_count)) return false;
    if(collisionRecurse_aabb_obbcount(root->children[0], query, cdata, aabb_count))
        return true; 
    if(collisionRecurse_aabb_obbcount(root->children[1], query, cdata, aabb_count))
        return true;
    return false;
}


template <typename S>
bool collisionRecurse_aabb(typename CollisionEnv<S>::AABBNode* root,  AABB<S>* query, void* cdata){
    if(root->isLeaf()){
        if(!root->bv.overlap(*query)) return false;
        return true;
    }
    if(!root->bv.overlap(*query)) return false;
    if(collisionRecurse_aabb(root->children[0], query, cdata))
        return true; 
    if(collisionRecurse_aabb(root->children[1], query, cdata))
        return true;
    return false;
}

template <typename S>
bool collisionRecurse(typename CollisionEnv<S>::AABBNode* root,  CollisionObject<S>* query, void* cdata){
    if(root->isLeaf()){
        if(!root->bv.overlap(query->getAABB())) return false;
        if(!root->data->obb.overlap(query->getOBB())) return false;
        return true;
    }

    if(!root->bv.overlap(query->getAABB())) return false;
    
    if(collisionRecurse(root->children[0], query, cdata))
        return true; 

    if(collisionRecurse(root->children[1], query, cdata))
        return true;

    return false;
}

template <typename S>
bool collisionRecurse_gpu(typename CollisionEnv<S>::AABBNode* root,  CollisionObject<S>* query, void* cdata){
    if(root->isLeaf()){
        if(!root->bv.overlap(query->getAABB())) return false;
        if(!root->data->obb.overlap_gpu(query->getOBB())) return false;
        return true;
    }

    if(!root->bv.overlap(query->getAABB())) return false;
    
    if(collisionRecurse_gpu(root->children[0], query, cdata))
        return true; 

    if(collisionRecurse_gpu(root->children[1], query, cdata))
        return true;

    return false;

}

template <typename S>
bool CollisionEnv<S>::collide_obbcount(CollisionObject<S>* obj, void* cdata, int &aabb_count, int &obb_count) const{
    if(dtree.size() == 0) return false;
    return collisionRecurse_obbcount(dtree.getRoot(), obj, cdata, aabb_count, obb_count);
}

template <typename S>
bool CollisionEnv<S>::collide(CollisionObject<S>* obj, void* cdata, int &timer) const{
    if(dtree.size() == 0) return false;
    return collisionRecurse(dtree.getRoot(), obj, cdata, timer);
}

template <typename S>
bool CollisionEnv<S>::collide_aabb_obbcount(AABB<S>* obj, void* cdata, int &aabb_count) const{
    if(dtree.size() == 0) return false;
    return collisionRecurse_aabb_obbcount(dtree.getRoot(), obj, cdata, aabb_count);
}


template <typename S>
bool CollisionEnv<S>::collide_aabb(AABB<S>* obj, void* cdata, int &timer) const{
    if(dtree.size() == 0) return false;
    return collisionRecurse_aabb(dtree.getRoot(), obj, cdata, timer);
}

template <typename S>
bool CollisionEnv<S>::collide_aabb(AABB<S>* obj, void* cdata) const{
    if(dtree.size() == 0) return false;
    return collisionRecurse_aabb(dtree.getRoot(), obj, cdata);
}

template <typename S>
bool CollisionEnv<S>::collide(CollisionObject<S>* obj, void* cdata) const{
    // std::cout << "happened" << std::endl;
    if(dtree.size() == 0) return false;
    return collisionRecurse(dtree.getRoot(), obj, cdata);
}

template <typename S>
bool CollisionEnv<S>::collide_gpu(CollisionObject<S>* obj, void* cdata) const{
    if(dtree.size() == 0) return false;
    return collisionRecurse_gpu(dtree.getRoot(), obj, cdata);
}

template <typename S>
size_t CollisionEnv<S>::size() const{
    return dtree.size();
}

template class CollisionEnv<double>;