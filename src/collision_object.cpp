// src/collision_object.cpp.cpp

#include "collision_object.h"
#include "aabb.h"
#include <iostream>
#include "types.h"
#include "obb.h"

template <typename S>
void CollisionObject<S>::computeCollisionAABB(){
    // //计算变化后的中心点
    // Vector3<S> center = t * aabb_center;
    // // Vector3<S> center = t.translation();
    // // Vector3<S> center = aabb_center;
    // Vector3<S> delta = Vector3<S>::Constant(aabb_radius);

    // aabb.min_ = center - delta;
    // aabb.max_ = center + delta;
    // obb.To = t * obb.center();
    Eigen::Matrix<S, 3, 3> abs_axis = obb.axis.cwiseAbs();
    Eigen::Matrix<S, 3, 1> radius = abs_axis * obb.extent;
    aabb.min_ = obb.To - radius;
    aabb.max_ = obb.To + radius;
}

template <typename S>
void CollisionObject<S>::computeCollisionOBB(){
    //将其AABB转化为OBB
    // 根据 AABB 创建 OBB
    Vector3<S> min = aabb.min_;
    Vector3<S> max = aabb.max_;
    
    Vector3<S> center = (min + max) / (2.0);
    Vector3<S> extents = (max - min) / (2.0);

    obb.To = center;
    obb.axis = Eigen::Matrix<S, 3, 3>::Identity(); // 轴对齐
    obb.extent = extents;
}

template void CollisionObject<double>::computeCollisionAABB();
template void CollisionObject<float>::computeCollisionAABB();
template void CollisionObject<double>::computeCollisionOBB();
template void CollisionObject<float>::computeCollisionOBB();