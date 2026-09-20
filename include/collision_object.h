// include/collision_object.h
#ifndef COLLISION_OBJECT_H
#define COLLISION_OBJECT_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include <random>

#include "types.h"
#include "aabb.h"
#include "obb.h"

#include "urdf_model.h"

// CollisionObject 类定义
template <typename S>
class CollisionObject
{
public:    

    CollisionObject(const Vector3<S>& center, S radius, const Eigen::Transform<S, 3, Eigen::Isometry>& transform)
        : aabb_center(center), aabb_radius(radius), t(transform)
    {   
        computeCollisionAABB();
        computeCollisionOBB();
    }

    CollisionObject(const AABB<S>& aabb_) : aabb(aabb_)
    {
        computeCollisionOBB();
    }

    CollisionObject(const AABB<S>& aabb_, const OBB<S>& obb_) : aabb(aabb_), obb(obb_)
    {

    }

    CollisionObject(const CollisionGeom& geom)
        : CollisionObject(geom, *geom.parentLink) {}

    CollisionObject(const CollisionGeom& geom, const Link& pose)
        : aabb_center(geom.aabb.center()), aabb_radius(geom.aabb_radius),
          t(Eigen::Transform<S, 3, Eigen::Isometry>::Identity())
    {
        // 初始化 transform
        // t.translation() = geom.parentLink->position + geom.parentLink->rotation * geom.collisionPositions;
        // t.linear() = (geom.parentLink->rotation * geom.collisionRotations).toRotationMatrix();

        t.translation() = pose.position;
        t.linear() = pose.rotation.toRotationMatrix();
        
        obb = geom.obb;
        // obb.To = obb.center();
        obb.To = t * obb.center();
        // obb.To = t.translation();
        obb.axis = t.linear() * obb.axis;
        // obb.To = obb.center();
        // obb.axis = obb.axis;

        // 计算 AABB 和 OBB
        computeCollisionAABB();
        // computeCollisionOBB(); 已经有OBB了，不需要再计算了
    }

    AABB<S> getAABB(){
        return aabb;
    }

    OBB<S> getOBB(){
        return obb;
    }

    void computeCollisionAABB();

    void computeCollisionOBB();

    //the position of the center of aabb
    Vector3<S> aabb_center;

    //the radius of aabb 
    S aabb_radius;

    //the transform matrix of AABB (can be zero)
    Eigen::Transform<S, 3, Eigen::Isometry> t;

    //the value of collision obj
    AABB<S> aabb;

    OBB<S> obb;
};

template <typename S>
std::vector<CollisionObject<S>*> randomCollisionObjects(int num_obj) {
    std::vector<CollisionObject<S>*> collision_objects;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_size(0.0, 0.3);
    std::uniform_real_distribution<> dis_pos(-1.5, 1.5);

    for (int i = 0; i < num_obj; ) {
        Vector3<S> size(dis_size(gen), dis_size(gen), dis_size(gen));
        Vector3<S> position(dis_pos(gen), dis_pos(gen), dis_pos(gen));
        Vector3<S> min = position - size / 2;
        Vector3<S> max = position + size / 2;
        AABB<S> aabb(min, max);

        bool overlap = false;
        for (const auto& obj : collision_objects) {
            if (aabb.overlap(obj->getAABB())) {
                overlap = true;
                break;
            }
        }

        if (!overlap) {
            collision_objects.push_back(new CollisionObject<S>(aabb));
            i++; 
        }
    }

    return collision_objects;
}


// 读取盒子数据的函数
template <typename S>
std::vector<CollisionObject<S>*> readBoxesFromFile(const std::string& filename) {
    std::vector<Vector3<S>> sizes;
    std::vector<Vector3<S>> positions;

    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)) {
        
        if (line.find("Size:") != std::string::npos) {
            Vector3<S> size;
            std::stringstream ss(line.substr(line.find(":") + 1));
            char ignore;
            ss >> ignore >> size[0] >> ignore >> size[1] >> ignore >> size[2] >> ignore;
            sizes.push_back(size);
        } else if (line.find("Position:") != std::string::npos) {
            Vector3<S> position;
            std::stringstream ss(line.substr(line.find(":") + 1));
            char ignore;
            ss >> ignore >> position[0] >> ignore >> position[1] >> ignore >> position[2] >> ignore;
            positions.push_back(position);
        }
    }

    infile.close();

    std::vector<CollisionObject<S>*> collision_objects;
    for (size_t i = 0; i < sizes.size(); ++i) {
        Vector3<S> min = positions[i] - sizes[i] / 2;
        Vector3<S> max = positions[i] + sizes[i] / 2;
        AABB<S> aabb(min, max);
        collision_objects.push_back(new CollisionObject<S>(aabb));
    }

    // std::cout << sizes.size() << std::endl;

    return collision_objects;
}
#endif // collision_object
