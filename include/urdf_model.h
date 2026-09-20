#ifndef URDF_MODEL_H
#define URDF_MODEL_H

#include <iostream>
#include <tinyxml2.h>
#include <vector>
#include <string>
#include <map>
#include <memory>

#include <Eigen/Dense>

#include <fstream>
#include <sstream>
#include <string>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>

#include "aabb.h"
#include "obb.h"

using namespace tinyxml2;

struct Link; 

struct CollisionGeom {
    
    using S = double;

    enum ShapeType { BOX, SPHERE, CYLINDER, MESH };
    ShapeType type;
    Eigen::Vector3d collisionPositions; //用于表征真实向量
    Eigen::Quaterniond collisionRotations; //用于表征现实中的旋转,四元数
    std::shared_ptr<bodies::Body> body;
    AABB<S> aabb;
    OBB<S> obb;
    S aabb_radius;
    Link* parentLink;

    void printAABB_toram(std::ofstream& outfile) const {
        auto to_fixed_point = [](S value) -> int32_t {
            return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
        };

        // 格式化输出
        outfile << std::hex << std::setfill('0');
        
        // 打印中心点
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.center().x()) << " "
                << "0x" << std::setw(8) << to_fixed_point(aabb.center().y()) << " "
                << "0x" << std::setw(8) << to_fixed_point(aabb.center().z()) << " ";
        
        // 打印AABB半径
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb_radius) << std::endl;
    }

    void printOBB_toram(std::ofstream& outfile) const{
        auto to_fixed_point = [](S value) -> int32_t {
            return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
        };

        outfile << std::hex << std::setfill('0');
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                outfile << "0x" << std::setw(8) << to_fixed_point(obb.axis(i, j)) << " ";
            }
        }
        // 写入中心点
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.z()) << " ";
        // 写入尺寸
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.z()) << " ";
        
        outfile << std::endl;

    }

};

struct Link { //Link是没有相对坐标的，因为由JOINT所决定，所以只需要一个绝对坐标
    std::string name;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    std::vector<std::shared_ptr<CollisionGeom>> collisions;
};

struct Joint {
    int serial = -1;
    std::string name;
    std::string parent;
    std::string child;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    std::string type;
    Eigen::Vector3d axis;
};

class URDFModel{

public:

    std::map<std::string, Link> links;
    std::vector<Joint> joints;
    std::map<std::string, double> jointAngles;
    std::map<std::string, Link> worldLinks;
    std::vector<std::shared_ptr<CollisionGeom>> collisionGeometries;

    void parseURDF(const std::string& filePath);
    void computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot);
    void computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot, std::ofstream& outfile);
    void computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot);
    void computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot, int &timer);
    void computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot, std::ofstream& outfile);
    void computeWorldCoordinates_toram(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot, std::ofstream& outfile);


    bool loadURDF(const std::string& filePath);
    void setJointAngles(const std::map<std::string, double>& joint_angles);
    void calculateWorldCoordinates(const std::string& rootLink, int &timer);
    void calculateWorldCoordinates(const std::string& rootLink);
    void calculateWorldCoordinates(std::ofstream& outfile, const std::string& rootLink);
    void calculateWorldCoordinates_toram(std::ofstream& outfile, const std::string& rootLink);
    void printModel() const;
    void printCollisionCoordinates() const;

    void computeAABB();
    void computeOBB();

    void configureCollisionOBB(std::shared_ptr<CollisionGeom>& collision);
    void configureCollisionAABB(std::shared_ptr<CollisionGeom>& collision);

    void configureCollisionAABBRadius(std::shared_ptr<CollisionGeom>& collision);
    
    int getCollisionNum();

    void assignJointSerialNumbers(const std::string& rootLinkName);
};

#endif