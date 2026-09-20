#include "urdf_model.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <set>
#include <functional>
#include <filesystem>
#include <stdexcept>

using namespace tinyxml2;

bool URDFModel::loadURDF(const std::string& filePath) {
    parseURDF(filePath);
    return !links.empty() && !joints.empty();
}

void URDFModel::setJointAngles(const std::map<std::string, double>& joint_angles) {
    jointAngles = joint_angles;
}

void URDFModel::calculateWorldCoordinates(std::ofstream& outfile, const std::string& rootLink) {
    if (links.empty() || joints.empty()) {
        std::cerr << "Links or joints are not loaded properly." << std::endl;
        return;
    }

    // const std::string rootLink = "panda_link0"; // Replace with actual root link if different
    // const std::string rootLink = "prbt_base_link"; // Replace with actual root link if different
    // computeWorldCoordinates(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0), outfile);
    computeWorldCoordinates(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity(), outfile);
}

void URDFModel::calculateWorldCoordinates_toram(std::ofstream& outfile, const std::string& rootLink) {
    if (links.empty() || joints.empty()) {
        std::cerr << "Links or joints are not loaded properly." << std::endl;
        return;
    }
    // const std::string rootLink = "panda_link0"; // Replace with actual root link if different
    // const std::string rootLink = "prbt_base_link"; // Replace with actual root link if different
    // computeWorldCoordinates(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0), outfile);
    computeWorldCoordinates_toram(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity(), outfile);
}

void URDFModel::calculateWorldCoordinates(const std::string& rootLink, int &timer){
    if(links.empty() || joints.empty()){
        std::cerr << "Links or joints are not loaded properly." << std::endl;
        return;
    }
    computeWorldCoordinates(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity(), timer);
}

void URDFModel::calculateWorldCoordinates(const std::string& rootLink) {
    if (links.empty() || joints.empty()) {
        std::cerr << "Links or joints are not loaded properly." << std::endl;
        return;
    }
    // const std::string rootLink = "panda_link0"; // Replace with actual root link if different
    // const std::string rootLink = "prbt_base_link"; // Replace with actual root link if different
    // computeWorldCoordinates(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0));
    computeWorldCoordinates(rootLink, Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity());
    // Eigen::Matrix3d::Identity()
}

void URDFModel::printModel() const {
    for (const auto& [name, link] : worldLinks) {
        std::cout << "Link: " << name << ", Position: (" << link.position.x() << ", " << link.position.y() << ", " << link.position.z() << ")" << std::endl;
    }
}

void URDFModel::printCollisionCoordinates() const {
    for (const auto& [name, link] : worldLinks) {
        for (const auto& collision : link.collisions) {
            Eigen::Vector3d worldCollisionPos = link.position + link.rotation * collision->collisionPositions;
            Eigen::Quaterniond worldCollisionRot = link.rotation * collision->collisionRotations;

            std::cout << "Link: " << name << ", Collision Position: (" << worldCollisionPos.x() << ", " << worldCollisionPos.y() << ", " << worldCollisionPos.z() << ")" << std::endl;

             // 打印旋转矩阵
            Eigen::Matrix3d rotationMatrix = worldCollisionRot.toRotationMatrix();
            std::cout << "Collision Rotation Matrix: \n" << rotationMatrix << std::endl;

        }
    }
}

void URDFModel::parseURDF(const std::string& filePath) {
    XMLDocument doc;
    if (doc.LoadFile(filePath.c_str()) != XML_SUCCESS) {
        std::cerr << "Failed to load URDF file: " << filePath << std::endl;
        return;
    }

    XMLElement* robot = doc.FirstChildElement("robot");
    if (!robot) {
        std::cerr << "No robot element found in URDF file!" << std::endl;
        return;
    }

    for (XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
        const char* name = link->Attribute("name");
        if (name) {
            Link l;
            l.name = name;
            l.position = Eigen::Vector3d(0, 0, 0);
            l.rotation = Eigen::Quaterniond(1, 0, 0, 0);

            // 添加到links映射中
            links[name] = l;

            // 获取插入后的Link的引用
            Link& storedLink = links[name];

            for (XMLElement* collision = link->FirstChildElement("collision"); collision; collision = collision->NextSiblingElement("collision")) {
                XMLElement* origin = collision->FirstChildElement("origin");
                Eigen::Vector3d collPos = Eigen::Vector3d(0, 0, 0);
                Eigen::Quaterniond collRot = Eigen::Quaterniond(1, 0, 0, 0);

                if (origin) {
                    const char* xyz = origin->Attribute("xyz");
                    if (xyz) {
                        sscanf(xyz, "%lf %lf %lf", &collPos.x(), &collPos.y(), &collPos.z());
                    }

                    const char* rpy = origin->Attribute("rpy");
                    if (rpy) {
                        double roll, pitch, yaw;
                        sscanf(rpy, "%lf %lf %lf", &roll, &pitch, &yaw);
                        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                        collRot = yawAngle * pitchAngle * rollAngle;
                    } 
                }

                XMLElement* geometry = collision->FirstChildElement("geometry");
                if(geometry){
                    std::shared_ptr<bodies::Body> body;
                    auto collisionGeom = std::make_shared<CollisionGeom>();
                    collisionGeom->collisionPositions = collPos;
                    collisionGeom->collisionRotations = collRot;
                    collisionGeom->parentLink = &storedLink; // 设置指向存储在links中的Link的指针

                    XMLElement* box = geometry->FirstChildElement("box");
                    if(box){
                        const char* size = box->Attribute("size");
                        if(size){
                            Eigen::Vector3d boxSize;
                            sscanf(size, "%lf %lf %lf", &boxSize.x(), &boxSize.y(), &boxSize.z());
                            shapes::Box shape(boxSize.x(), boxSize.y(), boxSize.z());
                            body = std::make_shared<bodies::Box>(&shape);
                            body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                            collisionGeom->type = CollisionGeom::BOX;
                            collisionGeom->body = body;
                            collisionGeom->aabb_radius = boxSize.norm() / 2;
                            storedLink.collisions.push_back(collisionGeom);
                            collisionGeometries.push_back(collisionGeom);
                        }
                    }
                    
                    XMLElement* sphere = geometry->FirstChildElement("sphere");
                    if (sphere) {
                        const char* radius = sphere->Attribute("radius");
                        if (radius) {
                            double sphereRadius;
                            sscanf(radius, "%lf", &sphereRadius);
                            shapes::Sphere shape(sphereRadius);
                            body = std::make_shared<bodies::Sphere>(&shape);
                            body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                            collisionGeom->type = CollisionGeom::SPHERE;
                            collisionGeom->body = body;
                            collisionGeom->aabb_radius = sphereRadius; //MYTODO: 这里修改了radius，别忘记了
                            storedLink.collisions.push_back(collisionGeom);
                            collisionGeometries.push_back(collisionGeom);
                        }
                    }

                    XMLElement* cylinder = geometry->FirstChildElement("cylinder");
                    if (cylinder) {
                        const char* radius = cylinder->Attribute("radius");
                        const char* length = cylinder->Attribute("length");
                        if (radius && length) {
                            double cylinderRadius, cylinderLength;
                            sscanf(radius, "%lf", &cylinderRadius);
                            sscanf(length, "%lf", &cylinderLength);
                            shapes::Cylinder shape(cylinderRadius, cylinderLength);
                            body = std::make_shared<bodies::Cylinder>(&shape);
                            body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                            collisionGeom->type = CollisionGeom::CYLINDER;
                            collisionGeom->body = body;
                            storedLink.collisions.push_back(collisionGeom);
                            collisionGeometries.push_back(collisionGeom);
                        }
                    }

                    XMLElement* mesh = geometry->FirstChildElement("mesh");
                    if (mesh) {
                        // const char* filename = mesh->Attribute("filename");
                        std::string filename = mesh->Attribute("filename") ? mesh->Attribute("filename") : "";
                        const char* scale = mesh->Attribute("scale");
                        Eigen::Vector3d meshScale(1, 1, 1);
                        if (scale) {
                            sscanf(scale, "%lf %lf %lf", &meshScale.x(), &meshScale.y(), &meshScale.z());
                        }
                        if (!filename.empty()) {
                            std::string resource = filename;
                            if (resource.find("://") == std::string::npos) {
                                std::filesystem::path mesh_path(resource);
                                if (mesh_path.is_relative())
                                    mesh_path = std::filesystem::path(filePath).parent_path() / mesh_path;
                                resource = "file://" + std::filesystem::absolute(mesh_path).lexically_normal().string();
                            }
                            shapes::Mesh* shape = shapes::createMeshFromResource(resource, meshScale);
                            if (!shape)
                                throw std::runtime_error("Cannot load collision mesh: " + resource);
                            if (shape) {
                                body = std::make_shared<bodies::ConvexMesh>(shape);
                                body->setPose(Eigen::Isometry3d(Eigen::Translation3d(collPos) * collRot));
                                collisionGeom->type = CollisionGeom::MESH;
                                collisionGeom->body = body;

                                //计算aabb_radius
                                double max_radius = 0.0;
                                unsigned int vertex_count = shape->vertex_count;
                                for(unsigned int j = 0; j < vertex_count; j += 3){
                                    double x = shape->vertices[j];
                                    double y = shape->vertices[j + 1];
                                    double z = shape->vertices[j + 2];
                                    double distance = std::sqrt(x * x + y * y + z * z);
                                    if (distance > max_radius) {
                                        max_radius = distance;
                                    }
                                }
                                
                                collisionGeom->aabb_radius = max_radius;

                                storedLink.collisions.push_back(collisionGeom);
                                collisionGeometries.push_back(collisionGeom);
                            }
                        }
                    }    
                }
            }

            // 最后将完整的Link插入到links中
            links[name] = storedLink;
        }
    }

    for (XMLElement* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
        const char* name = joint->Attribute("name");
        const char* parent = joint->FirstChildElement("parent")->Attribute("link");
        const char* child = joint->FirstChildElement("child")->Attribute("link");
        const char* type = joint->Attribute("type");
        if (parent && child && name && type) {
            Joint j;
            j.name = name;
            j.parent = parent;
            j.child = child;
            j.type = type;

            XMLElement* origin = joint->FirstChildElement("origin");
            if (origin) {
                const char* xyz = origin->Attribute("xyz");
                if (xyz) {
                    sscanf(xyz, "%lf %lf %lf", &j.position.x(), &j.position.y(), &j.position.z());
                } else {
                    j.position = Eigen::Vector3d(0, 0, 0);
                }

                const char* rpy = origin->Attribute("rpy");
                if (rpy) {
                    double roll, pitch, yaw;
                    sscanf(rpy, "%lf %lf %lf", &roll, &pitch, &yaw);
                    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                    j.rotation = yawAngle * pitchAngle * rollAngle;
                } else {
                    j.rotation = Eigen::Quaterniond(1, 0, 0, 0);
                }
            } else {
                j.position = Eigen::Vector3d(0, 0, 0);
                j.rotation = Eigen::Quaterniond(1, 0, 0, 0);
            }

            XMLElement* axis = joint->FirstChildElement("axis");
            if (axis) {
                const char* xyz = axis->Attribute("xyz");
                if (xyz) {
                    sscanf(xyz, "%lf %lf %lf", &j.axis.x(), &j.axis.y(), &j.axis.z());
                } else {
                    j.axis = Eigen::Vector3d(0, 0, 1);
                }
            } else {
                j.axis = Eigen::Vector3d(0, 0, 1);
            }

            joints.push_back(j);
        }
    }

    computeOBB();
    computeAABB();

}

void URDFModel::computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot, std::ofstream& outfile) {
    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link& link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot;

    worldLinks[linkName] = link;

    // Helper function to convert to fixed-point representation
    auto to_fixed_point = [](double value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    Eigen::Matrix4d parentTF = Eigen::Matrix4d::Identity();
    parentTF.block<3, 3>(0, 0) = parentRot.toRotationMatrix();
    parentTF.block<3, 1>(0, 3) = parentPos;

    outfile << std::hex << std::setfill('0');

    // 遍历每个子关节
    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            Eigen::Quaterniond jointRot = joint.rotation;
            double angle = 0;
            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { 
                    angle = angleIt->second;
                }
            }
            
            //打印parentTF
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    outfile << "0x" << std::setw(8) << to_fixed_point(parentTF(row, col)) << " ";
                }
            }

            Eigen::AngleAxisd jointAngleRotation(angle, joint.axis);
            jointRot = joint.rotation * jointAngleRotation;

            Eigen::Matrix4d jointTF = Eigen::Matrix4d::Identity();
            jointTF.block<3, 3>(0, 0) = joint.rotation.toRotationMatrix();
            jointTF.block<3, 1>(0, 3) = joint.position;
            
            //打印jointTF
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    outfile << "0x" << std::setw(8) << to_fixed_point(jointTF(row, col)) << " ";
                }
            }

            //打印angle
            outfile << "0x" << std::setw(8) << to_fixed_point(angle) << " ";

            //打印joint.axis()
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.x()) << " ";
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.y()) << " ";
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.z()) << " ";

            Eigen::Vector3d childPos = parentPos + parentRot * joint.position;
            Eigen::Quaterniond childRot = parentRot * jointRot;

            Eigen::Matrix4d childTF = Eigen::Matrix4d::Identity();
            childTF.block<3, 3>(0, 0) = childRot.toRotationMatrix();
            childTF.block<3, 1>(0, 3) = childPos;

            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    outfile << "0x" << std::setw(8) << to_fixed_point(childTF(row, col)) << " ";
                }
            }

            outfile << std::endl;  

            computeWorldCoordinates(joint.child, childPos, childRot, outfile);
        }
    }
}

void URDFModel::computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Quaterniond& parentRot) {
    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link& link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot;

    worldLinks[linkName] = link;

    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            Eigen::Quaterniond jointRot = joint.rotation;
            double angle = 0;
            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { //说明找到了对应的角度
                    angle = angleIt->second;
                }
            }
            
            Eigen::AngleAxisd jointAngleRotation(angle, joint.axis);
            jointRot = joint.rotation * jointAngleRotation;


            Eigen::Vector3d childPos = parentPos + parentRot * joint.position;
            Eigen::Quaterniond childRot = parentRot * jointRot;    

            computeWorldCoordinates(joint.child, childPos, childRot);
        }
    }
}

void URDFModel::computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot, int &timer) {

    Eigen::Quaterniond parentRot_tmp(parentRot);

    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link& link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot_tmp;

    worldLinks[linkName] = link;

    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            double angle = 0;
            double transangle = 0;
            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { //说明找到了对应的角度
                    angle = angleIt->second;
                }
                timer += 5;
            }   
            else if(joint.type == "prismatic"){
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { 
                    transangle = angleIt->second;
                }
                timer += 5;
            }

            Eigen::AngleAxisd jointAngleRotation1(angle, joint.axis);
            Eigen::Matrix3d jointRotMat = joint.rotation.toRotationMatrix() * jointAngleRotation1.toRotationMatrix();
            Eigen::Vector3d jointTrans = joint.position + transangle * joint.axis;

            Eigen::Vector3d childPos = parentPos + parentRot * jointTrans;
            Eigen::Matrix3d childRot = parentRot * jointRotMat;

            Eigen::Vector3d childPos_temp = parentRot * joint.position;
            Eigen::Matrix3d childRot_temp = parentRot * joint.rotation.toRotationMatrix();

            //parentRot * jointRot * R
            computeWorldCoordinates(joint.child, childPos, childRot, timer);
        }
    }

    timer ++;

}

void URDFModel::computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot) {

    Eigen::Quaterniond parentRot_tmp(parentRot);

    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link& link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot_tmp;

    worldLinks[linkName] = link;

    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            double angle = 0;
            double transangle = 0;
            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { //说明找到了对应的角度
                    angle = angleIt->second;
                }
            }   
            else if(joint.type == "prismatic"){
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { 
                    transangle = angleIt->second;
                }
            }

            Eigen::AngleAxisd jointAngleRotation1(angle, joint.axis);
            Eigen::Matrix3d jointRotMat = joint.rotation.toRotationMatrix() * jointAngleRotation1.toRotationMatrix();
            Eigen::Vector3d jointTrans = joint.position + transangle * joint.axis;

            Eigen::Vector3d childPos = parentPos + parentRot * jointTrans;
            Eigen::Matrix3d childRot = parentRot * jointRotMat;

            Eigen::Vector3d childPos_temp = parentRot * joint.position;
            Eigen::Matrix3d childRot_temp = parentRot * joint.rotation.toRotationMatrix();

            //parentRot * jointRot * R
            computeWorldCoordinates(joint.child, childPos, childRot);

            // Eigen::AngleAxisd jointAngleRotation(angle, joint.axis);
            // Eigen::Quaterniond jointRot = joint.rotation * jointAngleRotation;

            // Eigen::Vector3d childPos = parentPos + parentRot_tmp * joint.position;
            // Eigen::Quaterniond childRot = parentRot_tmp * jointRot;    

            // computeWorldCoordinates(joint.child, childPos, childRot.toRotationMatrix());
        }
    }
    // auto linkIt = links.find(linkName);
    // if (linkIt == links.end()) return;

    // Link link = linkIt->second;
    // link.position = parentPos;
    // link.rotation = Eigen::Quaterniond(parentRot);


    // worldLinks[linkName] = link;

    // for (const auto& joint : joints) {
    //     if (joint.parent == linkName) {
    //         // 将 joint.rotation 从 Quaterniond 转换为 Matrix3d
    //         Eigen::Matrix3d jointRotMatrix = joint.rotation.toRotationMatrix();
    //         double angle = 0;
    //         if (joint.type == "revolute" || joint.type == "continuous") {
    //             auto angleIt = jointAngles.find(joint.name);
    //             if (angleIt != jointAngles.end()) {
    //                 angle = angleIt->second;
    //             }
    //         }

    //         Eigen::Matrix3d jointAngleRotationMatrix;
    //         jointAngleRotationMatrix = Eigen::AngleAxisd(angle, joint.axis).toRotationMatrix();
    //         jointRotMatrix = jointRotMatrix * jointAngleRotationMatrix;

    //         Eigen::Vector3d childPos = parentPos + parentRot * joint.position;
    //         Eigen::Matrix3d childRot = joint.rotation.toRotationMatrix() * jointRotMatrix;
            
    //         computeWorldCoordinates(joint.child, childPos, childRot);
    //     }
    // }
}

void URDFModel::computeWorldCoordinates(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot, std::ofstream& outfile) {

    Eigen::Quaterniond parentRot_tmp(parentRot);

    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link& link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot_tmp;

    worldLinks[linkName] = link;

    // Helper function to convert to fixed-point representation
    auto to_fixed_point = [](double value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    outfile << std::hex << std::setfill('0');

    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            double angle = 0;
            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { 
                    angle = angleIt->second;
                }
            }
            // 打印 parentRot 矩阵
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    outfile << "0x" << std::setw(8) << to_fixed_point(parentRot(i, j)) << " ";
                }
            }

            // 打印 parentPos 向量
            for (int i = 0; i < 3; ++i) {
                outfile << "0x" << std::setw(8) << to_fixed_point(parentPos(i)) << " ";
            }

            for(int row = 0; row < 3; ++row){
                for(int col = 0; col < 3; ++col){
                    outfile << "0x" << std::setw(8) << to_fixed_point(joint.rotation.toRotationMatrix()(row, col)) << " ";
                }
            }

            for(int i = 0; i < 3; ++i){
                outfile << "0x" << std::setw(8) << to_fixed_point(joint.position(i)) << " ";
            }

            // 计算 joint 的旋转矩阵
            Eigen::AngleAxisd jointAngleRotation(angle, joint.axis);
            Eigen::Matrix3d jointRotMat = joint.rotation.toRotationMatrix() * jointAngleRotation;

            // 保存并打印 parentRot * joint.position 的中间结果
            // Eigen::Vector3d rotatedJointPos = parentRot * joint.position;
            // for (int i = 0; i < 3; ++i) {
            //     outfile << "0x" << std::setw(8) << to_fixed_point(rotatedJointPos(i)) << " ";
            // }

            // 打印 jointRot 矩阵
            // for (int i = 0; i < 3; ++i) {
            //     for (int j = 0; j < 3; ++j) {
            //         outfile << "0x" << std::setw(8) << to_fixed_point(jointRotMat(i, j)) << " ";
            //     }
            // }

            outfile << "0x" << std::setw(8) << to_fixed_point(angle) << " ";

            //打印joint.axis()
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.x()) << " ";
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.y()) << " ";
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.z()) << " ";

            // 计算 childPos 和 childRot
            Eigen::Vector3d childPos = parentPos + parentRot * joint.position;
            Eigen::Matrix3d childRot = parentRot * jointRotMat;
            
            Eigen::Matrix3d childRot_temp = parentRot * jointRotMat;
            
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    outfile << "0x" << std::setw(8) << to_fixed_point(childRot(i, j)) << " ";
                }
            }

            for (int i = 0; i < 3; ++i) {
                outfile << "0x" << std::setw(8) << to_fixed_point(childPos(i)) << " ";
            }

            outfile << std::endl;

            // 递归计算子关节
            computeWorldCoordinates(joint.child, childPos, childRot, outfile);
        }
    }
}


void URDFModel::computeWorldCoordinates_toram(const std::string& linkName, const Eigen::Vector3d& parentPos, const Eigen::Matrix3d& parentRot, std::ofstream& outfile) {

    Eigen::Quaterniond parentRot_tmp(parentRot);

    auto linkIt = links.find(linkName);
    if (linkIt == links.end()) return;

    Link& link = linkIt->second;
    link.position = parentPos;
    link.rotation = parentRot_tmp;

    worldLinks[linkName] = link;

    // Helper function to convert to fixed-point representation
    auto to_fixed_point = [](double value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    outfile << std::hex << std::setfill('0');

    for (const auto& joint : joints) {
        if (joint.parent == linkName) {
            double angle = 0;
            double transangle = 0;
            if (joint.type == "revolute" || joint.type == "continuous") {
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { 
                    angle = angleIt->second;
                }
            }
            else if(joint.type == "prismatic"){
                auto angleIt = jointAngles.find(joint.name);
                if (angleIt != jointAngles.end()) { 
                    transangle = angleIt->second;
                }
            }
            // 打印 parentRot 矩阵
            // for (int i = 0; i < 3; ++i) {
            //     for (int j = 0; j < 3; ++j) {
            //         outfile << "0x" << std::setw(8) << to_fixed_point(parentRot(i, j)) << " ";
            //     }
            // }

            // 打印 parentPos 向量
            // for (int i = 0; i < 3; ++i) {
            //     outfile << "0x" << std::setw(8) << to_fixed_point(parentPos(i)) << " ";
            // }

            for(int row = 0; row < 3; ++row){
                for(int col = 0; col < 3; ++col){
                    outfile << "0x" << std::setw(8) << to_fixed_point(joint.rotation.toRotationMatrix()(row, col)) << " ";
                }
            }

            for(int i = 0; i < 3; ++i){
                outfile << "0x" << std::setw(8) << to_fixed_point(joint.position(i)) << " ";
            }

            // 计算 joint 的旋转矩阵
            Eigen::AngleAxisd jointAngleRotation(angle, joint.axis);
            Eigen::Matrix3d jointRotMat = joint.rotation.toRotationMatrix() * jointAngleRotation;

            // 保存并打印 parentRot * joint.position 的中间结果
            // Eigen::Vector3d rotatedJointPos = parentRot * joint.position;
            // for (int i = 0; i < 3; ++i) {
            //     outfile << "0x" << std::setw(8) << to_fixed_point(rotatedJointPos(i)) << " ";
            // }

            // 打印 jointRot 矩阵
            // for (int i = 0; i < 3; ++i) {
            //     for (int j = 0; j < 3; ++j) {
            //         outfile << "0x" << std::setw(8) << to_fixed_point(jointRotMat(i, j)) << " ";
            //     }
            // }

            // outfile << "0x" << std::setw(8) << to_fixed_point(angle) << " ";

            //打印joint.axis()
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.x()) << " ";
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.y()) << " ";
            outfile << "0x" << std::setw(8) << to_fixed_point(joint.axis.z()) << " ";

            // 计算 childPos 和 childRot
            Eigen::Vector3d childPos = parentPos + parentRot * joint.position;
            Eigen::Matrix3d childRot = parentRot * jointRotMat;
            
            Eigen::Matrix3d childRot_temp = parentRot * jointRotMat;
            
            // for (int i = 0; i < 3; ++i) {
            //     for (int j = 0; j < 3; ++j) {
            //         outfile << "0x" << std::setw(8) << to_fixed_point(childRot(i, j)) << " ";
            //     }
            // }

            // for (int i = 0; i < 3; ++i) {
            //     outfile << "0x" << std::setw(8) << to_fixed_point(childPos(i)) << " ";
            // }

            outfile << std::endl;

            // 递归计算子关节
            computeWorldCoordinates_toram(joint.child, childPos, childRot, outfile);
        }
    }
}

void URDFModel::computeAABB(){
    for(auto& collisionGeom : collisionGeometries){
        if(collisionGeom->type == CollisionGeom::SPHERE || collisionGeom->type == CollisionGeom::BOX || collisionGeom->type == CollisionGeom::MESH){  
            configureCollisionAABB(collisionGeom);
        }else{
            configureCollisionAABB(collisionGeom);
            configureCollisionAABBRadius(collisionGeom);
        }
    }
}

void URDFModel::computeOBB(){
    for(auto& collisionGeom : collisionGeometries){
        configureCollisionOBB(collisionGeom);
    }
}

void URDFModel::configureCollisionOBB(std::shared_ptr<CollisionGeom>& collision){
    if(!collision->body){
        std::cerr << "Error: Collision body is not set." << std::endl;
        return;      
    }

    bodies::OBB obb;
    collision->body->computeBoundingBox(obb);

    collision->obb.extent = obb.getExtents() / 2.0;
    //注意：这里定义的两个OBB的格式不一样，一个是半长度，一个是长度，需要设置转换代码 ！！！！ 

    Eigen::Isometry3d obbPose = obb.getPose();
    collision->obb.axis = obbPose.rotation();
    collision->obb.To = obbPose.translation();

    return;
}

void URDFModel::configureCollisionAABB(std::shared_ptr<CollisionGeom>& collision){
    if(!collision->body){
        std::cerr << "Error: Collision body is not set." << std::endl;
        return;      
    }

    bodies::AABB aabb;
    collision->body->computeBoundingBox(aabb);

    collision->aabb.min_ = aabb.min();
    collision->aabb.max_ = aabb.max();
    return; 
}

void URDFModel::configureCollisionAABBRadius(std::shared_ptr<CollisionGeom>& collision){
    if(!collision->body){
        std::cerr << "Error: Collision body is not set." << std::endl;
        return;      
    }
    collision->aabb_radius = (collision->obb.extent).norm();
    // collision->aabb_radius = (collision->aabb.max_ - collision->aabb.min_).norm() / 2.0;
}

int URDFModel::getCollisionNum(){
    return collisionGeometries.size();
}


void URDFModel::assignJointSerialNumbers(const std::string& rootLinkName) {
    // 创建从父链接到其子关节的映射
    std::map<std::string, std::vector<Joint*>> parentToJoints;
    for (auto& joint : joints) {
        parentToJoints[joint.parent].push_back(&joint);
    }

    int serialCounter = 0;
    std::set<std::string> visitedLinks;

    std::function<void(const std::string&)> dfs = [&](const std::string& linkName) {
        visitedLinks.insert(linkName);
        auto it = parentToJoints.find(linkName);
        if (it != parentToJoints.end()) {
            for (auto jointPtr : it->second) {
                jointPtr->serial = serialCounter++;
                if (visitedLinks.find(jointPtr->child) == visitedLinks.end()) {
                    dfs(jointPtr->child);
                }
            }
        }
    };

    dfs(rootLinkName);
}