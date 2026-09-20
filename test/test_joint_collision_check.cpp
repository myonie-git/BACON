#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>

int main(){

    //read the env from boxes.txt or build a random env
    using S = double;
    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);
    // std::vector<CollisionObject<S>*> collision_objects = randomCollisionObjects<S>(32);

    //buid and print the env
    CollisionEnv<S> env;
    env.InitTree(collision_objects);
    // env.dtree.print();
    
    //read the robot model
    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/go1.urdf"; 
    // std::string urdfFilePath = BACON_SOURCE_DIR "/panda_description.urdf"; 
    // std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; 
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    //set joint value 
    std::map<std::string, double> jointAngles = {
        {"panda_joint1", 0.0},
        {"panda_joint2", 0.0},
        {"panda_joint3", 0.0},
        {"panda_joint4", 0.0},
        {"panda_joint5", 0.0},
        {"panda_joint6", 0.0},
        {"panda_joint7", 0.0}
    };

    //set joint value 
    //joint的名称要跟prbt_description.urdf中设置的joint名称一致
    // std::map<std::string, double> jointAngles = {
    //     {"prbt_joint_1", 0.0},
    //     {"prbt_joint_2", 0.0},
    //     {"prbt_joint_3", 0.0},
    //     {"prbt_joint_4", 0.0},
    //     {"prbt_joint_5", 0.0},
    //     {"prbt_joint_6", 0.0}
    // };
    // jointAngles["prbt_joint_1"] = -0.57;
    // jointAngles["prbt_joint_2"] = 1.43;
    // jointAngles["prbt_joint_3"] = -0.57;
    // jointAngles["prbt_joint_4"] = -0.57;
    // jointAngles["prbt_joint_5"] = -0.57;
    // jointAngles["prbt_joint_6"] = 1.43;
    
    // std::string rootLink = "prbt_base_link";
    // std::string rootLink = "panda_link0";
    std::string rootLink = "base";
    model.setJointAngles(jointAngles);
    model.calculateWorldCoordinates(rootLink);
    
    std::cout << std::endl << std::endl << std::endl << std::endl ;
    model.printModel();
    std::cout << std::endl;
    model.printCollisionCoordinates();

    std::vector<CollisionObject<S>> collision_geometry_;
    // for(int i = 0; i < model.collisionGeometries.size(); i++){
    // for (const auto& collisionGeomPtr : model.collisionGeometries) {
    //     CollisionObject<S> collisionObject(*collisionGeomPtr);
    //     // CollisionObject<S> collisionObject(*model.collisionGeometries[i]);
    //     collision_geometry_.push_back(std::move(collisionObject));
    // }

    for (int i = 0; i < model.collisionGeometries.size(); i++) {
        const auto& collisionGeomPtr = model.collisionGeometries[i];
        CollisionObject<S> collisionObject(*collisionGeomPtr);
        collision_geometry_.push_back(std::move(collisionObject));
        // if(i == 0){
        // std::cout << "AABB Information:" << i << std::endl;
        // (*collisionGeomPtr).aabb.printAABB();
        std::cout << "AABB Information:" << i << std::endl;
        collisionObject.aabb.printAABB();
        // std::cout << "t:" << std::endl << collisionObject.t.matrix() << std::endl;
    
        // std::cout << "OBB Information:" << i << std::endl;
        // (*collisionGeomPtr).obb.printOBB();

        // std::cout << "OBB Information:" << i << std::endl;
        // collisionObject.obb.printOBB();
}


    // bool result = false;
    // // for(int i = 0; i < collision_geometry_.size() && !result; i++){
    //     int i = 9;
    //     result = env.collide(&collision_geometry_[i], nullptr);

    //     std::cout << "AABB Information:" << i << std::endl;
    //     collision_geometry_[i].aabb.printAABB();
    //         // std::cout << "t:" << std::endl << collisionObject.t.matrix() << std::endl;
    //     // }
        
    //     std::cout << "OBB Information:" << i << std::endl;
    //     collision_geometry_[i].obb.printOBB();
    //     collision_geometry_[i].obb.computeVertices();

    //     if(result == true){
    //         std::cout << "Collision AT " << i << std::endl; 
    //     }
    // // }
    // std::cout <<  "Collision Detection: " << (result ? "Env Collision" : "No Collision") << std::endl;

    // -1.57, -1.57, -1.57, -0.37, -1.57, 0.03
    // -0.57, 1.43, -0.57, -0.57, -0.57, 0.43
    // 定义角度范围和步长
//   double start_angle = -1.57;
//   double end_angle = 1.57;
//   double step_size = 1.0;

// // //     // 嵌套循环，扫描多个自由度的角度，以验证正确性
//     for (double joint1 = start_angle; joint1 <= end_angle; joint1 += step_size)
//     {
//         for (double joint2 = start_angle; joint2 <= end_angle; joint2 += step_size)
//         {
//             for (double joint3 = start_angle; joint3 <= end_angle; joint3 += step_size)
//             {
//                 for (double joint4 = start_angle; joint4 <= end_angle; joint4 += step_size)
//                 {
//                     for (double joint5 = start_angle; joint5 <= end_angle; joint5 += step_size)
//                     {
//                         for (double joint6 = start_angle; joint6 <= end_angle; joint6 += step_size)
//                         {
//                             // panda_description的测试信息
//                             jointAngles["panda_joint1"] = joint1;
//                             jointAngles["panda_joint2"] = joint2;
//                             jointAngles["panda_joint3"] = joint3;
//                             jointAngles["panda_joint4"] = joint4;
//                             jointAngles["panda_joint5"] = joint5;
//                             jointAngles["panda_joint6"] = joint6;
                            
//                             //prbt_description的测试信息
//                             // jointAngles["prbt_joint_1"] = joint1;
//                             // jointAngles["prbt_joint_2"] = joint2;
//                             // jointAngles["prbt_joint_3"] = joint3;
//                             // jointAngles["prbt_joint_4"] = joint4;
//                             // jointAngles["prbt_joint_5"] = joint5;
//                             // jointAngles["prbt_joint_6"] = joint6;

//                             model.setJointAngles(jointAngles);
//                             model.calculateWorldCoordinates(rootLink);

//                             std::vector<CollisionObject<S>> collision_geometry_;
//                             // 循环将当前角度下的每个机械臂的collision生成为CollisionObject,用于后续的碰撞检测
//                             for (int i = 0; i < model.collisionGeometries.size(); i++) {
//                                 const auto& collisionGeomPtr = model.collisionGeometries[i];
//                                 CollisionObject<S> collisionObject(*collisionGeomPtr);
//                                 collision_geometry_.push_back(std::move(collisionObject));
//                                 // if(i == 0){
                                
//                                     // std::cout << "AABB Information:" << i << std::endl;
//                                     // collisionObject.aabb.printAABB();
//                                     // // std::cout << "t:" << std::endl << collisionObject.t.matrix() << std::endl;
//                                     // // }
                                
//                                     // std::cout << "OBB Information:" << i << std::endl;
//                                     // collisionObject.obb.printOBB();
//                             }

//                             //将每个CollisionObject跟环境进行碰撞检测，后续可在此设计调度策略
//                             bool result = false;
//                             for(int i = 0; i < collision_geometry_.size() && !result; i++){
//                                 result = env.collide(&collision_geometry_[i], nullptr);

//                                 // std::cout << "AABB Information:" << i << std::endl;
//                                 // collision_geometry_[i].aabb.printAABB();
//                                  // std::cout << "t:" << std::endl << collisionObject.t.matrix() << std::endl;
//                                 // }
                                
//                                 // std::cout << "OBB Information:" << i << std::endl;
//                                 // collision_geometry_[i].obb.printOBB();

//                                 if(result == true){
//                                     std::cout << "Collision AT " << i << std::endl; 
//                                 }
//                             }
//                             std::cout << "Joint Angles: " << joint1 << ", " << joint2 << ", " << joint3 << ", " << joint4 << ", " << joint5 << ", " << joint6 << " - Collision Detection: " << (result ? "Env Collision" : "No Collision") << std::endl;
//                         }
//                     }
//                 }
//             }
//         }
//     }
//     return 0;
}