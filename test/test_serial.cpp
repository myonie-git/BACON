#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <collision_object.h>
#include <collision_env.h>
#include "aabb.h"
#include "obb.h"
#include "types.h"

using S = double;

int main(){
    
    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/panda_description.urdf"; // Replace with your URDF file path
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }
    model.assignJointSerialNumbers("panda_link0");
    // model.assignJointSerialNumbers("prbt_base_link"); 
    for (const auto& joint : model.joints) {
        std::cout << "Joint Name: " << joint.name << ", Serial: " << joint.serial << std::endl;
    }

    return 0;

}