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

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> random_joint(-3.14, 3.14);

    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);
    
    CollisionEnv<S> env;
    env.InitTree(collision_objects);

    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; // Replace with your URDF file path
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    std::map<std::string, double> jointAngles = {
        {"prbt_joint_1", 0.0},
        {"prbt_joint_2", 0.0},
        {"prbt_joint_3", 0.0},
        {"prbt_joint_4", 0.0},
        {"prbt_joint_5", 0.0},
        {"prbt_joint_6", 0.0}
    };

    std::ofstream outfile(BACON_SOURCE_DIR "/data/compute_tf_test_results.txt");
    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    std::ofstream jointfile(BACON_SOURCE_DIR "/data/compute_tf_joint_file.txt");
    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    for(int i = 0; i < 50; i++){
        jointAngles["prbt_joint_1"] = random_joint(gen);
        jointAngles["prbt_joint_2"] = random_joint(gen);
        jointAngles["prbt_joint_3"] = random_joint(gen);
        jointAngles["prbt_joint_4"] = random_joint(gen);
        jointAngles["prbt_joint_5"] = random_joint(gen);
        jointAngles["prbt_joint_6"] = random_joint(gen);

        jointfile << jointAngles["prbt_joint_1"] << " ";
        jointfile << jointAngles["prbt_joint_2"] << " ";
        jointfile << jointAngles["prbt_joint_3"] << " ";
        jointfile << jointAngles["prbt_joint_4"] << " ";
        jointfile << jointAngles["prbt_joint_5"] << " ";
        jointfile << jointAngles["prbt_joint_6"] << " ";
        jointfile << std::endl;
        
        
        std::string rootLink = "prbt_base_link";
        model.setJointAngles(jointAngles);
        model.calculateWorldCoordinates(outfile, rootLink);

    }

    jointfile.close();
    outfile.close();
    std::cout << "Random tree traversal test data and results have been written to compute_tf_results.txt" << std::endl;

}