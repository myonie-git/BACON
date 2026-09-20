#include "urdf_model.h"

int main() {
    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; // Replace with your URDF file path

    if (model.loadURDF(urdfFilePath)) {
        std::cout << "Successfully loaded URDF file: " << urdfFilePath << std::endl;

        std::map<std::string, double> jointAngles = {
            {"prbt_joint_1", 0.0},
            {"prbt_joint_2", 0.0},
            {"prbt_joint_3", 0.0},
            {"prbt_joint_4", 0.0},
            {"prbt_joint_5", 0.0},
            {"prbt_joint_6", 0.0}
        };
        std::string rootLink = "prbt_base_link";
        model.setJointAngles(jointAngles);
        model.calculateWorldCoordinates(rootLink);
        model.printModel();
        model.printCollisionCoordinates();
    } else {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        return -1;
    }

    return 0;
}