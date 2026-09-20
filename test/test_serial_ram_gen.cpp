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
    using S = double;

    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/prbt_description.urdf"; // Replace with your URDF file path
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    // Assign serial numbers to joints
    // Replace "prbt_base_link" with the actual root link name of your robot
    model.assignJointSerialNumbers("prbt_base_link");

    std::ofstream outfile(BACON_SOURCE_DIR "/data/test_serial_ram_gen_results.txt");
    if (!outfile.is_open()) {
        std::cerr << "Failed to open output file." << std::endl;
        return -1;
    }
    outfile << std::hex << std::setfill('0');
    // Iterate over each collision geometry
    for(int i = 0; i < model.collisionGeometries.size(); i++){
        const auto& collisionGeomPtr = model.collisionGeometries[i];
        const Link* parentLink = collisionGeomPtr->parentLink;
        
        if(parentLink){
            // Find the joint that connects the parent link to its parent
            int serial = -1; // Default value if no joint is found
            std::string parentLinkName = parentLink->name;

            if(parentLink->name == "prbt_base_link"){
                serial = 0;
            }

            for(const auto& joint : model.joints){
                if(joint.child == parentLinkName){
                    serial = joint.serial + 1;
                    break;
                }
            }

            
            outfile << "0x" << std::hex << std::setw(8) << std::setfill('0') << serial <<  std::endl;

            // // Print the parent link's name and the serial number to the console
            // std::cout << "Collision Geometry " << i 
            //           << ": Parent Link: " << parentLinkName 
            //           << ", Parent Joint Serial: " << serial << std::endl;

            // // Optionally, write the information to the output file
            // outfile << "Collision Geometry " << i 
            //         << ": Parent Link: " << parentLinkName 
            //         << ", Parent Joint Serial: " << serial << std::endl;
        } 
        // else {
        //     std::cout << "Collision Geometry " << i 
        //               << ": No parent link found." << std::endl;
        //     outfile << "Collision Geometry " << i 
        //             << ": No parent link found." << std::endl;
        // }
    }

    outfile.close();

    return 0;
}