#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>
#include <filesystem>
#include <getopt.h>
#include <chrono>
#include <omp.h>
#include <cstdlib>  // for atoi

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    using S = double;
    std::string directory = BACON_SOURCE_DIR "/env/48";  
    int filenum = 100; 
    int thread_count = 4;

    static struct option long_options[] = {
        {"directory", required_argument, 0, 'd'},
        {"threads", required_argument, 0, 't'},
        {0, 0, 0, 0} 
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:t:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                directory = optarg;
                break;
            case 't':
                thread_count = std::atoi(optarg);
                break;
            default:
                break;
        }
    }

    URDFModel model;
    std::string urdfFilePath = BACON_SOURCE_DIR "/go1.urdf"; 
    std::string rootLink = "base";
    if (!model.loadURDF(urdfFilePath)) {
        std::cerr << "Failed to load URDF file: " << urdfFilePath << std::endl;
        assert(0);
        return -1;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file()) { 
            std::string filename = entry.path().string();
            std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);

            CollisionEnv<S> env;
            env.InitTree(collision_objects);

            double start_angle = -1.57;
            double end_angle = 1.57;
            double step_size = 1.0;
            int steps = static_cast<int>((end_angle - start_angle) / step_size) + 1;

            #pragma omp parallel for collapse(6) schedule(dynamic) num_threads(thread_count)
            for (int i1 = 0; i1 < steps; i1++) {
                for (int i2 = 0; i2 < steps; i2++) {
                    for (int i3 = 0; i3 < steps; i3++) {
                        for (int i4 = 0; i4 < steps; i4++) {
                            for (int i5 = 0; i5 < steps; i5++) {
                                for (int i6 = 0; i6 < steps; i6++) {
                                    URDFModel local_model = model;
                                    std::map<std::string, double> jointAngles = {
                                        {"FL_calf_joint", start_angle + i1 * step_size},
                                        {"FL_hip_joint", start_angle + i2 * step_size},
                                        {"FL_thigh_joint", start_angle + i3 * step_size},
                                        {"FR_calf_joint", start_angle + i4 * step_size},
                                        {"FR_hip_joint", start_angle + i5 * step_size},
                                        {"FR_thigh_joint", start_angle + i6 * step_size},
                                        {"RL_calf_joint", 0.0},
                                        {"RL_hip_joint", 0.0},
                                        {"RL_thigh_joint", 0.0},
                                        {"RR_calf_joint", 0.0},
                                        {"RR_hip_joint", 0.0},
                                        {"RR_thigh_joint", 0.0}
                                    };

                                    local_model.setJointAngles(jointAngles);
                                    local_model.calculateWorldCoordinates(rootLink);

                                    std::vector<CollisionObject<S>> collision_geometry_;
                                    for (size_t j = 0; j < local_model.collisionGeometries.size(); j++) {
                                        const auto& collisionGeomPtr = local_model.collisionGeometries[j];
                                        CollisionObject<S> collisionObject(*collisionGeomPtr,
                                            local_model.links.at(collisionGeomPtr->parentLink->name));
                                        collision_geometry_.push_back(std::move(collisionObject));
                                    }

                                    bool result = false;
                                    for (size_t j = 0; j < collision_geometry_.size() && !result; j++) {
                                        result = env.collide(&collision_geometry_[j], nullptr);
                                    }

                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    std::cout << elapsed_seconds.count() << std::endl;
    // std::cout << elapsed_seconds.count() * thread_count << std::endl;

    return 0;
}
