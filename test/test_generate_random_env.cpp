#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "types.h"
#include <assert.h>
#include <getopt.h>

int main(int argc, char *argv[]){
    
    using S = double;
    std::string filepath = BACON_SOURCE_DIR "/env/4/";
    int collnum = 4;
    int filenum = 100;

    static struct option long_options[] = {
        {"file", required_argument, 0, 'f'},
        {"collnum", required_argument, 0, 'c'},
        {"filenum", required_argument, 0, 'n'},
        {0, 0, 0, 0}  // 结束标志
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "f:c:n:", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'f':
                filepath = optarg;
                break;
            case 'c':
                collnum = std::stoi(optarg);
                break;
            case 'n':
                filenum = std::stoi(optarg);
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-f file] [-c coll_num] [-n filenum]" << std::endl;
                return 1;
        }
    }



    for (int i = 1; i <= filenum; ++i) {

        std::vector<CollisionObject<S>*> collision_objects = randomCollisionObjects<S>(collnum);
        CollisionEnv<S> env;
        env.InitTree(collision_objects);


        std::ostringstream filename;
        filename << filepath << i; 
        
        std::ofstream file(filename.str());
        if (!file) {
            std::cerr << "Error creating file: " << filename.str() << std::endl;
            return 1;
        }
                
        for(int i = 0; i < collnum; i++){
            collision_objects[i]->aabb.printEnv(file);
        }
        file.close();

        std::cout << "Created file: " << filename.str() << std::endl;
    }

    return 0;
}