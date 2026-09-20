#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <boost/filesystem.hpp>
#include "resources/config.h"
#include <random_numbers/random_numbers.h>
#include <iostream>

void testMeshes2Box(){
    std::vector<shapes::Mesh*> loaded_meshes;
    std::vector<bodies::Body*> loaded_convex_meshes;
    random_numbers::RandomNumberGenerator rng;
    // loaded_meshes.push_back(shapes::createMeshFromResource(
    //     "file://" + (boost::filesystem::path(TEST_RESOURCES_DIR) / "/cube.stl").string()));
    
    const std::string absolute_path = BACON_SOURCE_DIR "/resources/fanuc_description/meshes/collision/link_1.stl";
    loaded_meshes.push_back(shapes::createMeshFromResource("file://" + absolute_path));

    // resources/fanuc_description/meshes/collision/link_1.stl
    loaded_convex_meshes.push_back(new bodies::ConvexMesh(loaded_meshes.back()));

    for(size_t i = 0; i < loaded_meshes.size(); ++i){
        shapes::Mesh* load_ms = loaded_meshes[i];
        bodies::ConvexMesh body(load_ms);
        bodies::AABB bbox;
        body.computeBoundingBox(bbox);
        bodies::OBB obbox;
        body.computeBoundingBox(obbox);

        std::cout << "aabbox.min().x() = " << bbox.min().x() << std::endl;
        std::cout << "aabbox.min().y() = " << bbox.min().y() << std::endl;
        std::cout << "aabbox.min().z() = " << bbox.min().z() << std::endl;
        std::cout << "aabbox.max().x() = " << bbox.max().x() << std::endl;
        std::cout << "aabbox.max().y() = " << bbox.max().y() << std::endl;
        std::cout << "aabbox.max().z() = " << bbox.max().z() << std::endl;

        std::cout << "obbox.getExtents().x() = " << obbox.getExtents().x() << std::endl;
        std::cout << "obbox.getExtents().y() = " << obbox.getExtents().y() << std::endl;
        std::cout << "obbox.getExtents().z() = " << obbox.getExtents().z() << std::endl;
        std::cout << "obbox.getPose().translation().x() = " << obbox.getPose().translation().x() << std::endl;
        std::cout << "obbox.getPose().translation().y() = " << obbox.getPose().translation().y() << std::endl;
        std::cout << "obbox.getPose().translation().z() = " << obbox.getPose().translation().z() << std::endl;
        std::cout << "obbox.getPose().linear() = " << std::endl << obbox.getPose().linear() << std::endl;  //表示旋转矩阵
    }
}

int main(){
    testMeshes2Box();
}