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

int32_t from_hex_string(const std::string& hex_str) {
    return static_cast<int32_t>(std::stoul(hex_str, nullptr, 16));
}

bool read_obb(std::stringstream& ss, OBB<S>& obb) {
    std::string hex_value;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            ss >> hex_value;
            obb.axis(i, j) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        }
    }
    ss >> hex_value; obb.To.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; obb.To.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; obb.To.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; obb.extent.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; obb.extent.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; obb.extent.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

    return !ss.fail();
}

bool read_aabb(std::stringstream& ss, AABB<S>& aabb) {
    std::string hex_value;
    ss >> hex_value; aabb.min_.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; aabb.min_.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; aabb.min_.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; aabb.max_.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; aabb.max_.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
    ss >> hex_value; aabb.max_.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

    return !ss.fail();
}

int main() {
    // 打开文件读取测试数据
    std::ifstream infile(BACON_SOURCE_DIR "/data/tree_traversal_test_results.txt");
    if (!infile.is_open()) {
        std::cerr << "Failed to open the file: data/tree_traversal_test_results.txt" << std::endl;
        return 1;
    }

    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);
    
    CollisionEnv<S> env;
    env.InitTree(collision_objects);

    std::string line;
    int line_count = 0;
    int passed_tests = 0, failed_tests = 0;

    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        OBB<S> obb;
        AABB<S> aabb;
        int expected_result;

        if (!read_aabb(ss, aabb)) {
            std::cerr << "Failed to read AABB at line " << line_count + 1 << std::endl;
            break;
        }
        if (!read_obb(ss, obb)) {
            std::cerr << "Failed to read OBB at line " << line_count + 1 << std::endl;
            break;
        }

        ss >> expected_result;

        // 重新创建CollisionObject对象并计算是否重叠
        CollisionObject<S> collisionObject(aabb, obb);
        bool result = env.collide(&collisionObject, nullptr);

        // 比较计算结果与存储的结果
        if (result == expected_result) {
            ++passed_tests;
        } else {
            ++failed_tests;
            std::cerr << "Test failed at line " << line_count + 1 << ": expected " << expected_result << ", but got " << result << std::endl;
        }

        ++line_count;
    }

    infile.close();

    std::cout << "Total tests: " << line_count << std::endl;
    std::cout << "Passed tests: " << passed_tests << std::endl;
    std::cout << "Failed tests: " << failed_tests << std::endl;

    return 0;
}
