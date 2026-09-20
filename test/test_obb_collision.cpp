#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <iomanip>
#include <sstream>
#include "obb.h"

using S = double;  // 使用double类型

int32_t from_hex_string(const std::string& hex_str) {
    return static_cast<int32_t>(std::stoul(hex_str, nullptr, 16));
}

int main() {
    // 打开文件读取测试数据
    std::ifstream infile(BACON_SOURCE_DIR "/data/obb_test_results.txt");

    if (!infile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    std::string line;
    int line_count = 0;
    int passed_tests = 0, failed_tests = 0;

    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string hex_value;
        Eigen::Matrix3d axis1, axis2;
        Eigen::Vector3d center1, center2;
        Eigen::Vector3d extent1, extent2;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ss >> hex_value;
                axis1(i, j) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }

        ss >> hex_value; center1.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center1.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center1.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        ss >> hex_value; extent1.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent1.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent1.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取第二个OBB的旋转矩阵
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ss >> hex_value;
                axis2(i, j) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }
        // 读取第二个OBB的中心点
        ss >> hex_value; center2.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center2.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center2.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取第二个OBB的尺寸
        ss >> hex_value; extent2.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent2.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent2.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取存储的碰撞结果
        int expected_overlap;
        ss >> expected_overlap;

        // 重新创建OBB对象并计算是否重叠
        OBB<S> obb1(axis1, center1, extent1);
        OBB<S> obb2(axis2, center2, extent2);
        bool overlap = obb1.overlap_gpu(obb2);
        bool overlap1 = obb1.overlap(obb2);

        // 比较计算结果与存储的结果
        // if (overlap == expected_overlap) {
        if(overlap1 == overlap){
            ++passed_tests;
        } else {
            ++failed_tests;
            std::cout << "Test failed at line " << line_count + 1 << ": expected " << overlap1 << ", but got " << overlap << std::endl;
        }
        ++line_count;
    }

    infile.close();

    std::cout << "Total tests: " << line_count << std::endl;
    std::cout << "Passed tests: " << passed_tests << std::endl;
    std::cout << "Failed tests: " << failed_tests << std::endl;

    return 0;
}