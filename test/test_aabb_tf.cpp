#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <iomanip>
#include <sstream>
#include <cmath> // 用于std::abs函数

using S = double;  // 使用 double 类型

int32_t from_hex_string(const std::string& hex_str) {
    return static_cast<int32_t>(std::stoul(hex_str, nullptr, 16));
}

int main() {
    // 打开文件读取测试数据
    std::ifstream infile(BACON_SOURCE_DIR "/data/aabb_tf_test_results.txt");

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
        Eigen::Matrix4d tf;
        Eigen::Vector3d aabb_center, aabb_center_expected;
        S aabb_radius, aabb_radius_expected;
        Eigen::Vector3d aabb_min_expected, aabb_max_expected;

        // 读取变换矩阵
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                ss >> hex_value;
                tf(row, col) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }

        // 读取 AABB 的中心点和半径
        ss >> hex_value; aabb_center.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_center.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_center.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_radius = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取预期的变换后 AABB 最小点和最大点
        ss >> hex_value; aabb_min_expected.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_min_expected.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_min_expected.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_max_expected.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_max_expected.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; aabb_max_expected.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 计算变换后的 AABB 中心点
        Eigen::Vector3d transformed_center_temp = tf.block<3, 3>(0, 0) * aabb_center;
        Eigen::Vector3d transformed_center = tf.block<3, 3>(0, 0) * aabb_center + tf.block<3, 1>(0, 3);
        S transformed_radius = aabb_radius;  // 半径在变换过程中保持不变

        // 计算变换后的 AABB 的最小和最大点
        Eigen::Vector3d transformed_min = transformed_center - Eigen::Vector3d(transformed_radius, transformed_radius, transformed_radius);
        Eigen::Vector3d transformed_max = transformed_center + Eigen::Vector3d(transformed_radius, transformed_radius, transformed_radius);

        // 比较计算结果与预期结果
        bool test_passed = true;

        // // 验证中心点
        // if (!transformed_center.isApprox(aabb_center_expected, 1e-3)) {
        //     std::cerr << "Test failed at line " << line_count + 1 << ": center mismatch." << std::endl;
        //     test_passed = false;
        // }

        // // 验证半径
        // if (std::abs(transformed_radius - aabb_radius_expected) > 1e-3) {
        //     std::cerr << "Test failed at line " << line_count + 1 << ": radius mismatch." << std::endl;
        //     test_passed = false;
        // }

        // 验证最小点
        if (!transformed_min.isApprox(aabb_min_expected, 1e-3)) {
            std::cerr << "Test failed at line " << line_count + 1 << ": min point mismatch." << std::endl;
            test_passed = false;
        }

        // 验证最大点
        if (!transformed_max.isApprox(aabb_max_expected, 1e-3)) {
            std::cerr << "Test failed at line " << line_count + 1 << ": max point mismatch." << std::endl;
            test_passed = false;
        }

        // 根据测试结果更新通过或失败的计数
        if (test_passed) {
            ++passed_tests;
        } else {
            ++failed_tests;
        }

        ++line_count;
    }

    infile.close();

    // 输出测试结果
    std::cout << "Total tests: " << line_count << std::endl;
    std::cout << "Passed tests: " << passed_tests << std::endl;
    std::cout << "Failed tests: " << failed_tests << std::endl;

    return 0;
}
