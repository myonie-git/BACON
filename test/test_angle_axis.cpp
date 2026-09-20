#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <iomanip>
#include <sstream>

using S = double;  // 使用 double 类型

int32_t from_hex_string(const std::string& hex_str) {
    return static_cast<int32_t>(std::stoul(hex_str, nullptr, 16));
}

int main() {
    
    std::ifstream infile(BACON_SOURCE_DIR "/data/angle_axis_test_results.txt");

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
        S angle;
        Eigen::Vector3d axis;
        Eigen::Matrix3d expected_rotation_matrix, calculated_rotation_matrix;

        // 读取角度
        ss >> hex_value;
        angle = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取轴向量
        ss >> hex_value; axis.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; axis.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; axis.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取存储的旋转矩阵
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ss >> hex_value;
                expected_rotation_matrix(i, j) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }

        // 使用Eigen库计算旋转矩阵
        Eigen::AngleAxisd angle_axis(angle, axis);
        calculated_rotation_matrix = angle_axis.toRotationMatrix();

        // 比较计算的旋转矩阵与存储的旋转矩阵
        bool matrices_match = true;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (std::abs(calculated_rotation_matrix(i, j) - expected_rotation_matrix(i, j)) > 1e-4) {
                    matrices_match = false;
                    break;
                }
            }
            if (!matrices_match) break;
        }

        // 统计测试结果
        if (matrices_match) {
            ++passed_tests;
        } else {
            ++failed_tests;
            std::cerr << "Test failed at line " << line_count + 1 << std::endl;
        }

        ++line_count;
    }

    infile.close();

    std::cout << "Total tests: " << line_count << std::endl;
    std::cout << "Passed tests: " << passed_tests << std::endl;
    std::cout << "Failed tests: " << failed_tests << std::endl;

    return 0;
}
