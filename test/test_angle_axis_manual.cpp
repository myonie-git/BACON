#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <sstream>

using S = double;  // 使用 double 类型

int32_t from_hex_string(const std::string& hex_str) {
    return static_cast<int32_t>(std::stoul(hex_str, nullptr, 16));
}

void compute_rotation_matrix(S angle, const S axis_x, const S axis_y, const S axis_z, S R[3][3]) {
    S cos_angle = std::cos(angle);
    S sin_angle = std::sin(angle);
    S one_minus_cos = 1.0 - cos_angle;

    S norm = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
    S x = axis_x / norm;
    S y = axis_y / norm;
    S z = axis_z / norm;

    R[0][0] = cos_angle + x * x * one_minus_cos;
    R[0][1] = x * y * one_minus_cos - z * sin_angle;
    R[0][2] = x * z * one_minus_cos + y * sin_angle;

    R[1][0] = y * x * one_minus_cos + z * sin_angle;
    R[1][1] = cos_angle + y * y * one_minus_cos;
    R[1][2] = y * z * one_minus_cos - x * sin_angle;

    R[2][0] = z * x * one_minus_cos - y * sin_angle;
    R[2][1] = z * y * one_minus_cos + x * sin_angle;
    R[2][2] = cos_angle + z * z * one_minus_cos;
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
        S axis_x, axis_y, axis_z;
        S expected_rotation_matrix[3][3], calculated_rotation_matrix[3][3];

        // 读取角度
        ss >> hex_value;
        angle = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取轴向量
        ss >> hex_value; axis_x = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; axis_y = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; axis_z = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取存储的旋转矩阵
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ss >> hex_value;
                expected_rotation_matrix[i][j] = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }

        // 手动计算旋转矩阵
        compute_rotation_matrix(angle, axis_x, axis_y, axis_z, calculated_rotation_matrix);

        // 比较计算的旋转矩阵与存储的旋转矩阵
        bool matrices_match = true;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (std::abs(calculated_rotation_matrix[i][j] - expected_rotation_matrix[i][j]) > 1e-4) {
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
