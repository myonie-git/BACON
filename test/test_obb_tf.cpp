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
    // 打开文件读取测试数据
    std::ifstream infile(BACON_SOURCE_DIR "/data/obb_tf_test_results.txt");

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
        Eigen::Matrix3d axis1, axis2_expected;
        Eigen::Vector3d center1, center2_expected;
        Eigen::Vector3d extent1, extent2_expected;

        // 读取变换矩阵
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                ss >> hex_value;
                tf(row, col) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }

        // 读取第一个OBB的旋转矩阵
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ss >> hex_value;
                axis1(i, j) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }
        // 读取第一个OBB的中心点
        ss >> hex_value; center1.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center1.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center1.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取第一个OBB的尺寸
        ss >> hex_value; extent1.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent1.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent1.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取预期的变换后OBB的旋转矩阵
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ss >> hex_value;
                axis2_expected(i, j) = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
            }
        }
        // 读取预期的变换后OBB的中心点
        ss >> hex_value; center2_expected.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center2_expected.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; center2_expected.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 读取预期的变换后OBB的尺寸
        ss >> hex_value; extent2_expected.x() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent2_expected.y() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);
        ss >> hex_value; extent2_expected.z() = static_cast<S>(from_hex_string(hex_value)) / (1 << 16);

        // 重新计算变换后的 OBB
        Eigen::Vector3d center2_computed = tf.block<3, 3>(0, 0) * center1 + tf.block<3, 1>(0, 3);
        Eigen::Matrix3d axis2_computed = tf.block<3, 3>(0, 0) * axis1;
        Eigen::Vector3d extent2_computed = extent1;  // 范围不允许为负值，取绝对值

        // 比较计算结果与预期结果
        bool test_passed = true;

        if (!axis2_computed.isApprox(axis2_expected, 1e-3)) {
            std::cerr << "Test failed at line " << line_count + 1 << ": axis mismatch." << std::endl;
            test_passed = false;
        }
        if (!center2_computed.isApprox(center2_expected, 1e-3)) {
            std::cerr << "Test failed at line " << line_count + 1 << ": center mismatch." << std::endl;
            test_passed = false;
        }
        if (!extent2_computed.isApprox(extent2_expected, 1e-3)) {
            std::cerr << "Test failed at line " << line_count + 1 << ": extent mismatch." << std::endl;
            test_passed = false;
        }

        if (test_passed) {
            ++passed_tests;
        } else {
            ++failed_tests;
        }

        ++line_count;
    }

    infile.close();

    std::cout << "Total tests: " << line_count << std::endl;
    std::cout << "Passed tests: " << passed_tests << std::endl;
    std::cout << "Failed tests: " << failed_tests << std::endl;

    return 0;
}
