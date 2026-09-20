#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <iomanip>
#include <random>

using S = double;  // 使用 double 类型

int main() {
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> dis_angle(-M_PI, M_PI); // 角度范围在 -π 到 π 之间
    std::uniform_real_distribution<S> dis_axis(-1.0, 1.0);    // 轴向量的范围在 -1.0 到 1.0 之间

    // 打开文件写入测试数据
    std::ofstream outfile(BACON_SOURCE_DIR "/data/angle_axis_test_results.txt");

    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    auto write_matrix = [&](const Eigen::Matrix3d& R) {
        outfile << std::hex << std::setfill('0');
        // 写入旋转矩阵元素
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                outfile << "0x" << std::setw(8) << to_fixed_point(R(i, j)) << " ";
            }
        }
    };

    // 生成多个随机 angle_axis 测试用例
    for (int i = 0; i < 10000; ++i) {  
        // 生成随机角度和轴
        S angle = dis_angle(gen);
        Eigen::Vector3d axis(dis_axis(gen), dis_axis(gen), dis_axis(gen));

        // 归一化轴向量
        axis.normalize();

        // 计算旋转矩阵
        Eigen::AngleAxisd angle_axis(angle, axis);
        Eigen::Matrix3d rotation_matrix = angle_axis.toRotationMatrix();

        // 写入角度和轴（固定点表示）
        outfile << std::hex << std::setfill('0');
        outfile << "0x" << std::setw(8) << to_fixed_point(angle) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(axis.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(axis.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(axis.z()) << " ";

        // std::cout << "axis: " << axis << std::endl;
        // std::cout << "angle:" << angle << std::endl;
        // std::cout << "rotation_matrix:" << rotation_matrix << std::endl;

        // 写入旋转矩阵
        write_matrix(rotation_matrix);
        
        // 换行
        outfile << std::endl;
    }

    outfile.close();

    std::cout << "Random angle_axis test data and results have been written to angle_axis_test_results.txt" << std::endl;

    return 0;
}
