#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <limits>
#include <iomanip>
#include <random>

using S = double;  // 使用 double 类型

int main() {
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> dis_center(-1.0, 1.0);  // 中心坐标随机数范围在 -1.0 到 1.0 之间
    std::uniform_real_distribution<S> dis_extent(0.1, 1.0);   // 尺寸范围在 0.1 到 1.0 之间，用于生成 extent
    std::uniform_real_distribution<S> dis_transform(-1.0, 1.0); // 用于生成变换矩阵的随机数

    // 打开文件写入测试数据
    std::ofstream outfile(BACON_SOURCE_DIR "/data/obb_tf_test_results.txt");

    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    auto write_obb = [&](const Eigen::Matrix3d& axis, const Eigen::Vector3d& center, const Eigen::Vector3d& extent) {
        outfile << std::hex << std::setfill('0');
        // 写入旋转矩阵元素
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                outfile << "0x" << std::setw(8) << to_fixed_point(axis(i, j)) << " ";
            }
        }
        // 写入中心点
        outfile << "0x" << std::setw(8) << to_fixed_point(center.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(center.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(center.z()) << " ";
        // 写入尺寸
        outfile << "0x" << std::setw(8) << to_fixed_point(extent.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(extent.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(extent.z()) << " ";
    };

    // 生成多个随机 OBB 和变换矩阵测试用例
    for (int i = 0; i < 10; ++i) {  // 生成10000个随机测试用例
        Eigen::Matrix3d R;
        Eigen::Vector3d center(dis_center(gen), dis_center(gen), dis_center(gen));
        Eigen::Vector3d extent(dis_extent(gen), dis_extent(gen), dis_extent(gen));

        R = Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitZ());

        
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                tf(row, col) = dis_transform(gen);  
            }
            tf(row, 3) = dis_transform(gen);  
        }

        // 写入变换矩阵
        outfile << std::hex << std::setfill('0');
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                outfile << "0x" << std::setw(8) << to_fixed_point(tf(row, col)) << " ";
            }
        }

        // 写入原始 OBB
        write_obb(R, center, extent);
        
        // 计算变换后的 OBB
        Eigen::Vector3d new_center = tf.block<3, 3>(0, 0) * center + tf.block<3, 1>(0, 3);
        Eigen::Matrix3d new_axis = tf.block<3, 3>(0, 0) * R;
        Eigen::Vector3d new_extent = extent;  

        // 写入变换后的 OBB
        write_obb(new_axis, new_center, new_extent);
        
        outfile << std::endl;
    }

    outfile.close();

    std::cout << "Random OBB and transform test data have been written to obb_tf_test_results.txt" << std::endl;

    return 0;
}
