#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <limits>
#include <iomanip>
#include <random>
#include "obb.h"

using S = double;  // 使用double类型

int main() {
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> dis_center(-1.0, 1.0);  // 中心坐标随机数范围在 -1.0 到 1.0 之间
    std::uniform_real_distribution<S> dis_extent(0.1, 1.0);   // 尺寸范围在 0.1 到 1.0 之间，用于生成extent

    // 打开文件写入测试数据
    std::ofstream outfile(BACON_SOURCE_DIR "/data/obb_test_results.txt");

    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    auto write_obb = [&](const OBB<S>& obb) {
        outfile << std::hex << std::setfill('0');
        // 写入旋转矩阵元素
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                outfile << "0x" << std::setw(8) << to_fixed_point(obb.axis(i, j)) << " ";
            }
        }
        // 写入中心点
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.z()) << " ";
        // 写入尺寸
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.z()) << " ";
    };

    // 生成多个随机OBB测试用例
    for (int i = 0; i < 100; ++i) {  // 生成1000个随机OBB
        Eigen::Matrix3d R1, R2;
        Eigen::Vector3d center1(dis_center(gen), dis_center(gen), dis_center(gen));
        Eigen::Vector3d center2(dis_center(gen), dis_center(gen), dis_center(gen));
        Eigen::Vector3d extent1(dis_extent(gen), dis_extent(gen), dis_extent(gen));
        Eigen::Vector3d extent2(dis_extent(gen), dis_extent(gen), dis_extent(gen));

        // 随机生成旋转矩阵
        R1 = Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitZ());

        R2 = Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(dis_center(gen) * M_PI, Eigen::Vector3d::UnitZ());

        OBB<S> obb1(R1, center1, extent1);
        OBB<S> obb2(R2, center2, extent2);
        
        // 写入第一个OBB
        write_obb(obb1);
        
        // 写入第二个OBB
        write_obb(obb2);
        
        // 计算是否重叠
        bool overlap; 
        for(int k = 0; k < 100000000; k++){
            overlap = obb1.overlap(obb2);
        }
        // 写入是否重叠
        outfile << overlap << std::endl;
    }

    outfile.close();

    std::cout << "Random OBB test data and results have been written to obb_test_results.txt" << std::endl;

    return 0;
}
