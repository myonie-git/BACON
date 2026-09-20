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

using S = double;  // 使用 double 类型

int main() {
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> dis_center(-1.0, 1.0);  // 中心坐标随机数范围在 -1.0 到 1.0 之间
    std::uniform_real_distribution<S> dis_radius(0.1, 1.0);   // 半径范围在 0.1 到 1.0 之间
    std::uniform_real_distribution<S> dis_transform(-1.0, 1.0); // 变换矩阵元素范围

    // 打开文件写入测试数据
    std::ofstream outfile(BACON_SOURCE_DIR "/data/aabb_tf_test_results.txt");

    if (!outfile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 bits for fractional part
    };

    auto write_aabb = [&](const AABB<S>& aabb) {
        outfile << std::hex << std::setfill('0');
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.min_.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.min_.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.min_.z()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.max_.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.max_.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(aabb.max_.z()) << " ";
    };

    // 生成测试用例
    for (int i = 0; i < 10; ++i) {  // 生成10个随机测试用例
        Eigen::Vector3d center(dis_center(gen), dis_center(gen), dis_center(gen));
        S radius = dis_radius(gen);

        Eigen::Matrix3d R;  // 旋转矩阵
        Eigen::Vector3d T;  // 平移向量

        // 随机生成旋转矩阵 R
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                R(row, col) = dis_transform(gen);
            }
        }

        // 随机生成平移向量 T
        for (int row = 0; row < 3; ++row) {
            T(row) = dis_transform(gen);
        }

        // 写入旋转矩阵 R
        outfile << std::hex << std::setfill('0');
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                outfile << "0x" << std::setw(8) << to_fixed_point(R(row, col)) << " ";
            }
        }

        // 写入平移向量 T
        for (int row = 0; row < 3; ++row) {
            outfile << "0x" << std::setw(8) << to_fixed_point(T(row)) << " ";
        }

        // 写入 AABB 中心点和半径
        outfile << "0x" << std::setw(8) << to_fixed_point(center.x()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(center.y()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(center.z()) << " ";
        outfile << "0x" << std::setw(8) << to_fixed_point(radius) << " ";

        // 创建一个 CollisionObject 实例并计算转换后的 AABB
        Eigen::Transform<S, 3, Eigen::Isometry> transform;
        transform.linear() = R;  // 设置旋转矩阵
        transform.translation() = T;  // 设置平移向量
        CollisionObject<S> obj(center, radius, transform);

        // 写入计算后的 AABB 最小和最大点
        write_aabb(obj.getAABB());

        outfile << std::endl;
    }

    outfile.close();

    std::cout << "Random AABB and transform test data have been written to aabb_tf_test_results.txt" << std::endl;

    return 0;
}
