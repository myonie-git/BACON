#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <limits>
#include <iomanip>
#include <random>
#include "aabb.h"

using S = double;  // 使用double类型

int main() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<S> dis(-1.0, 1.0);

    std::ofstream outfile(BACON_SOURCE_DIR "/data/aabb_test_results.txt");

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

    // 生成多个随机AABB测试用例
    for (int i = 0; i < 100; ++i) {  // 生成5个随机AABB
        Eigen::Vector3d v1(dis(gen), dis(gen), dis(gen));
        Eigen::Vector3d v2(dis(gen), dis(gen), dis(gen));
        Eigen::Vector3d v3(dis(gen), dis(gen), dis(gen));
        Eigen::Vector3d v4(dis(gen), dis(gen), dis(gen));

        AABB<S> aabb1(v1, v2);
        AABB<S> aabb2(v3, v4);
        
        // 写入第一个AABB
        write_aabb(aabb1);
        
        // 写入第二个AABB
        write_aabb(aabb2);
        
        // 计算是否重叠
        bool overlap; 
        for(int k = 0; k < 100000000; k++){
            overlap = aabb1.overlap(aabb2);
        }
        
        // 写入是否重叠
        outfile << overlap << std::endl;
    }

    outfile.close();

    std::cout << "Random AABB test data and results have been written to aabb_test_results.txt" << std::endl;

    return 0;
}
