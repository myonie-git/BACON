// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <Eigen/Geometry> 
// #include <collision_object.h>
// #include <collision_env.h>
// #include "aabb.h"
// #include "types.h"
// #include <unordered_map>

// int main() {
//     using S = double;
//     const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
//     std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);
    
//     CollisionEnv<S> env;
//     env.InitTree(collision_objects);

//     // 打开文件写入数据
//     std::ofstream outfile(BACON_SOURCE_DIR "/data/bvh_tree_ram_data.txt");

//     if (!outfile.is_open()) {
//         std::cerr << "无法打开文件。" << std::endl;
//         return 1;
//     }

//     // 定点数转换函数
//     auto to_fixed_point = [](S value) -> int32_t {
//         return static_cast<int32_t>(value * (1 << 16));  // 16 位小数部分
//     };

//     std::unordered_map<NodeBase<AABB<S>>*, int> node_index_map;
//     int current_index = 0;

//     // 遍历节点并将其写入文件
//     auto write_node = [&](NodeBase<AABB<S>>* node) {

//         if (node == nullptr) return;

//         // 将节点指针映射到索引
//         node_index_map[node] = current_index++;
        
//         outfile << std::hex << std::setfill('0');

//         // 写入 AABB 的最小和最大值
//         outfile << "0x" << std::setw(8) << to_fixed_point(node->bv.min_.x()) << " ";
//         outfile << "0x" << std::setw(8) << to_fixed_point(node->bv.min_.y()) << " ";
//         outfile << "0x" << std::setw(8) << to_fixed_point(node->bv.min_.z()) << " ";
//         outfile << "0x" << std::setw(8) << to_fixed_point(node->bv.max_.x()) << " ";
//         outfile << "0x" << std::setw(8) << to_fixed_point(node->bv.max_.y()) << " ";
//         outfile << "0x" << std::setw(8) << to_fixed_point(node->bv.max_.z()) << " ";

//         // 如果是叶子节点，写入 OBB 数据
//         if (node->isLeaf()) {
//             OBB<S> obb = node->data->getOBB();  // 获取 OBB 数据
//             for (int i = 0; i < 3; ++i) {
//                 for (int j = 0; j < 3; ++j) {
//                     outfile << "0x" << std::setw(8) << to_fixed_point(obb.axis(i, j)) << " ";
//                 }
//             }
//             outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.x()) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.y()) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(obb.To.z()) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.x()) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.y()) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(obb.extent.z()) << " ";
//         } else {
//             for (int i = 0; i < 3; ++i) {
//                 for (int j = 0; j < 3; ++j) {
//                     outfile << "0x" << std::setw(8) << to_fixed_point(0) << " ";
//                 }
//             }
//             outfile << "0x" << std::setw(8) << to_fixed_point(0) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(0) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(0) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(0) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(0) << " ";
//             outfile << "0x" << std::setw(8) << to_fixed_point(0) << " ";
//         }

//         // 写入子节点的地址
//         for (int i = 0; i < 2; ++i) {
//             if (node->children[i] != nullptr) {
//                 // 输出子节点的索引
//                 outfile << "0x" << std::setw(8) << node_index_map[node->children[i]] << " ";
//             } else {
//                 outfile << "0x00000000 ";
//             }
//         }
//         outfile << std::endl;
//     };

//     // 遍历 BVH 树并将数据写入文件
//     std::function<void(NodeBase<AABB<S>>*)> traverse_and_write = [&](NodeBase<AABB<S>>* node) {
//         if (node == nullptr) return;
//         write_node(node);
//         traverse_and_write(node->children[0]);
//         traverse_and_write(node->children[1]);
//     };

//     // 从根节点开始遍历和写入
//     traverse_and_write(env.dtree.getRoot());

//     outfile.close();

//     std::cout << "BVH 树数据已保存到 bvh_tree_ram_data.txt 文件中。" << std::endl;

//     return 0;
// }


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Geometry> 
#include <collision_object.h>
#include <collision_env.h>
#include "aabb.h"
#include "types.h"
#include <unordered_map>

int main() {
    using S = double;
    const std::string filename = BACON_SOURCE_DIR "/boxes.txt";
    std::vector<CollisionObject<S>*> collision_objects = readBoxesFromFile<S>(filename);
    
    CollisionEnv<S> env;
    env.InitTree(collision_objects);

    // 打开多个文件写入数据
    std::ofstream outfile_aabb_min_x(BACON_SOURCE_DIR "/data/ram/aabb_min_x.txt");
    std::ofstream outfile_aabb_min_y(BACON_SOURCE_DIR "/data/ram/aabb_min_y.txt");
    std::ofstream outfile_aabb_min_z(BACON_SOURCE_DIR "/data/ram/aabb_min_z.txt");
    std::ofstream outfile_aabb_max_x(BACON_SOURCE_DIR "/data/ram/aabb_max_x.txt");
    std::ofstream outfile_aabb_max_y(BACON_SOURCE_DIR "/data/ram/aabb_max_y.txt");
    std::ofstream outfile_aabb_max_z(BACON_SOURCE_DIR "/data/ram/aabb_max_z.txt");

    std::ofstream outfile_obb_axes(BACON_SOURCE_DIR "/data/ram/obb_axes_ram.txt");
    std::ofstream outfile_obb_extents(BACON_SOURCE_DIR "/data/ram/obb_extents_ram.txt");
    std::ofstream outfile_obb_center(BACON_SOURCE_DIR "/data/ram/obb_center_ram.txt");

    std::ofstream outfile_child_addr(BACON_SOURCE_DIR "/data/ram/child_addr_ram.txt");
    std::ofstream outfile_is_leaf(BACON_SOURCE_DIR "/data/ram/is_leaf_ram.txt");

    // 检查文件是否成功打开
    if (!outfile_aabb_min_x.is_open()) {
        std::cerr << "无法打开一个或多个文件。" << std::endl;
        return 1;
    }

    // 定点数转换函数
    auto to_fixed_point = [](S value) -> int32_t {
        return static_cast<int32_t>(value * (1 << 16));  // 16 位小数部分
    };

    std::unordered_map<NodeBase<AABB<S>>*, int> node_index_map;
    int current_index = 0;

    // 遍历节点并将其写入文件
    auto write_node = [&](NodeBase<AABB<S>>* node) {
        if (node == nullptr) return;

        // // 将节点指针映射到索引
        // node_index_map[node] = current_index++;

        // 写入 AABB 的最小和最大值
        outfile_aabb_min_x << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(node->bv.min_.x()) << std::endl;
        outfile_aabb_min_y << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(node->bv.min_.y()) << std::endl;
        outfile_aabb_min_z << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(node->bv.min_.z()) << std::endl;
        outfile_aabb_max_x << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(node->bv.max_.x()) << std::endl;
        outfile_aabb_max_y << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(node->bv.max_.y()) << std::endl;
        outfile_aabb_max_z << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(node->bv.max_.z()) << std::endl;

        // 如果是叶子节点，写入 OBB 数据
        if (node->isLeaf()) {
            OBB<S> obb = node->data->getOBB();  // 获取 OBB 数据
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    outfile_obb_axes << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(obb.axis(i, j)) << std::endl;
                }
                // outfile_obb_axes << std::endl;
            }
            outfile_obb_center << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(obb.To.x()) << std::endl;
            outfile_obb_center << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(obb.To.y()) << std::endl;
            outfile_obb_center << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(obb.To.z()) << std::endl;
            outfile_obb_extents << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(obb.extent.x()) << std::endl;
            outfile_obb_extents << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(obb.extent.y()) << std::endl;
            outfile_obb_extents << "0x" << std::hex << std::setw(8) << std::setfill('0') << to_fixed_point(obb.extent.z()) << std::endl;
            outfile_is_leaf << "0x00000001" << std::endl;
        } else {
            for (int i = 0; i < 3; ++i) {
                outfile_obb_axes << "0x00000000 0x00000000 0x00000000" << std::endl;
            }
            outfile_obb_center << "0x00000000" << std::endl;
            outfile_obb_center << "0x00000000" << std::endl;
            outfile_obb_center << "0x00000000" << std::endl;
            outfile_obb_extents << "0x00000000" << std::endl;
            outfile_obb_extents << "0x00000000" << std::endl;
            outfile_obb_extents << "0x00000000" << std::endl;
            outfile_is_leaf << "0x00000000" << std::endl;
        }

        for (int i = 0; i < 2; ++i) {
            if (!node->isLeaf()) {
                // 输出子节点的索引
                outfile_child_addr << "0x" << std::hex << std::setw(8) << std::setfill('0') << node_index_map[node->children[i]] << " ";
            } else {
                outfile_child_addr << "0x00000000 ";
            }
        }
        outfile_child_addr << std::endl;
    };

    // 遍历 BVH 树并将数据写入文件
    std::function<void(NodeBase<AABB<S>>*)> traverse_and_write = [&](NodeBase<AABB<S>>* node) {
        if (node == nullptr) return;
        write_node(node);
        traverse_and_write(node->children[0]);
        traverse_and_write(node->children[1]);
    };

    // 从根节点开始遍历和写入

    // 遍历树，预写入节点数据，请完成这里的代码，先遍历一遍树，然后写入地址，然后
    std::function<void(NodeBase<AABB<S>>*)> pre_traverse = [&](NodeBase<AABB<S>>* node) {
        if (node == nullptr) return;
        node_index_map[node] = current_index++;
        pre_traverse(node->children[0]);
        pre_traverse(node->children[1]);
    };

    pre_traverse(env.dtree.getRoot());
    traverse_and_write(env.dtree.getRoot());

    // 关闭文件
    outfile_aabb_min_x.close();
    outfile_aabb_min_y.close();
    outfile_aabb_min_z.close();
    outfile_aabb_max_x.close();
    outfile_aabb_max_y.close();
    outfile_aabb_max_z.close();
    outfile_obb_axes.close();
    outfile_obb_extents.close();
    outfile_obb_center.close();
    outfile_child_addr.close();
    outfile_is_leaf.close();

    std::cout << "BVH 树数据已保存到多个文件中。" << std::endl;

    return 0;
}
