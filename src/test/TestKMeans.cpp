// /*
//     Copyright [2019] Jian ZhiQiang
// */

// #include "Common.hpp"

// int main(int argc, char** argv) {
//     // 初始化ros程序
//     ros::init(argc, argv, "motion_planning_node");
//     ros::NodeHandle nh("~");
//     std::vector<double> data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     std::vector<std::vector<size_t>> clusters;
//     std::vector<double> centers = Tools::kMeans(data, 3, clusters);
//     for (size_t i = 0; i < 3; i++) {
//         std::cout << "center " << i << " is: " << centers[i] << std::endl;
//         std::cout << "values: " << std::endl;
//         for (size_t j = 0; j < clusters[i].size(); j++) {
//             std::cout << data[clusters[i][j]] << std::endl;
//         }
//     }
    
//     return 0;
// }