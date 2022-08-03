/*
    Copyright [2019] Jian ZhiQiang
*/

// #include "Common.hpp"

// int main(int argc, char** argv) {
//     // 初始化ros程序
//     ros::init(argc, argv, "motion_planning_node");
//     ros::NodeHandle nh("~");
//     // 定义第一个区间
//     Section axis_section;
//     SectionSet axis_section_set_1;
//     axis_section = {2, 8};
//     axis_section_set_1.push_back(axis_section);
//     axis_section = {9, 14};
//     axis_section_set_1.push_back(axis_section);
//     axis_section = {15, 20};
//     axis_section_set_1.push_back(axis_section);
//     axis_section = {22, 25.0};
//     axis_section_set_1.push_back(axis_section);

//     // 定义第二个区间
//     SectionSet axis_section_set_2;
//     axis_section = {1, 10};
//     axis_section_set_2.push_back(axis_section);
//     axis_section = {12, 16};
//     axis_section_set_2.push_back(axis_section);
//     axis_section = {19, 25};
//     axis_section_set_2.push_back(axis_section);
//     axis_section = {24.9, 40};
//     axis_section_set_2.push_back(axis_section);
//     // 求交集,结果应该是{2,8;9,10;12,14;15,16;19,20;22,25}
//     SectionSet intersection = Tools::getIntersection(axis_section_set_1, axis_section_set_2);
//     for (size_t i = 0; i < intersection.size(); i++) {
//         std::cout << "区间下限: " << intersection[i].min_ << "，区间上限： " << intersection[i].max_ << std::endl;
//     }
//     std::cout << "输出中文会不会乱码" << std::endl;
//     return 0;
// }
