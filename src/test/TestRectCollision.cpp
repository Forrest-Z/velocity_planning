// /*
//     Copyright [2019] Jian ZhiQiang
// */

// #include "Common.hpp"

// // 测试程序，测试矩形碰撞是否正确, 100000计算用时58ms
// int main(int argc, char** argv) {
//     // 初始化ros程序
//     ros::init(argc, argv, "motion_planning_node");
//     ros::NodeHandle nh("~");
//     // 初始化可视化节点
//     ros::Publisher visualization_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/motion_planning/debug/vis", 10);
//     // 初始化两个矩形
//     Rectangle rect1, rect2;
//     // rect1.center_x_ = -11333.8;
//     // rect1.center_y_ = -2621.72;
//     // rect1.width_ = 3.42;
//     // rect1.length_ = 4.64356;
//     // rect1.rotation_ = 0.449365;
//     // rect2.center_x_ = -11336.7;
//     // rect2.center_y_ = -2624.9;
//     // rect2.width_ = 2.1436;
//     // rect2.length_ = 5.6925;
//     // rect2.rotation_ = 0.623883;
//     rect1.center_x_ = 0.0;
//     rect1.center_y_ = 0.0;
//     rect1.width_ = 3.42;
//     rect1.length_ = 4.64356;
//     rect1.rotation_ = 0.0;
//     rect2.center_x_ = 1.0;
//     rect2.center_y_ = 2.0;
//     rect2.width_ = 0.01;
//     rect2.length_ = 5.6925;
//     rect2.rotation_ = 0.623883;
//     // 判断两个矩形是否相交，计算100000次看时间开销
//     bool result;
//     clock_t start_time, end_time;
//     start_time = clock();
//     for (size_t i = 0; i < 100000; i++) {
//         result = Tools::isRectangleOverlap(rect1, rect2, 1.0, 1.0);
//     }
//     end_time = clock();
//     std::cout << "is two rectangle collision: " << result << ", and time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;
//     // 可视化两个矩形
//     visualization_msgs::MarkerArray marker_array;
//     std_msgs::ColorRGBA color;
//     color.r = 1;
//     color.g = 0;
//     color.b = 0;
//     color.a = 1;
//     marker_array.markers.push_back(VisualizationMethods::visualizeRectToMarker(rect1.center_x_, rect1.center_y_, rect1.rotation_, rect1.width_, rect1.length_, 0.5, color, 0));
//     color.r = 0;
//     color.g = 0;
//     color.b = 1;
//     color.a = 1;
//     marker_array.markers.push_back(VisualizationMethods::visualizeRectToMarker(rect2.center_x_, rect2.center_y_, rect2.rotation_, rect2.width_, rect2.length_, 0.5, color, 1));
//     while (ros::ok()) {
//         visualization_pub_.publish(marker_array);
//     }
//     return 0;
// }
