/*
    Copyright [2019] Jian ZhiQiang
*/

// #include "Common.hpp"

// int main(int argc, char** argv) {
//     path_planning_msgs::PathPoint p1, p2, p3, p4;
//     // p1.x = 1;
//     // p1.y = 4;
//     // p2.x = 3;
//     // p2.y = 0;
//     // p3.x = 0;
//     // p3.y = 1;
//     // p4.x = 4;
//     // p4.y = 3;
//     p1.x = 0;
//     p1.y = 0;
//     p2.x = 3;
//     p2.y = 3;
//     p3.x = 2;
//     p3.y = 2;
//     p4.x = 4;
//     p4.y = 4;
//     // p1.x = -11286.7;
//     // p1.y = -2602.99;
//     // p2.x = -11286.6;
//     // p2.y = -2602.95;
//     // p3.x = -11250.6;
//     // p3.y = -2583.73;
//     // p4.x = -11166.6;
//     // p4.y = -2543.2;
//     bool flag = false;
//     flag = Tools::isLineSegmentInteracted(p1, p2, p3, p4);
//     std::cout << "两条线段相交结果为: " << flag << std::endl;
//     // 初始化ros程序
//     ros::init(argc, argv, "motion_planning_node");
//     ros::NodeHandle nh("~");
//     // 初始化可视化节点
//     ros::Publisher visualization_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/motion_planning/debug/vis", 10);
//     // 可视化两个线段
//     visualization_msgs::MarkerArray marker_array;
//     visualization_msgs::Marker marker;
//     std_msgs::ColorRGBA color;
//     color.r = 1;
//     color.g = 0;
//     color.b = 0;
//     color.a = 1;
//     marker.header.frame_id = "world";
//     marker.header.stamp = ros::Time::now();
//     marker.type = visualization_msgs::Marker().LINE_LIST;
//     marker.color = color;
//     marker.id = 0;
//     geometry_msgs::Vector3 v3r;
//     v3r.x = 0.02;
//     marker.scale = v3r;
//     geometry_msgs::Point point1, point2, point3, point4;
//     point1.x = p1.x;
//     point1.y = p1.y;
//     point2.x = p2.x;
//     point2.y = p2.y;
//     point3.x = p3.x;
//     point3.y = p3.y;
//     point4.x = p4.x;
//     point4.y = p4.y;
//     marker.points.push_back(point1);
//     marker.points.push_back(point2);
//     marker.points.push_back(point3);
//     marker.points.push_back(point4);
//     marker_array.markers.push_back(marker);
//     while (ros::ok()) {
//         visualization_pub_.publish(marker_array);
//     }
//     return 0;
// }
