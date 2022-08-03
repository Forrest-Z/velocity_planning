/*
    Copyright [2019] Jian ZhiQiang
*/

// #include "Common.hpp"

// int main(int argc, char** argv) {
//     double max_kappa = 0.0;
//     double max_kappa_vel;
//     double velocity_interaction = 0.0;
//     while (velocity_interaction < 25.0) {
//         // 给定起点
//         PathPlanningUtilities::VehicleState start_point;
//         start_point.position_.x_ = 0.0;
//         start_point.position_.y_ = 0.0;
//         start_point.theta_ = 0.0;
//         start_point.kappa_ = 0.0;
//         // 给定起始速度
//         double start_velocity  = velocity_interaction;
//         // 计算规划距离
//         double plan_length = Tools::normalMotionDistance(start_velocity, 2.0, 15.0);
//         std::cout << "plan length is " << plan_length << std::endl;
//         // 给定终点
//         PathPlanningUtilities::CurvePoint goal_point;
//         goal_point.position_.x_ = plan_length;
//         goal_point.position_.y_ = 3.5;
//         goal_point.theta_ = 0.0;
//         goal_point.kappa_ = 0.0;
//         // 进行规划
//         PathPlanningUtilities::Curve trajectory_curve;
//         PathPlanningUtilities::QParameters params;
//         double distance = PathPlanningUtilities::calcDistance(start_point.position_, goal_point.position_);
//         PathPlanningUtilities::QuinticSplinePlanning::getParametersByMileageConstraint(start_point, goal_point, distance, 5, params);
//         int point_number = distance/LANE_GAP_DISTANCE;
//         PathPlanningUtilities::pathGenerator* pg_ = new PathPlanningUtilities::pathGenerator(point_number);
//         pg_->calcCurve(params, trajectory_curve);
//         // 计算最大曲率
//         double current_max_kappa = 0.0;
//         for(size_t i = 0; i < trajectory_curve.size(); i++) {
//             double kappa = trajectory_curve[i].kappa_;
//             if (Tools::isLarge(std::fabs(kappa), std::fabs(current_max_kappa))) {
//                 // 如果大于最大曲率
//                 current_max_kappa = kappa;
//             }
//         }
//         if (Tools::isLarge(std::fabs(current_max_kappa), std::fabs(max_kappa))) {
//             max_kappa = current_max_kappa;
//             max_kappa_vel = velocity_interaction;
//         }
//         std::cout << "current max kappa " << current_max_kappa << ", velocity is " << velocity_interaction << std::endl;
//         double steering_angle = atan((2.8498 + 0.007 * velocity_interaction * velocity_interaction) * current_max_kappa) * 14.8 / 3.14 * 180.0;
//         std::cout << "steering angle is " << steering_angle << " degree" << std::endl;
//         velocity_interaction = velocity_interaction + 1.0;
//     }
//     std::cout << "final max kappa is " << max_kappa << ", and final velocity is " << max_kappa_vel << std::endl;

// }
