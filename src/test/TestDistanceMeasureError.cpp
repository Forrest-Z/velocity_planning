/*
    Copyright [2019] Jian ZhiQiang
*/

// #include "Common.hpp"

// int main(int argc, char** argv) {
//     double max_error = 0.0;
//     double max_error_velo = -1.0;
//     double velocity_interaction = 0.0;
//     while (velocity_interaction < 25.0) {
//         // 给定起点
//         PathPlanningUtilities::VehicleState start_point;
//         start_point.position_.x_ = 0.0;
//         start_point.position_.y_ = 0.0;
//         start_point.theta_ = 0.7;
//         start_point.kappa_ = 0.0;
//         // 给定起始速度
//         double start_velocity  = velocity_interaction;
//         // 计算规划距离
//         double plan_length = Tools::normalMotionDistance(start_velocity, 2.0, 15.0);
//         // 给定终点
//         PathPlanningUtilities::CurvePoint goal_point;
//         goal_point.position_.x_ = plan_length;
//         goal_point.position_.y_ = 1.0;
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
//         // 计算最大误差
//         double truth_distance = 0.0;
//         double current_max_error = 0.0;
//         double error_perception = 0.0;
//         for(size_t i = 0; i < trajectory_curve.size(); i++) {
//             double expected_distance = static_cast<double>(i) * LANE_GAP_DISTANCE;
//             if (i > 0) {
//                 truth_distance = truth_distance + PathPlanningUtilities::calcDistance(trajectory_curve[i].position_, trajectory_curve[i - 1].position_);
//             }
//             double error = std::fabs(expected_distance - truth_distance);
//             if (Tools::isLarge(error, current_max_error)) {
//                 // 如果大于最大误差
//                 current_max_error = error;
//                 error_perception = current_max_error / truth_distance;
//             }
//         }
//         if (Tools::isLarge(current_max_error, max_error)) {
//             max_error = current_max_error;
//             max_error_velo = velocity_interaction;
//         }
//         std::cout << "current max error" << current_max_error << ", error percent is " << error_perception << ", velocity is " << velocity_interaction << std::endl;
//         velocity_interaction = velocity_interaction + 1.0;
//     }
//     std::cout << "final max error is " << max_error << ", and final velocity is " << max_error_velo << std::endl;

// }
