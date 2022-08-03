/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

// 将道路转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeLaneToMarker(const Lane &lane, const std_msgs::ColorRGBA &color, int id) {
    std::vector<PathPlanningUtilities::CoordinationPoint> lane_center = lane.getLaneCoordnation();
    visualization_msgs::Marker lane_marker;
    lane_marker.header.frame_id = "world";
    lane_marker.header.stamp = ros::Time::now();
    lane_marker.type = visualization_msgs::Marker().LINE_LIST;
    lane_marker.color = color;
    lane_marker.id = id;
    geometry_msgs::Vector3 v3r;
    v3r.x = 0.02;
    lane_marker.scale = v3r;
    for (size_t i = 0; i < lane_center.size(); i++) {
        geometry_msgs::Point left_point;
        geometry_msgs::Point right_point;
        PathPlanningUtilities::CoordinationPoint coordination_point = lane_center[i];
        left_point.x = coordination_point.worldpos_.position_.x_ - sin(coordination_point.worldpos_.theta_) * std::fabs(coordination_point.max_height_);
        left_point.y = coordination_point.worldpos_.position_.y_ + cos(coordination_point.worldpos_.theta_) * std::fabs(coordination_point.max_height_);
        right_point.x = coordination_point.worldpos_.position_.x_ + sin(coordination_point.worldpos_.theta_) * std::fabs(coordination_point.min_height_);
        right_point.y = coordination_point.worldpos_.position_.y_ - cos(coordination_point.worldpos_.theta_) * std::fabs(coordination_point.min_height_);
        lane_marker.points.push_back(left_point);
        lane_marker.points.push_back(right_point);
    }
    return lane_marker;
}

// 将路径转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeCurvesToMarker(const PathPlanningUtilities::Curve &curve, const std_msgs::ColorRGBA &color, int id) {
    visualization_msgs::Marker curve_marker;
    curve_marker.header.frame_id = "world";
    curve_marker.header.stamp = ros::Time::now();
    curve_marker.type = visualization_msgs::Marker().LINE_STRIP;
    curve_marker.color = color;
    curve_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = 0.15;
    curve_marker.scale = v3c;
    for (size_t i = 0; i < curve.size(); i++) {
        PathPlanningUtilities::CurvePoint curve_point = curve[i];
        geometry_msgs::Point point;
        point.x = curve_point.position_.x_;
        point.y = curve_point.position_.y_;
        curve_marker.points.push_back(point);
    }
    return curve_marker;
}

// 将矩形框转为marker
visualization_msgs::Marker VisualizationMethods::visualizeRectToMarker(double position_x, double position_y, double theta, double width, double length, double center_scale, const std_msgs::ColorRGBA &color, int id) {
    visualization_msgs::Marker rect_marker;
    rect_marker.header.frame_id = "world";
    rect_marker.header.stamp = ros::Time::now();
    rect_marker.type = visualization_msgs::Marker().LINE_STRIP;
    rect_marker.color = color;
    rect_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = 0.1;
    rect_marker.scale = v3c;
    geometry_msgs::Point point_1, point_2, point_3, point_4;
    point_1.x = position_x + length*center_scale*cos(theta) - width/2.0*sin(theta);
    point_1.y = position_y + length*center_scale*sin(theta) + width/2.0*cos(theta);
    rect_marker.points.push_back(point_1);
    point_2.x = position_x + length*center_scale*cos(theta) + width/2.0*sin(theta);
    point_2.y = position_y + length*center_scale*sin(theta) - width/2.0*cos(theta);
    rect_marker.points.push_back(point_2);
    point_3.x = position_x - length*(1.0 - center_scale)*cos(theta) + width/2.0*sin(theta);
    point_3.y = position_y - length*(1.0 - center_scale)*sin(theta) - width/2.0*cos(theta);
    rect_marker.points.push_back(point_3);
    point_4.x = position_x - length*(1.0 - center_scale)*cos(theta) - width/2.0*sin(theta);
    point_4.y = position_y - length*(1.0 - center_scale)*sin(theta) + width/2.0*cos(theta);
    rect_marker.points.push_back(point_4);
    rect_marker.points.push_back(point_1);
    return rect_marker;
}

// 将文本转为为marker
visualization_msgs::Marker VisualizationMethods::visualizeStringToMarker(const std::string &text, double position_x, double position_y, const std_msgs::ColorRGBA &color, int id) {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.type = visualization_msgs::Marker().TEXT_VIEW_FACING;
    text_marker.color = color;
    text_marker.id = id;
    text_marker.pose.position.x = position_x;
    text_marker.pose.position.y = position_y;
    text_marker.pose.position.z = 0;
    text_marker.pose.orientation.w = 1;
    text_marker.scale.x = 0.8;
    text_marker.scale.y = 0.8;
    text_marker.scale.z = 0.8;
    text_marker.text = text;
    return text_marker;
}

// 删除可视化marker
visualization_msgs::Marker VisualizationMethods::visualizedeleteMarker(int id) {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETE;
    delete_marker.id = id;
    return delete_marker;
}

// 删除从起始id开始的全部可视化marker
visualization_msgs::Marker VisualizationMethods::visualizedeleteAllMarker(int start_id) {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.id = start_id;
    return delete_marker;
}

// 将状态机可视化在车辆规划位置附近
void VisualizationMethods::visualizeStates(const std::vector<DecisionMaking::StandardState> &state_set, const DecisionMaking::StandardState &choosed_state, double position_x, double position_y, int start_id, const ros::Publisher &publisher) {
    visualization_msgs::MarkerArray states_marker_array;

    // 发布新的marker array
    for (size_t i = 0; i < state_set.size(); i++) {
        visualization_msgs::MarkerArray state_marker;
        switch (i) {
            case DecisionMaking::StateNames::STOP:
            {
                // 给出颜色，红色代表选中，白色代表可行，灰色代表不可行
                std_msgs::ColorRGBA color;
                if (choosed_state.getStateName() == i) {
                    color.r = 1;
                    color.g = 0;
                    color.b = 0;
                    color.a = 1;
                } else {
                    if (state_set[i].getCapability() && state_set[i].getSafety()) {
                        color.r = 1;
                        color.g = 1;
                        color.b = 1;
                        color.a = 1;
                    } else {
                        color.r = 0.75;
                        color.g = 0.75;
                        color.b = 0.75;
                        color.a = 0.75;
                    }
                }
                states_marker_array.markers.push_back(visualizeStringToMarker("STOP", position_x, position_y + i, color, start_id + i));
                break;
            }
            case DecisionMaking::StateNames::TURN_LEFT:
            {
                // 给出颜色，红色代表选中，白色代表可行，灰色代表不可行
                std_msgs::ColorRGBA color;
                if (choosed_state.getStateName() == i) {
                    color.r = 1;
                    color.g = 0;
                    color.b = 0;
                    color.a = 1;
                } else {
                    if (state_set[i].getCapability() && state_set[i].getSafety()) {
                        color.r = 1;
                        color.g = 1;
                        color.b = 1;
                        color.a = 1;
                    }  else {
                        color.r = 0.75;
                        color.g = 0.75;
                        color.b = 0.75;
                        color.a = 0.75;
                    }
                }
                states_marker_array.markers.push_back(visualizeStringToMarker("TURN_LEFT", position_x, position_y + i, color, start_id + i));
                break;
            }
            case DecisionMaking::StateNames::TURN_RIGHT:
            {
                // 给出颜色，红色代表选中，白色代表可行，灰色代表不可行
                std_msgs::ColorRGBA color;
                if (choosed_state.getStateName() == i) {
                    color.r = 1;
                    color.g = 0;
                    color.b = 0;
                    color.a = 1;
                } else {
                    if (state_set[i].getCapability() && state_set[i].getSafety()) {
                        color.r = 1;
                        color.g = 1;
                        color.b = 1;
                        color.a = 1;
                    } else {
                        color.r = 0.75;
                        color.g = 0.75;
                        color.b = 0.75;
                        color.a = 0.75;
                    }
                }
                states_marker_array.markers.push_back(visualizeStringToMarker("TURN_RIGHT", position_x, position_y + i, color, start_id + i));
                break;
            }
            case DecisionMaking::StateNames::FORWARD:
            {
                // 给出颜色，红色代表选中，白色代表可行，灰色代表不可行
                std_msgs::ColorRGBA color;
                if (choosed_state.getStateName() == i) {
                    color.r = 1;
                    color.g = 0;
                    color.b = 0;
                    color.a = 1;
                } else {
                    if (state_set[i].getCapability() && state_set[i].getSafety()) {
                        color.r = 1;
                        color.g = 1;
                        color.b = 1;
                        color.a = 1;
                    } else {
                        color.r = 0.75;
                        color.g = 0.75;
                        color.b = 0.75;
                        color.a = 0.75;
                    }
                }
                states_marker_array.markers.push_back(visualizeStringToMarker("FORWARD", position_x, position_y + i, color, start_id + i));
                break;
            }
            case DecisionMaking::StateNames::AVOIDANCE:
            {
                // 给出颜色，红色代表选中，白色代表可行，灰色代表不可行
                std_msgs::ColorRGBA color;
                if (choosed_state.getStateName() == i) {
                    color.r = 1;
                    color.g = 0;
                    color.b = 0;
                    color.a = 1;
                } else {
                    if (state_set[i].getCapability() && state_set[i].getSafety()) {
                        color.r = 1;
                        color.g = 1;
                        color.b = 1;
                        color.a = 1;
                    } else {
                        color.r = 0.75;
                        color.g = 0.75;
                        color.b = 0.75;
                        color.a = 0.75;
                    }
                }
                states_marker_array.markers.push_back(visualizeStringToMarker("AVOIDANCE", position_x, position_y + i, color, start_id + i));
                break;
            }
            case DecisionMaking::StateNames::REVERSE:
            {
                // 给出颜色，红色代表选中，白色代表可行，灰色代表不可行
                std_msgs::ColorRGBA color;
                if (choosed_state.getStateName() == i) {
                    color.r = 1;
                    color.g = 0;
                    color.b = 0;
                    color.a = 1;
                } else {
                    if (state_set[i].getCapability() && state_set[i].getSafety()) {
                        color.r = 1;
                        color.g = 1;
                        color.b = 1;
                        color.a = 1;
                    } else {
                        color.r = 0.75;
                        color.g = 0.75;
                        color.b = 0.75;
                        color.a = 0.75;
                    }
                }
                states_marker_array.markers.push_back(visualizeStringToMarker("REVERSE", position_x, position_y + i, color, start_id + i));
                break;
            }
            default:
                break;
        }
    }
    publisher.publish(states_marker_array);
}

// 可视化选中的路径及车辆延路径行驶的边界
void VisualizationMethods::visualizeChoosedCurveWithBoundary(const PathPlanningUtilities::Curve &choosed_curve, double width, double length, double center_scale, int start_id, int gap, const ros::Publisher &publisher) {
    visualization_msgs::MarkerArray choosed_curve_with_boundary_marker_array;
    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    for (size_t i = 0; i < choosed_curve.size(); i+=gap) {
        choosed_curve_with_boundary_marker_array.markers.push_back(visualizeRectToMarker(choosed_curve[i].position_.x_, choosed_curve[i].position_.y_, Tools::centerYawToRearYaw(choosed_curve[i].theta_, choosed_curve[i].kappa_, DISTANCE_FROM_REAR_TO_CENTER), width, length, center_scale, color, start_id));
        start_id++;
        if (i + gap >= choosed_curve.size()) {
            choosed_curve_with_boundary_marker_array.markers.push_back(visualizeRectToMarker(choosed_curve.back().position_.x_,choosed_curve.back().position_.y_, Tools::centerYawToRearYaw(choosed_curve.back().theta_, choosed_curve.back().kappa_, DISTANCE_FROM_REAR_TO_CENTER), width, length, center_scale, color, start_id));
        }
    }
    publisher.publish(choosed_curve_with_boundary_marker_array);
}

// 可视化车辆状态信息
void VisualizationMethods::visualizeVehicleState(double position_x, double position_y, const std::string &velocity, const std::string &wheel_angle, const std::string &safety, const ros::Publisher &publisher) {
    visualization_msgs::MarkerArray vehicle_state_marker_array;
    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 0.4;
    color.b = 0;
    color.a = 1;
    vehicle_state_marker_array.markers.push_back(visualizeStringToMarker(velocity, position_x + 4, position_y + 6, color, VisualizationID::VEHICLE_INFO_VEL));
    vehicle_state_marker_array.markers.push_back(visualizeStringToMarker(wheel_angle, position_x + 4, position_y + 5, color, VisualizationID::VEHICLE_INFO_ANGLE));
    vehicle_state_marker_array.markers.push_back(visualizeStringToMarker(safety, position_x + 4, position_y + 4, color, VisualizationID::VEHICLE_INFO_SAFETY));
    publisher.publish(vehicle_state_marker_array);
}

// 可视化交通规则
void VisualizationMethods::visualizeTrafficRules(const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_obstacles, const ros::Publisher &publisher) {
    visualization_msgs::MarkerArray traffic_rule_marker_array;
    std_msgs::ColorRGBA color;
    for (size_t i = 0; i < traffic_obstacles.size(); i++) {
        // 交通规则线体
        if (traffic_obstacles[i].mode == vec_map_cpp_msgs::VirtualObstacle::PERMANENT || traffic_obstacles[i].mode == vec_map_cpp_msgs::VirtualObstacle::BEYOND_GOAL) {
            color.r = 0;
            color.g = 0;
            color.b = 1;
            color.a = 1;
        } else if (traffic_obstacles[i].mode == vec_map_cpp_msgs::VirtualObstacle::NOT_PERMANENT) {
            color.r = 1;
            color.g = 0.855;
            color.b = 0.565;
            color.a = 1.0;
        } else if (traffic_obstacles[i].mode == vec_map_cpp_msgs::VirtualObstacle::CONDITION_DECISION) {
            color.r = 1;
            color.g = 0.855;
            color.b = 0.565;
            color.a = 1.0;
        } else if (traffic_obstacles[i].mode == vec_map_cpp_msgs::VirtualObstacle::INVALID){
            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1;
        }
        visualization_msgs::Marker traffic_rule_marker;
        traffic_rule_marker.header.frame_id = "world";
        traffic_rule_marker.header.stamp = ros::Time::now();
        traffic_rule_marker.type = visualization_msgs::Marker().LINE_STRIP;
        traffic_rule_marker.color = color;
        traffic_rule_marker.id = VisualizationMethods::VisualizationID::TRAFFIC_RULE_START_ID + i;
        geometry_msgs::Vector3 v3c;
        v3c.x = 0.3;
        traffic_rule_marker.scale = v3c;
        for (size_t j = 0; j < traffic_obstacles[i].points.size(); j++) {
            geometry_msgs::Point point;
            point.x = traffic_obstacles[i].points[j].x;
            point.y = traffic_obstacles[i].points[j].y;
            traffic_rule_marker.points.push_back(point);
        }
        traffic_rule_marker_array.markers.push_back(traffic_rule_marker);
    }
    publisher.publish(traffic_rule_marker_array);
}

// 可视化本车占用区域
void VisualizationMethods::visualizeSubvehicleOccupationArea(const DecisionMaking::StandardState &choosed_state, double center_scale, const ros::Publisher &publisher, size_t gap) {
    visualization_msgs::MarkerArray occupation_marker_array;
    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    color.a = 1;
    size_t length = std::min(static_cast<size_t>(Tools::normalSubvehicleOccupationDistance(choosed_state.getExpectedVelocityCurrent(), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE), choosed_state.getTrajectoryLength() + choosed_state.getExtendedTrajectoryLength() - choosed_state.getVehicleCurrentPositionIndexInTrajectory());
    for (size_t i = choosed_state.getVehicleCurrentPositionIndexInTrajectory(); i < choosed_state.getVehicleCurrentPositionIndexInTrajectory() + length; i += gap) {
        PathPlanningUtilities::CurvePoint curve_point;
        if (i < choosed_state.getTrajectoryLength()) {
            curve_point = choosed_state.getTrajectory()[choosed_state.getChoosedTrajectoryIndex()][i];
        } else {
            curve_point = choosed_state.getExtendedTrajectory()[choosed_state.getChoosedTrajectoryIndex()][i - choosed_state.getTrajectoryLength()];
        }
        occupation_marker_array.markers.push_back(visualizeRectToMarker(curve_point.position_.x_, curve_point.position_.y_, Tools::centerYawToRearYaw(curve_point.theta_, curve_point.kappa_, DISTANCE_FROM_REAR_TO_CENTER), choosed_state.getVehicleWidth(), choosed_state.getVehicleLength(), center_scale, color, i + VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
    }
    publisher.publish(occupation_marker_array);
}

// 将障碍物形状转化为maker，障碍物类型为obstacle
visualization_msgs::Marker VisualizationMethods::visualizeObstacleShape(const DecisionMaking::Obstacle &obstacle, int id) {
    visualization_msgs::Marker obstacle_marker;
    obstacle_marker.header.frame_id = "world";
    obstacle_marker.header.stamp = ros::Time::now();
    obstacle_marker.type = visualization_msgs::Marker().CUBE;
    obstacle_marker.id = id;
    obstacle_marker.scale.x = std::max(obstacle.getObstacleLength(), 0.1);
    obstacle_marker.scale.y = std::max(obstacle.getObstacleWidth(), 0.1);
    obstacle_marker.scale.z = 2.0;
    obstacle_marker.pose.position.x = obstacle.getObstaclePosition().x_;
    obstacle_marker.pose.position.y = obstacle.getObstaclePosition().y_;
    obstacle_marker.pose.position.z = 0.0;
    obstacle_marker.pose.orientation.x = 0.0;
    obstacle_marker.pose.orientation.y = 0.0;
    obstacle_marker.pose.orientation.z = sin(obstacle.getObstacleOrientation()/2.0);
    obstacle_marker.pose.orientation.w = cos(obstacle.getObstacleOrientation()/2.0);
    if (Tools::isZero(obstacle.getObstacleVelocity())) {
        // 这里表示障碍物是静止障碍物
        std_msgs::ColorRGBA color;
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 0.8;
        obstacle_marker.color = color;
    } else {
        // 这里表示障碍物不是静止障碍物
        std_msgs::ColorRGBA color;
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 0.8;
        obstacle_marker.color = color;
    }
    return obstacle_marker;
}

// 将障碍物预测轨迹转化为marker组
std::vector<visualization_msgs::Marker> VisualizationMethods::visualizeObstacleTrajectory(const DecisionMaking::Obstacle &obstacle, int start_id) {
    std_msgs::ColorRGBA color;
    color.r = 0.4;
    color.g = 0.7;
    color.b = 0.2;
    color.a = 0.8;
    geometry_msgs::Vector3 v3c;
    v3c.x = 0.05;
    std::vector<visualization_msgs::Marker> obstacle_trajectory_markers;
    for (size_t i = 0; i < obstacle.getPredictedTrajectoryNumber(); i++) {
        visualization_msgs::Marker obstacle_trajectory_marker;
        obstacle_trajectory_marker.header.frame_id = "world";
        obstacle_trajectory_marker.header.stamp = ros::Time::now();
        obstacle_trajectory_marker.type = visualization_msgs::Marker().LINE_LIST;
        obstacle_trajectory_marker.color = color;
        obstacle_trajectory_marker.scale = v3c;
        obstacle_trajectory_marker.id = start_id + i;
        for (size_t j = 0; j < obstacle.getPredictedTrajectory(i).size(); j++) {
            PathPlanningUtilities::CurvePoint curve_point = obstacle.getPredictedTrajectory(i)[j];
            geometry_msgs::Point left_point;
            left_point.x = curve_point.position_.x_ - sin(curve_point.theta_) * 0.5 * obstacle.getObstacleOccupationWidth();
            left_point.y = curve_point.position_.y_ + cos(curve_point.theta_) * 0.5 * obstacle.getObstacleOccupationWidth();
            geometry_msgs::Point right_point;
            right_point.x = curve_point.position_.x_ + sin(curve_point.theta_) * 0.5 *  obstacle.getObstacleOccupationWidth();
            right_point.y = curve_point.position_.y_ - cos(curve_point.theta_) * 0.5 * obstacle.getObstacleOccupationWidth();
            obstacle_trajectory_marker.points.push_back(left_point);
            obstacle_trajectory_marker.points.push_back(right_point);
        }
        obstacle_trajectory_markers.push_back(obstacle_trajectory_marker);
    }
    return obstacle_trajectory_markers;
}

// 将障碍物速度转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeObstacleVelocity(const DecisionMaking::Obstacle &obstacle, int id) {
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 0;
    color.b = 1;
    color.a = 1;
    std::stringstream ss;
    ss << "velocity: " << obstacle.getObstacleVelocity() << std::endl;
    return visualizeStringToMarker(ss.str(), obstacle.getObstaclePosition().x_, obstacle.getObstaclePosition().y_, color, id);
}

// 将障碍物速度方向转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeObstacleVelocityDirection(const DecisionMaking::Obstacle &obstacle) {
    visualization_msgs::Marker velocity_direction_marker;
    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    color.a = 1;
    velocity_direction_marker.header.frame_id = "world";
    velocity_direction_marker.header.stamp = ros::Time::now();
    velocity_direction_marker.type = visualization_msgs::Marker::ARROW;
    velocity_direction_marker.id = VisualizationID::OBSTACLE_VELOCITY_DIRECTION_START_ID + obstacle.getID();
    velocity_direction_marker.color = color;
    velocity_direction_marker.pose.position.x = obstacle.getObstaclePosition().x_;
    velocity_direction_marker.pose.position.y = obstacle.getObstaclePosition().y_;
    velocity_direction_marker.pose.position.z = 2.0;
    velocity_direction_marker.pose.orientation.x = 0;
    velocity_direction_marker.pose.orientation.y = 0;
    velocity_direction_marker.pose.orientation.z = sin(obstacle.getObstacleVelocityDirection()/2.0);
    velocity_direction_marker.pose.orientation.w = cos(obstacle.getObstacleVelocityDirection()/2.0);
    return velocity_direction_marker;
}

// 将障碍物可视化
void VisualizationMethods::visualizeObstacles(const std::vector<DecisionMaking::Obstacle> &obstacles, const ros::Publisher &publisher) {
    // 首先清空之前的可视化
    visualization_msgs::MarkerArray delete_marker_array, obstacle_marker_array;
    delete_marker_array.markers.push_back(visualizedeleteAllMarker(0));
    publisher.publish(delete_marker_array);
    // 开始可视化
    int count = 0;
    // 首先是障碍物本身
    for (size_t i = 0; i < obstacles.size(); i++) {
        obstacle_marker_array.markers.push_back(visualizeObstacleShape(obstacles[i], count));
        count++;
    }
    // 然后是障碍物的速度
    for (size_t i = 0; i < obstacles.size(); i++) {
        obstacle_marker_array.markers.push_back(visualizeObstacleVelocity(obstacles[i], count));
        count++;
    }
    // 最后是障碍物轨迹
    for (size_t i = 0; i < obstacles.size(); i++) {
        std::vector<visualization_msgs::Marker> predicted_trajectory_markers = visualizeObstacleTrajectory(obstacles[i], count);
        count += predicted_trajectory_markers.size();
        obstacle_marker_array.markers.insert(obstacle_marker_array.markers.end(), predicted_trajectory_markers.begin(), predicted_trajectory_markers.end());
    }
    publisher.publish(obstacle_marker_array);
}

// 本车碰撞点可视化
void VisualizationMethods::visualizeInteractionPosition(std::vector<Rectangle> *rectangles, const ros::Publisher &publisher) {
    visualization_msgs::MarkerArray interaction_position_marker_array, delete_marker_array, interaction_center_marker_array;
    // 清空之前的可视化
    delete_marker_array.markers.push_back(visualizedeleteAllMarker(0));
    publisher.publish(delete_marker_array);
    // 开始可视化
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.75;
    color.b = 0.796;
    color.a = 1.0;
    for (size_t i = 0; i < rectangles->size(); i++) {
        interaction_position_marker_array.markers.push_back(visualizeRectToMarker((*rectangles)[i].center_x_, (*rectangles)[i].center_y_, (*rectangles)[i].rotation_,(*rectangles)[i].width_, (*rectangles)[i].length_, 0.5, color, i));
    }
    for (size_t i = 0; i < rectangles->size(); i++) {
        visualization_msgs::Marker interaction_center_marker;
        interaction_center_marker.header.frame_id = "world";
        interaction_center_marker.header.stamp = ros::Time::now();
        interaction_center_marker.type = visualization_msgs::Marker::CYLINDER;
        interaction_center_marker.id = i + rectangles->size();
        interaction_center_marker.color = color;
        interaction_center_marker.pose.position.x = (*rectangles)[i].center_x_;
        interaction_center_marker.pose.position.y = (*rectangles)[i].center_y_;
        interaction_center_marker.pose.position.z = 0.0;
        interaction_center_marker.pose.orientation.x = 0.0;
        interaction_center_marker.pose.orientation.y = 0.0;
        interaction_center_marker.pose.orientation.z = 0.0;
        interaction_center_marker.pose.orientation.w = 1.0;
        interaction_center_marker.scale.x = 0.5;
        interaction_center_marker.scale.y = 0.5;
        interaction_center_marker.scale.z = 1;
        interaction_center_marker_array.markers.push_back(interaction_center_marker);
    }
    publisher.publish(interaction_position_marker_array);
    publisher.publish(interaction_center_marker_array);
    // 清空rectangle
    std::vector<Rectangle>().swap(*rectangles);
    return;
}

// 多条轨迹可视化
void VisualizationMethods::visualizeMultiCurves(const std::vector<PathPlanningUtilities::Curve> &curves, const ros::Publisher &publisher) {
    visualization_msgs::MarkerArray curves_marker_array, delete_marker_array;
    // 清空之前的可视化
    delete_marker_array.markers.push_back(visualizedeleteAllMarker(0));
    publisher.publish(delete_marker_array);
    // 开始可视化
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0;
    color.a = 0.75;
    for (size_t i = 0; i < curves.size(); i++) {
        curves_marker_array.markers.push_back(visualizeCurvesToMarker(curves[i], color, i));
    }
    publisher.publish(curves_marker_array);
}

// 在特定位置生成圆球marker
visualization_msgs::Marker VisualizationMethods::visualizePosition(double position_x, double position_y, std_msgs::ColorRGBA color, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = id;
    marker.color = color;
    marker.pose.position.x = position_x;
    marker.pose.position.y = position_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    return marker;
}

// 可视化有效障碍物
void VisualizationMethods::visualizeInfluenceObstacles(const std::vector<InfluenceObstacle> &influence_obstacles, const ros::Publisher &publisher) {
    visualization_msgs::MarkerArray influence_obstacles_marker_array, delete_marker_array;
    // 清空之前的可视化
    delete_marker_array.markers.push_back(visualizedeleteAllMarker(0));
    publisher.publish(delete_marker_array);
    // 开始可视化
    std_msgs::ColorRGBA color_car;
    color_car.r = 0.66;
    color_car.g = 0.0;
    color_car.b = 1.0;
    color_car.a = 1.0;
    std_msgs::ColorRGBA color_obs;
    color_obs.r = 0.66;
    color_obs.g = 1.0;
    color_obs.b = 1.0;
    color_obs.a = 1.0;
    for (size_t i = 0; i < influence_obstacles.size(); i++) {
        // 第一步生成障碍物当前位置marker
        influence_obstacles_marker_array.markers.push_back(visualizePosition(influence_obstacles[i].getInfluenceObstacleCenterPosition().x_, influence_obstacles[i].getInfluenceObstacleCenterPosition().y_, color_obs, i * 4));
        // 第二步生成障碍物轨迹marker
        influence_obstacles_marker_array.markers.push_back(visualizeCurvesToMarker(influence_obstacles[i].getInfluenceObstaclePredictedPath(), color_obs, i * 4 + 1));
        // 第三步生成障碍物碰撞点marker
        influence_obstacles_marker_array.markers.push_back(visualizePosition(influence_obstacles[i].getInfluenceObstacleCollisionPosition().x_, influence_obstacles[i].getInfluenceObstacleCollisionPosition().y_, color_obs, i * 4 + 2));
        // 第四步生成本车碰撞点marker
        influence_obstacles_marker_array.markers.push_back(visualizePosition(influence_obstacles[i].getInfluenceSubvehicleCollisionPosition().x_, influence_obstacles[i].getInfluenceSubvehicleCollisionPosition().y_, color_car, i * 4 + 3));
    }
    publisher.publish(influence_obstacles_marker_array);
}
