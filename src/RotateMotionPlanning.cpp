#include "Common.hpp"

// 生成转向状态
int DecisionMaking::SubVehicle::generateRotateState() {
    // 转向状态只有在停车时才能进行
    // 转向状态生成圆弧路径
    // 转向状态需要在原地打方向盘
    // 转向状态完成后,需要将方向盘回正
    // 得到当前的位置
    PathPlanningUtilities::VehicleState current_point_in_world;
    this->current_vehicle_world_position_mutex_.lock();
    current_point_in_world = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    // 得到当前速度
    PathPlanningUtilities::VehicleMovementState current_movement_state;
    this->current_vehicle_movement_mutex_.lock();
    current_movement_state = this->current_vehicle_movement_;
    this->current_vehicle_movement_mutex_.unlock();
    // 得到当前障碍物情况
    this->obstacle_mutex_.lock();
    std::vector<Obstacle> obstacles = this->obstacles_;
    this->obstacle_mutex_.unlock();

    // 判断当前速度是否为0
    if (Tools::isZero(current_movement_state.velocity_) && this->current_state_.getStateName() == StateNames::STOP) {
        // 可以进行转向规划
        // 获取是否允许转向标志为
        bool is_rotate_allowed = this->ROTATE_AND_REVERSE_ENABLE_FLAG_;
        if (!is_rotate_allowed) {
            LOG(INFO) << "不允许进行转向规划";
            return -1;
        } else {
            LOG(INFO) << "尝试进行转向";
        }
        // 确定转向的曲率
        std::vector<double> rotate_curvatures = {-MAX_CURVATURE, MAX_CURVATURE};
        std::vector<double> enable_rotates;
        std::vector<PathPlanningUtilities::Curve> arc_curves;
        // 进行曲线的生成
        double gap = 0.1;
        for (auto rotate_curvature: rotate_curvatures) {
            // 首先给出曲率对应的新朝向角
            double rear_yaw = Tools::centerYawToRearYaw(current_point_in_world.theta_, current_point_in_world.kappa_, DISTANCE_FROM_REAR_TO_CENTER);
            // 计算对应的后轴中心曲率
            double rear_curvature = Tools::centerToRearCurvature(rotate_curvature, DISTANCE_FROM_REAR_TO_CENTER);
            // 计算新曲率对应的几何中心角
            double new_center_yaw = Tools::rearYawToCenterYaw(rear_yaw, rear_curvature, DISTANCE_FROM_REAR_TO_CENTER);
            // 根据给出的曲率生成圆弧
            PathPlanningUtilities::Curve arc_curve;
            
            // 计算最大可转角度
            // 得到当前定位
            this->current_vehicle_world_position_mutex_.lock();
            PathPlanningUtilities::VehicleState start_point_in_world = this->current_vehicle_world_position_;
            this->current_vehicle_world_position_mutex_.unlock();
            // 得到当前道路
            std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = this->center_lane_.getLaneCoordnation();
            size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, start_point_in_world.position_);
            // 得到当前车位朝向
            double expected_yaw = this->center_lane_.getLaneCoordnation()[start_index_of_lane].worldpos_.theta_;

            LOG(INFO) << "车位朝向" << expected_yaw << ", 当前朝向" << start_point_in_world.theta_;

            // 得到期望转动角度
            double max_rotate = PI / 4.0;
            // 判断转向角
            if (!Tools::isSmall(std::abs(PathPlanningUtilities::rectifyAngle(expected_yaw - start_point_in_world.theta_)), max_rotate)) {
                LOG(INFO) << "当前朝向与道路朝向差过大,不允许转向";
                return -2;
            }
            // 计算可转动角度
            double enable_rotate;
            if (Tools::isLarge(rotate_curvature, 0.0)) {
                enable_rotate = std::min(std::abs(PathPlanningUtilities::rectifyAngle(expected_yaw + max_rotate - start_point_in_world.theta_)), max_rotate);
            } else {
                enable_rotate = std::min(std::abs(PathPlanningUtilities::rectifyAngle(expected_yaw - max_rotate - start_point_in_world.theta_)), max_rotate);
            }
            LOG(INFO) << "最大可转动角度为" << enable_rotate;

            enable_rotate = enable_rotate / MAX_CURVATURE;
            if (Tools::isSmall(enable_rotate, 0.5)) {
                return -2;
            }
            enable_rotates.push_back(enable_rotate);

            max_rotate = std::min(max_rotate / MAX_CURVATURE, 1.5 * enable_rotate);

            PathPlanningUtilities::CurvePoint curve_point;
            curve_point.position_ = current_point_in_world.position_;
            curve_point.theta_ = new_center_yaw;
            curve_point.kappa_ = rotate_curvature;
            LOG(INFO) << "转向初始位置为" << curve_point;
            LOG(INFO) << "后轴朝向为" << rear_yaw << ", 后轴曲率为" << rear_curvature;
            for (size_t i = 0; i < static_cast<size_t>(max_rotate / gap); i++) {
                curve_point.position_.x_ += gap * cos(curve_point.theta_);
                curve_point.position_.y_ += gap * sin(curve_point.theta_);
                curve_point.theta_ += rotate_curvature * gap;
                arc_curve.push_back(curve_point);
            }
            arc_curves.push_back(arc_curve);
        }

        if (arc_curves.size() == 0) {
            return -2;
        }

        // 进行可视化
        // 可视化路径
        // 清空之前的可视化
        visualization_msgs::MarkerArray delete_marker_array;
        delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
        this->vis_multi_curve_pub_.publish(delete_marker_array);
        visualization_msgs::MarkerArray marker_array;
        std_msgs::ColorRGBA color;
        color.r = 138.0 / 255.0;
        color.g = 43.0 / 255.0;
        color.b = 226.0 / 255.0;
        color.a = 1.0;
        int hhh = 0;
        for (auto arc_curve: arc_curves) {
            marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(arc_curve, color, hhh));
            hhh++;
        }
        this->vis_multi_curve_pub_.publish(marker_array);

        std::vector<PathPlanningUtilities::Curve> no_collision_curves;
        for (auto curve: arc_curves) {
            bool is_collision_with_obstacle = false;
            bool is_collision_with_traffic = false;
            // 判断曲线与障碍物之间的关系
            RSS::OccupationArea subvehicle_occupation_area = RSS::OccupationArea(curve, this->vehicle_width_, this->vehicle_length_, 1);
            for (auto obstacle: obstacles) {
                for (size_t predict_index = 0; predict_index < obstacle.getPredictedTrajectoryNumber(); predict_index++) {
                    RSS::OccupationArea obstacle_occupation_area = RSS::OccupationArea(obstacle, predict_index, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE, false);
                    LOG(INFO) << "障碍物占用区为" << obstacle_occupation_area.getOccupationArea()[0].length_ << ", " << obstacle_occupation_area.getOccupationArea()[0].width_;
                    // 得到交点
                    size_t subvehicle_interact_index, obstacle_interact_index;
                    if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                        // 发生碰撞
                        is_collision_with_obstacle = true;
                        break;
                    } else {
                        // 不发生碰撞
                    }
                }
                // 如果发生碰撞,则退出
                if (is_collision_with_obstacle) {
                    break;
                }
            }
            // 判断曲线与交通障碍物之间关系
            for (auto traffic_rule: this->traffic_rule_obstacles_) {
                RSS::OccupationArea obstacle_occupation_area = RSS::OccupationArea(traffic_rule, 1);
                // 得到交点
                size_t subvehicle_interact_index, obstacle_interact_index;
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    // 发生碰撞
                    is_collision_with_traffic = true;
                    break;
                } else {
                    // 未发生碰撞
                }
            }

            // 判断路径是否发生碰撞
            if (!is_collision_with_obstacle && !is_collision_with_traffic) {
                // 路径没有发生碰撞
                no_collision_curves.push_back(curve);
                LOG(INFO) << "转向曲率" << curve[0].kappa_ << "不碰撞";
            } else {
                LOG(INFO) << "转向曲率" << curve[0].kappa_ << "会发生碰撞";
                if (is_collision_with_obstacle) {
                    LOG(INFO) << "障碍物碰撞";
                } else {
                    LOG(INFO) << "交通碰撞";
                }
            }
        }

        if (no_collision_curves.size() == 0) {
            LOG(INFO) << "转向路径全部发生碰撞";
            return -2;
        }

        LOG(INFO) << "未发生碰撞转向路径数量" << no_collision_curves.size();
        // 进行优先级排序
        size_t best_index = 0;
        for (size_t i = 0; i < no_collision_curves.size(); i++) {
            if (Tools::isSmall(std::abs(no_collision_curves[i][0].kappa_), std::abs(no_collision_curves[best_index][0].kappa_))) {
                best_index = i;
            }
        }

        LOG(INFO) << "最优转向曲率为" << no_collision_curves[best_index][0].kappa_;

        // 得到最终路径(是无碰撞路的一半长度)
        PathPlanningUtilities::Curve final_rotate_curve = no_collision_curves[0];

        // 提供车辆目标点在路径中的下标(设置停车位置，是停车状态中最重要的环节)
        size_t cut_index = std::min(final_rotate_curve.size() * 2 / 3, static_cast<size_t>(0.9 * enable_rotates[best_index] / gap));

        if (cut_index < 5) {
            LOG(INFO) << "转向过小,不进行转向动作";
            return -3;
        }

        // 可视化选中路径
        visualization_msgs::MarkerArray reverse_curve_marker_array;
        std_msgs::ColorRGBA reverse_curve_visualization_color;
        reverse_curve_visualization_color.r = 0.0;
        reverse_curve_visualization_color.g = 0.0;
        reverse_curve_visualization_color.b = 1.0;
        reverse_curve_visualization_color.a = 1.0;
        reverse_curve_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(final_rotate_curve, reverse_curve_visualization_color, VisualizationMethods::VisualizationID::REVERSE_ID));
        this->visualization_pub_.publish(reverse_curve_marker_array);
        // 开始重新构建转向状态
        // 填充转向状态
        // 转向状态为有效状态
        this->states_set_[StateNames::ROTATE].enable();
        // 提供车辆形状信息
        this->states_set_[StateNames::ROTATE].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
        // 提供起始点信息
        this->states_set_[StateNames::ROTATE].setVehicleStartState(current_point_in_world);
        this->states_set_[StateNames::ROTATE].setVehicleStartMovement(current_movement_state);
        // 确定速度上下限制,速度固定为0(无效值)
        this->states_set_[StateNames::ROTATE].setVelocityLimitation(current_movement_state.velocity_, 0.0);
        // 确定加速度上下限制,固定为0(无效值)
        this->states_set_[StateNames::ROTATE].setAccelerationLimitationMax(0.0);
        this->states_set_[StateNames::ROTATE].setAccelerationLimitationMin(0.0);
        // 确定速度期望值(无效值)
        this->states_set_[StateNames::ROTATE].setExpectedVelocity(0.0, 0.0);
        // 设置安全性(无效值)
        this->states_set_[StateNames::ROTATE].setSafety(true);
        // 设置优先级(TOFIX)(无效值)
        this->states_set_[StateNames::ROTATE].setPriority(0.0);
        // 设置路径信息(选取直行道路作为停车的路径)(TOFIX)
        std::vector<PathPlanningUtilities::Curve> curve_set;
        curve_set.push_back(final_rotate_curve);
        this->states_set_[StateNames::ROTATE].setTrajectory(curve_set);
        // 设置延伸路径信息(选取直行道路作为停车的延伸路径)(无延伸路径)
        std::vector<PathPlanningUtilities::Curve> extended_curve_set;
        extended_curve_set.push_back(PathPlanningUtilities::Curve());
        this->states_set_[StateNames::ROTATE].setExtendedTrajectory(extended_curve_set);
        // 提供车辆当前速度信息
        this->states_set_[StateNames::ROTATE].setVehicleCurrentMovement(current_movement_state);
        // 提供车辆当前点在路径中的下标
        this->states_set_[StateNames::ROTATE].setVehicleCurrentPositionIndexInTrajectory(0);
        this->states_set_[StateNames::ROTATE].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(cut_index);
        // 提供车辆目标点速度
        this->states_set_[StateNames::ROTATE].setGoalVelocity(0.0);
        // 计算最大允许速度
        // 首先是初始值
        double max_allowable_velocity = 0.5;
        this->states_set_[StateNames::ROTATE].setAllowMaxVelocity(max_allowable_velocity);

        // 设置期望保持的加速度值（无效值）
        this->states_set_[StateNames::ROTATE].setVehicleDynamicPlanningExpectedAcceleration(0.0);
        // 设置是否进行保持
        this->states_set_[StateNames::ROTATE].setStateMaintain(true);

        LOG(INFO) << "转向状态补充完毕，转向曲率为" <<final_rotate_curve[0].kappa_ << ", 转向长度为" << cut_index * LANE_GAP_DISTANCE;
        // 选中状态为停车状态
        this->choosed_state_ = this->states_set_[StateNames::ROTATE];
        return 0;

    } else {
        return -1;
    }
}