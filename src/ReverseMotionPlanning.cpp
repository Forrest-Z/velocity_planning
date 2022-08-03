#include "Common.hpp"

// 生成倒车状态
int DecisionMaking::SubVehicle::generateReverseState() {
    // 倒车状态只有在停车的时候才能执行
    // 倒车状态只有在特定区域才执行(特殊标志位)
    // 倒车状态只生成直线向后的状态
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

    // 判断当前状态是否为停车状态,并且速度为0
    if (Tools::isZero(current_movement_state.velocity_) && this->current_state_.getStateName() == StateNames::STOP) {
        // 获取是否允许倒车标志为
        bool is_reverse_allowed = this->ROTATE_AND_REVERSE_ENABLE_FLAG_;
        if (!is_reverse_allowed) {
            LOG(INFO) << "不允许进行倒车规划";
            return -1;
        }
        LOG(INFO) << "进行倒车规划";
        // 首先进行朝向翻转
        PathPlanningUtilities::VehicleState reverse_init_odometry;
        reverse_init_odometry.position_ = current_point_in_world.position_;
        reverse_init_odometry.theta_ = PathPlanningUtilities::rectifyAngle(PI + current_point_in_world.theta_);
        reverse_init_odometry.kappa_ = 0.0;

        LOG(INFO) << "当期朝向" << current_point_in_world.theta_ << "反向朝向" << reverse_init_odometry.theta_;

        // 确定规划长度
        double planning_distance = 8.0;
        
        // 寻找规划终点
        PathPlanningUtilities::CurvePoint goal_point;
        goal_point.position_.x_ = reverse_init_odometry.position_.x_ + planning_distance * cos(reverse_init_odometry.theta_);
        goal_point.position_.y_ = reverse_init_odometry.position_.y_ + planning_distance * sin(reverse_init_odometry.theta_);
        goal_point.theta_ = reverse_init_odometry.theta_;
        goal_point.kappa_ = 0.0;

        // 开始进行路径规划
        // 用quintic polynomial方法进行路径生成(distance 是调整参数 TODO)
        PathPlanningUtilities::Curve trajectory_curve;
        PathPlanningUtilities::QParameters params;
        PathPlanningUtilities::QuinticSplinePlanning::getParametersByMileageConstraint(reverse_init_odometry, goal_point, planning_distance, 5, params);
        int point_number = planning_distance / (LANE_GAP_DISTANCE * 0.1);
        PathPlanningUtilities::pathGenerator* pg_ = new PathPlanningUtilities::pathGenerator(point_number);
        pg_->calcCurve(params, trajectory_curve);

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
        marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(trajectory_curve, color, 0));
        this->vis_multi_curve_pub_.publish(marker_array);

        // 得到路径,进行静态障碍物碰撞判断
        RSS::OccupationArea subvehicle_occupation_area = RSS::OccupationArea(trajectory_curve, this->vehicle_width_, this->vehicle_length_);
        // 最终交点
        size_t min_interact_index = trajectory_curve.size() - 1;
        for (auto obstacle: obstacles) {
            for (size_t predict_index = 0; predict_index < obstacle.getPredictedTrajectoryNumber(); predict_index++) {
                RSS::OccupationArea obstacle_occupation_area = RSS::OccupationArea(obstacle, predict_index, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE, false);
                // 得到交点
                size_t subvehicle_interact_index, obstacle_interact_index;
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    if (subvehicle_interact_index < min_interact_index) {
                        min_interact_index = subvehicle_interact_index;
                        LOG(INFO) << "与障碍物的交点为" << subvehicle_interact_index;
                    }
                }
            }
        }
        // 计算与静态障碍物的交点
        // 判断曲线与交通障碍物之间关系
        for (auto traffic_rule: this->traffic_rule_obstacles_) {
            RSS::OccupationArea obstacle_occupation_area = RSS::OccupationArea(traffic_rule, 1);
            // 得到交点
            size_t subvehicle_interact_index, obstacle_interact_index;
            if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                // 发生碰撞
                // 判断碰撞点
                if (subvehicle_interact_index < min_interact_index) {
                    min_interact_index = subvehicle_interact_index;
                    LOG(INFO) << "与交通障碍物的交点为" << subvehicle_interact_index;
                }
            }
        }

        LOG(INFO) << "最终与障碍物交点为" << min_interact_index;

        int cut_index = 2.5 / (LANE_GAP_DISTANCE * 0.1);

        // 计算倒车距离
        cut_index = std::max(0, std::min(cut_index, static_cast<int>(min_interact_index) - static_cast<int>(0.3 / (LANE_GAP_DISTANCE * 0.1))));

        if (cut_index < 100) {
            LOG(INFO) << "倒车距离小于10cm,放弃进行倒车";
            return -2;
        }

        // 开始重新构建停车状态
        // 填充停车状态
        // 停车状态为有效状态
        this->states_set_[StateNames::REVERSE].enable();
        // 提供车辆形状信息
        this->states_set_[StateNames::REVERSE].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
        // 提供起始点信息
        this->states_set_[StateNames::REVERSE].setVehicleStartState(current_point_in_world);
        this->states_set_[StateNames::REVERSE].setVehicleStartMovement(current_movement_state);
        // 确定速度上下限制,速度固定为0(无效值)
        this->states_set_[StateNames::REVERSE].setVelocityLimitation(current_movement_state.velocity_, 0.0);
        // 确定加速度上下限制,固定为0(无效值)
        this->states_set_[StateNames::REVERSE].setAccelerationLimitationMax(0.0);
        this->states_set_[StateNames::REVERSE].setAccelerationLimitationMin(0.0);
        // 确定速度期望值(无效值)
        this->states_set_[StateNames::REVERSE].setExpectedVelocity(0.0, 0.0);
        // 设置安全性(无效值)
        this->states_set_[StateNames::REVERSE].setSafety(true);
        // 设置优先级(TOFIX)(无效值)
        this->states_set_[StateNames::REVERSE].setPriority(0.0);
        // 设置路径信息(选取直行道路作为停车的路径)(TOFIX)
        std::vector<PathPlanningUtilities::Curve> curve_set;
        curve_set.push_back(trajectory_curve);
        this->states_set_[StateNames::REVERSE].setTrajectory(curve_set);
        // 设置延伸路径信息(选取直行道路作为停车的延伸路径)(无延伸路径)
        std::vector<PathPlanningUtilities::Curve> extended_curve_set;
        extended_curve_set.push_back(PathPlanningUtilities::Curve());
        this->states_set_[StateNames::REVERSE].setExtendedTrajectory(extended_curve_set);
        // 提供车辆当前速度信息
        this->states_set_[StateNames::REVERSE].setVehicleCurrentMovement(current_movement_state);
        // 提供车辆当前点在路径中的下标
        this->states_set_[StateNames::REVERSE].setVehicleCurrentPositionIndexInTrajectory(0);
        // 提供车辆目标点在路径中的下标(设置停车位置，是停车状态中最重要的环节)
        this->states_set_[StateNames::REVERSE].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(cut_index);
        // 提供车辆目标点速度
        this->states_set_[StateNames::REVERSE].setGoalVelocity(0.0);
        // 计算最大允许速度
        // 首先是初始值
        double max_allowable_velocity = 0.5;
        this->states_set_[StateNames::REVERSE].setAllowMaxVelocity(max_allowable_velocity);

        // 设置期望保持的加速度值（无效值）
        this->states_set_[StateNames::REVERSE].setVehicleDynamicPlanningExpectedAcceleration(0.0);
        // 设置是否进行保持
        this->states_set_[StateNames::REVERSE].setStateMaintain(true);

        LOG(INFO) << "倒车状态补充完毕，移动距离为" << static_cast<double>(cut_index * LANE_GAP_DISTANCE * 0.1) << "米";
        // 选中状态为停车状态
        this->choosed_state_ = this->states_set_[StateNames::REVERSE];
        return 0;
    } else {
        // 不满足进行停车状态的逻辑
        return -1;
    }
}