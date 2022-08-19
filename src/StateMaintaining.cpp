#include "Common.hpp"

extern std::vector<Rectangle> collision_rectangles;

// 进行车辆和障碍物状态更新，判断状态是否完成，判断当前状态是否安全，判断期望状态是否可行，进行动态速度规划。
void DecisionMaking::SubVehicle::maintainStates() {
    // 1. 更新当前的位置和速度信息，更新障碍物信息
    // 2. 判断是否要开始重新路径规划。
    // 3. 判断是否要开始重新速度规划。
    // 4. 判断当前状态是否安全。
    // 5. 更新期望状态并判断期望状态是否可行。
    ros::Rate loop_rate(STATE_MAINTAIN_FREQUENCY);
    int re_dynamic_motion_planning_count = 0;
    while (ros::ok()) {
        clock_t start_time, end_time;
        loop_rate.sleep();
        start_time = clock();
        LOG(INFO) << "----------------------------- one turn of state maintain start --------------------------------";
        this->emergency_break_flag_mutex_.lock();
        bool check_emergency_break = this->IS_EMERGENCY_BREAK_FLAG_;
        this->emergency_break_flag_mutex_.unlock();
        if (check_emergency_break) {
            return;
        }
        // 开启转向灯
        // if (this->choosed_state_.getRespondingLane().getTurn() == vec_map_cpp_msgs::LocalLane::LEFT) {
        //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
        //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::LEFT;
        //     this->turn_signal_pub_.publish(turn_signal_cmd);
        // } else if (this->choosed_state_.getRespondingLane().getTurn() == vec_map_cpp_msgs::LocalLane::RIGHT) {
        //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
        //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::RIGHT;
        //     this->turn_signal_pub_.publish(turn_signal_cmd);
        // } else {
        if (this->choosed_state_.getStateName() == StateNames::TURN_LEFT) {
            dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::LEFT;
            this->turn_signal_pub_.publish(turn_signal_cmd);
        }else if (this->choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
            dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::RIGHT;
            this->turn_signal_pub_.publish(turn_signal_cmd);
        } else if (this->choosed_state_.getStateName() == StateNames::FORWARD) {
            // if (this->guidance_type_ == vec_map_cpp_msgs::GetGuidedCurvesResponse::CHANGE_LEFT) {
            //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::LEFT;
            //     this->turn_signal_pub_.publish(turn_signal_cmd);
            // } else if (this->guidance_type_ == vec_map_cpp_msgs::GetGuidedCurvesResponse::CHANGE_RIGHT) {
            //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::RIGHT;
            //     this->turn_signal_pub_.publish(turn_signal_cmd);
            // } else {
            //     // // 曲率大也要打灯
            //     // if (Tools::isSmall(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()])].kappa_, -0.1)) {
            //     //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            //     //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::RIGHT;
            //     //     this->turn_signal_pub_.publish(turn_signal_cmd);
            //     //     std::cout << "右打灯" << std::endl;
            //     // } else if (Tools::isLarge(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()])].kappa_, 0.1)) {
            //     //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            //     //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::LEFT;
            //     //     this->turn_signal_pub_.publish(turn_signal_cmd);
            //     //     std::cout << "左打灯" << std::endl;
            //     // }
            // }
        }
        // }

        // 更新障碍物信息
        this->updateValidateTrafficRuleInformation();
        this->updateObstacleInformation();
        // 得到当前的障碍物信息
        this->obstacle_mutex_.lock();
        std::vector<Obstacle> obstacles = this->obstacles_;
        this->obstacle_mutex_.unlock();

        // 1. 更新选中状态的信息
        // 获取世界坐标系下车辆的信息
        PathPlanningUtilities::VehicleState current_position_in_world;
        this->current_vehicle_world_position_mutex_.lock();
        current_position_in_world = this->current_vehicle_world_position_;
        this->current_vehicle_world_position_mutex_.unlock();
        PathPlanningUtilities::VehicleMovementState current_movement_state;
        this->current_vehicle_movement_mutex_.lock();
        current_movement_state = this->current_vehicle_movement_;
        this->current_vehicle_movement_mutex_.unlock();
        this->current_vehicle_kappa_mutex_.lock();
        double current_vehicle_kappa = this->current_vehicle_kappa_;
        current_position_in_world.kappa_ = current_vehicle_kappa;
        this->current_vehicle_kappa_mutex_.unlock();

        // 得到当前位置路径下标和速度更新
        size_t current_position_index_in_trajectory = Tools::findNearestPositionIndexInCurve(this->choosed_state_.getTotalTrajectory(), current_position_in_world.position_, this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory());
        // 更新选中状态的信息
        // 当前速度信息
        this->choosed_state_.setVehicleCurrentMovement(current_movement_state);
        // 当前位置信息
        this->choosed_state_.setVehicleCurrentPositionIndexInTrajectory(current_position_index_in_trajectory);
        // 当前状态的速度期望
        Lane choosed_lane;
        if (this->choosed_state_.getStateName() == StateNames::FORWARD) {
            choosed_lane = this->center_lane_;
        } else if (this->choosed_state_.getStateName() == StateNames::TURN_LEFT) {
            choosed_lane = this->left_lane_;
        } else if (this->choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
            choosed_lane = this->right_lane_;
        } else {
            std::cout << "[Error] choosed lane name error" << std::endl;
            LOG(INFO) << "[Error] choosed lane name error";
            exit(0);
        }
        // 得到当前位置在道路上的下标
        size_t current_position_index_in_choosed_lane = choosed_lane.findCurrenPositionIndexInLane(current_position_in_world.position_.x_, current_position_in_world.position_.y_);
        // 更新当前位置速度上限
        // 判断当前位置道路限速是否发生异变，如果发生，立刻进行重新路径规划
        if (this->expected_velocity_upper_bound_ != choosed_lane.getLaneVelocityLimitation()[current_position_index_in_choosed_lane]) {
            LOG(INFO) << "道路速度上限发生改变，由" << choosed_lane.getLaneVelocityLimitation()[current_position_index_in_choosed_lane] << "变化为了" << this->expected_velocity_upper_bound_ << "立刻进行重新运动规划";
            // this->current_state_ = this->choosed_state_;
            // this->current_state_.setStateCompleted(false);
            // return;
            this->expected_velocity_upper_bound_ = choosed_lane.getLaneVelocityLimitation()[current_position_index_in_choosed_lane];
            re_dynamic_motion_planning_count = static_cast<int>(STATE_MAINTAIN_FREQUENCY / VELOCITY_UPDATE_FREQUENCY_IN_STATE_MAINTAIN);
        }
        // 得到未来下标
        size_t future_position_index_in_choosed_lane = current_position_index_in_choosed_lane + static_cast<size_t>(Tools::normalForeseenDistance(std::max(current_movement_state.velocity_, this->expected_velocity_upper_bound_), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
        std::cout << "当前道路下标为" << current_position_index_in_choosed_lane << "未来道路下标为" << future_position_index_in_choosed_lane << std::endl;
        // assert(future_position_index_in_choosed_lane < choosed_lane.getLaneVelocityLimitation().size());
        // 得到当前下标到未来下标中最低速度限制
        double new_speed_limitation = choosed_lane.getLaneVelocityLimitation()[current_position_index_in_choosed_lane];
        for (size_t i = current_position_index_in_choosed_lane; i < std::min(future_position_index_in_choosed_lane, choosed_lane.getLaneVelocityLimitation().size()); i++) {
            double velocity_limitation = choosed_lane.getLaneVelocityLimitation()[i];
            if (Tools::isSmall(velocity_limitation, new_speed_limitation)) {
                new_speed_limitation = velocity_limitation;
            }
        }
        double new_lowest_speed = std::min(choosed_lane.getLowVelocity()[current_position_index_in_choosed_lane], choosed_lane.getLowVelocity()[future_position_index_in_choosed_lane]);
        // 如果给出的速度上限过小，会报错
        if (!Tools::isLarge(new_speed_limitation, new_lowest_speed)) {
            std::cout << "[Error] speed limitation is too small" << new_speed_limitation << "||" << new_lowest_speed << std::endl;
            LOG(INFO) << "[Error] speed limitation is too small";
            LOG(INFO) << "发生错误的坐标为(" << current_position_in_world.position_.x_ << ", " << current_position_in_world.position_.y_ << "), 错误的速度上限为" << new_speed_limitation;
            exit(0);
        }
        // 更新当前选中状态道路的速度期望
        this->choosed_state_.setExpectedVelocity(choosed_lane.getLaneVelocityLimitation()[current_position_index_in_choosed_lane], new_speed_limitation);
        // 更新车辆速度限制
        // 确定车辆速度限制，车辆速度上限由当前速度、横向加速度上限、横向加加速度上限决定
        double subvehicle_max_velocity = std::min(this->choosed_state_.getVehicleStartMovement().velocity_ + VELOCITY_MAX_INCREASE_VALUE, new_speed_limitation);
        size_t foreseen_length = static_cast<size_t>(Tools::normalSubvehicleOccupationDistance(this->choosed_state_.getExpectedVelocityCurrent(), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
        // 计算横向加速度限速
        double state_max_kappa = this->calcMaxKappaForState(this->choosed_state_, foreseen_length);
        double state_curvature_velocity_limtation = Tools::calcVelocityForMaxNormalAcceleration(state_max_kappa);
        // 计算横向加加速度限速
        double state_max_curvature_change_rate = calcMaxCurvatureChangeRateForState(this->choosed_state_, foreseen_length);
        double state_curvature_change_rate_velocity_limtation = Tools::calcVelocityForMaxNormalJerk(state_max_curvature_change_rate);
        // 得到最终限速
        subvehicle_max_velocity = std::min(std::min(subvehicle_max_velocity, state_curvature_velocity_limtation), state_curvature_change_rate_velocity_limtation);
        // 设置限速
        this->choosed_state_.setVelocityLimitation(subvehicle_max_velocity, std::min(new_lowest_speed, 0.9 * subvehicle_max_velocity));
        // 更新车辆加速度限制
        // 确定车辆的加速度上下限制，加速度上限由横向加加速度上限决定
        double subvehicle_max_acceleration, subvehicle_min_acceleration;
        subvehicle_max_acceleration = std::min(Tools::velocityToAccelerationReflection(current_movement_state.velocity_), Tools::calcAccelerationForMaxNormalJerk(state_max_kappa, subvehicle_max_velocity));
        subvehicle_min_acceleration = -COMMON_DECCELERATION;
        this->choosed_state_.setAccelerationLimitationMax(subvehicle_max_acceleration);
        this->choosed_state_.setAccelerationLimitationMin(subvehicle_min_acceleration);
        // 记录日志信息
        LOG(INFO) << "车辆到达当前路径的第" << current_position_index_in_trajectory << "个点，速度期望点为路径上第" << future_position_index_in_choosed_lane << "个点，速度上界限制为"<< subvehicle_max_velocity << "米/秒，当前速度为"<< current_movement_state.velocity_ <<"米/秒，路径的终点为" << this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].size();
        LOG(INFO) << "此时速度期望为" << new_speed_limitation << "米/秒";
        LOG(INFO) << "此时加速度限制为" << subvehicle_min_acceleration << "到" << subvehicle_max_acceleration;
        // // 2. 更新车辆状态信息可视化
        // // 删除之前状态显示
        // visualization_msgs::MarkerArray delete_marker_array;
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_VEL));
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_ANGLE));
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_SAFETY));
        // this->visualization_pub_.publish(delete_marker_array);
        // // 构造可视化状态字符串
        // std::stringstream ss_velocity;
        // ss_velocity << "velocity: " << current_movement_state.velocity_ << " m/s";
        // std::stringstream ss_wheel_angle;
        // ss_wheel_angle << "steering angle: " << atan(current_vehicle_kappa * (2.8498 + 0.007 * current_movement_state.velocity_ * current_movement_state.velocity_)) * 14.8 / PI * 180.0 << " deg";
        // std::stringstream ss_safety;
        // ss_safety << "safety: " << this->choosed_state_.getSafety();
        // VisualizationMethods::visualizeVehicleState(current_position_in_world.position_.x_, current_position_in_world.position_.y_, ss_velocity.str(), ss_wheel_angle.str(), ss_safety.str(), this->visualization_pub_);
        // 可视化本车占用区域
        // 清空之前的可视化
        visualization_msgs::MarkerArray delete_occupation_marker_array;
        delete_occupation_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
        this->vis_occupation_pub_.publish(delete_occupation_marker_array);
        VisualizationMethods::visualizeSubvehicleOccupationArea(this->choosed_state_, this->vehicle_rear_axis_center_scale_, this->vis_occupation_pub_);

        // 3. 判断状态是否已完成，如果完成（本车当前位置到达状态的目标点表示完成）则开始重新路径规划
        if (current_position_index_in_trajectory >= this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory()) {
            LOG(INFO) << "状态完成，重新进行路径规划";
            LOG(INFO) << "结束时接收的GPS坐标为" << current_position_in_world.position_.x_ << "||" << current_position_in_world.position_.y_ << "||" << current_position_in_world.theta_;
            LOG(INFO) << "结束时方向盘给出的曲率为" << current_vehicle_kappa;
            LOG(INFO) << "结束时对应的路径上最近点的坐标为" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.x_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.y_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].theta_;
            LOG(INFO) << "结束时对应的路径上最近点的曲率为" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].kappa_;
            this->current_state_ = this->choosed_state_;
            return;
        }

        // 4. 如果期望状态和选中状态不一致且期望状态是选中状态的邻居，更新期望状态信息，判断期望状态是否可行，如果可行，切换当前状态为期望状态，并跳过之后的步骤。（期望状态只能是三大状态）
        if (this->choosed_state_.getStateName() == StateNames::FORWARD && this->expected_state_.getStateName() != this->choosed_state_.getStateName() && Tools::searchNeighborStates(this->choosed_state_.getNeighborStates(), this->expected_state_.getStateName())) {
            LOG(INFO) << "期望状态和选中状态不一致且期望状态是选中状态的邻居，更新期望状态信息，判断期望状态是否可行";
            // 判断期望状态的毫米波是否报警
            bool is_expected_state_radar_safe = true;
            if (this->IS_SURROUND_RADAR_ENABLE_FLAG_) {
                if (this->expected_state_.getStateName() == StateNames::TURN_LEFT) {
                    this->left_alert_mutex_.lock();
                    bool is_left_alert = this->left_alert_;
                    this->left_alert_mutex_.unlock();
                    if (is_left_alert) {
                        is_expected_state_radar_safe = false;
                        LOG(INFO) << "左侧毫米波报警，期望状态放弃";
                    }
                } else if (this->expected_state_.getStateName() == StateNames::TURN_RIGHT) {
                    this->right_alert_mutex_.lock();
                    bool is_right_alert = this->right_alert_;
                    this->right_alert_mutex_.unlock();
                    if (is_right_alert) {
                        is_expected_state_radar_safe = false;
                        LOG(INFO) << "右侧毫米波报警，期望状态放弃";
                    }
                }
            }
            if (is_expected_state_radar_safe) {
                // 对期望状态进行状态初始化
                //  首先得到期望状态对应的道路信息
                Lane expected_lane = this->expected_state_.getRespondingLane();
                // 判断对应道路信息是否存在
                if (expected_lane.getLaneExistance()) {
                    LOG(INFO) << "进行期望状态的更新";
                    // 只有在道路信息存在时才有效
                    // 期望状态为有效状态
                    this->expected_state_.enable();
                    // 提供车辆信息
                    this->expected_state_.setVehicleSize(this->vehicle_length_, this->vehicle_width_);
                    // 提供起始点信息
                    this->expected_state_.setVehicleStartState(current_position_in_world);
                    this->expected_state_.setVehicleStartMovement(current_movement_state);
                    // 提供车辆起点速度信息
                    this->expected_state_.setVehicleCurrentMovement(current_movement_state);
                    // 提供车辆当前位置在轨迹中下标信息(此处一定是0)
                    this->expected_state_.setVehicleCurrentPositionIndexInTrajectory(0);
                    // 对期望状态进行重新的路径规划
                    this->updateLaneTrajectoryforStates(&(this->expected_state_), expected_lane, current_position_in_world, current_movement_state, current_vehicle_kappa);
                    // 确定状态的优先级(TODO)
                    this->expected_state_.setPriority(PRIORITY_INIT_VALUE_MAX);
                    // 初始化状态的安全性
                    this->expected_state_.setSafety(true);
                    // 确定道路速度期望
                    // 得到当前位置下标
                    size_t current_position_index_in_expected_lane = expected_lane.findCurrenPositionIndexInLane(current_position_in_world.position_.x_, current_position_in_world.position_.y_);
                    // 得到未来下标
                    size_t future_position_index_in_expected_lane = current_position_index_in_expected_lane + static_cast<size_t>(Tools::normalForeseenDistance(std::max(current_movement_state.velocity_, this->expected_velocity_upper_bound_), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
                    assert(future_position_index_in_expected_lane < expected_lane.getLaneVelocityLimitation().size());
                    // 得到当前下标到未来下标中最低速度限制
                    double velocity_limitation_max = expected_lane.getLaneVelocityLimitation()[current_position_index_in_expected_lane];
                    for (size_t i = current_position_index_in_expected_lane; i < future_position_index_in_expected_lane; i++) {
                        double velocity_limitation = expected_lane.getLaneVelocityLimitation()[i];
                        if (Tools::isSmall(velocity_limitation, velocity_limitation_max)) {
                            velocity_limitation_max = velocity_limitation;
                        }
                    }
                    double velocity_limitation_min = std::min(expected_lane.getLowVelocity()[current_position_index_in_expected_lane], expected_lane.getLowVelocity()[future_position_index_in_expected_lane]);
                    assert(Tools::isLarge(velocity_limitation_max, velocity_limitation_min));
                    this->expected_state_.setExpectedVelocity(expected_lane.getLaneVelocityLimitation()[current_position_index_in_expected_lane], velocity_limitation_max);
                    // 确定车辆速度限制，车辆速度上限由当前速度、横向加速度上限决定
                    double expected_subvehicle_max_velocity = std::min(this->expected_state_.getVehicleStartMovement().velocity_ + VELOCITY_MAX_INCREASE_VALUE, velocity_limitation_max);
                    // 计算横向加速度限速
                    double expected_state_max_kappa = this->calcMaxKappaForState(this->expected_state_, foreseen_length);
                    double expected_state_curvature_velocity_limtation = Tools::calcVelocityForMaxNormalAcceleration(expected_state_max_kappa);
                    // 计算横向加加速度限速
                    double expected_state_max_curvature_change_rate = calcMaxCurvatureChangeRateForState(this->expected_state_, foreseen_length);
                    double expected_state_curvature_change_rate_velocity_limtation = Tools::calcVelocityForMaxNormalJerk(expected_state_max_curvature_change_rate);
                    // 得到最终限速
                    expected_subvehicle_max_velocity = std::min(std::min(expected_subvehicle_max_velocity, expected_state_curvature_velocity_limtation), expected_state_curvature_change_rate_velocity_limtation);
                    // 设置限速
                    this->expected_state_.setVelocityLimitation(expected_subvehicle_max_velocity, std::min(velocity_limitation_min, 0.9 * expected_subvehicle_max_velocity));
                    // 确定车辆的加速度上下限制，加速度上限由横向加加速度上限决定
                    double subvehicle_max_acceleration, subvehicle_min_acceleration;
                    subvehicle_max_acceleration = std::min(Tools::velocityToAccelerationReflection(current_movement_state.velocity_), Tools::calcAccelerationForMaxNormalJerk(expected_state_max_kappa, expected_subvehicle_max_velocity));
                    subvehicle_min_acceleration = -COMMON_DECCELERATION;
                    this->expected_state_.setAccelerationLimitationMax(subvehicle_max_acceleration);
                    this->expected_state_.setAccelerationLimitationMin(subvehicle_min_acceleration);
                    // 设置目标点初始值
                    this->expected_state_.setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->expected_state_.getTrajectoryLength());
                    // 设置目标速度初始值(无效值)
                    this->expected_state_.setGoalVelocity(0.0);
                    // 设置期望加速度初始值(无效值)
                    this->expected_state_.setVehicleDynamicPlanningExpectedAcceleration(0.0);

                    // 判断是否遵循交通规则
                    DecisionMaking::RSS::trafficRuleCheck(&(this->expected_state_), this->traffic_rule_obstacles_);

                    // // 对期望状态进行动态速度规划
                    // this->velocityPlanningForState(&(this->expected_state_), obstacles, false);

                    VelocityPlanning::VelocityPlanner* v_planner = new VelocityPlanning::VelocityPlanner(&(this->expected_state_));
                    v_planner->runOnce(obstacles);

                    // if (!(&expected_state_)->velocity_profile_generation_state_) {
                    //     velocityPlanningForState(&(expected_state_), obstacles, false);
                    //     (&(expected_state_))->velocity_planning_from_previous_version_ = true;
                    // }

                    

                    // 进行动态速度规划和碰撞可视化
                    visualization_msgs::MarkerArray delete_collsion_point, collision_point;
                    delete_collsion_point.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
                    std_msgs::ColorRGBA collision_color;
                    collision_color.r = 220.0 / 255.0;
                    collision_color.g = 20.0 / 255.0;
                    collision_color.b = 60.0 / 255.0;
                    collision_color.a = 1.0;
                    for (size_t i = 0; i < collision_rectangles.size(); i++) {
                        Rectangle rectangle = collision_rectangles[i];
                        collision_point.markers.push_back(VisualizationMethods::visualizeRectToMarker(rectangle.center_x_, rectangle.center_y_, rectangle.rotation_, rectangle.width_, rectangle.length_, 0.5, collision_color, i));
                    }
                    std::vector<Rectangle>().swap(collision_rectangles);
                    this->vis_collision_pub_.publish(delete_collsion_point);
                    this->vis_collision_pub_.publish(collision_point);
                    // 期望状态信息更新完毕，
                    if (this->expected_state_.getCapability() && this->expected_state_.getSafety()) {
                        // 期望状态可行且安全，立刻选择期望状态并发布，并重新开始状态保持
                        LOG(INFO) << "期望状态可行且安全，立刻选择期望状态并发布";
                        this->choosed_state_ = this->expected_state_;
                        // 发布新版路径，保持加速度模式
                        this->choosed_state_.publishCurveMsgVelocityMaintain(this->motion_planning_curve_pub_);

                        // Reload
                        if (choosed_state_.getStateName() == StateNames::FORWARD) {
                            (&states_set_[StateNames::FORWARD])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                            (&states_set_[StateNames::FORWARD])->s_ = choosed_state_.s_;
                            (&states_set_[StateNames::FORWARD])->v_ = choosed_state_.v_;
                            (&states_set_[StateNames::FORWARD])->a_ = choosed_state_.a_;
                        } else if (choosed_state_.getStateName() == StateNames::TURN_LEFT) {
                            (&states_set_[StateNames::TURN_LEFT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                            (&states_set_[StateNames::TURN_LEFT])->s_ = choosed_state_.s_;
                            (&states_set_[StateNames::TURN_LEFT])->v_ = choosed_state_.v_;
                            (&states_set_[StateNames::TURN_LEFT])->a_ = choosed_state_.a_;
                        } else if (choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
                            (&states_set_[StateNames::TURN_RIGHT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                            (&states_set_[StateNames::TURN_RIGHT])->s_ = choosed_state_.s_;
                            (&states_set_[StateNames::TURN_RIGHT])->v_ = choosed_state_.v_;
                            (&states_set_[StateNames::TURN_RIGHT])->a_ = choosed_state_.a_;
                        } else {
                            assert(false);
                        }

                        // 重新进行可视化
                        // 清空之前的可视化
                        visualization_msgs::MarkerArray delete_marker_array;
                        delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
                        this->visualization_pub_.publish(delete_marker_array);

                        // 1.可视化选中状态的路径
                        PathPlanningUtilities::Curve raw_choosed_curve, choosed_curve;
                        if (this->choosed_state_.getExtendedTrajectory().size() == 0) {
                            // 如果没有延伸路径（倒车没有延伸路径）
                            raw_choosed_curve = this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()];
                        } else {
                            // 如果有延伸路径
                            raw_choosed_curve = this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()];
                            raw_choosed_curve.insert(raw_choosed_curve.end(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
                        }
                        choosed_curve.assign(raw_choosed_curve.begin() + this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory(), raw_choosed_curve.begin() + this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory());
                        VisualizationMethods::visualizeChoosedCurveWithBoundary(choosed_curve, this->vehicle_width_, this->vehicle_length_, this->vehicle_rear_axis_center_scale_, VisualizationMethods::VisualizationID::CHOOSED_CURVE_START_ID, 7, this->visualization_pub_);

                        // 2. 可视化道路
                        visualization_msgs::MarkerArray lanes_marker_array;
                        std_msgs::ColorRGBA lane_visualization_color;
                        if (this->center_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->center_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::CENTER_LANE_ID));
                        }
                        if (this->left_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->left_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::LEFT_LANE_ID));
                        }
                        if (this->right_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->right_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::RIGHT_LANE_ID));
                        }
                        if (lanes_marker_array.markers.size() > 0) {
                            this->visualization_pub_.publish(lanes_marker_array);
                        }

                        // 3. 可视化状态机
                        double position_x = this->choosed_state_.getVehicleCurrentPosition().position_.x_;
                        double position_y = this->choosed_state_.getVehicleCurrentPosition().position_.y_;
                        VisualizationMethods::visualizeStates(this->states_set_, this->choosed_state_, position_x, position_y, VisualizationMethods::VisualizationID::STATES_START_ID, this->visualization_pub_);

                        // 4. 可视化交通规则
                        VisualizationMethods::visualizeTrafficRules(this->traffic_rule_obstacles_raw_, this->visualization_pub_);

                        // 5. 可视化本车占用区域
                        // 清空之前的可视化
                        visualization_msgs::MarkerArray delete_occupation_marker_array;
                        delete_occupation_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
                        this->vis_occupation_pub_.publish(delete_occupation_marker_array);
                        VisualizationMethods::visualizeSubvehicleOccupationArea(this->choosed_state_, this->vehicle_rear_axis_center_scale_, this->vis_occupation_pub_);
                        continue;
                    } else {
                        LOG(INFO) << "期望状态不可行或不安全";
                    }
                } else {
                    // 道路不存在时，状态直接失效
                    this->expected_state_.disable();
                    LOG(INFO) << "期望状态不可行";
                }
            }
        }

        // 5. 判断选中状态是否遵循交通规则
        DecisionMaking::RSS::trafficRuleCheck(&(this->choosed_state_), this->traffic_rule_obstacles_);
        if (!this->choosed_state_.getCapability() || !this->choosed_state_.getSafety()) {
            // 如果违法交通规则，立刻进行路径重新规划
            LOG(INFO) << "选中状态违反交通规则，立刻进行重新规划";
            this->current_state_ = this->choosed_state_;
            this->current_state_.setStateCompleted(false);
            return;
        }

        // 6. 选中状态如果是三大状态时，进行动态速度规划（每隔2秒进行一次）和安全性判断。
        if (this->choosed_state_.getStateName() == StateNames::FORWARD || this->choosed_state_.getStateName() == StateNames::TURN_LEFT || this->choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
            // 判断是否达到了速度规划的时机(TOFIX)->a. 如果经过了2秒钟，进行一次重新速度规划。b. 道路限速发生变化时，进行一次重新速度规划
            re_dynamic_motion_planning_count++;
            LOG(INFO) << "re_dynamic_motion_planning_count" << re_dynamic_motion_planning_count;
            if (re_dynamic_motion_planning_count >= static_cast<int>(STATE_MAINTAIN_FREQUENCY / VELOCITY_UPDATE_FREQUENCY_IN_STATE_MAINTAIN)) {
                // 如果到了，则进行速度规划
                LOG(INFO) << "重新进行动态速度规划";
                re_dynamic_motion_planning_count = 0;
                // this->velocityPlanningForState(&(this->choosed_state_), obstacles, true);
                
                VelocityPlanning::VelocityPlanner* v_planner = new VelocityPlanning::VelocityPlanner(&(this->choosed_state_));
                v_planner->runOnce(obstacles);

                // if (!(&choosed_state_)->velocity_profile_generation_state_) {
                //     velocityPlanningForState(&(choosed_state_), obstacles, false);
                //     (&choosed_state_)->velocity_planning_from_previous_version_ = true;
                // }
                

                if (!this->choosed_state_.getSafety() || !this->choosed_state_.getCapability()) {
                    // LOG(INFO) << "动态速度规划，选中状态不可行，进行安全性判断";
                    // if (!DecisionMaking::RSS::stateSafetyJudgement(this->choosed_state_, obstacles)) {
                    //     // 安全性判断失败，装填不可行
                    //     LOG(INFO) << "安全性判断失败，状态不可行，重新规划";
                    //     this->current_state_ = this->choosed_state_;
                    //     this->current_state_.setStateCompleted(false);
                    //     return;
                    // }else {
                    //     // 安全性判断成功，使用原来运动
                    //     LOG(INFO) << "安全性判断成功，沿用之前状态";
                    //     // continue;
                    // }
                    LOG(INFO) << "动态速度规划，选中状态不可行，重新规划";
                    this->current_state_ = this->choosed_state_;
                    this->current_state_.setStateCompleted(false);
                    // return;
                } else {
                    // 如果重新进行速度规划后发现选中状态可行，发布新的路径
                    // 发布新版路径，保持加速度模式
                    this->choosed_state_.publishCurveMsgVelocityMaintain(this->motion_planning_curve_pub_);
                    // continue;
                        // Reload
                    if (choosed_state_.getStateName() == StateNames::FORWARD) {
                        (&states_set_[StateNames::FORWARD])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                        (&states_set_[StateNames::FORWARD])->s_ = choosed_state_.s_;
                        (&states_set_[StateNames::FORWARD])->v_ = choosed_state_.v_;
                        (&states_set_[StateNames::FORWARD])->a_ = choosed_state_.a_;
                    } else if (choosed_state_.getStateName() == StateNames::TURN_LEFT) {
                        (&states_set_[StateNames::TURN_LEFT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                        (&states_set_[StateNames::TURN_LEFT])->s_ = choosed_state_.s_;
                        (&states_set_[StateNames::TURN_LEFT])->v_ = choosed_state_.v_;
                        (&states_set_[StateNames::TURN_LEFT])->a_ = choosed_state_.a_;
                    } else if (choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
                        (&states_set_[StateNames::TURN_RIGHT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                        (&states_set_[StateNames::TURN_RIGHT])->s_ = choosed_state_.s_;
                        (&states_set_[StateNames::TURN_RIGHT])->v_ = choosed_state_.v_;
                        (&states_set_[StateNames::TURN_RIGHT])->a_ = choosed_state_.a_;
                    } else {
                        assert(false);
                    }
                }
            } else {
                // 如进行安全性判断,如果不安全立刻进行速度规划，如果速度规划得不到安全结果则进行路径规划。
                LOG(INFO) << "进行安全性判别";
                if (!DecisionMaking::RSS::stateSafetyJudgement(this->choosed_state_, obstacles)) {
                    LOG(INFO) << "不安全状态需要重新规划";
                    // 当前状态不安全，则重新开始速度规划
                    // this->velocityPlanningForState(&(this->choosed_state_), obstacles, true);

                    VelocityPlanning::VelocityPlanner* v_planner = new VelocityPlanning::VelocityPlanner(&(this->choosed_state_));
                    v_planner->runOnce(obstacles);

                    // if (!(&choosed_state_)->velocity_profile_generation_state_) {
                    //     velocityPlanningForState(&(choosed_state_), obstacles, false);
                    //     (&choosed_state_)->velocity_planning_from_previous_version_ = true;
                    // }

                    if (!this->choosed_state_.getSafety() || !this->choosed_state_.getCapability()) {
                        // 如果重新进行速度规划后发现选中状态不可行，则要重新开始路径规划
                        LOG(INFO) << "状态不安全且动态速度修正失败，选中状态不可行，重新进行路径规划";
                        this->current_state_ = this->choosed_state_;
                        this->current_state_.setStateCompleted(false);
                        return;
                    } else {
                        LOG(INFO) << "状态不安全，动态速度修正成功";
                        // 如果重新进行速度规划后发现选中状态可行，发布新的路径
                        // 重规划标志位归零
                        re_dynamic_motion_planning_count = 0;
                        // 发布新版路径，保持加速度模式
                        this->choosed_state_.publishCurveMsgVelocityMaintain(this->motion_planning_curve_pub_);

                        // Reload
                        if (choosed_state_.getStateName() == StateNames::FORWARD) {
                            (&states_set_[StateNames::FORWARD])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                            (&states_set_[StateNames::FORWARD])->s_ = choosed_state_.s_;
                            (&states_set_[StateNames::FORWARD])->v_ = choosed_state_.v_;
                            (&states_set_[StateNames::FORWARD])->a_ = choosed_state_.a_;
                        } else if (choosed_state_.getStateName() == StateNames::TURN_LEFT) {
                            (&states_set_[StateNames::TURN_LEFT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                            (&states_set_[StateNames::TURN_LEFT])->s_ = choosed_state_.s_;
                            (&states_set_[StateNames::TURN_LEFT])->v_ = choosed_state_.v_;
                            (&states_set_[StateNames::TURN_LEFT])->a_ = choosed_state_.a_;
                        } else if (choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
                            (&states_set_[StateNames::TURN_RIGHT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
                            (&states_set_[StateNames::TURN_RIGHT])->s_ = choosed_state_.s_;
                            (&states_set_[StateNames::TURN_RIGHT])->v_ = choosed_state_.v_;
                            (&states_set_[StateNames::TURN_RIGHT])->a_ = choosed_state_.a_;
                        } else {
                            assert(false);
                        }
                        // continue;
                    }
                } else {
                }
            }
        } else {
            std::cout << "[Error] error choosed state name" << std::endl;
            LOG(INFO) << "[Error] error choosed state name";
            exit(0);
        }

        end_time = clock();
        if (re_dynamic_motion_planning_count == 0) {
            LOG(INFO) << "------------------ one turn of state maintain end, speed replan, time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << "ms ----------------------";
        } else {
            LOG(INFO) << "------------------ one turn of state maintain end, without speed replan, time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << "ms -------------------";
        }
    }
}

// 判断停车状态是否需要继续，还是重新进行规划
void DecisionMaking::SubVehicle::stopStateMaintain() {
    // TOFIX
    // 函数以2HZ频率运行
    // 停车状态保持分为了四步，第一步是更新信息，包括车辆状态信息和障碍物信息；第二步是判断状态是否完成，如果当前速度等于0，则状态完成；第三步是判断剩余距离，如果剩余距离小于阈值，不会退出状态进行重新规划；第三步，如果剩余距离大于阈值，则判断已经走过的距离是否足够，如果足够，进行重新规划；第四步，判断当前路径是否安全，如果不安全需要重新规划。
    ros::Rate loop_rate(STATE_MAINTAIN_FREQUENCY);
    while (ros::ok()) {
        loop_rate.sleep();
        clock_t start_time, end_time;
        LOG(INFO) << "----------------------------- one turn of stop state maintain start --------------------------------";
        std::cout << "----------------------------- one turn of stop state maintain start --------------------------------" << std::endl;
        this->emergency_break_flag_mutex_.lock();
        bool check_emergency_break = this->IS_EMERGENCY_BREAK_FLAG_;
        this->emergency_break_flag_mutex_.unlock();
        if (check_emergency_break) {
            return;
        }

        if (!this->choosed_state_.getStateMaintain()) {
            // 不进行状态保持
            this->current_state_ = this->choosed_state_;
            return;
        }

        // 判断停车状态给的路径长度足够，如果不够，直接进行重新规划
        if (this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory() <= 16 && Tools::isZero(this->choosed_state_.getVehicleStartMovement().velocity_)) {
            LOG(INFO) << "停车距离小于1米且当前速度为0，直接无视";
            this->current_state_ = this->choosed_state_;
            this->current_state_.setVehicleCurrentMovement(this->choosed_state_.getVehicleStartMovement());
            // sleep(1);
            return;
        }

        start_time = clock();
        // 更新障碍物信息
        this->updateValidateTrafficRuleInformation();
        this->updateObstacleInformation();
        // 得到当前的障碍物信息
        this->obstacle_mutex_.lock();
        std::vector<Obstacle> obstacles = this->obstacles_;
        this->obstacle_mutex_.unlock();
        
        // 更新选中状态的信息
        // 获取世界坐标系下车辆的信息
        PathPlanningUtilities::VehicleState current_position_in_world;
        this->current_vehicle_world_position_mutex_.lock();
        current_position_in_world = this->current_vehicle_world_position_;
        this->current_vehicle_world_position_mutex_.unlock();
        PathPlanningUtilities::VehicleMovementState current_movement_state;
        this->current_vehicle_movement_mutex_.lock();
        current_movement_state = this->current_vehicle_movement_;
        this->current_vehicle_movement_mutex_.unlock();
        this->current_vehicle_kappa_mutex_.lock();
        double current_vehicle_kappa = this->current_vehicle_kappa_;
        current_position_in_world.kappa_ = current_vehicle_kappa;
        this->current_vehicle_kappa_mutex_.unlock();
        // 得到当前位置路径下标和速度更新
        size_t current_position_index_in_trajectory = Tools::findNearestPositionIndexInCurve(this->choosed_state_.getTotalTrajectory(), current_position_in_world.position_, this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory());
        // 更新选中状态的信息
        // 当前速度信息
        this->choosed_state_.setVehicleCurrentMovement(current_movement_state);
        // 当前位置信息
        this->choosed_state_.setVehicleCurrentPositionIndexInTrajectory(current_position_index_in_trajectory);

        // // 更新车辆状态信息可视化
        // // 删除之前状态显示
        // visualization_msgs::MarkerArray delete_marker_array;
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_VEL));
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_ANGLE));
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_SAFETY));
        // this->visualization_pub_.publish(delete_marker_array);
        // // 构造可视化状态字符串
        // std::stringstream ss_velocity;
        // ss_velocity << "velocity: " << current_movement_state.velocity_ << " m/s";
        // std::stringstream ss_wheel_angle;
        // ss_wheel_angle << "steering angle: " << atan(current_vehicle_kappa * (2.8498 + 0.007 * current_movement_state.velocity_ * current_movement_state.velocity_)) * 14.8 / PI * 180.0 << " deg";
        // std::stringstream ss_safety;
        // ss_safety << "safety: " << this->choosed_state_.getSafety();
        // VisualizationMethods::visualizeVehicleState(current_position_in_world.position_.x_, current_position_in_world.position_.y_, ss_velocity.str(), ss_wheel_angle.str(), ss_safety.str(), this->visualization_pub_);
        
        // // 清空之前的可视化
        // visualization_msgs::MarkerArray delete_occupation_marker_array;
        // delete_occupation_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
        // this->vis_occupation_pub_.publish(delete_occupation_marker_array);
        // // 可视化本车占用区域
        // VisualizationMethods::visualizeSubvehicleOccupationArea(this->choosed_state_, this->vehicle_rear_axis_center_scale_, this->vis_occupation_pub_);
        LOG(INFO) << "current position index is " << current_position_index_in_trajectory;

        // 判断当前选中状态是否完成(停车分为两种情况，分别是高速停车和低速停车)
        if (Tools::isLarge(this->choosed_state_.getVehicleStartMovement().velocity_, VELOCITY_THRESHOLD)) {
            // 高速停车，如果速度低于阈值，或到达目标点都算完成
            if (Tools::isSmall(current_movement_state.velocity_, VELOCITY_THRESHOLD) || this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory() >= this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory() ) {
                LOG(INFO) << "高速停车状态完成，重新进行规划" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.x_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.y_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].theta_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].kappa_;;
                this->current_state_ = this->choosed_state_;
                return;
            }
        } else {
            // 判断条件
            double is_stop_finished = false;
            if (Tools::isLarge(this->choosed_state_.getVehicleStartMovement().velocity_, 0.0)) {
                // 如果初始速度大于0
                if(Tools::isZero(current_movement_state.velocity_)) {
                    // 如果当前车速等于0认为状态完成，开始重新规划
                    is_stop_finished = true;
                    double error_distance = (static_cast<int>(this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory()) - static_cast<int>(this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory())) * LANE_GAP_DISTANCE;
                    LOG(INFO) << "停车误差距离为" << error_distance;
                }
            } else {
                // 如果初始速度等于0
                if ((Tools::isZero(current_movement_state.velocity_) && this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory() >= static_cast<int>(STOP_STATE_FINISH_MIN_TRAVELED_DISTANCE / LANE_GAP_DISTANCE))) {
                    // 如果当前车速等于0，并且走过了超过1米的距离，认为状态完成
                    is_stop_finished = true;
                    double error_distance = (static_cast<int>(this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory()) - static_cast<int>(this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory())) * LANE_GAP_DISTANCE;
                    LOG(INFO) << "停车误差距离为" << error_distance;
                }
            }
            // 低速停车，如果速度等于0且走过了1米，或到达离目标点还有1米都算完成（在此条件下，控制要能追踪1m路径）
            if (is_stop_finished) {
                LOG(INFO) << "低速停车状态已完成，延时1秒后重新进行规划" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.x_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.y_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].theta_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].kappa_;;
                this->current_state_ = this->choosed_state_;
                // 停车状态切出前要延时1秒
                // sleep(1);
                return;
            }
        }

        double maintain_distance = AVOIDANCE_MIN_DISTANCE;
        // if (GLOBAL_IS_IN_CHECK_) {
        //     maintain_distance = AVOIDANCE_MIN_DISTANCE / 3.0;
        // }
        // 如果选中状态未完成，开始判断是否保持选中状态（也分为高速停车和低速停车两种情况）
        if (this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory() < static_cast<size_t>(maintain_distance / LANE_GAP_DISTANCE) + this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory() || Tools::isLarge(this->choosed_state_.getVehicleStartMovement().velocity_, VELOCITY_THRESHOLD)) {
            // 保持选中状态
            LOG(INFO) << "停车状态进入保持模式";
        } else {
            // 无需保持选中状态，此时如果走过了总路程的一半，则认为走过了足够的距离
            if (this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory() >= static_cast<size_t>(this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory() / 2)) {
                // 已经走过了足够的距离
                LOG(INFO) << "已经走过了足够距离，且低速停车状态无需进行保持，进行重新规划";
                this->current_state_ = choosed_state_;
                return;
            }
        }

        Lane expected_lane;
        bool ignore_offset = false;
        LOG(INFO) << "选择状态是否为继续状态" << this->choosed_state_.getStateContinue() << "，继续的状态名为" << DIC_STATE_NAME[this->current_state_.getStateName()];
        if (this->choosed_state_.getStateContinue() && (this->current_state_.getStateName() == StateNames::TURN_LEFT || this->current_state_.getStateName() == StateNames::TURN_RIGHT)) {
            // 期望状态就是之前没能够完成的状态
            expected_lane = this->current_state_.getRespondingLane();
            ignore_offset = true;
            
        } else {
            expected_lane = this->center_lane_;
        }
        // 判断中间道路是否可行（期望状态是否可行）
        {
            LOG(INFO) << "更新期望状态信息，判断期望状态是否可行";
            // 对期望状态进行状态初始化
            // 判断对应道路信息是否存在
            if (expected_lane.getLaneExistance()) {
                LOG(INFO) << "进行期望状态的更新";
                // 只有在道路信息存在时才有效
                // 期望状态为有效状态
                this->expected_state_.enable();
                // 提供车辆信息
                this->expected_state_.setVehicleSize(this->vehicle_length_, this->vehicle_width_);
                // 提供起始点信息
                this->expected_state_.setVehicleStartState(current_position_in_world);
                this->expected_state_.setVehicleStartMovement(current_movement_state);
                // 提供车辆起点速度信息
                this->expected_state_.setVehicleCurrentMovement(current_movement_state);
                // 提供车辆当前位置在轨迹中下标信息(此处一定是0)
                this->expected_state_.setVehicleCurrentPositionIndexInTrajectory(0);
                // 对期望状态进行重新的路径规划
                this->updateLaneTrajectoryforStates(&(this->expected_state_), expected_lane, current_position_in_world, current_movement_state, current_vehicle_kappa);
                // 判断期望状态起始点是否可行
                double head_offset = this->expected_state_.getFrenetTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][0].position_.y_ + sin(this->expected_state_.getFrenetTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][0].theta_) * 0.5 * this->vehicle_length_;
                double theta_offset = this->expected_state_.getFrenetTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][0].theta_;
                LOG(INFO) << "头偏移量为" << head_offset << ",角度偏移量为" << theta_offset;
                if (!Tools::isSmall(std::fabs(head_offset), TO_EXPECT_STATE_MAX_HEAD_OFFSET) && !Tools::isSmall(std::fabs(theta_offset), TO_EXPECT_STATE_MAX_THETA_OFFSET) && !ignore_offset) {
                    LOG(INFO) << "偏移过大，不考虑切换到期望状态";
                    this->expected_state_.disable();
                } else {
                    // 确定状态的优先级(TODO)
                    this->expected_state_.setPriority(PRIORITY_INIT_VALUE_MAX);
                    // 初始化状态的安全性
                    this->expected_state_.setSafety(true);
                    // 确定道路速度期望
                    // 得到当前位置下标
                    size_t current_position_index_in_expected_lane = expected_lane.findCurrenPositionIndexInLane(current_position_in_world.position_.x_, current_position_in_world.position_.y_);
                    // 得到未来下标
                    size_t future_position_index_in_expected_lane = current_position_index_in_expected_lane + static_cast<size_t>(Tools::normalForeseenDistance(std::max(current_movement_state.velocity_, this->expected_velocity_upper_bound_), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
                    assert(future_position_index_in_expected_lane < expected_lane.getLaneVelocityLimitation().size());
                    // 得到当前下标到未来下标中最低速度限制
                    double velocity_limitation_max = expected_lane.getLaneVelocityLimitation()[current_position_index_in_expected_lane];
                    for (size_t i = current_position_index_in_expected_lane; i < future_position_index_in_expected_lane; i++) {
                        double velocity_limitation = expected_lane.getLaneVelocityLimitation()[i];
                        if (Tools::isSmall(velocity_limitation, velocity_limitation_max)) {
                            velocity_limitation_max = velocity_limitation;
                        }
                    }
                    double velocity_limitation_min = std::min(expected_lane.getLowVelocity()[current_position_index_in_expected_lane], expected_lane.getLowVelocity()[future_position_index_in_expected_lane]);
                    assert(Tools::isLarge(velocity_limitation_max, velocity_limitation_min));
                    this->expected_state_.setExpectedVelocity(expected_lane.getLaneVelocityLimitation()[current_position_index_in_expected_lane], velocity_limitation_max);
                    // 确定车辆速度限制，车辆速度上限由当前速度、横向加速度上限决定
                    double expected_subvehicle_max_velocity = std::min(this->expected_state_.getVehicleStartMovement().velocity_ + VELOCITY_MAX_INCREASE_VALUE, velocity_limitation_max);
                    // 计算横向加速度限速
                    double expected_state_max_kappa =  this->expected_state_.getTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->expected_state_.getTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()])].kappa_;
                    double expected_state_curvature_velocity_limtation = Tools::calcVelocityForMaxNormalAcceleration(expected_state_max_kappa);
                    // 计算横向加加速度限速
                    double expected_state_max_curvature_change_rate = Tools::getCurveMaxCurvatureChangeRate(this->expected_state_.getTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()]);
                    double expected_state_curvature_change_rate_velocity_limtation = Tools::calcVelocityForMaxNormalJerk(expected_state_max_curvature_change_rate);
                    // 得到最终限速
                    expected_subvehicle_max_velocity = std::min(std::min(expected_subvehicle_max_velocity, expected_state_curvature_velocity_limtation), expected_state_curvature_change_rate_velocity_limtation);
                    // 设置限速
                    this->expected_state_.setVelocityLimitation(expected_subvehicle_max_velocity, std::min(velocity_limitation_min, 0.9 * expected_subvehicle_max_velocity));
                    // 确定车辆的加速度上下限制，加速度上限由横向加加速度上限决定
                    double subvehicle_max_acceleration, subvehicle_min_acceleration;
                    subvehicle_max_acceleration = std::min(Tools::velocityToAccelerationReflection(current_movement_state.velocity_), Tools::calcAccelerationForMaxNormalJerk(expected_state_max_kappa, expected_subvehicle_max_velocity));
                    subvehicle_min_acceleration = -COMMON_DECCELERATION;
                    this->expected_state_.setAccelerationLimitationMax(subvehicle_max_acceleration);
                    this->expected_state_.setAccelerationLimitationMin(subvehicle_min_acceleration);
                    // 设置目标点初始值
                    this->expected_state_.setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->expected_state_.getTrajectoryLength());
                    // 设置目标速度初始值(无效值)
                    this->expected_state_.setGoalVelocity(0.0);
                    // 设置期望加速度初始值(无效值)
                    this->expected_state_.setVehicleDynamicPlanningExpectedAcceleration(0.0);

                    // 判断是否遵循交通规则
                    DecisionMaking::RSS::trafficRuleCheck(&(this->expected_state_), this->traffic_rule_obstacles_);

                    // 对期望状态进行动态速度规划
                    // this->velocityPlanningForState(&(this->expected_state_), obstacles, true);

                    VelocityPlanning::VelocityPlanner* v_planner = new VelocityPlanning::VelocityPlanner(&(this->expected_state_));
                    v_planner->runOnce(obstacles);

                    // 进行动态速度规划和碰撞可视化
                    visualization_msgs::MarkerArray delete_collsion_point, collision_point;
                    delete_collsion_point.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
                    std_msgs::ColorRGBA collision_color;
                    collision_color.r = 220.0 / 255.0;
                    collision_color.g = 20.0 / 255.0;
                    collision_color.b = 60.0 / 255.0;
                    collision_color.a = 1.0;
                    for (size_t i = 0; i < collision_rectangles.size(); i++) {
                        Rectangle rectangle = collision_rectangles[i];
                        collision_point.markers.push_back(VisualizationMethods::visualizeRectToMarker(rectangle.center_x_, rectangle.center_y_, rectangle.rotation_, rectangle.width_, rectangle.length_, 0.5, collision_color, i));
                    }
                    std::vector<Rectangle>().swap(collision_rectangles);
                    this->vis_collision_pub_.publish(delete_collsion_point);
                    this->vis_collision_pub_.publish(collision_point);

                    // 期望状态信息更新完毕，
                    if (this->expected_state_.getCapability() && this->expected_state_.getSafety()) {
                        // 期望状态可行且安全，立刻选择期望状态并发布，并重新开始状态保持
                        LOG(INFO) << "期望状态可行且安全，立刻选择期望状态并发布";
                        this->choosed_state_ = this->expected_state_;
                        // 发布新版路径，保持加速度模式
                        this->choosed_state_.publishCurveMsgVelocityMaintain(this->motion_planning_curve_pub_);

                        // 重新进行可视化
                        // 清空之前的可视化
                        visualization_msgs::MarkerArray delete_marker_array;
                        delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
                        this->visualization_pub_.publish(delete_marker_array);

                        // 1.可视化选中状态的路径
                        PathPlanningUtilities::Curve raw_choosed_curve, choosed_curve;
                        if (this->choosed_state_.getExtendedTrajectory().size() == 0) {
                            // 如果没有延伸路径（倒车没有延伸路径）
                            raw_choosed_curve = this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()];
                        } else {
                            // 如果有延伸路径
                            raw_choosed_curve = this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()];
                            raw_choosed_curve.insert(raw_choosed_curve.end(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
                        }
                        choosed_curve.assign(raw_choosed_curve.begin() + this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory(), raw_choosed_curve.begin() + this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory());
                        VisualizationMethods::visualizeChoosedCurveWithBoundary(choosed_curve, this->vehicle_width_, this->vehicle_length_, this->vehicle_rear_axis_center_scale_, VisualizationMethods::VisualizationID::CHOOSED_CURVE_START_ID, 7, this->visualization_pub_);

                        // 2. 可视化道路
                        visualization_msgs::MarkerArray lanes_marker_array;
                        std_msgs::ColorRGBA lane_visualization_color;
                        if (this->center_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->center_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::CENTER_LANE_ID));
                        }
                        if (this->left_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->left_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::LEFT_LANE_ID));
                        }
                        if (this->right_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->right_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::RIGHT_LANE_ID));
                        }
                        if (lanes_marker_array.markers.size() > 0) {
                            this->visualization_pub_.publish(lanes_marker_array);
                        }

                        // 3. 可视化状态机
                        double position_x = this->choosed_state_.getVehicleCurrentPosition().position_.x_;
                        double position_y = this->choosed_state_.getVehicleCurrentPosition().position_.y_;
                        VisualizationMethods::visualizeStates(this->states_set_, this->choosed_state_, position_x, position_y, VisualizationMethods::VisualizationID::STATES_START_ID, this->visualization_pub_);

                        // 4. 可视化交通规则
                        VisualizationMethods::visualizeTrafficRules(this->traffic_rule_obstacles_raw_, this->visualization_pub_);

                        // 5. 可视化本车占用区域
                        // 清空之前的可视化
                        visualization_msgs::MarkerArray delete_occupation_marker_array;
                        delete_occupation_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
                        this->vis_occupation_pub_.publish(delete_occupation_marker_array);
                        VisualizationMethods::visualizeSubvehicleOccupationArea(this->choosed_state_, this->vehicle_rear_axis_center_scale_, this->vis_occupation_pub_);

                        return;
                    } else {
                        LOG(INFO) << "期望状态不可行或不安全";
                    }
                }
            } else {
                // 道路不存在时，状态直接失效
                this->expected_state_.disable();
                LOG(INFO) << "期望状态不可行";
            }
        }

        // 判断路径的安全性
        // 如果已经过了刹车距离，安全性不进行保证
        if (this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory() >= this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory()) {
            LOG(INFO) << "停车超过目标点，不保证安全";
        } else {
            // GLOBAL_IN_GROUND_MUTEX_.lock();
            // bool is_in_ground =  GLOBAL_IS_IN_GROUND_;
            // GLOBAL_IN_GROUND_MUTEX_.unlock();
            // if (!is_in_ground) {
            //     // 不在园区内
            PathPlanningUtilities::Curve remain_curve, total_curve;
            total_curve.assign(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
            total_curve.insert(total_curve.end(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
            remain_curve.assign(total_curve.begin() + this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory(), total_curve.begin() + this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory());
            // 判断是否发生碰撞
            bool is_collision_occur = false;
            size_t traffic_rule_cut_index, obstacle_cut_index, dynamic_obstacle_block_index;
            if (RSS::collisionPositionIndexInCurve(remain_curve, this->vehicle_width_, this->vehicle_length_, this->traffic_rule_obstacles_, &traffic_rule_cut_index)) {
                is_collision_occur = true;
            } else if (RSS::collisionPositionIndexInCurve(remain_curve, this->vehicle_width_, this->vehicle_length_,current_movement_state.velocity_, obstacles, &obstacle_cut_index)){
                is_collision_occur = true;
            } else if (RSS::collisionWithDynamicObstacles(remain_curve, current_movement_state.velocity_,this->vehicle_width_, this->vehicle_length_, obstacles, &dynamic_obstacle_block_index)) {
                is_collision_occur = true;
            }

            if (is_collision_occur) {
                LOG(INFO) << "停车状态不安全，立刻进行重新规划";
                this->current_state_ = this->choosed_state_;
                this->current_state_.setStateCompleted(false);
                return;
            }
            // } else {
            //     // 在园区内，停车动态规划
            //     PathPlanningUtilities::Curve remain_curve, total_curve;
            //     total_curve.assign(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
            //     total_curve.insert(total_curve.end(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
            //     remain_curve.assign(total_curve.begin() + this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory(), total_curve.begin() + this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory());
            //     // 判断是否发生碰撞
            //     bool is_collision_occur = false;
            //     size_t traffic_rule_cut_index, obstacle_cut_index, dynamic_obstacle_block_index;
            //     if (RSS::collisionPositionIndexInCurve(remain_curve, this->vehicle_width_, this->vehicle_length_, this->traffic_rule_obstacles_, &traffic_rule_cut_index)) {
            //         is_collision_occur = true;
            //     } else if (RSS::collisionPositionIndexInCurve(remain_curve, this->vehicle_width_, this->vehicle_length_,current_movement_state.velocity_, obstacles, &obstacle_cut_index)){
            //         is_collision_occur = true;
            //     } else if (RSS::collisionWithDynamicObstacles(remain_curve, current_movement_state.velocity_,this->vehicle_width_, this->vehicle_length_, obstacles, &dynamic_obstacle_block_index)) {
            //         is_collision_occur = true;
            //     }
            //     if (is_collision_occur) {
            //         LOG(INFO) << "停车状态不安全，立刻进行重新规划";
            //         this->current_state_ = this->choosed_state_;
            //         this->current_state_.setStateCompleted(false);
            //         return;
            //     }
            // }

        }

        end_time = clock();

        LOG(INFO) << "------------------ one turn of stop state maintain end, time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << "ms -------------------";
        std::cout << "------------------ one turn of stop state maintain end, time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << "ms -------------------" << std::endl;
    }
}

// 判断蔽障状态是否需要继续，还是重新进行规划
void DecisionMaking::SubVehicle::avoidanceStateMaintain() {
    // TOFIX
    // 函数以2HZ频率运行
    // 蔽障状态保持分为了三步，第一步是更新信息，包括车辆状态信息和障碍物信息；第二步是判断是否沿着蔽障路径走过了足够的距离，如果距离足够，进行重新规划；第三步是当距离不够时，判断当前路径是否安全。
    ros::Rate loop_rate(STATE_MAINTAIN_FREQUENCY);
    while (ros::ok()) {
        loop_rate.sleep();
        if (!this->choosed_state_.getStateMaintain()) {
            // 不进行状态保持
            this->current_state_ = this->choosed_state_;
            return;
        }
        clock_t start_time, end_time;
        LOG(INFO) << "----------------------------- one turn of avoidance state maintain start --------------------------------";
        std::cout << "----------------------------- one turn of avoidance state maintain start --------------------------------" << std::endl;
        this->emergency_break_flag_mutex_.lock();
        bool check_emergency_break = this->IS_EMERGENCY_BREAK_FLAG_;
        this->emergency_break_flag_mutex_.unlock();
        if (check_emergency_break) {
            return;
        }
        start_time = clock();
        // 更新障碍物信息
        this->updateValidateTrafficRuleInformation();
        this->updateObstacleInformation();
        // 得到当前的障碍物信息
        this->obstacle_mutex_.lock();
        std::vector<Obstacle> obstacles = this->obstacles_;
        this->obstacle_mutex_.unlock();
        
        // 更新选中状态的信息
        // 获取世界坐标系下车辆的信息
        PathPlanningUtilities::VehicleState current_position_in_world;
        this->current_vehicle_world_position_mutex_.lock();
        current_position_in_world = this->current_vehicle_world_position_;
        this->current_vehicle_world_position_mutex_.unlock();
        PathPlanningUtilities::VehicleMovementState current_movement_state;
        this->current_vehicle_movement_mutex_.lock();
        current_movement_state = this->current_vehicle_movement_;
        this->current_vehicle_movement_mutex_.unlock();
        this->current_vehicle_kappa_mutex_.lock();
        double current_vehicle_kappa = this->current_vehicle_kappa_;
        current_position_in_world.kappa_ = current_vehicle_kappa;
        this->current_vehicle_kappa_mutex_.unlock();
        // 得到当前位置路径下标和速度更新
        size_t current_position_index_in_trajectory = Tools::findNearestPositionIndexInCurve(this->choosed_state_.getTotalTrajectory(), current_position_in_world.position_, this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory());
        // 更新选中状态的信息
        // 当前速度信息
        this->choosed_state_.setVehicleCurrentMovement(current_movement_state);
        // 当前位置信息
        this->choosed_state_.setVehicleCurrentPositionIndexInTrajectory(current_position_index_in_trajectory);
        LOG(INFO) << "current position index is " << current_position_index_in_trajectory;
        // // 更新车辆状态信息可视化
        // // 删除之前状态显示
        // visualization_msgs::MarkerArray delete_marker_array;
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_VEL));
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_ANGLE));
        // delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_INFO_SAFETY));
        // this->visualization_pub_.publish(delete_marker_array);
        // // 构造可视化状态字符串
        // std::stringstream ss_velocity;
        // ss_velocity << "velocity: " << current_movement_state.velocity_ << " m/s";
        // std::stringstream ss_wheel_angle;
        // ss_wheel_angle << "steering angle: " << atan(current_vehicle_kappa * (2.8498 + 0.007 * current_movement_state.velocity_ * current_movement_state.velocity_)) * 14.8 / PI * 180.0 << " deg";
        // std::stringstream ss_safety;
        // ss_safety << "safety: " << this->choosed_state_.getSafety();
        // VisualizationMethods::visualizeVehicleState(current_position_in_world.position_.x_, current_position_in_world.position_.y_, ss_velocity.str(), ss_wheel_angle.str(), ss_safety.str(), this->visualization_pub_);
        // // 可视化本车占用区域
        // // 清空之前的可视化
        // visualization_msgs::MarkerArray delete_occupation_marker_array;
        // delete_occupation_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
        // this->vis_occupation_pub_.publish(delete_occupation_marker_array);
        // // 可视化本车占用区
        // VisualizationMethods::visualizeSubvehicleOccupationArea(this->choosed_state_, this->vehicle_rear_axis_center_scale_, this->vis_occupation_pub_);

        // 判断状态是否已经完成(蔽障状态完成条件为走到了目标点所在位置)
        if (current_position_index_in_trajectory >= this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory()) {
            // 状态已经完成
            LOG(INFO) << "避障状态完成，重新进行路径规划，结束点信息为" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.x_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].position_.y_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].theta_ << "||" << this->choosed_state_.getTotalTrajectory()[current_position_index_in_trajectory].kappa_;
            this->current_state_ = this->choosed_state_;
            return;
        }

        // 判断中间道路是否可行（期望状态是否可行）
        LOG(INFO) << "选择状态是否为继续状态" << this->choosed_state_.getStateContinue() << "，继续的状态名为" << DIC_STATE_NAME[this->current_state_.getStateName()];
        Lane expected_lane;
        bool ignore_offset = false;
        if (this->choosed_state_.getStateContinue() && (this->current_state_.getStateName() == StateNames::TURN_LEFT || this->current_state_.getStateName() == StateNames::TURN_RIGHT)) {
            // 期望状态就是之前没能够完成的状态
            expected_lane = this->current_state_.getRespondingLane();
            ignore_offset = true;
        } else {
            expected_lane = this->center_lane_;
        }
        
        // 计算左侧道路起点偏移量
        // 计算右侧起点偏移量
        {
            LOG(INFO) << "更新期望状态信息，判断期望状态是否可行";
            // 对期望状态进行状态初始化
            //  首先得到期望状态对应的道路信息
            
            // 判断对应道路信息是否存在
            if (expected_lane.getLaneExistance()) {
                LOG(INFO) << "进行期望状态的更新";
                // 只有在道路信息存在时才有效
                // 期望状态为有效状态
                this->expected_state_.enable();
                // 提供车辆信息
                this->expected_state_.setVehicleSize(this->vehicle_length_, this->vehicle_width_);
                // 提供起始点信息
                this->expected_state_.setVehicleStartState(current_position_in_world);
                this->expected_state_.setVehicleStartMovement(current_movement_state);
                // 提供车辆起点速度信息
                this->expected_state_.setVehicleCurrentMovement(current_movement_state);
                // 提供车辆当前位置在轨迹中下标信息(此处一定是0)
                this->expected_state_.setVehicleCurrentPositionIndexInTrajectory(0);
                // 对期望状态进行重新的路径规划
                this->updateLaneTrajectoryforStates(&(this->expected_state_), expected_lane, current_position_in_world, current_movement_state, current_vehicle_kappa);
                // 判断期望状态起始点是否可行
                double head_offset = this->expected_state_.getFrenetTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][0].position_.y_ + sin(this->expected_state_.getFrenetTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][0].theta_) * 0.5 * this->vehicle_length_;
                double theta_offset = this->expected_state_.getFrenetTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][0].theta_;
                if (!Tools::isSmall(std::fabs(theta_offset), TO_EXPECT_STATE_MAX_THETA_OFFSET) && !Tools::isSmall(std::fabs(head_offset), TO_EXPECT_STATE_MAX_HEAD_OFFSET) && !ignore_offset) {
                    LOG(INFO) << "偏移过大，不考虑切换到期望状态";
                    this->expected_state_.disable();
                } else {
                    // 确定状态的优先级(TODO)
                    this->expected_state_.setPriority(PRIORITY_INIT_VALUE_MAX);
                    // 初始化状态的安全性
                    this->expected_state_.setSafety(true);
                    // 确定道路速度期望
                    // 得到当前位置下标
                    size_t current_position_index_in_expected_lane = expected_lane.findCurrenPositionIndexInLane(current_position_in_world.position_.x_, current_position_in_world.position_.y_);
                    // 得到未来下标
                    size_t future_position_index_in_expected_lane = current_position_index_in_expected_lane + static_cast<size_t>(Tools::normalForeseenDistance(std::max(current_movement_state.velocity_, this->expected_velocity_upper_bound_), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
                    assert(future_position_index_in_expected_lane < expected_lane.getLaneVelocityLimitation().size());
                    // 得到当前下标到未来下标中最低速度限制
                    double velocity_limitation_max = expected_lane.getLaneVelocityLimitation()[current_position_index_in_expected_lane];
                    for (size_t i = current_position_index_in_expected_lane; i < future_position_index_in_expected_lane; i++) {
                        double velocity_limitation = expected_lane.getLaneVelocityLimitation()[i];
                        if (Tools::isSmall(velocity_limitation, velocity_limitation_max)) {
                            velocity_limitation_max = velocity_limitation;
                        }
                    }
                    double velocity_limitation_min = std::min(expected_lane.getLowVelocity()[current_position_index_in_expected_lane], expected_lane.getLowVelocity()[future_position_index_in_expected_lane]);
                    assert(Tools::isLarge(velocity_limitation_max, velocity_limitation_min));
                    this->expected_state_.setExpectedVelocity(expected_lane.getLaneVelocityLimitation()[current_position_index_in_expected_lane], velocity_limitation_max);
                    // 确定车辆速度限制，车辆速度上限由当前速度、横向加速度上限决定
                    double expected_subvehicle_max_velocity = std::min(this->expected_state_.getVehicleStartMovement().velocity_ + VELOCITY_MAX_INCREASE_VALUE, velocity_limitation_max);
                    // 计算横向加速度限速
                    double expected_state_max_kappa =  this->expected_state_.getTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->expected_state_.getTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()])].kappa_;
                    double expected_state_curvature_velocity_limtation = Tools::calcVelocityForMaxNormalAcceleration(expected_state_max_kappa);
                    // 计算横向加加速度限速
                    double expected_state_max_curvature_change_rate = Tools::getCurveMaxCurvatureChangeRate(this->expected_state_.getTrajectory()[this->expected_state_.getChoosedTrajectoryIndex()]);
                    double expected_state_curvature_change_rate_velocity_limtation = Tools::calcVelocityForMaxNormalJerk(expected_state_max_curvature_change_rate);
                    // 得到最终限速
                    expected_subvehicle_max_velocity = std::min(std::min(expected_subvehicle_max_velocity, expected_state_curvature_velocity_limtation), expected_state_curvature_change_rate_velocity_limtation);
                    // 设置限速
                    this->expected_state_.setVelocityLimitation(expected_subvehicle_max_velocity, std::min(velocity_limitation_min, 0.9 * expected_subvehicle_max_velocity));
                    // 确定车辆的加速度上下限制，加速度上限由横向加加速度上限决定
                    double subvehicle_max_acceleration, subvehicle_min_acceleration;
                    subvehicle_max_acceleration = std::min(Tools::velocityToAccelerationReflection(current_movement_state.velocity_), Tools::calcAccelerationForMaxNormalJerk(expected_state_max_kappa, expected_subvehicle_max_velocity));
                    subvehicle_min_acceleration = -COMMON_DECCELERATION;
                    this->expected_state_.setAccelerationLimitationMax(subvehicle_max_acceleration);
                    this->expected_state_.setAccelerationLimitationMin(subvehicle_min_acceleration);
                    // 设置目标点初始值
                    this->expected_state_.setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->expected_state_.getTrajectoryLength());
                    // 设置目标速度初始值(无效值)
                    this->expected_state_.setGoalVelocity(0.0);
                    // 设置期望加速度初始值(无效值)
                    this->expected_state_.setVehicleDynamicPlanningExpectedAcceleration(0.0);

                    // 判断是否遵循交通规则
                    DecisionMaking::RSS::trafficRuleCheck(&(this->expected_state_), this->traffic_rule_obstacles_);

                    // 对期望状态进行动态速度规划
                    // this->velocityPlanningForState(&(this->expected_state_), obstacles, true);

                    VelocityPlanning::VelocityPlanner* v_planner = new VelocityPlanning::VelocityPlanner(&(this->expected_state_));
                    v_planner->runOnce(obstacles);

                    // 进行动态速度规划和碰撞可视化
                    visualization_msgs::MarkerArray delete_collsion_point, collision_point;
                    delete_collsion_point.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
                    std_msgs::ColorRGBA collision_color;
                    collision_color.r = 220.0 / 255.0;
                    collision_color.g = 20.0 / 255.0;
                    collision_color.b = 60.0 / 255.0;
                    collision_color.a = 1.0;
                    for (size_t i = 0; i < collision_rectangles.size(); i++) {
                        Rectangle rectangle = collision_rectangles[i];
                        collision_point.markers.push_back(VisualizationMethods::visualizeRectToMarker(rectangle.center_x_, rectangle.center_y_, rectangle.rotation_, rectangle.width_, rectangle.length_, 0.5, collision_color, i));
                    }
                    std::vector<Rectangle>().swap(collision_rectangles);
                    this->vis_collision_pub_.publish(delete_collsion_point);
                    this->vis_collision_pub_.publish(collision_point);

                    // 期望状态信息更新完毕，
                    if (this->expected_state_.getCapability() && this->expected_state_.getSafety()) {
                        // 期望状态可行且安全，立刻选择期望状态并发布，并重新开始状态保持
                        LOG(INFO) << "期望状态可行且安全，立刻选择期望状态并发布";
                        this->choosed_state_ = this->expected_state_;
                        // 发布新版路径，保持加速度模式
                        this->choosed_state_.publishCurveMsgVelocityMaintain(this->motion_planning_curve_pub_);
                        // 重新进行可视化
                        // 清空之前的可视化
                        visualization_msgs::MarkerArray delete_marker_array;
                        delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
                        this->visualization_pub_.publish(delete_marker_array);

                        // 1.可视化选中状态的路径
                        PathPlanningUtilities::Curve raw_choosed_curve, choosed_curve;
                        if (this->choosed_state_.getExtendedTrajectory().size() == 0) {
                            // 如果没有延伸路径（倒车没有延伸路径）
                            raw_choosed_curve = this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()];
                        } else {
                            // 如果有延伸路径
                            raw_choosed_curve = this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()];
                            raw_choosed_curve.insert(raw_choosed_curve.end(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
                        }
                        choosed_curve.assign(raw_choosed_curve.begin() + this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory(), raw_choosed_curve.begin() + this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory());
                        VisualizationMethods::visualizeChoosedCurveWithBoundary(choosed_curve, this->vehicle_width_, this->vehicle_length_, this->vehicle_rear_axis_center_scale_, VisualizationMethods::VisualizationID::CHOOSED_CURVE_START_ID, 7, this->visualization_pub_);

                        // 2. 可视化道路
                        visualization_msgs::MarkerArray lanes_marker_array;
                        std_msgs::ColorRGBA lane_visualization_color;
                        if (this->center_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->center_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::CENTER_LANE_ID));
                        }
                        if (this->left_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->left_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::LEFT_LANE_ID));
                        }
                        if (this->right_lane_.getLaneExistance()) {
                            lane_visualization_color.r = 0.5;
                            lane_visualization_color.g = 0.5;
                            lane_visualization_color.b = 0;
                            lane_visualization_color.a = 0.75;
                            lanes_marker_array.markers.push_back(VisualizationMethods::visualizeLaneToMarker(this->right_lane_, lane_visualization_color, VisualizationMethods::VisualizationID::RIGHT_LANE_ID));
                        }
                        if (lanes_marker_array.markers.size() > 0) {
                            this->visualization_pub_.publish(lanes_marker_array);
                        }

                        // 3. 可视化状态机
                        double position_x = this->choosed_state_.getVehicleCurrentPosition().position_.x_;
                        double position_y = this->choosed_state_.getVehicleCurrentPosition().position_.y_;
                        VisualizationMethods::visualizeStates(this->states_set_, this->choosed_state_, position_x, position_y, VisualizationMethods::VisualizationID::STATES_START_ID, this->visualization_pub_);

                        // 4. 可视化交通规则
                        VisualizationMethods::visualizeTrafficRules(this->traffic_rule_obstacles_raw_, this->visualization_pub_);

                        // 5. 可视化本车占用区域
                        // 清空之前的可视化
                        visualization_msgs::MarkerArray delete_occupation_marker_array;
                        delete_occupation_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
                        this->vis_occupation_pub_.publish(delete_occupation_marker_array);
                        VisualizationMethods::visualizeSubvehicleOccupationArea(this->choosed_state_, this->vehicle_rear_axis_center_scale_, this->vis_occupation_pub_);
                        return;
                    } else {
                        LOG(INFO) << "期望状态不可行或不安全";
                    }
                }
            } else {
                // 道路不存在时，状态直接失效
                this->expected_state_.disable();
                LOG(INFO) << "期望状态不可行";
            }
        }

        // 如果状态未完成判断当前路径的安全性
        PathPlanningUtilities::Curve remain_curve;
        remain_curve.assign(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin() + current_position_index_in_trajectory, this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
        remain_curve.insert(remain_curve.end(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].begin(), this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].end());
        // 判断是否发生碰撞
        bool is_collision_occur = false;
        size_t traffic_rule_cut_index, obstacle_cut_index, dynamic_obstacle_block_index;
        if (RSS::collisionPositionIndexInCurve(remain_curve, this->vehicle_width_, this->vehicle_length_, this->traffic_rule_obstacles_, &traffic_rule_cut_index)) {
            is_collision_occur = true;
        } else if (RSS::collisionPositionIndexInCurve(remain_curve, this->vehicle_width_, this->vehicle_length_, current_movement_state.velocity_, obstacles, &obstacle_cut_index)){
            is_collision_occur = true;
        } else if (RSS::collisionWithDynamicObstacles(remain_curve, current_movement_state.velocity_, this->vehicle_width_, this->vehicle_length_, obstacles, &dynamic_obstacle_block_index)) {
            is_collision_occur = true;
        }

        if (is_collision_occur) {
            LOG(INFO) << "避障状态不安全，立刻进行重新规划";
            this->current_state_ = this->choosed_state_;
            this->current_state_.setStateCompleted(false);
            return;
        }

        end_time = clock();

        LOG(INFO) << "------------------ one turn of avoidance state maintain end, time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << "ms -------------------";
        std::cout << "------------------ one turn of avoidance state maintain end, time consuming is " << static_cast<double>(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC << "ms -------------------" << std::endl;
    }
}

// 特殊状态保持
void DecisionMaking::SubVehicle::specialStateMaintain() {
    // TOFIX
}