/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

// 更新地图信息，ros服务(TODO)
void DecisionMaking::SubVehicle::updateMapInformation() {
    do {
        // 初始化道路
        this->left_lane_ = Lane();
        this->right_lane_ = Lane();
        this->center_lane_ = Lane();
        // 调用地图服务，提供服务所需参数
        vec_map_cpp_msgs::GetGuidedCurves map_service;
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.frame_id = "world";
        current_pose.header.stamp = ros::Time::now();
        this->current_vehicle_world_position_mutex_.lock();
        // // 以车头的中心点作为中心道路锚点
        // double vehicle_head_x, vehicle_head_y, vehicle_rear_axis_center_scale;
        // this->nh_.getParam("vehicle_rear_axis_center_scale", vehicle_rear_axis_center_scale);
        // vehicle_head_x = this->current_vehicle_world_position_.position_.x_ + vehicle_rear_axis_center_scale * this->vehicle_length_ * cos(this->current_vehicle_world_position_.theta_/2.0);
        // vehicle_head_y = this->current_vehicle_world_position_.position_.y_ + vehicle_rear_axis_center_scale * this->vehicle_length_ * sin(this->current_vehicle_world_position_.theta_/2.0);
        // current_pose.pose.position.x = vehicle_head_x;
        // current_pose.pose.position.y = vehicle_head_y;
        current_pose.pose.position.x = this->current_vehicle_world_position_.position_.x_;
        current_pose.pose.position.y = this->current_vehicle_world_position_.position_.y_;
        current_pose.pose.orientation.x = 0.0;
        current_pose.pose.orientation.y = 0.0;
        current_pose.pose.orientation.z = sin(this->current_vehicle_world_position_.theta_/2.0);
        current_pose.pose.orientation.w = cos(this->current_vehicle_world_position_.theta_/2.0);
        this->current_vehicle_world_position_mutex_.unlock();
        current_pose.pose.position.z = 0;
        LOG(INFO) << "向地图请求的当前位置为" << std::setprecision(14) << current_pose.pose.position.x << "||" << std::setprecision(14) << current_pose.pose.position.y << "||" << std::setprecision(14) << std::atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w) * 2.0;
        std::cout << "向地图请求的当前位置为" << std::setprecision(14) << current_pose.pose.position.x << "||" << std::setprecision(14) << current_pose.pose.position.y << "||" << std::setprecision(14) << std::atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w) * 2.0 << std::endl;
        map_service.request.current_pose = current_pose;
        bool current_pose_ignore_orientation;
        this->nh_.getParam("current_pose_ignore_orientation", current_pose_ignore_orientation);
        map_service.request.current_pose_orientation_unknown = current_pose_ignore_orientation;
        // this->destination_mutex_.lock();
        // map_service.request.goal_pose = this->destination_pose_;
        // this->destination_mutex_.unlock();
        // bool goal_pose_ignore_orientation;
        // this->nh_.getParam("goal_pose_ignore_orientation", goal_pose_ignore_orientation);
        // map_service.request.goal_pose_orientation_unknown = goal_pose_ignore_orientation;
        map_service.request.point_margin = LANE_GAP_DISTANCE;
        std::vector<size_t> ignore_traffic_obstacle_ids;
        for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
            if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::INVALID) {
                ignore_traffic_obstacle_ids.push_back(this->traffic_rule_obstacles_raw_[i].id);
                LOG(INFO) << "删除的id为" << this->traffic_rule_obstacles_raw_[i].id;
            }
        }

        map_service.request.ignored_ids = ignore_traffic_obstacle_ids;
        
        int failed_call_map_service_num = 0;
        while (true) {
            bool is_success = this->map_service_client_.call(map_service);
            if (is_success) {
                break;
            }
            failed_call_map_service_num += 1;
            if (failed_call_map_service_num >= 100) {
                LOG(INFO) << "Called the map service failed 100 times, process has exited.";
                std::cout << "Called the map service failed 100 times, process has exited." <<std::endl;
                exit(0);
            }
        }
        // Judge whether the goal point is unreachable
        if (map_service.response.status == vec_map_cpp_msgs::GetGuidedCurvesResponse::GOAL_UNREACHABLE) {
            LOG(INFO) << "The goal is unreachable";
            // 如果任务结束，重置目标点和开始任务标志位
            this->mission_start_mutex_.lock();
            this->MISSION_START_FLAG_ = false;
            this->mission_start_mutex_.unlock();
            // 调用到达目标点服务
            std_srvs::Trigger destination_reached_service;
            this->destination_reached_service_client_.call(destination_reached_service);
            if (destination_reached_service.response.success == true) {
                LOG(INFO) << "Mission failed due to the unreacheable goal, skip to the next mission";
            } else {
                LOG(INFO) << "任务结束失败";
            }
            break;
        }
        // 获取服务返回值，对于起点和终点的判断
        if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::WRONG_ORIENTATION) {
            LOG(INFO) << "地图服务得到的起点方向错误";
            continue;
        } else if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::OUT_MAP) {
            LOG(INFO) << "地图服务得到的起点在地图外";
            continue;
        }

        // 判断车道长度
        if (map_service.response.center_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
            LOG(INFO) << "给出中间道路过短" << map_service.response.center_lane.geometry.points.size();
            continue;
        }

        // 判断是否在停车位内
        this->ROTATE_AND_REVERSE_ENABLE_FLAG_ = false;
        if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::PARKING_ISLAND) {
            LOG(INFO) << "在停车位内,允许转向和倒车";
            this->ROTATE_AND_REVERSE_ENABLE_FLAG_ = true;
        }

        // if (map_service.response.goal_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::WRONG_ORIENTATION) {
        //     LOG(INFO) << "地图服务得到的终点方向错误";
        // } else if (map_service.response.goal_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::OUT_MAP) {
        //     LOG(INFO) << "地图服务得到的终点在地图外";
        // }

        // if (map_service.response.goal_reachable == false) {
        //     LOG(INFO) << "地图服务得到的终点不可达";
        // }

        // 获取服务的返回值,首先是中间车道
        this->center_lane_.enable();
        // 判断是否需要连续换道
        if (map_service.response.multiple_lane_changes) {
            // 需要
            std::vector<double> max_speeds, min_speeds;
            for (auto max_speed: map_service.response.center_lane.max_speeds) {
                max_speeds.push_back(0.5 * max_speed);
            }
            this->center_lane_.setLaneVelocityLimitation(max_speeds);

            for (auto min_speed: map_service.response.center_lane.min_speeds) {
                min_speeds.push_back(0.5 * min_speed);
            }
            this->center_lane_.setLowVelocity(min_speeds);
        } else {
            // 不需要
            this->center_lane_.setLaneVelocityLimitation(map_service.response.center_lane.max_speeds);
            this->center_lane_.setLowVelocity(map_service.response.center_lane.min_speeds);
        }
        
        // 附加打灯情况
        this->center_lane_.setTurn(map_service.response.center_lane.turn);
        // for (size_t i = 0; i < map_service.response.center_lane.max_speeds.size(); i++) {
        //     std::cout << "asfwgax " << map_service.response.center_lane.min_speeds[i] << std::endl;
        // }
        this->center_lane_.generateLaneCenter(map_service.response.center_lane.geometry);
        this->center_lane_.generateLaneTransMatrix();
        // 判断左侧车道是否存在
        if (map_service.response.left_lane_exist) {
            // 判断左道长度
            if (map_service.response.left_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
                LOG(INFO) << "给出左侧道路过短" << map_service.response.left_lane.geometry.points.size();
                continue;
            }
            this->left_lane_.enable();
            // 判断是否需要连续换道
            if (map_service.response.multiple_lane_changes) {
                // 需要
                std::vector<double> max_speeds, min_speeds;
                for (auto max_speed: map_service.response.left_lane.max_speeds) {
                    max_speeds.push_back(0.5 * max_speed);
                }
                this->left_lane_.setLaneVelocityLimitation(max_speeds);

                for (auto min_speed: map_service.response.left_lane.min_speeds) {
                    min_speeds.push_back(0.5 * min_speed);
                }
                this->left_lane_.setLowVelocity(min_speeds);
            } else {
                // 不需要
                this->left_lane_.setLaneVelocityLimitation(map_service.response.left_lane.max_speeds);
                this->left_lane_.setLowVelocity(map_service.response.left_lane.min_speeds);
            }
            this->left_lane_.generateLaneCenter(map_service.response.left_lane.geometry);
            this->left_lane_.generateLaneTransMatrix();
            // 附加打灯情况
            this->left_lane_.setTurn(map_service.response.left_lane.turn);
        } else {
            this->left_lane_.disable();
        }
        // 判断右侧车道是否存在
        if (map_service.response.right_lane_exist) {
            // 判断右道长度
            if (map_service.response.right_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
                LOG(INFO) << "给出右侧道路过短" << map_service.response.right_lane.geometry.points.size();
                continue;
            }
            this->right_lane_.enable();
            // 判断是否需要连续换道
            if (map_service.response.multiple_lane_changes) {
                // 需要
                std::vector<double> max_speeds, min_speeds;
                for (auto max_speed: map_service.response.right_lane.max_speeds) {
                    max_speeds.push_back(0.5 * max_speed);
                }
                this->right_lane_.setLaneVelocityLimitation(max_speeds);

                for (auto min_speed: map_service.response.right_lane.min_speeds) {
                    min_speeds.push_back(0.5 * min_speed);
                }
                this->right_lane_.setLowVelocity(min_speeds);
            } else {
                // 不需要
                this->right_lane_.setLaneVelocityLimitation(map_service.response.right_lane.max_speeds);
                this->right_lane_.setLowVelocity(map_service.response.right_lane.min_speeds);
            }
            this->right_lane_.generateLaneCenter(map_service.response.right_lane.geometry);
            this->right_lane_.generateLaneTransMatrix();
            // 附加打灯情况
            this->right_lane_.setTurn(map_service.response.right_lane.turn);
        } else {
            this->right_lane_.disable();
        }
        // 确定引导道路类型
        this->guidance_type_ = map_service.response.guidance;
        LOG(INFO) << "guided type: "<< this->guidance_type_;
        // std::cout << "guided type raw: "<< Lane::GuidanceType::ALL_AVAILABLE << std::endl;
        if (this->guidance_type_ == Lane::GuidanceType::CHANGE_LEFT) {
            this->right_lane_.disable();
        } else if (this->guidance_type_ == Lane::GuidanceType::CHANGE_RIGHT) {
            this->left_lane_.disable();
        }

        // 设置道路优先级
        // 确定中间道的优先级
        switch (this->guidance_type_) {
            case Lane::GuidanceType::CHANGE_LEFT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
            case Lane::GuidanceType::KEEP_CENTER:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::CHANGE_RIGHT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
            case Lane::GuidanceType::CENTER_LEFT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::CENTER_RIGHT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::ALL_AVAILABLE:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            default:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
        }
        LOG(INFO) << "中间车道优先级" << this->center_lane_.getLanePriority();
        // 设置左道优先级
        if (this->left_lane_.getLaneExistance()) {
            // 如果左道存在
            switch (this->guidance_type_) {
                case Lane::GuidanceType::CHANGE_LEFT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                    break;
                case Lane::GuidanceType::KEEP_CENTER:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
                case Lane::GuidanceType::CHANGE_RIGHT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MIN);
                    break;
                case Lane::GuidanceType::CENTER_LEFT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                case Lane::GuidanceType::CENTER_RIGHT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_LOW);
                    break;
                case Lane::GuidanceType::ALL_AVAILABLE:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                default:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
            }
            LOG(INFO) << "左边车道优先级" << this->left_lane_.getLanePriority();
        }
        // 设置右道优先级
        if (this->right_lane_.getLaneExistance()) {
            switch (this->guidance_type_) {
                case Lane::GuidanceType::CHANGE_LEFT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MIN);
                    break;
                case Lane::GuidanceType::KEEP_CENTER:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
                case Lane::GuidanceType::CHANGE_RIGHT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                    break;
                case Lane::GuidanceType::CENTER_LEFT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_LOW);
                    break;
                case Lane::GuidanceType::CENTER_RIGHT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                case Lane::GuidanceType::ALL_AVAILABLE:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                default:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
            }
            LOG(INFO) << "右边车道优先级" << this->right_lane_.getLanePriority();
        }

        // 当前位置最大限速
        this->expected_velocity_upper_bound_ = this->center_lane_.getLaneVelocityLimitation()[0];

        // 获取交通规则生成的障碍物
        this->traffic_rule_obstacles_raw_ = map_service.response.virtual_obstacles;
        // for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
        //     if (this->traffic_rule_obstacles_raw_[i].points.size() > 0) {
        //         LOG(INFO) << " traffic obstacle is " << this->traffic_rule_obstacles_raw_[i].points[0].x << "||" << this->traffic_rule_obstacles_raw_[i].points[0].y;
        //         std::cout << " traffic obstacle is " << this->traffic_rule_obstacles_raw_[i].points[0].x << "||" << this->traffic_rule_obstacles_raw_[i].points[0].y << std::endl;
        //     } else {
        //         LOG(INFO) << "traffic obstacle size error";
        //         std::cout << "traffic obstacle size error" << std::endl;
        //     }

        // }

        // 判断是否为单车道
        if (!map_service.response.right_lane_exist && !map_service.response.left_lane_exist) {
            this->is_single_lane_ = true;
        } else {
            this->is_single_lane_ = false;
        }
        // std::cout << "hgugugu" << std::endl;

        // 获取是否允许避障标志位
        this->is_avoidance_capable_ = map_service.response.obstacle_avoidance_allowed;
        // this->is_avoidance_capable_ = true;
        // DEBUG
        // if (this->is_single_lane_) {
        //     this->is_avoidance_capable_ = false;
        // } else {
        //     this->is_avoidance_capable_ = true;
        // }
        // std::cout << "sgwbafsha" << std::endl;
        // 离终点的距离
        this->distance_to_goal_ = map_service.response.distance_to_goal;
        // TOFIX判断长度是否足够
        double shortest_distance = std::min(map_service.response.distance_to_stop, map_service.response.distance_to_goal);
        if (Tools::isLarge(shortest_distance, std::max(NAVIGATION_LENGTH_ENOUGH_MIN_VALUE, this->expected_velocity_upper_bound_ * NAVIGATION_LENGTH_ENOUGH_MIN_COEF))) {
            this->is_length_enough_ = true;
        } else {
            this->is_length_enough_ = false;
        }
        // this->is_length_enough_ = true;
        this->remain_distance_ = shortest_distance;
        LOG(INFO) << "可以自由行驶的距离还剩" << shortest_distance << "，是否足够" << this->is_length_enough_;
        std::cout << "可以自由行驶的距离还剩" << shortest_distance << "，是否足够" << this->is_length_enough_ << std::endl;

        // if (map_service.response.extra_flags != "") {
        //     if (map_service.response.extra_flags == "HIGHWAY_DOWN_MIDDLE") {
        //         GLOBAL_IS_IN_CHECK_ = true;
        //         // std::cout << "在临检区" << std::endl;
        //     } else if (map_service.response.extra_flags == "BLIND_ZONE"){
        //         GLOBAL_IS_IN_SLOW_ = true;
        //         // std::cout << "不在临检区" << std::endl;
        //     } else if (map_service.response.extra_flags == "OVERTAKE") {
        //         GLOBAL_IS_IN_OVERTAKE_ = true;
        //     }
        // } else {
        //     GLOBAL_IS_IN_CHECK_ = false;
        //     GLOBAL_IS_IN_SLOW_ = false;
        //     GLOBAL_IS_IN_OVERTAKE_ = false;
        //     GLOBAL_IS_IN_CHECK_COUNT_FLAG_ = 0;
        // }
        break;
    } while (true);

}

// 给每一个状态在不同道路上的路径进行更新
void DecisionMaking::SubVehicle::updateLaneTrajectoryforStates(StandardState *standard_state, const Lane &lane, const PathPlanningUtilities::VehicleState &start_point_in_world, const PathPlanningUtilities::VehicleMovementState &start_point_movement, double start_point_kappa) {
    // 0. 道路信息初始化(加快后续程序速度)
    std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane.getLaneCoordnation();
    std::vector<TransMatrix> trans_matrixes = lane.getLaneTransMatrix();
    // std::cout << "jojojo0: " << lane_coordination.size() << std::endl;
    // 1. 得到frenet坐标系起点
    size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, start_point_in_world.position_);
    TransMatrix trans_matrix = trans_matrixes[start_index_of_lane];
    Eigen::Vector2d start_point_in_world_v2d(start_point_in_world.position_.x_, start_point_in_world.position_.y_);
    Eigen::Vector2d start_point_position_in_frenet;
    Tools::transferPointCoordinateSystem(trans_matrix, start_point_in_world_v2d, &start_point_position_in_frenet);
    double start_point_theta_in_frenet;
    Tools::transferThetaCoordinateSystem(trans_matrix, start_point_in_world.theta_, &start_point_theta_in_frenet);
    double start_point_kappa_in_frenet;
    start_point_kappa_in_frenet = start_point_kappa - cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*1.0/(1.0/lane_coordination[start_index_of_lane].worldpos_.kappa_ - start_point_position_in_frenet(1));
    PathPlanningUtilities::VehicleState start_point_in_frenet;
    start_point_in_frenet.position_.x_ = start_point_position_in_frenet(0);
    start_point_in_frenet.position_.y_ = start_point_position_in_frenet(1);
    start_point_in_frenet.theta_ = start_point_theta_in_frenet;
    start_point_in_frenet.kappa_ = start_point_kappa_in_frenet;
    // std::cout << "jojojo1: " << start_point_in_frenet.position_.x_ << "||" << start_point_in_frenet.position_.y_ << "||" << start_point_in_frenet.theta_ << "||" << start_point_in_frenet.kappa_ << "||" << std::endl;
    // 2. 得到frenet坐标系下的终点组
    double motion_planning_length = Tools::normalMotionDistance(start_point_movement.velocity_, SUBVEHICLE_COMMON_DECCELERATION, CONSTANT_DISTANCE);
    size_t motion_planning_index_length = static_cast<int>(motion_planning_length/LANE_GAP_DISTANCE);
    size_t goal_index_of_lane = start_index_of_lane + motion_planning_index_length;
    LOG(INFO) <<"Total goal index of lane : "<< goal_index_of_lane+EXTENDED_PATH_SMOOTH_INDEXES << " lane coordination length : "<< lane_coordination.size();
    assert(goal_index_of_lane + EXTENDED_PATH_SMOOTH_INDEXES < lane_coordination.size());
    // // DEBUG计算道路的最大曲率
    // double lane_max_kappa = 0.0;
    // for (size_t i = start_index_of_lane; i <= goal_index_of_lane; i++) {
    //     double kappa = lane.getLaneCoordnation()[i].worldpos_.kappa_;
    //     if (Tools::isLarge(std::fabs(kappa), std::fabs(lane_max_kappa))) {
    //         lane_max_kappa = kappa;
    //     }
    // }
    // LOG(INFO) << "lane max kappa is " << lane_max_kappa;

    // 从起点到终点每间隔固定长度进行采样
    size_t sample_length = static_cast<size_t>(SAMPLE_LENGTH / LANE_GAP_DISTANCE);
    size_t sampled_goal_index_of_lane = start_index_of_lane + static_cast<size_t>(std::max(static_cast<int>(motion_planning_index_length * LOCAL_PLANNING_LONGITUDE_SAMPLE_COEF) - static_cast<int>(LOCAL_PLANNING_LONGITUDE_SAMPLE_MIN_LENGTH / LANE_GAP_DISTANCE), static_cast<int>(0)));
    assert(sampled_goal_index_of_lane + sample_length <= goal_index_of_lane);
    // 3. 开始规划本状态的路径组
    std::vector<PathPlanningUtilities::Curve> curves;
    std::vector<PathPlanningUtilities::Curve> frenet_curves;
    std::vector<PathPlanningUtilities::Curve> extended_curves;
    // std::cout << "jojo2: " << start_index_of_lane << " " << goal_index_of_lane << std::endl;
    while(sampled_goal_index_of_lane < goal_index_of_lane) {
        // 进行采样目标点更新
        sampled_goal_index_of_lane = sampled_goal_index_of_lane + sample_length;
        if (sampled_goal_index_of_lane + sample_length < goal_index_of_lane) {
            // do nothing
        } else {
            sampled_goal_index_of_lane = goal_index_of_lane;
        }
        // std::cout << "jojo3: " << sampled_goal_index_of_lane << std::endl;
        // 得到采样目标点信息
        PathPlanningUtilities::CurvePoint goal_point_in_frenet;
        goal_point_in_frenet.position_.x_ = lane.getLaneCenterPathInFrenet()[sampled_goal_index_of_lane].x_;
        goal_point_in_frenet.position_.y_ = lane.getLaneCenterPathInFrenet()[sampled_goal_index_of_lane].y_;
        goal_point_in_frenet.theta_ = 0.0;
        goal_point_in_frenet.kappa_ = 0.0;
        // 用quintic polynomial方法进行路径生成(distance 是调整参数 TODO)
        PathPlanningUtilities::Curve trajectory_curve_in_frenet;
        PathPlanningUtilities::QParameters params;
        double distance = PathPlanningUtilities::calcDistance(start_point_in_frenet.position_, goal_point_in_frenet.position_);
        PathPlanningUtilities::QuinticSplinePlanning::getParametersByMileageConstraint(start_point_in_frenet, goal_point_in_frenet, distance, 5, params);
        int point_number = sampled_goal_index_of_lane - start_index_of_lane + 1;
        PathPlanningUtilities::pathGenerator* pg_ = new PathPlanningUtilities::pathGenerator(point_number);
        pg_->calcCurve(params, trajectory_curve_in_frenet);
        // 将路径从frenet坐标系转化道world坐标系
        PathPlanningUtilities::Curve trajectory_curve_in_world;
        for (size_t i = 0; i < trajectory_curve_in_frenet.size(); i++) {
            PathPlanningUtilities::CurvePoint point_in_frenet = trajectory_curve_in_frenet[i];
            PathPlanningUtilities::CurvePoint point_in_world;
            // 第一步得到当前位置对应的映射
            TransMatrix corresponding_trans_matrix = trans_matrixes[i + start_index_of_lane];
            // 第二步变换坐标
            Eigen::Vector2d vector_frenet(point_in_frenet.position_.x_, point_in_frenet.position_.y_);
            Eigen::Vector2d vector_world;
            vector_world = corresponding_trans_matrix.rotation_.inverse() * (vector_frenet - corresponding_trans_matrix.trans_);
            point_in_world.position_.x_ = vector_world(0);
            point_in_world.position_.y_ = vector_world(1);
            // 第三步变换角度和曲率
            double world_sin_theta = corresponding_trans_matrix.rotation_.inverse()(1, 0)*cos(point_in_frenet.theta_) + corresponding_trans_matrix.rotation_.inverse()(1, 1)*sin(point_in_frenet.theta_);
            double world_cos_theta = -corresponding_trans_matrix.rotation_.inverse()(1, 0)*sin(point_in_frenet.theta_) + corresponding_trans_matrix.rotation_.inverse()(1, 1)*cos(point_in_frenet.theta_);
            point_in_world.theta_ = atan2(world_sin_theta, world_cos_theta);
            point_in_world.kappa_ = point_in_frenet.kappa_ + cos(point_in_frenet.theta_) * cos(point_in_frenet.theta_) * cos(point_in_frenet.theta_) / (1.0 / lane_coordination[i + start_index_of_lane].worldpos_.kappa_ - point_in_frenet.position_.y_);
            // 将点加入路径
            trajectory_curve_in_world.push_back(point_in_world);
        }

        // 给路径补点，直到路径长度等于目标长度
        for (size_t i = sampled_goal_index_of_lane + 1; i <= goal_index_of_lane; i++) {
            trajectory_curve_in_world.push_back(lane_coordination[i].worldpos_);
            // // 记录对应点
            // coordinate_indexes.push_back(i);
        }
        // 自此路径生成完毕，提出路径延伸段
        // 计算路径延伸段的长度
        PathPlanningUtilities::Curve extended_trajectory_curve_in_world;  // 最终延伸路径
        for (size_t i = goal_index_of_lane + 1; i < lane_coordination.size(); i++) {
            PathPlanningUtilities::CurvePoint curve_point;
            curve_point = lane_coordination[i].worldpos_;
            extended_trajectory_curve_in_world.push_back(curve_point);
        }

        // 验证路径中不存在重复点
        for (size_t i = 0; i < trajectory_curve_in_world.size(); i++) {
            if (i < trajectory_curve_in_world.size() - 1) {
                assert(!Tools::isZero(PathPlanningUtilities::calcDistance(trajectory_curve_in_world[i].position_, trajectory_curve_in_world[i + 1].position_)));
            } else {
                assert(!Tools::isZero(PathPlanningUtilities::calcDistance(trajectory_curve_in_world[i].position_, extended_trajectory_curve_in_world[0].position_)));
                assert(Tools::isZero(PathPlanningUtilities::calcDistance(trajectory_curve_in_world[i].position_, lane_coordination[goal_index_of_lane].worldpos_.position_)));
            }
        }
        
        // // DEBUG
        // assert(cut_trajectory_curve_in_world.size() == coordinate_indexes.size());
        // for (size_t i = 0; i < cut_trajectory_curve_in_world.size(); i++) {
        //     // 打印生成路径信息
        //     LOG(INFO) << "==++++++++++++++++++++生成路径的第" << i << "个点：++++++++++++++++++++++++++";
        //     LOG(INFO) << "坐标为" << cut_trajectory_curve_in_world[i].position_.x_ << "||" << cut_trajectory_curve_in_world[i].position_.y_ << "||" << cut_trajectory_curve_in_world[i].theta_ << "||" << cut_trajectory_curve_in_world[i].kappa_;
        //     LOG(INFO) << "对应道路点下标为" << coordinate_indexes[i] << "，坐标为" << lane.getLaneCoordnation()[i].worldpos_.position_.x_ << "||" << lane.getLaneCoordnation()[i].worldpos_.position_.y_ << "||" << lane.getLaneCoordnation()[i].worldpos_.theta_ << "||" << lane.getLaneCoordnation()[i].worldpos_.kappa_;
        // }

        // 进行赋值
        curves.push_back(trajectory_curve_in_world);
        frenet_curves.push_back(trajectory_curve_in_frenet);
        extended_curves.push_back(extended_trajectory_curve_in_world);
    }
    // 数据打印
    LOG(INFO) << "frenet start point: " << start_point_in_frenet.position_.x_ << "||" << start_point_in_frenet.position_.y_ << "||" << start_point_in_frenet.theta_ << "||" << start_point_in_frenet.kappa_;
    LOG(INFO) << "frenet start index: "<< start_index_of_lane;
    LOG(INFO) << "planning length: "<< motion_planning_length;
    LOG(INFO) << "frenet goal index: "<< goal_index_of_lane;
    LOG(INFO) << "frenet goal point: " << lane.getLaneCenterPathInFrenet()[goal_index_of_lane].x_ << "||" << lane.getLaneCenterPathInFrenet()[goal_index_of_lane].y_ << "||" << 0.0 << "||" << 0.0;
    // std::cout << "jojo4 " << curves.size() << std::endl;
    assert(curves.size() == frenet_curves.size() && curves.size() == extended_curves.size());
    // 4. 路径生成完成，给状态进行赋值
    standard_state->setTrajectory(curves);
    standard_state->setFrenetTrajectory(frenet_curves);
    standard_state->setExtendedTrajectory(extended_curves);

    // // 5. 确定路径组中的选中路径
    // // 确定条件一与永久墙体无撞击，条件二最大曲率较小
    // std::vector<size_t> uncollision_curves_index;
    // for (size_t i = 0; i < curves.size(); i++) {
    //     // 得到路径的占用区域
    //     RSS::OccupationArea subvehicle_occupation_area = RSS::OccupationArea(curves[i], this->vehicle_width_ * 1.0, this->vehicle_length_ * 1.05);
    //     bool is_collision = false;
    //     for (size_t j = 0; j < this->traffic_rule_obstacles_raw_.size(); j++) {
    //         if (this->traffic_rule_obstacles_raw_[j].mode == vec_map_cpp_msgs::VirtualObstacle::PERMANENT) {
    //             RSS::OccupationArea traffic_rule_obstacle_occupation_area = RSS::OccupationArea(this->traffic_rule_obstacles_raw_[j]);
    //             size_t subvehicle_interact_index, obstacle_interact_index;
    //             if (occupationInteractionJudgement(subvehicle_occupation_area, traffic_rule_obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
    //                 // 发生碰撞
    //                 is_collision = true;
    //                 break;
    //             }
    //         }
    //     }
    //     if (!is_collision) {
    //         uncollision_curves_index.push_back(i);
    //     }
    // }
    // std::cout << "jojo5 " << uncollision_curves_index.size() << std::endl;
    // if (uncollision_curves_index.size() == 0) {
    //     LOG(INFO) << "状态" << DIC_STATE_NAME[standard_state->getStateName()] << "全部路径与边界碰撞，状态不可行";
    //     standard_state->disable();
    // } else {
    //     // 从未碰撞的曲线中选出一条最优曲线
    //     // 1. 最大横向偏移越小越好
    //     // 2. 最大角度越小越好
    //     // 3. 最大曲率较小
    //     // 4. 最大初始横向加速度小于阈值
    //     // 5. 最大初始横向加加速度小于阈值

    // }
    // 5. 确定路径组中的选中路径
    // 第一步计算可行的最大曲率(正数)
    double tmp_velocity = std::max(start_point_movement.velocity_, MIN_VELOCITY_FOR_ACCEPTABLE_CURVATURE_CALCULATION);
    double max_acceptable_kappa = MAX_NORMAL_ACCELERATION / (tmp_velocity * tmp_velocity);

    LOG(INFO) << "max_acceptable_kappa " << max_acceptable_kappa;

    // // 第二步排除曲率大于阈值的线
    // std::vector<size_t> acceptable_curves_index;
    // for (size_t i = 0; i < curves.size(); i++) {
    //     LOG(INFO) << "curve " << i << "max kappa is " << curves[i][Tools::calcMaxKappaIndexForCurve(curves[i])].kappa_ << ", index is " << Tools::calcMaxKappaIndexForCurve(curves[i]);
    //     // for (size_t j = 0; j < frenet_curves[i].size(); j++) {
    //     //     LOG(INFO) << j << " theta: "<< frenet_curves[i][j].theta_ << ", frenet kappa: " << frenet_curves[i][j].kappa_;
            
    //     // }
    //     if (!Tools::isLarge(std::fabs(curves[i][Tools::calcMaxKappaIndexForCurve(curves[i])].kappa_),std::fabs(max_acceptable_kappa))) {
    //         acceptable_curves_index.push_back(i);
    //     }
    // }
    std::vector<size_t> acceptable_curves_index;
    for (size_t i = 0; i < curves.size(); i++) {
        acceptable_curves_index.push_back(i);
    }

    if (acceptable_curves_index.size() == 0) {
        LOG(INFO) << "无可选道路状态不可行";
        standard_state->setChoosedTrajectoryIndex(curves.size() - 1);
        standard_state->disable();
    } else {
        // 选出最优路径（角度小，偏移小）
        double min_score = MAX_VALUE;
        size_t choosed_index = 0; 
        for (size_t i = 0; i < acceptable_curves_index.size(); i++) {
            double average_theta = 0.0, average_offset = 0.0;
            for (size_t j = 0; j < frenet_curves[acceptable_curves_index[i]].size(); j++) {
                // 计算轨迹在frenet坐标系下的平均角度和平均偏移
                average_theta = average_theta + std::fabs(frenet_curves[acceptable_curves_index[i]][j].theta_);
                average_offset = average_offset + std::fabs(frenet_curves[acceptable_curves_index[i]][j].position_.y_ - frenet_curves[acceptable_curves_index[i]][0].position_.y_) + std::fabs(frenet_curves[acceptable_curves_index[i]][j].position_.y_ - frenet_curves[acceptable_curves_index[i]][frenet_curves[acceptable_curves_index[i]].size() - 1].position_.y_);
            }
            average_theta = average_theta / frenet_curves[acceptable_curves_index[i]].size();
            average_offset = average_offset / frenet_curves[acceptable_curves_index[i]].size();
            // LOG(INFO) << "index " << acceptable_curves_index[i] << " average_offset " << average_offset;
            // LOG(INFO) << "index " << acceptable_curves_index[i] << " average_theta " << average_theta;
            double score = average_offset + LONGITUDE_PATH_SELECTION_YAW_WEIGHT * average_theta;
            if (Tools::isSmall(score, min_score)) {
                min_score = score;
                choosed_index = acceptable_curves_index[i];
            }
        }
        LOG(INFO) << "choosed index is " << choosed_index;
        standard_state->setChoosedTrajectoryIndex(choosed_index);

        LOG(INFO) << "生成路径的第一个点坐标为" << standard_state->getTotalTrajectory()[0].position_.x_ << "||" << standard_state->getTotalTrajectory()[0].position_.y_ << "||" << standard_state->getTotalTrajectory()[0].theta_ << "||" << standard_state->getTotalTrajectory()[0].kappa_;
    }
}

// 更新状态
// 1. 根据当前状态判断可行状态（不包含停车）
// 2. 给每一个可行状态赋值起终点、起点速度和规划出的局部路径
// 3. 给每一个可行状态优先级
void DecisionMaking::SubVehicle::updateStates() {
    // 为规划提供地图信息
    this->updateMapInformation();
    // 打印信息
    std::cout << "MAP SERVICE FINISH" << std::endl;
    std::cout << "center lane length exist: " << this->center_lane_.getLaneExistance() << std::endl;
    if (this->center_lane_.getLaneExistance()) {
        std::cout << "center lane length: " << this->center_lane_.getLaneCoordnation().size() << std::endl;
    } else {
        return;
    }
    std::cout << "left lane length exist: " << this->left_lane_.getLaneExistance() << std::endl;
    if (this->left_lane_.getLaneExistance()) {
        std::cout << "left lane length: " << this->left_lane_.getLaneCoordnation().size() << std::endl;
    }
    std::cout << "right lane length exist: " << this->right_lane_.getLaneExistance() << std::endl;
    if (this->right_lane_.getLaneExistance()) {
        std::cout << "right lane length: " << this->right_lane_.getLaneCoordnation().size() << std::endl;
    }
    std::cout << "guidance type: " << GUIDE_TYPE_NAME[this->guidance_type_] << std::endl;
    std::cout << "current position lane velocity limitation: " << this->expected_velocity_upper_bound_ << std::endl;

    LOG(INFO) << "MAP SERVICE FINISH";
    LOG(INFO) << "center lane length exist: " << this->center_lane_.getLaneExistance();
    if (this->center_lane_.getLaneExistance()) {
        LOG(INFO) << "center lane length: " << this->center_lane_.getLaneCoordnation().size();
    } else {
        return;
    }
    LOG(INFO) << "left lane length exist: " << this->left_lane_.getLaneExistance();
    if (this->left_lane_.getLaneExistance()) {
        LOG(INFO) << "left lane length: " << this->left_lane_.getLaneCoordnation().size();
    }
    LOG(INFO) << "right lane length exist: " << this->right_lane_.getLaneExistance();
    if (this->right_lane_.getLaneExistance()) {
        LOG(INFO) << "right lane length: " << this->right_lane_.getLaneCoordnation().size();
    }
    LOG(INFO) << "guidance type: " << GUIDE_TYPE_NAME[this->guidance_type_];
    LOG(INFO) << "current position lane velocity limitation: " << this->expected_velocity_upper_bound_;

    // 获取世界坐标系下车辆的信息
    PathPlanningUtilities::VehicleState start_point_in_world;
    this->current_vehicle_world_position_mutex_.lock();
    start_point_in_world = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    PathPlanningUtilities::VehicleMovementState start_point_movement;
    this->current_vehicle_movement_mutex_.lock();
    start_point_movement = this->current_vehicle_movement_;
    this->current_vehicle_movement_mutex_.unlock();
    this->current_vehicle_kappa_mutex_.lock();
    double start_point_kappa = this->current_vehicle_kappa_;
    start_point_in_world.kappa_ = start_point_kappa;
    this->current_vehicle_kappa_mutex_.unlock();

    // 打印车辆信息
    LOG(INFO) << "开始时车辆的GPS坐标为 " << start_point_in_world.position_.x_ << "||" <<  start_point_in_world.position_.y_;
    LOG(INFO) << "开始时车辆的GPS朝向为 " << start_point_in_world.theta_;
    LOG(INFO) << "开始时车辆的方向盘给出的曲率为 " << start_point_in_world.kappa_;
    LOG(INFO) << "开始时车辆的速度为 " << start_point_movement.velocity_;

    // 初始化直行状态(查找直行是否在当前状态的邻居中,如果在则进行初始化)
    if (this->current_state_.getStateName() == StateNames::FORWARD || Tools::searchNeighborStates(this->current_state_.getNeighborStates(), StateNames::FORWARD)) {
        this->states_set_[StateNames::FORWARD].setRespondingLane(this->center_lane_);
        // 直行状态为有效状态
        this->states_set_[StateNames::FORWARD].enable();
        // 提供车辆形状信息
        this->states_set_[StateNames::FORWARD].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
        // 提供起始点信息
        this->states_set_[StateNames::FORWARD].setVehicleStartState(start_point_in_world);
        this->states_set_[StateNames::FORWARD].setVehicleStartMovement(start_point_movement);
        // 提供车辆当前速度信息
        this->states_set_[StateNames::FORWARD].setVehicleCurrentMovement(start_point_movement);
        // 提供车辆当前位置在轨迹中下标信息(此处一定是0)
        this->states_set_[StateNames::FORWARD].setVehicleCurrentPositionIndexInTrajectory(0);
        std::cout << "CENTER LANE PROCESSING" << std::endl;
        LOG(INFO) << "CENTER LANE PROCESSING";
        // 提供路径信息
        this->updateLaneTrajectoryforStates(&(this->states_set_[StateNames::FORWARD]), this->center_lane_, start_point_in_world, start_point_movement, start_point_kappa);
        // 初始化状态的安全性，初始化必须为true
        this->states_set_[StateNames::FORWARD].setSafety(true);
        // 确定状态的优先级(TODO)
        this->states_set_[StateNames::FORWARD].setPriority(this->center_lane_.getLanePriority());
        // 确定道路速度期望
        // 得到当前下标
        size_t current_position_index_in_lane = this->center_lane_.findCurrenPositionIndexInLane(start_point_in_world.position_.x_, start_point_in_world.position_.y_);
        // 得到未来下标
        size_t future_position_index_in_lane = current_position_index_in_lane + static_cast<size_t>(Tools::normalForeseenDistance(std::max(start_point_movement.velocity_, this->expected_velocity_upper_bound_), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
        // 计算当前下标到未来下标速度最低限制
        double velocity_limitation_max = this->center_lane_.getLaneVelocityLimitation()[current_position_index_in_lane];
        for (size_t i = current_position_index_in_lane; i < future_position_index_in_lane; i++) {
            double velocity_limitation = this->center_lane_.getLaneVelocityLimitation()[i];
            if (Tools::isSmall(velocity_limitation, velocity_limitation_max)) {
                velocity_limitation_max = velocity_limitation;
            }
        }
        double velocity_limitation_min = std::min(this->center_lane_.getLowVelocity()[current_position_index_in_lane], this->center_lane_.getLowVelocity()[future_position_index_in_lane]);
        assert(Tools::isLarge(velocity_limitation_max, velocity_limitation_min));
        this->states_set_[StateNames::FORWARD].setExpectedVelocity(this->center_lane_.getLaneVelocityLimitation()[current_position_index_in_lane], velocity_limitation_max);
        // 确定车辆速度限制，车辆速度上限由当前速度、横向加速度上限、横向加加速度上限决定
        double subvehicle_max_velocity = std::min(this->states_set_[StateNames::FORWARD].getVehicleStartMovement().velocity_ + VELOCITY_MAX_INCREASE_VALUE, velocity_limitation_max);
        double state_max_kappa = this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()])].kappa_;
        // 计算横向加速度限速
        double curvature_velocity_limitation = Tools::calcVelocityForMaxNormalAcceleration(state_max_kappa);
        // 计算横向加加速度限速
        double max_curvature_change_rate = Tools::getCurveMaxCurvatureChangeRate(this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()]);
        double curvature_change_rate_velocity_limitation = Tools::calcVelocityForMaxNormalJerk(max_curvature_change_rate);
        subvehicle_max_velocity = std::min(std::min(subvehicle_max_velocity, curvature_velocity_limitation), curvature_change_rate_velocity_limitation);
        LOG(INFO) << "中间车道最大曲率" << state_max_kappa << ", 曲率限速为" << curvature_velocity_limitation << ", 最大曲率变化率为" << max_curvature_change_rate << ", 曲率变化率限速为" << curvature_change_rate_velocity_limitation;
        // 设置速度限制
        this->states_set_[StateNames::FORWARD].setVelocityLimitation(subvehicle_max_velocity, std::min(velocity_limitation_min, 0.9 * subvehicle_max_velocity));
        // 确定车辆的加速度上下限制，加速度上限由横向加加速度上限决定
        double subvehicle_max_acceleration, subvehicle_min_acceleration;
        subvehicle_max_acceleration = std::min(Tools::velocityToAccelerationReflection(start_point_movement.velocity_), Tools::calcAccelerationForMaxNormalJerk(state_max_kappa, subvehicle_max_velocity));
        subvehicle_min_acceleration = -COMMON_DECCELERATION;
        this->states_set_[StateNames::FORWARD].setAccelerationLimitationMax(subvehicle_max_acceleration);
        this->states_set_[StateNames::FORWARD].setAccelerationLimitationMin(subvehicle_min_acceleration);
        // 设置目标点初始值(无效值)
        this->states_set_[StateNames::FORWARD].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->states_set_[StateNames::FORWARD].getTrajectoryLength());
        // 设置目标速度初始值(无效值)
        this->states_set_[StateNames::FORWARD].setGoalVelocity(0.0);
        // 设置期望加速度初始值(无效值)
        this->states_set_[StateNames::FORWARD].setVehicleDynamicPlanningExpectedAcceleration(0.0);

    } else {
         // 直行状态为无效状态
        this->states_set_[StateNames::FORWARD].disable();
    }

    LOG(INFO) << "CENTER LANE COMPLETE";
    LOG(INFO) << "center trajectory enable: " << this->states_set_[StateNames::FORWARD].getCapability();
    if (this->states_set_[StateNames::FORWARD].getCapability()) {
        LOG(INFO) << "center trajectory start point: " << this->states_set_[StateNames::FORWARD].getVehicleStartState().position_.x_ << "||" << this->states_set_[StateNames::FORWARD].getVehicleStartState().position_.y_ << "||" << this->states_set_[StateNames::FORWARD].getVehicleStartState().theta_ << "||" << this->states_set_[StateNames::FORWARD].getVehicleStartState().kappa_;
        LOG(INFO) << "center trajectories number: " << this->states_set_[StateNames::FORWARD].getTrajectory().size();
        if (this->states_set_[StateNames::FORWARD].getTrajectory().size() > 0) {
            LOG(INFO) << "center trajectory length: " << this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()].size();
            // 期望速度
            LOG(INFO) << "current position expected velocity is " << this->states_set_[StateNames::FORWARD].getExpectedVelocityCurrent();
            // 车道限制速度
            LOG(INFO) << "center lane velocity limitation is " << this->states_set_[StateNames::FORWARD].getExpectedVelocityLimitation();
            // 曲率限速
            LOG(INFO) << "center trajectory speed limitation for max normal acceleration is " << Tools::calcVelocityForMaxNormalAcceleration(this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()])].kappa_);
            // 限制速度
            LOG(INFO) << "vehicle velocity limitation is from " << this->states_set_[StateNames::FORWARD].getVelocityLimitationMin() << " to " << this->states_set_[StateNames::FORWARD].getVelocityLimitationMax();
            // 限制加速度
            LOG(INFO) << "vehicle acceleration limitation is from " << this->states_set_[StateNames::FORWARD].getAccelerationLimitationMin() << " to " << this->states_set_[StateNames::FORWARD].getAccelerationLimitationMax();

            double state_current_normal_acceleration = Tools::calcNormalAcceleration(this->states_set_[StateNames::FORWARD].getVehicleStartMovement().velocity_, this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()])].kappa_);
            LOG(INFO) << "current normal acceleration: " << state_current_normal_acceleration;
            if (Tools::isLarge(state_current_normal_acceleration, MAX_NORMAL_ACCELERATION)) {
                LOG(INFO) << "[WARN][WARN]normal acceleration out of upper boundary[WARN][WARN]";
            }
        }
    }

    // 初始化左换道状态(查找左换道是否在当前状态的邻居中且左道存在,如果在则进行初始化)
    if (this->left_lane_.getLaneExistance()) {
        this->states_set_[StateNames::TURN_LEFT].setRespondingLane(this->left_lane_);
        // 左转状态为有效状态
        this->states_set_[StateNames::TURN_LEFT].enable();
        // 提供车辆信息
        this->states_set_[StateNames::TURN_LEFT].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
        // 提供起始点信息
        this->states_set_[StateNames::TURN_LEFT].setVehicleStartState(start_point_in_world);
        this->states_set_[StateNames::TURN_LEFT].setVehicleStartMovement(start_point_movement);
        // 提供车辆起点速度信息
        this->states_set_[StateNames::TURN_LEFT].setVehicleCurrentMovement(start_point_movement);
        // 提供车辆当前位置在轨迹中下标信息(此处一定是0)
        this->states_set_[StateNames::TURN_LEFT].setVehicleCurrentPositionIndexInTrajectory(0);
        // 提供路径信息
        std::cout << "LEFT LANE PROCESSING" << std::endl;
        LOG(INFO) << "LEFT LANE PROCESSING";
        this->updateLaneTrajectoryforStates(&(this->states_set_[StateNames::TURN_LEFT]), this->left_lane_, start_point_in_world, start_point_movement, start_point_kappa);
        // 初始化状态的安全性，初始化必须为true
        this->states_set_[StateNames::TURN_LEFT].setSafety(true);
        // 确定状态的优先级(TODO)
        this->states_set_[StateNames::TURN_LEFT].setPriority(this->left_lane_.getLanePriority());
        // 确定道路速度期望
        // 得到当前下标
        size_t current_position_index_in_lane = this->left_lane_.findCurrenPositionIndexInLane(start_point_in_world.position_.x_, start_point_in_world.position_.y_);
        // 得到未来下标
        size_t future_position_index_in_lane = current_position_index_in_lane + static_cast<size_t>(Tools::normalForeseenDistance(std::max(start_point_movement.velocity_, this->expected_velocity_upper_bound_), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
        // 计算当前下标到未来下标速度最低限制
        double velocity_limitation_max = this->left_lane_.getLaneVelocityLimitation()[current_position_index_in_lane];
        for (size_t i = current_position_index_in_lane; i < future_position_index_in_lane; i++) {
            double velocity_limitation = this->left_lane_.getLaneVelocityLimitation()[i];
            if (Tools::isSmall(velocity_limitation, velocity_limitation_max)) {
                velocity_limitation_max = velocity_limitation;
            }
        }
        double velocity_limitation_min = std::min(this->left_lane_.getLowVelocity()[current_position_index_in_lane], this->left_lane_.getLowVelocity()[future_position_index_in_lane]);
        assert(Tools::isLarge(velocity_limitation_max, velocity_limitation_min));
        this->states_set_[StateNames::TURN_LEFT].setExpectedVelocity(this->left_lane_.getLaneVelocityLimitation()[current_position_index_in_lane], velocity_limitation_max);
        // 确定车辆速度限制，车辆速度上限由当前速度、横向加速度上限、横向加加速度上限决定
        double subvehicle_max_velocity = std::min(this->states_set_[StateNames::TURN_LEFT].getVehicleStartMovement().velocity_ + VELOCITY_MAX_INCREASE_VALUE, velocity_limitation_max);
        // 计算横向加速度限速
        double state_max_kappa = this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()])].kappa_;
        double curvature_velocity_limitation = Tools::calcVelocityForMaxNormalAcceleration(state_max_kappa);
        // 计算横向加加速度限速
        double max_curvature_change_rate = Tools::getCurveMaxCurvatureChangeRate(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()]);
        double curvature_change_rate_velocity_limitation = Tools::calcVelocityForMaxNormalJerk(max_curvature_change_rate);
        subvehicle_max_velocity = std::min(std::min(subvehicle_max_velocity, curvature_velocity_limitation), curvature_change_rate_velocity_limitation);
        LOG(INFO) << "左侧车道最大曲率" << state_max_kappa << ", 曲率限速为" << curvature_velocity_limitation << ", 最大曲率变化率为" << max_curvature_change_rate << ", 曲率变化率限速为" << curvature_change_rate_velocity_limitation;
        // 设置速度限制
        this->states_set_[StateNames::TURN_LEFT].setVelocityLimitation(subvehicle_max_velocity, std::min(velocity_limitation_min, 0.9 * subvehicle_max_velocity));
        // 确定车辆的加速度上下限制，加速度上限由横向加加速度上限决定
        double subvehicle_max_acceleration, subvehicle_min_acceleration;
        subvehicle_max_acceleration = std::min(Tools::velocityToAccelerationReflection(start_point_movement.velocity_), Tools::calcAccelerationForMaxNormalJerk(state_max_kappa, subvehicle_max_velocity));
        subvehicle_min_acceleration = -COMMON_DECCELERATION;
        this->states_set_[StateNames::TURN_LEFT].setAccelerationLimitationMax(subvehicle_max_acceleration);
        this->states_set_[StateNames::TURN_LEFT].setAccelerationLimitationMin(subvehicle_min_acceleration);
        // 设置目标点初始值(无效值)
        this->states_set_[StateNames::TURN_LEFT].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->states_set_[StateNames::TURN_LEFT].getTrajectoryLength());
        // 设置目标速度初始值(无效值)
        this->states_set_[StateNames::TURN_LEFT].setGoalVelocity(0.0);
        // 设置期望加速度初始值(无效值)
        this->states_set_[StateNames::TURN_LEFT].setVehicleDynamicPlanningExpectedAcceleration(0.0);
    } else {
         // 左转状态为无效状态
        this->states_set_[StateNames::TURN_LEFT].disable();
    }

    LOG(INFO) << "LEFT LANE COMPLETE";
    LOG(INFO) << "left trajectory enable: " << this->states_set_[StateNames::TURN_LEFT].getCapability();
    if (this->states_set_[StateNames::TURN_LEFT].getCapability()) {
        LOG(INFO) << "left trajectory start point: " << this->states_set_[StateNames::TURN_LEFT].getVehicleStartState().position_.x_ << "||" << this->states_set_[StateNames::TURN_LEFT].getVehicleStartState().position_.y_ << "||" << this->states_set_[StateNames::TURN_LEFT].getVehicleStartState().theta_ << "||" << this->states_set_[StateNames::TURN_LEFT].getVehicleStartState().kappa_;
        LOG(INFO) << "left trajectories number: " << this->states_set_[StateNames::TURN_LEFT].getTrajectory().size();
        if (this->states_set_[StateNames::TURN_LEFT].getTrajectory().size() > 0) {
            LOG(INFO) << "left trajectory length: " << this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()].size();
            LOG(INFO) << "left trajectory max normal acceleration: " << Tools::calcMaxNormalAccelerationForCurve(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()], this->states_set_[StateNames::TURN_LEFT].getVehicleCurrentMovement().velocity_);
            // DEBUG
            LOG(INFO) << "left trajectory max kappa: " << this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()])].kappa_;
            // 期望速度
            LOG(INFO) << "current position expected velocity is " << this->states_set_[StateNames::TURN_LEFT].getExpectedVelocityCurrent();
            // 车道限制速度
            LOG(INFO) << "left lane velocity limitation is " << this->states_set_[StateNames::TURN_LEFT].getExpectedVelocityLimitation();
            // 曲率限速
            LOG(INFO) << "left trajectory speed limitation for max normal acceleration is " << Tools::calcVelocityForMaxNormalAcceleration(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()])].kappa_);
            // 限制速度
            LOG(INFO) << "vehicle velocity limitation is from " << this->states_set_[StateNames::TURN_LEFT].getVelocityLimitationMin() << " to " << this->states_set_[StateNames::TURN_LEFT].getVelocityLimitationMax();
            // 限制加速度
            LOG(INFO) << "vehicle acceleration limitation is from " << this->states_set_[StateNames::TURN_LEFT].getAccelerationLimitationMin() << " to " << this->states_set_[StateNames::TURN_LEFT].getAccelerationLimitationMax();
            
            double state_current_normal_acceleration = Tools::calcNormalAcceleration(this->states_set_[StateNames::TURN_LEFT].getVehicleStartMovement().velocity_, this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()])].kappa_);
            LOG(INFO) << "current normal acceleration: " << state_current_normal_acceleration;
            if (Tools::isLarge(state_current_normal_acceleration, MAX_NORMAL_ACCELERATION)) {
                LOG(INFO) << "[WARN][WARN]normal acceleration out of upper boundary[WARN][WARN]";
            }
        }
    }

    // 初始化右换道状态(查找右换道是否在当前状态的邻居中且右道存在,如果在则进行初始化)
    if (this->right_lane_.getLaneExistance()) {
        this->states_set_[StateNames::TURN_RIGHT].setRespondingLane(this->right_lane_);
        // 右转状态为有效状态
        this->states_set_[StateNames::TURN_RIGHT].enable();
        // 提供车辆信息
        this->states_set_[StateNames::TURN_RIGHT].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
        // 提供起始点信息
        this->states_set_[StateNames::TURN_RIGHT].setVehicleStartState(start_point_in_world);
        this->states_set_[StateNames::TURN_RIGHT].setVehicleStartMovement(start_point_movement);
        // 提供车辆当前速度信息
        this->states_set_[StateNames::TURN_RIGHT].setVehicleCurrentMovement(start_point_movement);
        // 提供车辆当前位置在轨迹中下标信息(此处一定是0)
        this->states_set_[StateNames::TURN_RIGHT].setVehicleCurrentPositionIndexInTrajectory(0);
        // 提供路径信息
        std::cout << "RIGHT LANE PROCESSING";
        LOG(INFO) << "RIGHT LANE PROCESSING";
        this->updateLaneTrajectoryforStates(&(this->states_set_[StateNames::TURN_RIGHT]), this->right_lane_, start_point_in_world, start_point_movement, start_point_kappa);
        // 初始化状态的安全性，初始化必须为true
        this->states_set_[StateNames::TURN_RIGHT].setSafety(true);
        // 确定状态的优先级(TODO)
        this->states_set_[StateNames::TURN_RIGHT].setPriority(this->right_lane_.getLanePriority());
        // 确定道路速度期望
        // 得到当前下标
        size_t current_position_index_in_lane = this->right_lane_.findCurrenPositionIndexInLane(start_point_in_world.position_.x_, start_point_in_world.position_.y_);
        // 得到未来下标
        size_t future_position_index_in_lane = current_position_index_in_lane + static_cast<size_t>(Tools::normalForeseenDistance(std::max(start_point_movement.velocity_, this->expected_velocity_upper_bound_), MAX_DECCELERATION, CONSTANT_DISTANCE) / LANE_GAP_DISTANCE);
        // 计算当前下标到未来下标速度最低限制
        double velocity_limitation_max = this->right_lane_.getLaneVelocityLimitation()[current_position_index_in_lane];
        for (size_t i = current_position_index_in_lane; i < future_position_index_in_lane; i++) {
            double velocity_limitation = this->right_lane_.getLaneVelocityLimitation()[i];
            if (Tools::isSmall(velocity_limitation, velocity_limitation_max)) {
                velocity_limitation_max = velocity_limitation;
            }
        }
        double velocity_limitation_min = std::min(this->right_lane_.getLowVelocity()[current_position_index_in_lane], this->right_lane_.getLowVelocity()[future_position_index_in_lane]);
        assert(Tools::isLarge(velocity_limitation_max, velocity_limitation_min));
        this->states_set_[StateNames::TURN_RIGHT].setExpectedVelocity(this->right_lane_.getLaneVelocityLimitation()[current_position_index_in_lane], velocity_limitation_max);
        // 确定车辆速度限制，车辆速度上限由当前速度、横向加速度上限、横向加加速度上限决定
        double subvehicle_max_velocity = std::min(this->states_set_[StateNames::TURN_RIGHT].getVehicleStartMovement().velocity_ + VELOCITY_MAX_INCREASE_VALUE, velocity_limitation_max);
        // 计算横向加速度限速
        double state_max_kappa = this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()])].kappa_;
        double curvature_velocity_limitation = Tools::calcVelocityForMaxNormalAcceleration(state_max_kappa);
        // 计算横向加加速度限速
        double max_curvature_change_rate = Tools::getCurveMaxCurvatureChangeRate(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()]);
        double curvature_change_rate_velocity_limitation = Tools::calcVelocityForMaxNormalJerk(max_curvature_change_rate);
        subvehicle_max_velocity = std::min(std::min(subvehicle_max_velocity, curvature_velocity_limitation), curvature_change_rate_velocity_limitation);
        LOG(INFO) << "右侧车道最大曲率" << state_max_kappa << ", 曲率限速为" << curvature_velocity_limitation << ", 最大曲率变化率为" << max_curvature_change_rate << ", 曲率变化率限速为" << curvature_change_rate_velocity_limitation;
        // 设置速度限制
        this->states_set_[StateNames::TURN_RIGHT].setVelocityLimitation(subvehicle_max_velocity, std::min(velocity_limitation_min, 0.9 * subvehicle_max_velocity));
        // 确定车辆的加速度上下限制，加速度上限由横向加加速度上限决定
        double subvehicle_max_acceleration, subvehicle_min_acceleration;
        subvehicle_max_acceleration = std::min(Tools::velocityToAccelerationReflection(start_point_movement.velocity_), Tools::calcAccelerationForMaxNormalJerk(state_max_kappa, subvehicle_max_velocity));
        subvehicle_min_acceleration = -COMMON_DECCELERATION;
        this->states_set_[StateNames::TURN_RIGHT].setAccelerationLimitationMax(subvehicle_max_acceleration);
        this->states_set_[StateNames::TURN_RIGHT].setAccelerationLimitationMin(subvehicle_min_acceleration);
        // 设置目标点初始值(无效值)
        this->states_set_[StateNames::TURN_RIGHT].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->states_set_[StateNames::TURN_RIGHT].getTrajectoryLength());
        // 设置目标速度初始值(无效值)
        this->states_set_[StateNames::TURN_RIGHT].setGoalVelocity(0.0);
        // 设置期望加速度初始值(无效值)
        this->states_set_[StateNames::TURN_RIGHT].setVehicleDynamicPlanningExpectedAcceleration(0.0);
    } else {
        // 右转状态为无效状态
        this->states_set_[StateNames::TURN_RIGHT].disable();
    }

    LOG(INFO) << "RIGHT LANE COMPLETE";
    LOG(INFO) << "right trajectory enable: " << this->states_set_[StateNames::TURN_RIGHT].getCapability();
    if (this->states_set_[StateNames::TURN_RIGHT].getCapability()) {
        LOG(INFO) << "right trajectory start point: " << this->states_set_[StateNames::TURN_RIGHT].getVehicleStartState().position_.x_ << "||" << this->states_set_[StateNames::TURN_RIGHT].getVehicleStartState().position_.y_ << "||" << this->states_set_[StateNames::TURN_RIGHT].getVehicleStartState().theta_ << "||" << this->states_set_[StateNames::TURN_RIGHT].getVehicleStartState().kappa_;
        LOG(INFO) << "right trajectories number: " << this->states_set_[StateNames::TURN_RIGHT].getTrajectory().size();
        if (this->states_set_[StateNames::TURN_RIGHT].getTrajectory().size() > 0) {
            LOG(INFO) << "right trajectory length: " << this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()].size();
            LOG(INFO) << "right trajectory max normal acceleration: " << Tools::calcMaxNormalAccelerationForCurve(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()], this->states_set_[StateNames::TURN_RIGHT].getVehicleCurrentMovement().velocity_);
            // DEBUG
            LOG(INFO) << "right trajectory max kappa: " << this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()])].kappa_;
            // 期望速度
            LOG(INFO) << "current position expected velocity is " << this->states_set_[StateNames::TURN_RIGHT].getExpectedVelocityCurrent();
            // 车道限制速度
            LOG(INFO) << "right lane velocity limitation is " << this->states_set_[StateNames::TURN_RIGHT].getExpectedVelocityLimitation();
            // 曲率限速
            LOG(INFO) << "right trajectory speed limitation for max normal acceleration is " << Tools::calcVelocityForMaxNormalAcceleration(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()])].kappa_);
            // 限制速度
            LOG(INFO) << "vehicle velocity limitation is from " << this->states_set_[StateNames::TURN_RIGHT].getVelocityLimitationMin() << " to " << this->states_set_[StateNames::TURN_RIGHT].getVelocityLimitationMax();
            // 限制加速度
            LOG(INFO) << "vehicle acceleration limitation is from " << this->states_set_[StateNames::TURN_RIGHT].getAccelerationLimitationMin() << " to " << this->states_set_[StateNames::TURN_RIGHT].getAccelerationLimitationMax();

            double state_current_normal_acceleration = Tools::calcNormalAcceleration(this->states_set_[StateNames::TURN_RIGHT].getVehicleStartMovement().velocity_, this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()])].kappa_);

            LOG(INFO) << "current normal acceleration: " << state_current_normal_acceleration;
            if (Tools::isLarge(state_current_normal_acceleration, MAX_NORMAL_ACCELERATION)) {
                LOG(INFO) << "[WARN][WARN]normal acceleration out of upper boundary[WARN][WARN]";
            }
        }
    }

    // // 判断三个状态的可行性，如果都不可行，紧急停车并退出程序
    // if (this->states_set_[StateNames::TURN_RIGHT].getCapability() == false && this->states_set_[StateNames::TURN_LEFT].getCapability() == false && this->states_set_[StateNames::FORWARD].getCapability() == false) {
    //     LOG(INFO) << "[WARN][WARN][WARN][WARN]全部状态不可行，紧急停车[WARN][WARN][WARN][WARN]";
    //     // 紧急停车
    //     exit(0);
    // }

    // 可视化生成的全部轨迹
    // std::vector<PathPlanningUtilities::Curve> all_curve_for_vis;
    // for (size_t i = 0; i < this->states_set_[StateNames::TURN_RIGHT].getTrajectory().size(); i++) {
    //     all_curve_for_vis.push_back(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[i]);
    // }
    // for (size_t i = 0; i < this->states_set_[StateNames::TURN_LEFT].getTrajectory().size(); i++) {
    //     all_curve_for_vis.push_back(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[i]);
    // }
    // for (size_t i = 0; i < this->states_set_[StateNames::FORWARD].getTrajectory().size(); i++) {
    //     all_curve_for_vis.push_back(this->states_set_[StateNames::FORWARD].getTrajectory()[i]);
    // }

    // VisualizationMethods::visualizeMultiCurves(all_curve_for_vis, this->vis_multi_curve_pub_);
}
