/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

// // 特殊处理
// bool GLOBAL_IS_IN_GROUND_ = false;
// std::mutex GLOBAL_IN_GROUND_MUTEX_;
// Rectangle GLOBAL_GROUND_AREA = {-11337.9472656, -2507.84375, 133.0, 142.0, 2.0476333396251585};
// bool GLOBAL_IS_IN_JIFEI_ = false;
// std::mutex GLOBAL_IN_JIFEI_MUTEX_;
// Rectangle GLOBAL_JIFEI_AREA = {-11356.0869141, -2458.27685547, 101.0, 20.0, 2.0231007985729406};
// bool GLOBAL_IS_IN_CHECK_ = false;
// bool GLOBAL_IS_IN_SLOW_ = false;
// bool GLOBAL_IS_IN_OVERTAKE_ = false;
// int GLOBAL_IS_IN_CHECK_COUNT_FLAG_ = 0;

// 初始化并启动线程
void DecisionMaking::SubVehicle::runMotionPlanning() {
    // 初始化自检测对象
    this->self_test_.add("Program checking", this, &DecisionMaking::SubVehicle::selfTestForProgress);
    // 启动各线程,线程一为订阅ros节点和服务，获取车辆信息，线程二为motionplanning和decisionmaking，计算并保持轨迹
    std::thread ros_msgs_receiver_thread(&DecisionMaking::SubVehicle::listenRosMSGThread, this);
    std::thread motion_planning_thread(&DecisionMaking::SubVehicle::motionPlanningThread, this);
    ros_msgs_receiver_thread.join();
    motion_planning_thread.join();
}

// 状态机初始化
void DecisionMaking::SubVehicle::initVehicleStates() {
    std::vector<StandardState>().swap(this->states_set_);
    this->states_set_.resize(StateNames::STATE_SIZE);
    for (size_t i = StateNames::STOP; i < StateNames::STATE_SIZE; i++) {
        std::vector<size_t> neighbor_states;
        if (i == StateNames::STOP) {
            neighbor_states.push_back(StateNames::TURN_LEFT);
            neighbor_states.push_back(StateNames::TURN_RIGHT);
            neighbor_states.push_back(StateNames::FORWARD);
            neighbor_states.push_back(StateNames::AVOIDANCE);
            neighbor_states.push_back(StateNames::REVERSE);
            neighbor_states.push_back(StateNames::ROTATE);
            this->states_set_[i] = StandardState(i, neighbor_states);
        } else if (i == StateNames::TURN_LEFT) {
            neighbor_states.push_back(StateNames::STOP);
            neighbor_states.push_back(StateNames::FORWARD);
            this->states_set_[i] = StandardState(i, neighbor_states);
        } else if (i == StateNames::TURN_RIGHT) {
            neighbor_states.push_back(StateNames::STOP);
            neighbor_states.push_back(StateNames::FORWARD);
            this->states_set_[i] = StandardState(i, neighbor_states);
        } else if (i == StateNames::FORWARD) {
            neighbor_states.push_back(StateNames::STOP);
            neighbor_states.push_back(StateNames::TURN_LEFT);
            neighbor_states.push_back(StateNames::TURN_RIGHT);
            neighbor_states.push_back(StateNames::AVOIDANCE);
            this->states_set_[i] = StandardState(i, neighbor_states);
        } else if (i == StateNames::AVOIDANCE) {
            neighbor_states.push_back(StateNames::STOP);
            neighbor_states.push_back(StateNames::TURN_LEFT);
            neighbor_states.push_back(StateNames::TURN_RIGHT);
            neighbor_states.push_back(StateNames::FORWARD);
            this->states_set_[i] = StandardState(i, neighbor_states);
        } else if (i == StateNames::REVERSE) {
            neighbor_states.push_back(StateNames::STOP);
            neighbor_states.push_back(StateNames::ROTATE);
            neighbor_states.push_back(StateNames::AVOIDANCE);
            this->states_set_[i] = StandardState(i, neighbor_states);
        } else if (i == StateNames::ROTATE) {
            neighbor_states.push_back(StateNames::STOP);
            neighbor_states.push_back(StateNames::REVERSE);
            neighbor_states.push_back(StateNames::AVOIDANCE);
            this->states_set_[i] = StandardState(i, neighbor_states);
        } else {
            continue;
        }
    }
}

// 初始化ros节点
void DecisionMaking::SubVehicle::rosInit() {
    // 初始化可视化节点
    std::string visualization_topic, vis_vehicle_topic, vis_obstacle_topic, vis_collision_topic, vis_multi_curve_topic, vis_influence_obstacle_topic, vis_occupation_topic;
    this->nh_.getParam("visualization_topic", visualization_topic);
    this->nh_.getParam("vis_vehicle_topic", vis_vehicle_topic);
    this->nh_.getParam("vis_obstacle_topic", vis_obstacle_topic);
    this->nh_.getParam("vis_collision_topic", vis_collision_topic);
    this->nh_.getParam("vis_multi_curve_topic", vis_multi_curve_topic);
    this->nh_.getParam("vis_influence_obstacle_topic", vis_influence_obstacle_topic);
    this->nh_.getParam("vis_occupation_topic", vis_occupation_topic);
    this->visualization_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic, 10);
    this->vis_vehicle_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_vehicle_topic, 10);
    this->vis_obstacle_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_obstacle_topic, 10);
    this->vis_occupation_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_occupation_topic, 10);
    // debug
    this->vis_collision_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_collision_topic, 10);
    this->vis_multi_curve_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_multi_curve_topic, 10);
    this->vis_influence_obstacle_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(vis_influence_obstacle_topic, 10);

    // 获取tf
    this->tf_listener_ptr_ = new tf::TransformListener();

    // 获取位置topic
    std::string odometry_topic;
    this->nh_.getParam("odometry_topic", odometry_topic);
    this->odom_sub_ = this->nh_.subscribe(odometry_topic, 1, &DecisionMaking::SubVehicle::updateVehiclePosition, this);

    // 获取移动状态topic
    std::string movement_topic;
    this->nh_.getParam("movement_topic", movement_topic);
    this->movement_sub_ = this->nh_.subscribe(movement_topic, 1, &DecisionMaking::SubVehicle::updateVehicleMovement, this);


    // Get acceleration topic
    std::string acceleration_topic;
    this->nh_.getParam("acceleration_topic", acceleration_topic);
    acceleration_sub_ = nh_.subscribe(acceleration_topic, 1, &DecisionMaking::SubVehicle::updateVehicleAcceleration, this);

    // 获取曲率topic
    std::string curvature_topic;
    this->nh_.getParam("curvature_topic", curvature_topic);
    this->curvature_sub_ = this->nh_.subscribe(curvature_topic, 1, &DecisionMaking::SubVehicle::updateVehicleCurvature, this);

    // 获取控制状态topic
    std::string control_report_topic;
    this->nh_.getParam("control_report_topic", control_report_topic);
    this->control_report_sub_ = this->nh_.subscribe(control_report_topic, 1, &DecisionMaking::SubVehicle::updateControlReport, this);

    // 开始任务服务
    std::string mission_start_service_name;
    this->nh_.getParam("mission_start_service", mission_start_service_name);
    this->mission_start_service_server_ = this->nh_.advertiseService(mission_start_service_name, &DecisionMaking::SubVehicle::startMission, this);

    // 强制停车服务
    std::string forced_stop_service_name;
    this->nh_.getParam("forced_stop_service", forced_stop_service_name);
    this->forced_stop_service_server_ = this->nh_.advertiseService(forced_stop_service_name, &DecisionMaking::SubVehicle::forcedStop, this);

    // 障碍物topic
    std::string obstacle_topic;
    this->nh_.getParam("obstacle_topic", obstacle_topic);
    this->obstacle_sub_ = this->nh_.subscribe(obstacle_topic, 1, &DecisionMaking::SubVehicle::getObstacles, this);

    // 初始化地图服务
    std::string map_service_name;
    this->nh_.getParam("map_service", map_service_name);
    ros::service::waitForService(map_service_name);
    this->map_service_client_ = this->nh_.serviceClient<vec_map_cpp_msgs::GetGuidedCurves>(map_service_name);

    // 初始化障碍物轨迹预测服务
    std::string obstacle_trajectory_prediction_service_name;
    this->nh_.getParam("obstacle_trajectory_prediction_service", obstacle_trajectory_prediction_service_name);
    ros::service::waitForService(obstacle_trajectory_prediction_service_name);
    this->obstacle_trajectory_prediction_service_client_ = this->nh_.serviceClient<vec_map_cpp_msgs::GetPredictedTrajectory>(obstacle_trajectory_prediction_service_name);

    // 初始化到达目的地服务节点
    std::string destination_reached_service_name;
    this->nh_.getParam("destination_reached_service", destination_reached_service_name);
    ros::service::waitForService(destination_reached_service_name);
    this->destination_reached_service_client_ = this->nh_.serviceClient<std_srvs::Trigger>(destination_reached_service_name);

    // 初始化调用A星掉头服务节点
    std::string motion_planning_uncapable_service_name;
    this->nh_.getParam("motion_planning_uncapable_service", motion_planning_uncapable_service_name);
    this->motion_planning_uncapable_client_ = this->nh_.serviceClient<mission_msgs::RequireTurnAround>(motion_planning_uncapable_service_name, 10);

    // 初始化报告无法进行服务节点
    std::string motion_planning_failed_service_name;
    this->nh_.getParam("motion_planning_failed_service", motion_planning_failed_service_name);
    this->motion_planning_failed_client_ = this->nh_.serviceClient<std_srvs::Trigger>(motion_planning_failed_service_name, 10);

    // 初始化ros publish节点
    std::string motion_planning_curve_publish_topic;
    this->nh_.getParam("motion_planning_curve_publish_topic", motion_planning_curve_publish_topic);
    this->motion_planning_curve_pub_ = this->nh_.advertise<path_planning_msgs::MotionPlanningCurve>(motion_planning_curve_publish_topic, 10);

    this->history_curve_sub_ = this->nh_.subscribe(motion_planning_curve_publish_topic, 1, &DecisionMaking::SubVehicle::getHistoryCurve, this);

    // 初始化转向灯发布节点
    std::string turn_signal_publish_topic;
    this->nh_.getParam("turn_signal_publish_topic", turn_signal_publish_topic);
    this->turn_signal_pub_ = this->nh_.advertise<dbw_mkz_msgs::TurnSignalCmd>(turn_signal_publish_topic, 10);

    // 初始化紧急刹车发布节点
    std::string emergency_break_publish_topic;
    this->nh_.getParam("emergency_break_publish_topic", emergency_break_publish_topic);
    this->emergency_break_pub_ = this->nh_.advertise<std_msgs::Empty>(emergency_break_publish_topic, 10);

    if (this->TRAFFIC_LIGHT_USAGE_FLAG_) {
        // 初始化交通灯服务
        std::string traffic_light_service_name;
        this->nh_.getParam("traffic_light_service", traffic_light_service_name);
        ros::service::waitForService(traffic_light_service_name);
        this->traffic_light_service_client_ = this->nh_.serviceClient<traffic_light_msgs::traffic_lights>(traffic_light_service_name);
    }

    if (this->IS_SURROUND_RADAR_ENABLE_FLAG_) {
        // 初始化预警毫米波
        std::string surround_radar_topic;
        this->nh_.getParam("surround_radar_topic", surround_radar_topic);
        this->surround_radar_sub_ = this->nh_.subscribe(surround_radar_topic, 1, &DecisionMaking::SubVehicle::updateSurroundRadarInfo, this);
    }
}

// 启动ros订阅线程,50hz
void DecisionMaking::SubVehicle::listenRosMSGThread() {
    ros::Rate loop_rate(ROS_UPDATE_FREQUENCY);
    while (ros::ok()) {
        ros::spinOnce();
        this->self_test_.checkTest();
        loop_rate.sleep();
    }
}

// 更新车辆位置信息，ros节点
void DecisionMaking::SubVehicle::updateVehiclePosition(const nav_msgs::Odometry::ConstPtr odometry_msg) {
    // 获取当前位置信息(世界坐标系下)
    this->current_vehicle_world_position_mutex_.lock();
    this->current_vehicle_world_position_.position_.x_ = odometry_msg->pose.pose.position.x;
    this->current_vehicle_world_position_.position_.y_ = odometry_msg->pose.pose.position.y;
    tf::Quaternion quaternion;
    double raw, pitch, theta;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(raw, pitch, theta);
    this->current_vehicle_world_position_.theta_ = theta;
    this->current_vehicle_world_position_mutex_.unlock();
    // 确定车辆位置信息加载成功
    this->vehicle_position_ready_flag_mutex_.lock();
    if (!this->VEHICLE_POSITION_READY_FLAG_) {
        this->VEHICLE_POSITION_READY_FLAG_ = true;
        ROS_INFO("VEHICLE POSITION GOT");
        LOG(INFO) << "VEHICLE POSITION GOT";
    }
    this->vehicle_position_ready_flag_mutex_.unlock();

    // // 判断当前位置是否处于园区内
    // GLOBAL_IN_GROUND_MUTEX_.lock();
    // if (Tools::isRectangleOverlap(current_pose, GLOBAL_GROUND_AREA, 1.0, 1.0)) {
    //     // 在园区内
    //     GLOBAL_IS_IN_GROUND_ = true;
    //     // std::cout << "在园区内" << std::endl;
    // } else {
    //     GLOBAL_IS_IN_GROUND_ = false;
    //     // std::cout << "不在园区内" << std::endl;
    // }
    // GLOBAL_IN_GROUND_MUTEX_.unlock();
    // // 判断当前位置是否处于机非混行内
    // GLOBAL_IN_JIFEI_MUTEX_.lock();
    // if (Tools::isRectangleOverlap(current_pose, GLOBAL_JIFEI_AREA, 1.0, 1.0)) {
    //     // 在园区内
    //     GLOBAL_IS_IN_JIFEI_ = true;
    //     // std::cout << "在机非混行内" << std::endl;
    // } else {
    //     GLOBAL_IS_IN_JIFEI_ = false;
    //     // std::cout << "不在机非混行内" << std::endl;
    // }
    // GLOBAL_IN_JIFEI_MUTEX_.unlock();

    // 判断是否可以可视化车辆当前位置
    this->vehicle_curvature_ready_flag_mutex_.lock();
    this->vehicle_position_ready_flag_mutex_.lock();
    double pose_defined = this->VEHICLE_CURVATURE_READY_FLAG_ && this->VEHICLE_POSITION_READY_FLAG_;
    this->vehicle_position_ready_flag_mutex_.unlock();
    this->vehicle_curvature_ready_flag_mutex_.unlock();
    if (!pose_defined) {
        return;
    }
    // 车辆位置进行可视化
    this->current_vehicle_world_position_mutex_.lock();
    // 1.清除之前的位置
    visualization_msgs::MarkerArray delete_marker_array;
    delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteMarker(VisualizationMethods::VisualizationID::VEHICLE_ODOM));
    this->vis_vehicle_pub_.publish(delete_marker_array);
    // 2.加载新的位置信息
    visualization_msgs::MarkerArray marker_array;
    std_msgs::ColorRGBA color;
    color.r = 0;
    color.g = 1;
    color.b = 0;
    color.a = 1;
    marker_array.markers.push_back(VisualizationMethods::visualizeRectToMarker(this->current_vehicle_world_position_.position_.x_, this->current_vehicle_world_position_.position_.y_, Tools::centerYawToRearYaw(this->current_vehicle_world_position_.theta_, this->current_vehicle_world_position_.kappa_, DISTANCE_FROM_REAR_TO_CENTER), this->vehicle_width_, this->vehicle_length_, this->vehicle_rear_axis_center_scale_, color, VisualizationMethods::VisualizationID::VEHICLE_ODOM));
    this->vis_vehicle_pub_.publish(marker_array);
    this->current_vehicle_world_position_mutex_.unlock();
}

// 更新车辆速度和速度朝向，ros节点
void DecisionMaking::SubVehicle::updateVehicleMovement(const std_msgs::Float64::ConstPtr velocity_msg) {
    // 更新车辆速度信息
    this->current_vehicle_movement_mutex_.lock();
    this->current_vehicle_movement_.velocity_ = velocity_msg->data;
    this->current_vehicle_movement_.acceleration_ = 0.0;
    this->current_vehicle_movement_mutex_.unlock();
    // 确定车辆运动信息加载成功
    this->vehicle_movement_ready_flag_mutex_.lock();
    if (!this->VEHICLE_MOVEMENT_READY_FLAG_) {
        this->VEHICLE_MOVEMENT_READY_FLAG_ = true;
        ROS_INFO("VEHICLE MOVEMENT GOT");
        LOG(INFO) << "VEHICLE MOVEMENT GOT";
    }
    this->vehicle_movement_ready_flag_mutex_.unlock();
    // 特殊处理
    if (Tools::isLarge(this->current_vehicle_movement_.velocity_, 0.0)) {
        this->stop_count_recorder_ = 0;
        this->low_frequency_stop_count_recorder_ = 0;
    }
    
}

void DecisionMaking::SubVehicle::updateVehicleAcceleration(const std_msgs::Float64::ConstPtr acceleration_msg) {
    // Update acceleration information
    this->current_vehicle_movement_mutex_.lock();
    this->current_vehicle_movement_.acceleration_ = acceleration_msg->data;
    this->current_vehicle_movement_mutex_.unlock();
}

// 更新车辆曲率
void DecisionMaking::SubVehicle::updateVehicleCurvature(const std_msgs::Float64::ConstPtr curvature_msg) {
    // 记录车辆当前曲率
    this->current_vehicle_world_position_mutex_.lock();
    this->current_vehicle_world_position_.kappa_ = curvature_msg->data;
    this->current_vehicle_world_position_mutex_.unlock();
    this->current_vehicle_kappa_mutex_.lock();
    this->current_vehicle_kappa_ = curvature_msg->data;
    this->current_vehicle_kappa_mutex_.unlock();
    // 确定车辆曲率信息加载成功
    this->vehicle_curvature_ready_flag_mutex_.lock();
    if (!this->VEHICLE_CURVATURE_READY_FLAG_) {
        this->VEHICLE_CURVATURE_READY_FLAG_ = true;
        ROS_INFO("VEHICLE CURVATURE GOT");
        LOG(INFO) << "VEHICLE CURVATURE GOT";
    }
    this->vehicle_curvature_ready_flag_mutex_.unlock();
}

// 更新控制报告信息
void DecisionMaking::SubVehicle::updateControlReport(const control_msgs::CoreReport::ConstPtr control_report_msg) {
    if (control_report_msg->status == control_msgs::CoreReport::GOAL_REACHED) {
        // 控制报告完成
        this->control_finished_flag_mutex_.lock();
        this->CONTROL_FINISHED_FLAG_ = true;
        this->control_finished_flag_mutex_.unlock();
    }
}

// 更新毫米波雷达信息，ros节点
void DecisionMaking::SubVehicle::updateSurroundRadarInfo(const dbw_mkz_msgs::SurroundReport::ConstPtr radar_msgs) {
    this->right_alert_mutex_.lock();
    this->right_alert_ = radar_msgs->blis_right_alert;
    this->right_alert_mutex_.unlock();
    this->left_alert_mutex_.lock();
    this->left_alert_ = radar_msgs->blis_left_alert;
    this->left_alert_mutex_.unlock();
    this->vehicle_surround_radar_ready_flag_mutex_.lock();
    if (!this->VEHICLE_SURROUND_RADAR_READY_FLAG_) {
        this->VEHICLE_SURROUND_RADAR_READY_FLAG_ = true;
    }
    this->vehicle_surround_radar_ready_flag_mutex_.unlock();
}

// 开始任务节点
bool DecisionMaking::SubVehicle::startMission(mission_msgs::StartMainRequest &request ,mission_msgs::StartMainResponse &response) {
    this->mission_start_mutex_.lock();
    this->MISSION_START_FLAG_ = true;
    ROS_INFO("MISSION START");
    LOG(INFO) << "MISSION START";
    this->mission_start_mutex_.unlock();
    // 获取终点的位置
    this->destination_mutex_.lock();
    this->destination_pose_ = request.goal_pose;
    LOG(INFO) << "收到的目标点为" << std::setprecision(14) << this->destination_pose_.pose.position.x << "||" << std::setprecision(14) << this->destination_pose_.pose.position.y << "||" << std::setprecision(14) << std::atan2(this->destination_pose_.pose.orientation.z, this->destination_pose_.pose.orientation.w) * 2.0; 
    this->destination_mutex_.unlock();
    // 更新紧急停车标志
    this->emergency_break_flag_mutex_.lock();
    this->IS_EMERGENCY_BREAK_FLAG_ = false;
    this->emergency_break_flag_mutex_.unlock();
    return true;
}

// 紧急停车服务
bool DecisionMaking::SubVehicle::forcedStop(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response) {
    if (!this->IS_EMERGENCY_BREAK_FLAG_) {
        LOG(INFO) << "紧急停车";
        // 服务返回信息
        response.success = true;
        response.message = "get message";
        // 启动急刹
        std_msgs::Empty emergency_break_msg;
        this->emergency_break_pub_.publish(emergency_break_msg);
        // 修改标志位
        this->mission_start_mutex_.lock();
        this->MISSION_START_FLAG_ = false;
        this->mission_start_mutex_.unlock();
        // 修改紧急停车标志位
        this->emergency_break_flag_mutex_.lock();
        this->IS_EMERGENCY_BREAK_FLAG_ = true;
        this->emergency_break_flag_mutex_.unlock();
    } else {
        LOG(INFO) << "已经处于紧急停车状态中";
        // 服务返回信息
        response.success = true;
        response.message = "get message";
    }

    return true;
}

// 判断任务结束
bool DecisionMaking::SubVehicle::missionFinishJudgement() {
    // 得到当前位置和速度
    PathPlanningUtilities::VehicleState current_position_in_world;
    this->current_vehicle_world_position_mutex_.lock();
    current_position_in_world = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    this->current_vehicle_kappa_mutex_.lock();
    double current_vehicle_kappa = this->current_vehicle_kappa_;
    current_position_in_world.kappa_ = current_vehicle_kappa;
    this->current_vehicle_kappa_mutex_.unlock();
    PathPlanningUtilities::VehicleMovementState current_movement_state;
    this->current_vehicle_movement_mutex_.lock();
    current_movement_state = this->current_vehicle_movement_;
    this->current_vehicle_movement_mutex_.unlock();
    // 任务结束条件，1.当前速度为0 2.当前离终点很近 3.当前状态为停车
    bool is_mission_finished = false;
    this->destination_mutex_.lock();
    double distance = sqrt((current_position_in_world.position_.x_ - this->destination_pose_.pose.position.x) * (current_position_in_world.position_.x_ - this->destination_pose_.pose.position.x) + (current_position_in_world.position_.y_ - this->destination_pose_.pose.position.y) * (current_position_in_world.position_.y_ - this->destination_pose_.pose.position.y));
    this->destination_mutex_.unlock();
    std::cout << "当前处于停车状态，离目标点的距离为" << distance << std::endl;
    LOG(INFO) << "当前处于停车状态，离目标点的距离为" << distance;
    if (Tools::isSmall(distance, REACH_DESTINATION_MAX_DISTANCE)) {
        if (Tools::isZero(current_movement_state.velocity_) && this->current_state_.getStateName() == StateNames::STOP) {
            is_mission_finished = true;
        }
    }
    // 判断任务是否结束
    if (is_mission_finished) {
        // 如果任务结束，重置目标点和开始任务标志位
        this->mission_start_mutex_.lock();
        this->MISSION_START_FLAG_ = false;
        this->mission_start_mutex_.unlock();
        // 调用到达目标点服务
        std_srvs::Trigger destination_reached_service;
        this->destination_reached_service_client_.call(destination_reached_service);
        if (destination_reached_service.response.success == true) {
            LOG(INFO) << "任务结束成功";
        } else {
            LOG(INFO) << "任务结束失败";
        }
        
    }
    return is_mission_finished;
}

// 障碍物信息callback函数，ros节点
void DecisionMaking::SubVehicle::getObstacles(const ibeo_lidar_msgs::object_filter_data::ConstPtr &msg) {
    this->perception_object_mutex_.lock();
    // 更新障碍物信息
    this->perception_objects_ = msg->objects;
    // 障碍物可视化(TOFIX)
    this->perception_object_mutex_.unlock();
}

// 历史路径callback函数,ros节点
void DecisionMaking::SubVehicle::getHistoryCurve(const path_planning_msgs::MotionPlanningCurve &msg) {
    this->history_curve_mutex_.lock();
    PathPlanningUtilities::Curve().swap(this->history_curve_);
    for (auto point: msg.points) {
        PathPlanningUtilities::CurvePoint curve_point;
        curve_point.position_.x_ = point.x;
        curve_point.position_.y_ = point.y;
        curve_point.theta_ = point.theta;
        curve_point.kappa_ = point.kappa;
        this->history_curve_.push_back(curve_point);
    }
    this->history_curve_mutex_.unlock();
}

// 规划和决策线程,20hz
void DecisionMaking::SubVehicle::motionPlanningThread() {
    ros::Rate loop_rate(MOTION_PLANNING_FREQUENCY);
    // 进程等待直到数据准备完成
    while (ros::ok()) {
        this->vehicle_surround_radar_ready_flag_mutex_.lock();
        bool surround_radar_ready_flag = !this->IS_SURROUND_RADAR_ENABLE_FLAG_ || 
        this->VEHICLE_SURROUND_RADAR_READY_FLAG_;
        vehicle_surround_radar_ready_flag_mutex_.unlock();
        this->vehicle_position_ready_flag_mutex_.lock();
        this->vehicle_movement_ready_flag_mutex_.lock();
        this->vehicle_curvature_ready_flag_mutex_.lock();
        bool data_read_flag = this->VEHICLE_POSITION_READY_FLAG_ && this->VEHICLE_MOVEMENT_READY_FLAG_ && this->VEHICLE_CURVATURE_READY_FLAG_;
        this->vehicle_curvature_ready_flag_mutex_.unlock();
        this->vehicle_movement_ready_flag_mutex_.unlock();
        this->vehicle_position_ready_flag_mutex_.unlock();
        if (data_read_flag && surround_radar_ready_flag) {
            break;
        }
        loop_rate.sleep();
    }
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++ data prepare finished +++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    LOG(INFO) << "++++++++++++++++++++++++++++++++++++++++++++++++++ data prepare finished +++++++++++++++++++++++++++++++++++++++++++++++++++++";
    // 主程序入口
    while (ros::ok()) {
        // 判断任务是否开始
        this->mission_start_mutex_.lock();
        bool is_mission_start = this->MISSION_START_FLAG_;
        this->mission_start_mutex_.unlock();
        if (!is_mission_start) {
            loop_rate.sleep();
            // 如果任务还没有开始，并且当前速度为0，初始化当前状态为停止状态
            PathPlanningUtilities::VehicleMovementState current_movement_state;
            this->current_vehicle_movement_mutex_.lock();
            current_movement_state = this->current_vehicle_movement_;
            this->current_vehicle_movement_mutex_.unlock();
            if (Tools::isZero(current_movement_state.velocity_)) {
                std::vector<size_t> neighbor_states;
                neighbor_states.push_back(StateNames::TURN_LEFT);
                neighbor_states.push_back(StateNames::TURN_RIGHT);
                neighbor_states.push_back(StateNames::FORWARD);
                neighbor_states.push_back(StateNames::AVOIDANCE);
                neighbor_states.push_back(StateNames::REVERSE);
                neighbor_states.push_back(StateNames::ROTATE);
                this->current_state_ = StandardState(StateNames::STOP, neighbor_states);
            }
            this->history_curve_mutex_.lock();
            this->history_curve_ = PathPlanningUtilities::Curve();
            this->history_curve_mutex_.unlock();
            continue;
        }
        // 判断任务是否结束
        if (this->current_state_.getStateName() == StateNames::STOP && Tools::isZero(this->current_state_.getVehicleCurrentMovement().velocity_)) {
            if (missionFinishJudgement()) {
                std::cout << "mission finished" << std::endl;
                LOG(INFO) << "mission finished";
                loop_rate.sleep();
                continue;
            }
        }

        // 首先清空状态机
        std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++ initialization for new planning +++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "++++++++++++++++++++++++++++++++++++++++++++++++++ initialization for new planning +++++++++++++++++++++++++++++++++++++++++++++++++++++";
        // 修改自检测标志位
        this->SELF_TEST_INIT_ = true;
        this->SELF_TEST_PLANNING_ = false;
        this->SELF_TEST_DECISION_MAKING_ = false;
        this->SELF_TEST_STATE_MAINTAINING_ = false;

        clock_t start_init_time, end_init_time;
        start_init_time = clock();
        this->initVehicleStates();
        std::cout <<"INIT VEHICLE STATE MACHINE SUCCESS" << std::endl;
        LOG(INFO) << "INIT VEHICLE STATE MACHINE SUCCESS";
        std::cout << "CURRENT STATE IS " << DIC_STATE_NAME[this->current_state_.getStateName()] << std::endl;
        LOG(INFO) << "CURRENT STATE IS " << DIC_STATE_NAME[this->current_state_.getStateName()];
        for (size_t i = 0; i < this->current_state_.getNeighborStates().size(); i++) {
            std::cout << "NEIGHBOR STATE INCLUDE " << DIC_STATE_NAME[this->current_state_.getNeighborStates()[i]] << std::endl;
            LOG(INFO) << "NEIGHBOR STATE INCLUDE " << DIC_STATE_NAME[this->current_state_.getNeighborStates()[i]];
        }
        end_init_time = clock();
        std::cout << "++++++++++++++++++++++++++ end of initialization for new planning, time consuming is " << static_cast<double>(end_init_time - start_init_time) * 1000.0 / CLOCKS_PER_SEC << " ms +++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "++++++++++++++++++++++++++ end of initialization for new planning, time consuming is " << static_cast<double>(end_init_time - start_init_time) * 1000.0 / CLOCKS_PER_SEC << " ms +++++++++++++++++++++++++++++++++++";
        // 进行局部规划，补全状态机（路径，优先度、可行性）
        std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ motion planning part ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ motion planning part ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
        // 修改自检测标志位
        this->SELF_TEST_INIT_ = false;
        this->SELF_TEST_PLANNING_ = true;
        this->SELF_TEST_DECISION_MAKING_ = false;
        this->SELF_TEST_STATE_MAINTAINING_ = false;

        clock_t start_motion_planning_time, end_motion_planning_time;
        start_motion_planning_time = clock();
        this->updateStates();
        // 如果局部规划失败，报错
        if (!this->center_lane_.getLaneExistance()) {
            ROS_ERROR("NO LANE OBTAINED FROM MAP");
            LOG(ERROR) << "NO LANE OBTAINED FROM MAP";
            continue;
        }
        std::cout << "TRAJECTORY GENERATION SUCCESS" << std::endl;
        LOG(INFO) << "TRAJECTORY GENERATION SUCCESS";
        end_motion_planning_time = clock();
        std::cout << "+++++++++++++++++++++++++++++++++++ end of motion planning part, time consuming is " << static_cast<double>(end_motion_planning_time - start_motion_planning_time) * 1000.0 / CLOCKS_PER_SEC  << " ms ++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "+++++++++++++++++++++++++++++++++++ end of motion planning part, time consuming is " << static_cast<double>(end_motion_planning_time - start_motion_planning_time) * 1000.0 / CLOCKS_PER_SEC  << " ms ++++++++++++++++++++++++++++++++++++++++++";
        // 进行决策，把状态机剩余信息添加（安全性、目标速度）
        std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ decision making part ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ decision making part ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
        // 修改自检测标志位
        this->SELF_TEST_INIT_ = false;
        this->SELF_TEST_PLANNING_ = false;
        this->SELF_TEST_DECISION_MAKING_ = true;
        this->SELF_TEST_STATE_MAINTAINING_ = false;

        clock_t start_decision_making_time, end_decision_making_time;
        start_decision_making_time = clock();
        // 进行动态速度规划
        this->checkStates();
        std::cout << "SAFETY JUDGEMENT SUCCESS" << std::endl;
        LOG(INFO) << "SAFETY JUDGEMENT SUCCESS";
        // // 填充特殊状态信息
        // this->generateLowPriorityStates();
        // 进行状态选择，获取目标状态(也就是choosed_state),进行可视化
        this->chooseStates();
        std::cout << "STATES CHOOSE SUCCESS" << std::endl;
        LOG(INFO) << "STATES CHOOSE SUCCESS";
        end_decision_making_time = clock();
        std::cout << "+++++++++++++++++++++++++++++++++++ end of decision making part, time consuming is " << static_cast<double>(end_decision_making_time - start_decision_making_time) * 1000.0 / CLOCKS_PER_SEC <<" ms ++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "+++++++++++++++++++++++++++++++++++ end of decision making part, time consuming is " << static_cast<double>(end_decision_making_time - start_decision_making_time) * 1000.0 / CLOCKS_PER_SEC <<" ms ++++++++++++++++++++++++++++++++++++++++++";
        // 判断规划模块的能力
        if (this->motionPlanningUncapableJudgement()) {
            // 任务结束
            LOG(INFO) << "无法继续自动执行任务";
            std::cout << "无法继续自动执行任务" << std::endl;
            continue;
        }
        // 进行状态保持，内容包括1.当前状态何时变成目标状态（满足一定条件）2.当前状态是否安全，如果不，则退出状态保持 3.是否完成状态路径，如果是，退出状态保持，进行新的规划
        // 状态保持判断频率为2hz
        // 如果期望状态和选中状态不同，则还要判断期望状态是否安全（期望状态只能是换到与直行）
        std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ state maintain part ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ state maintain part ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
        // 修改自检测标志位
        this->SELF_TEST_INIT_ = false;
        this->SELF_TEST_PLANNING_ = false;
        this->SELF_TEST_DECISION_MAKING_ = false;
        this->SELF_TEST_STATE_MAINTAINING_ = true;

        // 进行车辆和障碍物状态更新，判断当前状态是否安全，判断期望状态是否可行，进行动态速度规划。
        // 当选中的是避障状态时
        if (this->choosed_state_.getStateName() == StateNames::AVOIDANCE){
            
            this->avoidanceStateMaintain();
        }
        // 当选中的是停车状态时
        if (this->choosed_state_.getStateName() == StateNames::STOP) {
            
            this->stopStateMaintain();
        }

        // 当满足三大状态时
        if (this->choosed_state_.getStateName() == StateNames::FORWARD || this->choosed_state_.getStateName() == StateNames::TURN_LEFT || this->choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
            this->maintainStates();
        }
        // 当选中的是倒车状态时
        if (this->choosed_state_.getStateName() == StateNames::REVERSE) {
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
            marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(this->choosed_state_.getTotalTrajectory(), color, 0));
            this->vis_multi_curve_pub_.publish(marker_array);
            // 发布路径
            this->choosed_state_.publishCurveMsgPointReach(this->motion_planning_curve_pub_);
            // 等待直到控制完成
            this->control_finished_flag_mutex_.lock();
            this->CONTROL_FINISHED_FLAG_ = false;
            this->control_finished_flag_mutex_.unlock();
            // 判断是否完成
            ros::Rate wait_rate(1);
            while (true) {
                // 计算当前位置
                this->current_vehicle_world_position_mutex_.lock();
                PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
                this->current_vehicle_world_position_mutex_.unlock();
                // 计算距离
                double distance = PathPlanningUtilities::calcDistance(current_vehicle_world_position.position_, this->choosed_state_.getTotalTrajectory()[this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory()].position_);
                LOG(INFO) << "倒车距离误差仍然存在" << distance;
                // 获取控制是否结束
                this->control_finished_flag_mutex_.lock();
                bool is_control_finished = this->CONTROL_FINISHED_FLAG_;
                this->control_finished_flag_mutex_.unlock();
                if (is_control_finished) {
                    LOG(INFO) << "倒车状态完成";
                    break;
                }
                wait_rate.sleep();
            }
        }
        // 选中的是转向状态时
        if (this->choosed_state_.getStateName() == StateNames::ROTATE) {
            // 发布目标角度
            {
                // 车速为0时,将车辆转向回正
                path_planning_msgs::MotionPlanningCurve rotate_msgs;
                rotate_msgs.header.frame_id = "world";
                rotate_msgs.header.stamp = ros::Time::now();
                rotate_msgs.mode = path_planning_msgs::MotionPlanningCurve::ROTATE;
                rotate_msgs.tracking_mode = path_planning_msgs::MotionPlanningCurve::CENTER;
                rotate_msgs.aim_curvature = this->choosed_state_.getTotalTrajectory()[0].kappa_;
                this->motion_planning_curve_pub_.publish(rotate_msgs);
                // 判断回正是否完成,完成则结束此状态
                ros::Rate wait_rate(1);
                while (true) {
                    // 计算当前曲率
                    this->current_vehicle_kappa_mutex_.lock();
                    double current_curvature = this->current_vehicle_kappa_;
                    this->current_vehicle_kappa_mutex_.unlock();
                    if (Tools::isSmall(std::abs(current_curvature - this->choosed_state_.getTotalTrajectory()[0].kappa_), 0.01)) {
                        LOG(INFO) << "转向完成";
                        break;
                    }
                    wait_rate.sleep();
                }
            }
            // 发布路径
            {
                this->choosed_state_.publishCurveMsgPointReach(this->motion_planning_curve_pub_);
                // 等待直到控制完成
                this->control_finished_flag_mutex_.lock();
                this->CONTROL_FINISHED_FLAG_ = false;
                this->control_finished_flag_mutex_.unlock();
                // 判断车速的是否为0
                ros::Rate wait_rate(1);
                while (true) {
                    // 计算当前位置
                    this->current_vehicle_world_position_mutex_.lock();
                    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
                    this->current_vehicle_world_position_mutex_.unlock();
                    // 计算距离
                    double distance = PathPlanningUtilities::calcDistance(current_vehicle_world_position.position_, this->choosed_state_.getTotalTrajectory()[this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory()].position_);
                    LOG(INFO) << "倒车距离误差仍然存在" << distance;
                    // 获取控制是否结束
                    this->control_finished_flag_mutex_.lock();
                    bool is_control_finished = this->CONTROL_FINISHED_FLAG_;
                    this->control_finished_flag_mutex_.unlock();
                    if (is_control_finished) {
                        LOG(INFO) << "转向到达终点";
                        break;
                    }
                    wait_rate.sleep();
                }
            }
            // 回正
            {
                // 车速为0时,将车辆转向回正
                path_planning_msgs::MotionPlanningCurve rotate_msgs;
                rotate_msgs.header.frame_id = "world";
                rotate_msgs.header.stamp = ros::Time::now();
                rotate_msgs.mode = path_planning_msgs::MotionPlanningCurve::ROTATE;
                rotate_msgs.tracking_mode = path_planning_msgs::MotionPlanningCurve::CENTER;
                rotate_msgs.aim_curvature = 0.0;
                this->motion_planning_curve_pub_.publish(rotate_msgs);
                // 判断回正是否完成,完成则结束此状态
                ros::Rate wait_rate(1);
                while (true) {
                    // 计算当前曲率
                    this->current_vehicle_kappa_mutex_.lock();
                    double current_curvature = this->current_vehicle_kappa_;
                    std::cout << "回正方向盘误差为" << current_curvature << std::endl;
                    this->current_vehicle_kappa_mutex_.unlock();
                    if (Tools::isSmall(std::abs(current_curvature), 0.01)) {
                        LOG(INFO) << "转向完成";
                        std::cout << "转向完成" << std::endl;
                        break;
                    }
                    wait_rate.sleep();
                }
            }

        }
        std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ old planning finished ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        LOG(INFO) << "+++++++++++++++++++++++++++++++++++++++++++++++++++++ old planning finished ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
        loop_rate.sleep();
    }
}

// 自检测函数，用于判断程序执行到那个位置
void DecisionMaking::SubVehicle::selfTestForProgress(diagnostic_updater::DiagnosticStatusWrapper& status) {
    // 根据程序进行的标志位情况来得到程序运行的状态
    if (!(this->SELF_TEST_INIT_ || this->SELF_TEST_PLANNING_ || this->SELF_TEST_DECISION_MAKING_ || this->SELF_TEST_STATE_MAINTAINING_)) {
        // 所有的标志位都是false
        // 程序还未开始
        status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "DATA IS NOT PREPARED OR START COMMAND IS NOT RECEIVED.");
    } else if (this->SELF_TEST_INIT_) {
        // 程序正在执行初始化
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "PROGRAM IS IN INITIALIZATION.");
    } else if (this->SELF_TEST_PLANNING_) {
        // 程序正在执行规划
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "PROGRAM IS IN PLANNING.");
    } else if (this->SELF_TEST_DECISION_MAKING_) {
        // 程序正在执行决策
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "PROGRAM IS IN DECISION MAKING.");
    } else {
        // 程序正在执行状态保持
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "PROGRAM IS IN STATE MAINTAIN.");
    }
}
