/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

// Debug 可视化碰撞点
extern std::vector<Rectangle> collision_rectangles;

// 补全障碍物全部信息(与感知部分结果对接)
void DecisionMaking::SubVehicle::updateObstacleInformation() {
    // 判断是否存在历史轨迹点，如果列表中存在相同ID障碍物则添加历史轨迹，反之没有。
    // 添加障碍物状态信息
    // 如果速度大于最小值，表示障碍物有速度时，添加障碍物的预测轨迹信息；如果速度小于最小值则表示障碍物无速度，障碍物没有轨迹信息。

    // DEBUG
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::Point2f current_position;
    current_position.x_ = this->current_vehicle_world_position_.position_.x_;
    current_position.y_ = this->current_vehicle_world_position_.position_.y_;
    this->current_vehicle_world_position_mutex_.unlock();
    LOG(INFO) << "Current update obstacle information position x: " << current_position.x_ << " y:" << current_position.y_; 

    this->obstacle_mutex_.lock();
    this->perception_object_mutex_.lock();
    std::vector<Obstacle>().swap(this->obstacles_);
    for (size_t i = 0; i < this->perception_objects_.size(); i++) {
        // std::cout << "new obstacle information is added" << std::endl;
        // LOG(INFO) << "new obstacle information is added";
        ibeo_lidar_msgs::object_filter obstacle = this->perception_objects_[i];
        // 判断是否处于机非混行区域
        // GLOBAL_IN_JIFEI_MUTEX_.lock();
        // bool is_in_jifei = GLOBAL_IS_IN_JIFEI_;
        // GLOBAL_IN_JIFEI_MUTEX_.unlock();
        // if (!is_in_jifei) {
        //     // 不在机非混行区
        // 首先判断是边界框障碍物还是轮廓点障碍物
        if (obstacle.ibeo_contours_box.size() > 0) {
            // 轮廓点障碍物（每两个轮廓点构成一个边界框，多个边对应一个id界框）
            for (size_t j = 0; j < obstacle.ibeo_contours_box.size(); j++) {
                ibeo_lidar_msgs::Contour_box contour_box = obstacle.ibeo_contours_box[j];
                Obstacle obs(obstacle.id);
                PathPlanningUtilities::Point2f position;
                position.x_ = contour_box.center.x;
                position.y_ = contour_box.center.y;
                double v_theta = contour_box.orientation;
                double v = 0.0;
                // 补全障碍物状态信息
                this->updateObstacleState(&obs, position, contour_box.width, contour_box.length, contour_box.orientation, v, v_theta, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
                // 打印障碍物信息
                LOG(INFO) << "Obstacle id: " << obstacle.id << " boundary points obstacle position is (" << position.x_ << ", " << position.y_ << "), orientation is " << obstacle.orientation << ", width is " << obstacle.width << ", length is " << obstacle.length;
                // 对于静态障碍物，其预测轨迹就是其所在的位置，占用宽度就是其宽度
                obs.setObstacleOccupationWidth(obs.getObstacleWidth());
                PathPlanningUtilities::CurvePoint curve_point;
                curve_point.position_ = position;
                curve_point.theta_ = contour_box.orientation;
                curve_point.kappa_ = 0.0;
                PathPlanningUtilities::Curve curve;
                curve.push_back(curve_point);
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(curve);
                obs.setPredictedTrajectorySet(curve_set);
                // 只有障碍物存在预测轨迹才加入到障碍物列表中
                if (obs.getPredictedTrajectoryNumber() != 0) {
                    this->obstacles_.push_back(obs);
                } else {
                    LOG(INFO) << "此障碍物不存在预测轨迹，不加入障碍物列表";
                }
            }
        } else {
            // 边界框障碍物（边界框障碍物只会出现一个，并且一个框对应一个id）
            Obstacle obs(obstacle.id);
            PathPlanningUtilities::Point2f position;
            position.x_ = obstacle.center.x;
            position.y_ = obstacle.center.y;
            double v_theta = atan2(obstacle.velocity.y, obstacle.velocity.x);
            double v = obstacle.v;
            // 如果障碍物速度小于最小速度，视为静止
            if (Tools::isZero(v)) {
                v_theta = obstacle.orientation;
            } else if (Tools::isSmall(v, MIN_SPEED)) {
                v = 0.0;
            }
            // 打印障碍物信息
            LOG(INFO) << "Obstacle id: " << obstacle.id << " bounding box obstacle position is (" << position.x_ << ", " << position.y_ << "), orientation is " << obstacle.orientation << ", width is " << obstacle.width << ", length is " << obstacle.length << ", velocity is " << v << ", velocity orientation: " << v_theta;
            // 补全障碍物预测信息,静态物体没有预测轨迹
            if (!Tools::isZero(v)) {
                // 补全障碍物状态信息
                this->updateObstacleState(&obs, position, obstacle.width, obstacle.length, obstacle.orientation, v, v_theta, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
                // 如果速度较大,根据地图生成预测路径
                this->updateObstaclePredictedInformation(&obs);
            } else {
                // 如果速度较小,根据速度方向生成预测路径
                // double predict_distance = 0.5 * Tools::normalObstacleVehicleOccupationDistance(obstacle.v, MAX_DECCELERATION);
                // obs.setObstacleOccupationWidth(obs.getObstacleWidth());
                // // 计算新的中心点
                // position.x_ = position.x_ + 0.5 * predict_distance * cos(v_theta);
                // position.y_ = position.y_ + 0.5 * predict_distance * sin(v_theta);
                // double new_width = obstacle.width;
                // double new_length = obstacle.length + predict_distance;
                // 补全障碍物状态信息
                this->updateObstacleState(&obs, position, obstacle.width, obstacle.length, v_theta, v, v_theta, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
                // 对于静态障碍物，其预测轨迹就是其所在的位置，占用宽度就是其宽度
                obs.setObstacleOccupationWidth(obs.getObstacleWidth());
                PathPlanningUtilities::CurvePoint curve_point;
                curve_point.position_ = position;
                curve_point.theta_ = v_theta;
                curve_point.kappa_ = 0.0;
                PathPlanningUtilities::Curve curve;
                curve.push_back(curve_point);
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(curve);
                obs.setPredictedTrajectorySet(curve_set);
            }
            // 只有障碍物存在预测轨迹才加入到障碍物列表中
            if (obs.getPredictedTrajectoryNumber() != 0) {
                this->obstacles_.push_back(obs);
            } else {
                LOG(INFO) << "此障碍物不存在预测轨迹，不加入障碍物列表";
            }
        }
    } 
    this->perception_object_mutex_.unlock();

    // 障碍物信息可视化
    VisualizationMethods::visualizeObstacles(this->obstacles_, this->vis_obstacle_pub_);
    // 打印新障碍物的信息
    // std::cout << "obstacle information updated, obstacle number is " << this->obstacles_.size() << std::endl;
    // for (size_t i = 0; i < this->obstacles_.size(); i++) {
    //     std::cout << "obstacle number " << i << ": " << std::endl;
    //     std::cout << "position: " << this->obstacles_[i].getObstaclePosition().x_ << "||" << this->obstacles_[i].getObstaclePosition().y_ << std::endl;
    //     std::cout << "velocity: " << this->obstacles_[i].getObstacleVelocity() << std::endl;
    //     std::cout << "predicted trajectory number is " << this->obstacles_[i].getPredictedTrajectoryNumber() << std::endl;
    //     for (size_t j = 0; j < this->obstacles_[i].getPredictedTrajectoryNumber(); j++) {
    //         std::cout << "-- trajectory " << j << " length is " << this->obstacles_[i].getPredictedTrajectory(j).size() << std::endl;
    //     }
    // }

    LOG(INFO) << "obstacle information updated, obstacle number is " << this->obstacles_.size();
    // for (size_t i = 0; i < this->obstacles_.size(); i++) {
    //     LOG(INFO) << "obstacle number " << i << ": ";
    //     LOG(INFO) << "position: " << this->obstacles_[i].getObstaclePosition().x_ << "||" << this->obstacles_[i].getObstaclePosition().y_;
    //     LOG(INFO) << "velocity: " << this->obstacles_[i].getObstacleVelocity();
    //     LOG(INFO) << "predicted trajectory number is " << this->obstacles_[i].getPredictedTrajectoryNumber();
    //     for (size_t j = 0; j < this->obstacles_[i].getPredictedTrajectoryNumber(); j++) {
    //         LOG(INFO) << "-- trajectory " << j << " length is " << this->obstacles_[i].getPredictedTrajectory(j).size();
    //     }
    // }
    this->obstacle_mutex_.unlock();
}

// 补全障碍物的状态信息，其中acceleration和class_name为无用值，只要给占位符即可
void DecisionMaking::SubVehicle::updateObstacleState(Obstacle* obstacle, const PathPlanningUtilities::Point2f position, double width, double length, double orientation, double velocity, double velocity_direction, double acceleration, size_t class_name) {
    obstacle->setObstaclePosition(position);
    obstacle->setObstacleShape(width, length);
    obstacle->setObstacleOrientation(orientation);
    obstacle->setObstacleVelocity(velocity);
    obstacle->setObstacleVelocityDirection(velocity_direction);
    obstacle->setObstacleAcceleration(acceleration);
    obstacle->setObstacleClass(class_name);
}

// 补全障碍物的预测信息、轨迹和位置(TOFIX)
void DecisionMaking::SubVehicle::updateObstaclePredictedInformation(Obstacle* obstacle) {
    // clock_t obstacle_service_start_time = clock();
    // 地图服务所需参数，本车位置
    geometry_msgs::PoseStamped vehicle_pose;
    vehicle_pose.header.frame_id = "world";
    vehicle_pose.header.stamp = ros::Time::now();
    this->current_vehicle_world_position_mutex_.lock();
    vehicle_pose.pose.position.x = this->current_vehicle_world_position_.position_.x_;
    vehicle_pose.pose.position.y = this->current_vehicle_world_position_.position_.y_;
    vehicle_pose.pose.position.z = 0.0;
    vehicle_pose.pose.orientation.x = 0.0;
    vehicle_pose.pose.orientation.y = 0.0;
    vehicle_pose.pose.orientation.z = sin(this->current_vehicle_world_position_.theta_/2.0);
    vehicle_pose.pose.orientation.w = cos(this->current_vehicle_world_position_.theta_/2.0);
    this->current_vehicle_world_position_mutex_.unlock();
    // 地图服务所需参数，位姿
    geometry_msgs::PoseStamped obstacle_pose;
    obstacle_pose.header.frame_id = "world";
    obstacle_pose.header.stamp = ros::Time::now();
    obstacle_pose.pose.position.x = obstacle->getObstaclePosition().x_;
    obstacle_pose.pose.position.y = obstacle->getObstaclePosition().y_;
    obstacle_pose.pose.position.z = 0.0;
    obstacle_pose.pose.orientation.x = 0.0;
    obstacle_pose.pose.orientation.y = 0.0;
    obstacle_pose.pose.orientation.z = sin(obstacle->getObstacleOrientation()/2.0);
    obstacle_pose.pose.orientation.w = cos(obstacle->getObstacleOrientation()/2.0);
    // 是否需要无视车辆正后方障碍物
    bool ignore_rear_side = true;
    // 障碍物id
    size_t id = obstacle->getID();
    // 地图服务所需参数，计算长度，长度为障碍物以当前速度行驶3秒的距离(vt + (v*t + v^2/2a + constant_distance) + constant_distance)
    double distance;
    // 障碍物速度不为0时，取障碍物的刹车距离
    if (obstacle->getObstacleClass() == vec_map_cpp_msgs::GetPredictedTrajectory::Request::PEDESTRIAN) {
        distance = CONSTANT_DISTANCE;
    } else {
        distance = Tools::normalObstacleVehicleOccupationDistance(obstacle->getObstacleVelocity(), COMMON_DECCELERATION);
    }
    // 地图服务所需参数，形状
    double width = obstacle->getObstacleWidth();
    double length = obstacle->getObstacleLength();
    // 地图服务所需参数，速度方向
    double direction = obstacle->getObstacleVelocityDirection();
    // 地图服务所需参数，点间距
    double point_margin = OBSTACLE_MARGIN;
    // 地图服务所需参数，类别(TOFIX)
    double obstacle_class = obstacle->getObstacleClass();

    // 调用地图服务（TOFIX）
    vec_map_cpp_msgs::GetPredictedTrajectory predict_trajectory_service;
    predict_trajectory_service.request.id = id;
    predict_trajectory_service.request.ignore_rear_side = ignore_rear_side;
    predict_trajectory_service.request.vehicle_pose = vehicle_pose;
    predict_trajectory_service.request.current_pose = obstacle_pose;
    predict_trajectory_service.request.width = width;
    predict_trajectory_service.request.length = length;
    predict_trajectory_service.request.speed_orientation = direction;
    predict_trajectory_service.request.request_length = distance;
    predict_trajectory_service.request.point_margin = point_margin;
    predict_trajectory_service.request.type = obstacle_class;
    predict_trajectory_service.request.current_pose_orientation_unknown = false;
    this->obstacle_trajectory_prediction_service_client_.call(predict_trajectory_service);
    // 地图服务返回state表示障碍物速度方向不正确或位置不正确或正常
    int state = predict_trajectory_service.response.state;
    // std::cout << "predict obstacle server result: " << is_out_map << std::endl;
    // clock_t obstacle_service_end_time = clock();
    // std::cout << "obstacle service time consuming is: " << (obstacle_service_end_time - obstacle_service_start_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;
    if (state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::NORMAL) {
        // 地图服务正常，返回轨迹和道路最小宽度
        LOG(INFO) << "此障碍物轨迹预测正常";
        obstacle->setObstacleOccupationWidth(std::max(predict_trajectory_service.response.road_width * OBSTACLE_OCCUPANCY_WIDTH_MAX_TO_LANE_WIDTH, obstacle->getObstacleWidth() + OBSTACLE_OCCUPANCY_WIDTH_EXPAND_CONST));
        // clock_t obstacle_processing_start_time, obstacle_processing_end_time;
        std::vector<path_planning_msgs::Curve> predicted_ros_curves = predict_trajectory_service.response.paths;
        // std::cout << "predicted trajectory number from map server: " << predicted_ros_paths.size() << std::endl;
        // for (size_t i = 0; i < predicted_ros_paths.size(); i++) {
        //     std::cout << "predicted trajectory " << i << " point number is " << predicted_ros_paths[i].points.size() << std::endl;
        //     std::cout << "predicted trajectory " << i << " distance is " << sqrt((predicted_ros_paths[i].points[0].x - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].x) * (predicted_ros_paths[i].points[0].x - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].x) + (predicted_ros_paths[i].points[0].y - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].y) * (predicted_ros_paths[i].points[0].y - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].y)) << std::endl;
        //     for (size_t j = 0; j < predicted_ros_paths[i].points.size(); j++) {
        //         std::cout << "point is " << predicted_ros_paths[i].points[j].x << "||" << predicted_ros_paths[i].points[j].y << std::endl;
        //     }
        // }
        std::vector<PathPlanningUtilities::Curve> predicted_trajectories;
        predicted_trajectories.resize(predicted_ros_curves.size());
        // obstacle_processing_start_time = clock();
        for (size_t i = 0; i < predicted_ros_curves.size(); i++) {
            PathPlanningUtilities::Curve predicted_trajectory_curve;
            PathPlanningUtilities::CurveConverter::fromRosMessage(predicted_ros_curves[i], predicted_trajectory_curve);
            predicted_trajectories[i] = predicted_trajectory_curve;
        }
        obstacle->setPredictedTrajectorySet(predicted_trajectories);
        // obstacle_processing_end_time = clock();
        // std::cout << "obstacle trajectory processing time consuming is: " << (obstacle_processing_end_time - obstacle_processing_start_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;
    } else if (state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::WRONG_ORIENTATION || state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::HIGH_UNCERTAINTY) {
        // 障碍物方向与地图路径方向存在较大差异性
        // TOFIX
        LOG(INFO) << "此障碍物方向与其所处道路朝向存在较大差异或不可信";
        // 得到占用宽度
        obstacle->setObstacleOccupationWidth(obstacle->getObstacleWidth() + OBSTACLE_OCCUPANCY_WIDTH_EXPAND_CONST);
        // 生成延障碍物速度方向的预测轨迹
        std::vector<PathPlanningUtilities::Curve> predicted_trajectories;
        PathPlanningUtilities::Curve predicted_trajectory_curve;
        size_t predicted_trajectory_size = static_cast<size_t>(distance / OBSTACLE_MARGIN);
        for (size_t i = 0; i < predicted_trajectory_size; i++) {
            PathPlanningUtilities::CurvePoint curve_point;
            curve_point.position_.x_ = obstacle->getObstaclePosition().x_ + i * OBSTACLE_MARGIN * cos(obstacle->getObstacleVelocityDirection());
            curve_point.position_.y_ = obstacle->getObstaclePosition().y_ + i * OBSTACLE_MARGIN * sin(obstacle->getObstacleVelocityDirection());
            curve_point.theta_ = obstacle->getObstacleVelocityDirection();
            curve_point.kappa_ = 0.0;
            predicted_trajectory_curve.push_back(curve_point);
        }
        predicted_trajectories.push_back(predicted_trajectory_curve);
        obstacle->setPredictedTrajectorySet(predicted_trajectories);
    } else if (state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::OUT_MAP || state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::IGNORED) {
        // 障碍物位置在地图外面
        // TOFIX
        LOG(INFO) << "此障碍物处于地图外或被无视" << obstacle->getObstaclePosition().x_ << "||" << obstacle->getObstaclePosition().y_;
    }
}

// 得到有效的交通障碍物列表
void DecisionMaking::SubVehicle::updateValidateTrafficRuleInformation() {
    // 首先清空之间的有效交通规则障碍物列表
    std::vector<vec_map_cpp_msgs::VirtualObstacle>().swap(this->traffic_rule_obstacles_);
    // 得到当前位置信息和速度信息
    PathPlanningUtilities::VehicleState current_position_in_world;
    this->current_vehicle_world_position_mutex_.lock();
    current_position_in_world = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    PathPlanningUtilities::VehicleMovementState current_movement_state;
    this->current_vehicle_movement_mutex_.lock();
    current_movement_state = this->current_vehicle_movement_;
    this->current_vehicle_movement_mutex_.unlock();
    // 遍历原始交通障碍物列表
    for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
        // 判断原始障碍物类型
        if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::PERMANENT || this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::BEYOND_GOAL) {
            // 如果是永久墙体，直接加入有效障碍物
            this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
        } else if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::NOT_PERMANENT) {
            // 如果是非永久墙体，进行判断
            if (this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_) {
                // 当满足条件1. 当前状态是停车状态，且速度为0。2. 当前位置离空气墙中心很近。此空气墙无效化
                bool is_validate = true;
                if (this->current_state_.getStateName() == StateNames::STOP && Tools::isZero(this->current_state_.getVehicleCurrentMovement().velocity_)) {
                    // 满足第一个条件后，开始判断第二个条件
                    // 计算空气墙中心点
                    PathPlanningUtilities::Point2f traffic_rule_obstacle_center;
                    traffic_rule_obstacle_center.x_ = 0.0;
                    traffic_rule_obstacle_center.y_ = 0.0;
                    for (size_t j = 0; j < this->traffic_rule_obstacles_raw_[i].points.size(); j++) {
                        traffic_rule_obstacle_center.x_ += this->traffic_rule_obstacles_raw_[i].points[j].x;
                        traffic_rule_obstacle_center.y_ += this->traffic_rule_obstacles_raw_[i].points[j].y;
                    }
                    traffic_rule_obstacle_center.x_ = traffic_rule_obstacle_center.x_ / this->traffic_rule_obstacles_raw_[i].points.size();
                    traffic_rule_obstacle_center.y_ = traffic_rule_obstacle_center.y_ / this->traffic_rule_obstacles_raw_[i].points.size();
                    // 计算距离
                    double distance = PathPlanningUtilities::calcDistance(current_position_in_world.position_, traffic_rule_obstacle_center);
                    if (Tools::isSmall(distance, MAX_DISTANCE_TO_NOT_PERMANENT_TRAFFIC_RULE_TO_MAKE_INVALID)) {
                        is_validate = false;
                    }
                }
                if (is_validate) {
                    this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                } else {
                    LOG(INFO) << "车辆当前处于停止状态，临时空气墙无效，并且将其从原始数据中删除";
                    this->traffic_rule_obstacles_raw_[i].mode = vec_map_cpp_msgs::VirtualObstacle::INVALID;
                }
            } else {
                LOG(INFO) << "临时空气墙未被使用，无效";
            }

        } else if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::CONDITION_DECISION) {
            // 如果是交通灯，首先判断是否进行了使用
            if (this->TRAFFIC_LIGHT_USAGE_FLAG_) {
                // 判断离灯距离
                PathPlanningUtilities::Point2f traffic_rule_center_point;
                traffic_rule_center_point.x_ = 0.0;
                traffic_rule_center_point.y_ = 0.0;
                for (size_t point_index = 0; point_index < this->traffic_rule_obstacles_raw_[i].points.size(); point_index++) {
                    traffic_rule_center_point.x_ += this->traffic_rule_obstacles_raw_[i].points[point_index].x;
                    traffic_rule_center_point.y_ += this->traffic_rule_obstacles_raw_[i].points[point_index].y;
                }
                traffic_rule_center_point.x_ = traffic_rule_center_point.x_ / static_cast<double>(this->traffic_rule_obstacles_raw_[i].points.size());
                traffic_rule_center_point.y_ = traffic_rule_center_point.y_ / static_cast<double>(this->traffic_rule_obstacles_raw_[i].points.size());
                LOG(INFO) << "traffic block center is" <<traffic_rule_center_point.x_ << " " << traffic_rule_center_point.y_;
                this->current_vehicle_world_position_mutex_.lock();
                double remain_distance = Tools::calcNewCoordinationPosition(this->current_vehicle_world_position_, traffic_rule_center_point).x_;
                LOG(INFO) << "current_position is " << this->current_vehicle_world_position_.position_.x_ << " " << this->current_vehicle_world_position_.position_.y_;
                double direction_distance = sqrt((this->current_vehicle_world_position_.position_.x_ - traffic_rule_center_point.x_)*(this->current_vehicle_world_position_.position_.x_ - traffic_rule_center_point.x_)+(this->current_vehicle_world_position_.position_.y_ - traffic_rule_center_point.y_)*(this->current_vehicle_world_position_.position_.y_ - traffic_rule_center_point.y_));
                this->current_vehicle_world_position_mutex_.unlock();
                if (Tools::isLarge(remain_distance, MAX_DISTANCE_TO_DETECT_TRAFFIC_LIGHT) || Tools::isLarge(direction_distance, MAX_DISTANCE_TO_DETECT_TRAFFIC_LIGHT)) {
                    LOG(INFO) << "离红绿灯超过60米，不进行处理";
                    continue;
                }
                LOG(INFO) << "traffic light remain distance is " << remain_distance;

                // 调用服务
                traffic_light_msgs::traffic_lights traffic_light_service;
                // 提供服务所需参数
                traffic_light_service.request.direction = this->traffic_rule_obstacles_raw_[i].traffic_light_mode;
                traffic_light_service.request.num = this->traffic_rule_obstacles_raw_[i].traffic_light_id;
                // 调用服务
                this->traffic_light_service_client_.call(traffic_light_service);
                LOG(INFO) << "traffic light number " << static_cast<int>(this->traffic_rule_obstacles_raw_[i].traffic_light_id) << " direction " << static_cast<int>(this->traffic_rule_obstacles_raw_[i].traffic_light_mode) << "is called";
                // 得到交通灯结果
                int traffic_light_result = traffic_light_service.response.move_signal;
                if (traffic_light_result == 1) {
                    // 绿灯，空气墙无效
                    LOG(INFO) << "方向" << this->traffic_rule_obstacles_raw_[i].traffic_light_mode << "前方绿灯，空气墙无效";
                } else {
                    LOG(INFO) << "方向" << this->traffic_rule_obstacles_raw_[i].traffic_light_mode << "前方红灯，空气墙有效, time consume is" << traffic_light_service.response.consume_time;
                    if (Tools::isLarge(traffic_light_service.response.consume_time, CONSUME_TIME_THRESHOLD_FOR_YELLOW_LIGHT_JUDGEMENT)) {
                        LOG(INFO) << "黄灯阶段";
                        // 判断红灯是否可以闯过去
                        // double remain_time = std::max(2.8 - traffic_light_service.response.consume_time, 0.0);
                        // std::cout << "remian time is" << remain_time << std::endl;                            
                        // // 判断是否可以超过, 1.以最大加速度无法刹车停下，此时选择通过 2. 减速到3.0米/秒仍然可以通过，此时选择通过
                        // this->current_vehicle_movement_mutex_.lock();
                        // double current_velocity = this->current_vehicle_movement_.velocity_;
                        // this->current_vehicle_movement_mutex_.unlock();
                        // LOG(INFO) << "黄灯还剩余时间" << remain_time << "剩余距离为" << remain_distance << ",当前速度" << current_velocity;
                        // double stop_distance = current_velocity * current_velocity / 5.0;
                        // if (Tools::isLarge(remain_distance, stop_distance + 0.5 * this->vehicle_length_)) {
                        //     // 能够以最大加速度刹车停下
                        //     LOG(INFO) << "刹车能够刹下来, 给停车";
                        //     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                        // } else {
                        //     // 以最大加速度无法刹车停下
                        //     LOG(INFO) << "当前速度无法刹车停下";
                        // }
                        
                        // if (Tools::isLarge(travellable_distance, remain_distance + this->vehicle_length_)) {
                        //     LOG(INFO) << "满足条件1，可以通过";
                        // } else if (Tools::isLarge(stop_distance, remain_distance - 0.5 * this->vehicle_length_)){
                        //     LOG(INFO) << "满足条件2，可以通过";
                        // } else {
                        //     LOG(INFO) << "不可以通过";
                        //     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                        // }
                        // 第一种情况车已经到停止线上，直接走
                        // 第二种情况车还没停止线，进行判断
                        // 黄灯阶段逻辑梳理
                        // 1. 已知数据：黄灯剩余时间，车辆中心离停止线距离，车长，车辆当前速度，车辆最低速度？
                        // 2. 如果在剩余时间内车辆中心能够超过停止线，则直接忽略红灯。
                        // 3. 如果不满足上诉条件，判断以最大加速度是否可以保持车头在停止线内，如果可以则停下。
                        // 4. 如果不满足上诉条件，直接忽略红灯
                        if (Tools::isSmall(remain_distance, VEHICLE_OUTOF_TRAFFIC_LINE_JUDGEMENT_COEF * this->vehicle_length_)) {
                            // 车已经到停止线上，直接走
                            LOG(INFO) << "车在线上，直接走";
                        } else {
                            // 黄灯的总时长
                            double total_time = YELLOW_LIGHT_DURATION;
                            // 计算黄的剩余的时长
                            double remain_time = std::max(total_time - traffic_light_service.response.consume_time, 0.0);
                            std::cout << "remain time is" << remain_time << std::endl;
                            this->current_vehicle_movement_mutex_.lock();
                            double current_velocity = this->current_vehicle_movement_.velocity_;
                            this->current_vehicle_movement_mutex_.unlock();
                            LOG(INFO) << "黄灯还剩余时间" << remain_time << "剩余距离为" << remain_distance << ",当前速度" << current_velocity;
                            // 判断闯黄灯时能走过的距离,采用的是匀变速运动
                            double max_travel_distance = current_velocity + std::min(current_velocity, MAX_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_YELLOW_LIGHT) * 0.5 * remain_time;
                            // 判断在黄灯前紧急停车需要的距离
                            double max_stop_distance = current_velocity * current_velocity / MAX_ACCELERATION_IN_YELLOW_LIGHT;
                            LOG(INFO) << "最大行驶距离为" << max_travel_distance;
                            LOG(INFO) << "最大刹车距离为" << max_stop_distance;
                            // 开始判断是刹车还是开过去
                            if (Tools::isSmall(remain_distance - max_travel_distance, VEHICLE_OUTOF_TRAFFIC_LINE_JUDGEMENT_COEF * this->vehicle_length_)) {
                                // 可以开过去
                                LOG(INFO) << "当前速度可以开过去";
                            } else {
                                if (Tools::isLarge(remain_distance - max_stop_distance, this->vehicle_length_ * 0.5)) {
                                    LOG(INFO) << "当前速度可以停下";
                                    this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                                } else {
                                    LOG(INFO) << "危险阶段，无法进行判断";
                                    if (Tools::isLarge(current_velocity, MIN_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_YELLOW_LIGHT_WITH_EMERGENCY)) {
                                        // 车速大于5.0米，直接过
                                    } else {
                                        //车速小于5.0米，刹车
                                        this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                                    }
                                }
                            }
                        }
                    } else {
                        LOG(INFO) << "非黄灯阶段";
                        // // 判断车辆刹车停下是否会超出停止线
                        // this->current_vehicle_movement_mutex_.lock();
                        // double current_velocity = this->current_vehicle_movement_.velocity_;
                        // this->current_vehicle_movement_mutex_.unlock();
                        // if (Tools::isSmall(remain_distance, this->vehicle_length_ * 0.5) && !Tools::isZero(current_velocity)) {
                        //     // 当前速度停下，车辆不会超出停止线
                        //     LOG(INFO) << "车头已经超线，直接走";
                        // } else {
                        //     LOG(INFO) << "无法保持停车不超过车道线";
                        //     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                        // }
                        // 如果车压在停止线上且速度大于2m/s，可以走
                        this->current_vehicle_movement_mutex_.lock();
                        double current_velocity = this->current_vehicle_movement_.velocity_;
                        this->current_vehicle_movement_mutex_.unlock();
                        if (Tools::isSmall(remain_distance, this->vehicle_length_ * VEHICLE_OUTOF_TRAFFIC_LINE_JUDGEMENT_COEF) && Tools::isLarge(current_velocity, MIN_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_RED_LIGHT)) {
                            // 走
                            LOG(INFO) << "闯红灯";
                        } else {
                            this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                        }
                        
                    }
                }
            } else {
                LOG(INFO) << "没有使用交通灯，交通灯空气墙当做临时空气墙处理";
                if (this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_) {
                    // 当满足条件1. 当前状态是停车状态，且速度为0。2. 当前位置离空气墙中心很近。此空气墙无效化
                    bool is_validate = true;
                    if (this->current_state_.getStateName() == StateNames::STOP && Tools::isZero(this->current_state_.getVehicleCurrentMovement().velocity_)) {
                        // 满足第一个条件后，开始判断第二个条件
                        // 计算空气墙中心点
                        PathPlanningUtilities::Point2f traffic_rule_obstacle_center;
                        traffic_rule_obstacle_center.x_ = 0.0;
                        traffic_rule_obstacle_center.y_ = 0.0;
                        for (size_t j = 0; j < this->traffic_rule_obstacles_raw_[i].points.size(); j++) {
                            traffic_rule_obstacle_center.x_ += this->traffic_rule_obstacles_raw_[i].points[j].x;
                            traffic_rule_obstacle_center.y_ += this->traffic_rule_obstacles_raw_[i].points[j].y;
                        }
                        traffic_rule_obstacle_center.x_ = traffic_rule_obstacle_center.x_ / this->traffic_rule_obstacles_raw_[i].points.size();
                        traffic_rule_obstacle_center.y_ = traffic_rule_obstacle_center.y_ / this->traffic_rule_obstacles_raw_[i].points.size();
                        // 计算距离
                        double distance = PathPlanningUtilities::calcDistance(current_position_in_world.position_, traffic_rule_obstacle_center);
                        // std::cout << "miaomiaomiao" << distance << std::endl;
                        // std::cout << "center_position is " << traffic_rule_obstacle_center.x_ << "||" << traffic_rule_obstacle_center.y_ << std::endl; 
                        // LOG(INFO) << "miaomiaomiao" << distance;
                        // LOG(INFO) << "center_position is " << traffic_rule_obstacle_center.x_ << "||" << traffic_rule_obstacle_center.y_;
                        if (Tools::isSmall(distance, MAX_DISTANCE_TO_NOT_PERMANENT_TRAFFIC_RULE_TO_MAKE_INVALID)) {
                            is_validate = false;
                        }
                    }
                    if (is_validate) {
                        this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
                    } else {
                        LOG(INFO) << "车辆当前处于停止状态，临时空气墙无效，并且将其从原始数据中删除";
                        this->traffic_rule_obstacles_raw_[i].mode = vec_map_cpp_msgs::VirtualObstacle::INVALID;
                    }
                } else {
                    LOG(INFO) << "临时空气墙未被使用，无效";
                }
            }
        } else if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::INVALID){
            // 空气墙模式为无效模式，不用处理
            LOG(INFO) << "无效空气墙";
        }
    }
}

// 构造障碍物类型的占用区，占用区为障碍物在在第lane_index条路径上的占用区
DecisionMaking::RSS::OccupationArea::OccupationArea(const Obstacle &obstacle, size_t lane_index, int sample_gap, bool as_occupation) {
    // 初始化占用区域
    if (!Tools::isZero(obstacle.getObstacleVelocity())) {
        // 如果不是静止障碍物
        // 1. 得到终点下标
        assert(obstacle.getPredictedTrajectory(lane_index).size() > 0);
        size_t end_index = obstacle.getPredictedTrajectory(lane_index).size();
        // 2. 构造占用区域
        this->occupation_area_.resize(end_index);
        if (as_occupation) {
            // 作为占用区存在，则障碍物宽度为道路宽度*0.9
            for (size_t i = 0; i < end_index; i++) {
                Rectangle rectangle = Rectangle();
                rectangle.center_x_ = obstacle.getPredictedTrajectory(lane_index)[i].position_.x_;
                rectangle.center_y_ = obstacle.getPredictedTrajectory(lane_index)[i].position_.y_;
                rectangle.width_ = obstacle.getObstacleOccupationWidth();
                rectangle.length_ = obstacle.getObstacleLength() + DYNAMIC_OBSTACLE_EXPAND_LENGTH_AS_OCCUPANCY;
                rectangle.rotation_ = obstacle.getPredictedTrajectory(lane_index)[i].theta_;
                this->occupation_area_[i] = rectangle;
            }
        } else {
            // 作为实际障碍物存在，障碍物宽度为其实际宽度1.2倍，长度也是1.2倍
            for (size_t i = 0; i < end_index; i++) {
                Rectangle rectangle;
                rectangle.center_x_ = obstacle.getPredictedTrajectory(lane_index)[i].position_.x_;
                rectangle.center_y_ = obstacle.getPredictedTrajectory(lane_index)[i].position_.y_;
                rectangle.width_ = obstacle.getObstacleWidth() + DYNAMIC_OBSTACLE_EXPAND_WIDTH_NOT_AS_OCCUPANCY;
                rectangle.length_ = obstacle.getObstacleLength() + DYNAMIC_OBSTACLE_EXPAND_LENGTH_NOT_AS_OCCUPANCY;
                rectangle.rotation_ = obstacle.getPredictedTrajectory(lane_index)[i].theta_;
                this->occupation_area_[i] = rectangle;
            }
        }
    } else {
        // 如果是静止障碍物
        // 2. 构造占用区域
        this->occupation_area_.resize(1);
        Rectangle rectangle;
        rectangle.center_x_ = obstacle.getObstaclePosition().x_;
        rectangle.center_y_ = obstacle.getObstaclePosition().y_;
        if (as_occupation) {
            // 作为占用区存在
            // if (GLOBAL_IS_IN_SLOW_) {
            //     // 盲区增大
            //     add_offset = 6.5;
            // } 
            // if (GLOBAL_IS_IN_OVERTAKE_) {
            //     // 超车减少
            //     add_offset = 1.2;
            // }
            rectangle.width_ = obstacle.getObstacleWidth() + STATIC_OBSTACLE_EXPAND_WIDTH_AS_OCCUPANCY;
            rectangle.length_ = obstacle.getObstacleLength() + STATIC_OBSTACLE_EXPAND_LENGTH_AS_OCCUPANCY;
        } else {
            // 作为实际障碍物存在
            rectangle.width_ = obstacle.getObstacleWidth() + STATIC_OBSTACLE_EXPAND_WIDTH_NOT_AS_OCCUPANCY;
            rectangle.length_ = obstacle.getObstacleLength() + STATIC_OBSTACLE_EXPAND_LENGTH_NOT_AS_OCCUPANCY;            
        }

        rectangle.rotation_ = obstacle.getObstacleOrientation();
        this->occupation_area_[0] = rectangle;
    }
    // 初始化采样占用区域
    this->samplingOccupationArea(sample_gap);
    // 设置默认优先级
    this->setPriority(false);
}

// 构造障碍物类型的占用区，占用区为障碍物在在第lane_index条路径上的占用区
DecisionMaking::RSS::OccupationArea::OccupationArea(const Obstacle &obstacle, size_t lane_index, double velocity, int sample_gap) {
    // 初始化占用区域
    if (!Tools::isZero(obstacle.getObstacleVelocity())) {
        // 如果不是静止障碍物
        // 1. 得到终点下标
        assert(obstacle.getPredictedTrajectory(lane_index).size() > 0);
        size_t end_index = obstacle.getPredictedTrajectory(lane_index).size();
        // 2. 构造占用区域
        this->occupation_area_.resize(end_index);
        // 作为实际障碍物存在，障碍物宽度为其实际宽度1.2倍，长度也是1.2倍
        for (size_t i = 0; i < end_index; i++) {
            Rectangle rectangle;
            rectangle.center_x_ = obstacle.getPredictedTrajectory(lane_index)[i].position_.x_;
            rectangle.center_y_ = obstacle.getPredictedTrajectory(lane_index)[i].position_.y_;
            rectangle.width_ = obstacle.getObstacleWidth() + DYNAMIC_OBSTACLE_EXPAND_WIDTH_NOT_AS_OCCUPANCY + std::min(MAX_EXTRA_EXPAND_FOR_VELOCITY, std::max(0.0, velocity - EXTRA_EXPAND_MIN_VELOCITY) * EXTRA_EXPAND_VELOCITY_COEF);
            rectangle.length_ = obstacle.getObstacleLength() + DYNAMIC_OBSTACLE_EXPAND_LENGTH_NOT_AS_OCCUPANCY + std::min(MAX_EXTRA_EXPAND_FOR_VELOCITY, std::max(0.0, velocity - EXTRA_EXPAND_MIN_VELOCITY) * EXTRA_EXPAND_VELOCITY_COEF);
            rectangle.rotation_ = obstacle.getPredictedTrajectory(lane_index)[i].theta_;
            this->occupation_area_[i] = rectangle;
        }
    } else {
        // 如果是静止障碍物
        // 2. 构造占用区域
        this->occupation_area_.resize(1);
        Rectangle rectangle;
        rectangle.center_x_ = obstacle.getObstaclePosition().x_;
        rectangle.center_y_ = obstacle.getObstaclePosition().y_;
        // 作为实际障碍物存在
        rectangle.width_ = obstacle.getObstacleWidth() + STATIC_OBSTACLE_EXPAND_WIDTH_NOT_AS_OCCUPANCY + std::min(MAX_EXTRA_EXPAND_FOR_VELOCITY, std::max(0.0, velocity - EXTRA_EXPAND_MIN_VELOCITY) * EXTRA_EXPAND_VELOCITY_COEF);
        rectangle.length_ = obstacle.getObstacleLength() + STATIC_OBSTACLE_EXPAND_LENGTH_NOT_AS_OCCUPANCY + std::min(MAX_EXTRA_EXPAND_FOR_VELOCITY, std::max(0.0, velocity - EXTRA_EXPAND_MIN_VELOCITY) * EXTRA_EXPAND_VELOCITY_COEF);            

        rectangle.rotation_ = obstacle.getObstacleOrientation();
        this->occupation_area_[0] = rectangle;
    }
    // 初始化采样占用区域
    this->samplingOccupationArea(sample_gap);
    // 设置默认优先级
    this->setPriority(false);
}

// 构造车道类型的占用区
DecisionMaking::RSS::OccupationArea::OccupationArea(const StandardState &judge_state, int sample_gap, double width_expanded_factor, double length_expanded_factor) {
    // 根据车辆当前位置得到起始下标点
    size_t start_index = judge_state.getVehicleCurrentPositionIndexInTrajectory();
    // 计算结束下标点
    double occupation_distance = Tools::normalSubvehicleOccupationDistance(judge_state.getExpectedVelocityCurrent(), MAX_DECCELERATION, CONSTANT_DISTANCE);
    size_t end_index = std::min(start_index + static_cast<size_t>(occupation_distance/LANE_GAP_DISTANCE), judge_state.getTrajectoryLength() + judge_state.getExtendedTrajectoryLength());

    // 初始化占用区域
    this->occupation_area_.resize(end_index - start_index);
    for (size_t i = start_index; i < end_index; i++) {
        Rectangle rectangle;
        if (i < judge_state.getTrajectoryLength()) {
            rectangle.center_x_ = judge_state.getTrajectory()[judge_state.getChoosedTrajectoryIndex()][i].position_.x_;
            rectangle.center_y_ = judge_state.getTrajectory()[judge_state.getChoosedTrajectoryIndex()][i].position_.y_;
            rectangle.rotation_ = Tools::centerYawToRearYaw(judge_state.getTrajectory()[judge_state.getChoosedTrajectoryIndex()][i].theta_, judge_state.getTrajectory()[judge_state.getChoosedTrajectoryIndex()][i].kappa_, DISTANCE_FROM_REAR_TO_CENTER) ;
        } else {
            rectangle.center_x_ = judge_state.getExtendedTrajectory()[judge_state.getChoosedTrajectoryIndex()][i - judge_state.getTrajectoryLength()].position_.x_;
            rectangle.center_y_ = judge_state.getExtendedTrajectory()[judge_state.getChoosedTrajectoryIndex()][i - judge_state.getTrajectoryLength()].position_.y_;
            rectangle.rotation_ = Tools::centerYawToRearYaw(judge_state.getExtendedTrajectory()[judge_state.getChoosedTrajectoryIndex()][i - judge_state.getTrajectoryLength()].theta_, judge_state.getExtendedTrajectory()[judge_state.getChoosedTrajectoryIndex()][i - judge_state.getTrajectoryLength()].kappa_, DISTANCE_FROM_REAR_TO_CENTER);
        }
        rectangle.width_ = judge_state.getVehicleWidth() * width_expanded_factor;
        rectangle.length_ = judge_state.getVehicleLength() * length_expanded_factor;
        this->occupation_area_[i - start_index] = rectangle;
    }
    // 初始化采样占用区域
    this->samplingOccupationArea(sample_gap);
    // 设置默认优先级
    this->setPriority(false);
}

// 构造空气墙类型的占用区
DecisionMaking::RSS::OccupationArea::OccupationArea(const vec_map_cpp_msgs::VirtualObstacle &obstacle, int sample_gap) {
    if (obstacle.points.size() == 0) {
        return;
    }
    // 初始化矩形框
    for (size_t i = 0; i < obstacle.points.size() - 1; i++) {
        Rectangle rectangle;
        path_planning_msgs::PathPoint point1 = obstacle.points[i];
        path_planning_msgs::PathPoint point2 = obstacle.points[i + 1];
        rectangle.center_x_ = (point1.x + point2.x) / 2.0;
        rectangle.center_y_ = (point1.y + point2.y) / 2.0;
        rectangle.rotation_ = atan2(point2.y - point1.y, point2.x - point1.x);
        rectangle.width_ = VIRTUAL_OBSTACLE_WIDTH;
        rectangle.length_ = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
        this->occupation_area_.push_back(rectangle);
    }
    // 初始化采样占用区域
    this->samplingOccupationArea(sample_gap);
    // 设置默认优先级
    this->setPriority(false);
}

// 构造曲线类型的占用区
DecisionMaking::RSS::OccupationArea::OccupationArea(const PathPlanningUtilities::Curve &curve, double area_width, double area_length, int sample_gap) {
    // 初始化矩形框
    for (size_t i = 0; i < curve.size(); i++) {
        Rectangle rectangle;
        rectangle.center_x_ = curve[i].position_.x_;
        rectangle.center_y_ = curve[i].position_.y_;
        rectangle.rotation_ = Tools::centerYawToRearYaw(curve[i].theta_, curve[i].kappa_, DISTANCE_FROM_REAR_TO_CENTER);
        rectangle.width_ = area_width;
        rectangle.length_ = area_length;
        this->occupation_area_.push_back(rectangle);
    }
    // 初始化采样占用区域
    this->samplingOccupationArea(sample_gap);
    // 设置默认优先级
    this->setPriority(false);
}

// 采样占用区域
void DecisionMaking::RSS::OccupationArea::samplingOccupationArea(int sampling_gap) {
    // 得到采样后的占用区域和对应未采样时的下标
    for (size_t i = 0; i < this->occupation_area_.size(); i += sampling_gap) {
        if (i + sampling_gap >= this->occupation_area_.size() && i < this->occupation_area_.size() - 1) {
            this->sampled_occupation_area_.push_back(this->occupation_area_[i]);
            this->sampled_occupation_area_bijection_indexes_.push_back(i);
            i = this->occupation_area_.size() - 1;
            this->sampled_occupation_area_.push_back(this->occupation_area_[i]);
            this->sampled_occupation_area_bijection_indexes_.push_back(i);
        } else {
            this->sampled_occupation_area_.push_back(this->occupation_area_[i]);
            this->sampled_occupation_area_bijection_indexes_.push_back(i);
        }
    }
}

// 判断两个占用区域是否产生相交, true表示相交，false表示不相交
bool DecisionMaking::RSS::occupationInteractionJudgement(const OccupationArea &subvehicle_occupation_area, const OccupationArea &obstacle_occupation_area, size_t *subvehicle_interact_index, size_t *obstacle_interact_index) {
    // clock_t start_calc_time = clock();
    // 遍历占用区域的矩形框，判断其是否重叠
    for (size_t i = 0; i < subvehicle_occupation_area.getSampledOccupationArea().size(); i++) {
        for (size_t j = 0; j < obstacle_occupation_area.getSampledOccupationArea().size(); j++) {
            if (Tools::isRectangleOverlap(subvehicle_occupation_area.getSampledOccupationArea()[i], obstacle_occupation_area.getSampledOccupationArea()[j], RECTANGLE_INTERACTION_EXPAND_RATION, RECTANGLE_INTERACTION_EXPAND_RATION)) {
                // 此处得到的是采样后的下标，还要转化为采样前的下标
                // *subvehicle_interact_index = i;
                // *obstacle_interact_index = j;
                // 转化为采样前的下标
                *subvehicle_interact_index = subvehicle_occupation_area.getSampledOccupationAreaBijectionIndex(i);
                *obstacle_interact_index = obstacle_occupation_area.getSampledOccupationAreaBijectionIndex(j);
                // 如果发生碰撞，进行精细化处理，得到本车更加精确的相交点下标（TOFIX）
                if (i == 0) {
                    // 如果相交点就在车辆当前所在位置，不需要进行精细化
                } else {
                    // 如果相交点不在车辆当前所在位置
                    size_t subvehicle_last_interact_index = subvehicle_occupation_area.getSampledOccupationAreaBijectionIndex(i - 1);
                    for (size_t k = subvehicle_last_interact_index; k <= *subvehicle_interact_index; k++) {
                        if (Tools::isRectangleOverlap(subvehicle_occupation_area.getOccupationArea()[k], obstacle_occupation_area.getSampledOccupationArea()[j], RECTANGLE_INTERACTION_EXPAND_RATION, RECTANGLE_INTERACTION_EXPAND_RATION)) {
                            *subvehicle_interact_index = k;
                            break;
                        }
                    }
                }
                return true;
            }
        }
    }
    // clock_t end_calc_time = clock();
    // LOG(INFO) << "轨迹相交判定所花时间为" << static_cast<double>(end_calc_time - start_calc_time) * 1000.0 / CLOCKS_PER_SEC << "ms，计算次数为" << subvehicle_occupation_area.getSampledOccupationArea().size() * obstacle_occupation_area.getSampledOccupationArea().size() << "次";
    return false;
}

// 判断两个占用区域是否产生相交, true表示相交，false表示不相交
bool DecisionMaking::RSS::occupationInteractionJudgement(const OccupationArea &subvehicle_occupation_area, const OccupationArea &obstacle_occupation_area, int* subvehicle_interact_start_index, int* subvehicle_interact_end_index, int* obstacle_start_interact_index, int* obstacle_end_interact_index) {
    int cur_ego_vehicle_start_interact_index = -1;
    int cur_ego_vehicle_end_interact_index = -1;
    int cur_obstacle_start_interact_index = -1;
    int cur_obstacle_end_interact_index = -1;
    bool is_collision = false;
    // clock_t start_calc_time = clock();
    // 遍历占用区域的矩形框，判断其是否重叠
    for (size_t i = 0; i < subvehicle_occupation_area.getSampledOccupationArea().size(); i++) {
        for (size_t j = 0; j < obstacle_occupation_area.getSampledOccupationArea().size(); j++) {
            if (Tools::isRectangleOverlap(subvehicle_occupation_area.getSampledOccupationArea()[i], obstacle_occupation_area.getSampledOccupationArea()[j], RECTANGLE_INTERACTION_EXPAND_RATION, RECTANGLE_INTERACTION_EXPAND_RATION)) {
                // 此处得到的是采样后的下标，还要转化为采样前的下标
                // *subvehicle_interact_index = i;
                // *obstacle_interact_index = j;
                // 转化为采样前的下标
                int this_subvehicle_interact_index = subvehicle_occupation_area.getSampledOccupationAreaBijectionIndex(i);
                int this_obstacle_interact_index = obstacle_occupation_area.getSampledOccupationAreaBijectionIndex(j);

                if (cur_ego_vehicle_start_interact_index == -1) cur_ego_vehicle_start_interact_index = this_subvehicle_interact_index;
                if (cur_obstacle_start_interact_index == -1) cur_obstacle_start_interact_index = this_obstacle_interact_index;
                cur_ego_vehicle_end_interact_index = std::max(this_subvehicle_interact_index, cur_ego_vehicle_end_interact_index);
                cur_obstacle_end_interact_index = std::max(this_obstacle_interact_index, cur_obstacle_end_interact_index);
                is_collision = true;
            }
        }
    }
    // clock_t end_calc_time = clock();
    // LOG(INFO) << "轨迹相交判定所花时间为" << static_cast<double>(end_calc_time - start_calc_time) * 1000.0 / CLOCKS_PER_SEC << "ms，计算次数为" << subvehicle_occupation_area.getSampledOccupationArea().size() * obstacle_occupation_area.getSampledOccupationArea().size() << "次";
    *subvehicle_interact_start_index = cur_ego_vehicle_start_interact_index;
    *subvehicle_interact_end_index = cur_ego_vehicle_end_interact_index;
    *obstacle_start_interact_index = cur_obstacle_start_interact_index;
    *obstacle_end_interact_index = cur_obstacle_end_interact_index;
    return is_collision;
}

// 判断状态是否安全（已获得expected acceleration的状态）
bool DecisionMaking::RSS::stateSafetyJudgement(const StandardState &judge_state, const std::vector<Obstacle> &obstacles) {
    double test_acceleration = judge_state.getVehicleDynamicPlanningExpectedAcceleration();
    // 首先判断是否满足速度限制的需求
    {
        double current_velocity = judge_state.getVehicleCurrentMovement().velocity_;
        double velocity_limitation_max = judge_state.getVelocityLimitationMax();

        if (Tools::isLarge(current_velocity, velocity_limitation_max)) {
            // 如果当前速度大于限制速度，必须是处于减速状态，否则状态不安全
            if (Tools::isLarge(test_acceleration, 0.0)) {
                LOG(INFO) << "当前状态超速且加速，不安全";
                return false;
            }
        }
    }

    // 获取本车行驶的最高速度和最低速度
    double subvehicle_min_velocity = judge_state.getVelocityLimitationMin();
    // 最高速度由道路期望速度与状态速度限制共同决定
    double subvehicle_max_velocity = judge_state.getVelocityLimitationMax();
    LOG(INFO) << "本车当前速度" << judge_state.getVehicleCurrentMovement().velocity_ << "，最大速度可到" << subvehicle_max_velocity << "，最小速度可到" << subvehicle_min_velocity;

    // 最后判断与障碍物的相交情况
    for (size_t i = 0; i < obstacles.size(); i++) {
        Obstacle obstacle = obstacles[i];
        for (size_t lane_index = 0; lane_index < obstacle.getPredictedTrajectoryNumber(); lane_index++) {
            if (!Tools::isZero(obstacle.getObstacleVelocity())) {
                // 如果障碍物速度不为0
                OccupationArea subvehicle_occupation_area = OccupationArea(judge_state);
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index);
                size_t subvehicle_interact_index, obstacle_interact_index;
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    // 如果相交
                    // 保存为碰撞点
                    collision_rectangles.push_back(subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index]);
                    // 判断是否为逆向行驶
                    double yaw_gap = std::abs(obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].rotation_ - subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].rotation_);
                    if (Tools::isLarge(yaw_gap, 4.0 / 5.0 * PI)) {
                        LOG(INFO) << "逆向来车";
                        return false;
                    }
                    // 第一种情况当障碍物到达交点时，本车离交点还有安全距离
                    // 加速度为0单独考虑
                    if (Tools::isZero(test_acceleration)) {
                        if (Tools::isSmall(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_min_velocity) || Tools::isLarge(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                            // 当前车速大于最大车速或小于最小车速时，加速度不能为0
                            LOG(INFO) << "选中状态不满足当前车速大于最大车速或小于最小车速时，加速度不能为0";
                            return false;
                        } else {
                            double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity();
                            double safe_distance = Tools::getWaitSafeDistance(judge_state.getVehicleCurrentMovement().velocity_, obstacle.getObstacleVelocity());
                            LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车经过此时间走过的距离为" << obstacle_reach_interaction_time_consume * judge_state.getVehicleCurrentMovement().velocity_ << "米，离交点的距离还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - obstacle_reach_interaction_time_consume * judge_state.getVehicleCurrentMovement().velocity_ << "米，而安全距离为" << safe_distance << "米";
                            if (Tools::isSmall(obstacle_reach_interaction_time_consume * judge_state.getVehicleCurrentMovement().velocity_ + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(obstacle_reach_interaction_time_consume * judge_state.getVehicleCurrentMovement().velocity_ + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件0";
                                continue;
                            } else {
                                LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件0";
                            }
                        }
                    } else {
                        // 如果加速度不为0
                        // 计算障碍物到达交点所花时间
                        double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity();
                        // 在此时间内，本车进行匀加速/减速运动
                        // 判断本车速度与最高/最低速度的关系
                        if (Tools::isLarge(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                            // 如果当前车速大于最高车速，那没法进行加速，只能减速，减到最大速度
                            if (!Tools::isSmall(test_acceleration, 0.0)) {
                                // 当前车速大于最大车速，加速度必须小于0
                                LOG(INFO) << "当前车速大于最大车速，而选中状态加速度不小于0";
                                return false;
                            } else {
                                // 这段时间内经过加速度，速度变为了如下值
                                double subvehicle_interaction_velocity = std::max(judge_state.getVehicleCurrentMovement().velocity_ + obstacle_reach_interaction_time_consume * test_acceleration, subvehicle_min_velocity);
                                if (Tools::isLarge(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                    // 经过加速度后，速度大于最小速度
                                    // 计算本车走过的距离vt+0.5at^2
                                    double subvehicle_traveled_distance = judge_state.getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                    // 根据当前车速计算安全距离
                                    double safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << safe_distance << "米";
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    if (Tools::isSmall(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件1";
                                        continue;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件1";
                                    }
                                } else {
                                    // 经过加速度后，速度可以减到最小速度
                                    // 计算走过的距离
                                    double subvehicle_traveled_distance = subvehicle_min_velocity * obstacle_reach_interaction_time_consume - (judge_state.getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) * (judge_state.getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) / (2.0 * test_acceleration);
                                    // 计算安全距离
                                    double safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << safe_distance << "米";
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    if (Tools::isSmall(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件2";
                                        continue;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件2";
                                    }
                                }
                            }
                        } else if (Tools::isSmall(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_min_velocity)) {
                            // 如果当前车速小于最低车速，那没法进行减速，只能加速，加到最大速度
                            if (!Tools::isLarge(test_acceleration, 0.0)) {
                                // 当前车速小于最低车速，加速度必须大于0
                                LOG(INFO) << "当前车速小于最低车速，而选中状态的加速度不满足大于0";
                                return false;
                            } else {
                                // 计算经过此加速度后，本车的速度
                                double subvehicle_interaction_velocity = std::min(judge_state.getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_max_velocity);
                                if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                    // 经过此加速度，速度加到了最大速度
                                    // 计算走过的距离
                                    double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state.getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                    // 计算安全距离
                                    double safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << safe_distance << "米";
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    if (Tools::isSmall(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件3";
                                        continue;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件3";
                                    }
                                } else {
                                    // 经过此加速度，速度小于最大车速
                                    // 计算走过的距离
                                    double subvehicle_traveled_distance = judge_state.getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << safe_distance << "米";
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    if (Tools::isSmall(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件4";
                                        continue;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件4";
                                    }
                                }
                            }
                        } else {
                            // 如果当前车速小于最高车速，大于最低车速
                            // 计算经过加速度后，车辆的速度
                            double subvehicle_interaction_velocity = std::min(std::max(judge_state.getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_min_velocity), subvehicle_max_velocity);
                            if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                // 如果车辆加速到最大速度
                                // 计算经过的距离
                                double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state.getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                // 计算安全距离
                                double safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << safe_distance << "米";
                                // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                if (Tools::isSmall(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                    LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件5";
                                    continue;
                                } else {
                                    // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                    LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件5";
                                }
                            } else if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                // 如果车辆减速到最小速度
                                // 计算经过的距离
                                double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (judge_state.getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) * (judge_state.getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) / (2.0 * test_acceleration);
                                // 计算安全距离
                                double safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << safe_distance << "米";
                                // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                if (Tools::isSmall(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                    LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件6";
                                    continue;
                                } else {
                                    // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                    LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件6";
                                }
                            } else {
                                // 最终车速在最大最小之间
                                // 计算经过的距离
                                double subvehicle_traveled_distance = judge_state.getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                // 计算安全距离
                                double safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                if (Tools::isSmall(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                    LOG(INFO) << "满足障碍物到达交点时，本车离交点还有安全距离的条件7";
                                    continue;
                                } else {
                                    // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                    LOG(INFO) << "不满足障碍物到达交点时，本车离交点还有安全距离的条件7";
                                }
                            }
                        }
                    }

                    // 第二种情况当本车到达交点时，障碍物离交点还有安全距离
                    // 加速度为0单独考虑
                    if (Tools::isZero(test_acceleration)) {
                        if (Tools::isSmall(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_min_velocity) || Tools::isLarge(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                            // 当前车速大于最大车速或小于最小车速时，加速度不能为0
                            LOG(INFO) << "选中状态不满足当前车速大于最大车速或小于最小车速时，加速度不能为0";
                            return false;
                        } else {
                            double subvehicle_reach_interaction_time_consume = subvehicle_interact_index * LANE_GAP_DISTANCE / judge_state.getVehicleCurrentMovement().velocity_;
                            double safe_distance = Tools::getOvertakeSafeDistance(judge_state.getVehicleCurrentMovement().velocity_, obstacle.getObstacleVelocity());
                            if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                LOG(INFO) << "本车以匀速" << judge_state.getVehicleCurrentMovement().velocity_ << "到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                continue;
                            } else {
                                LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                return false;
                            }
                        }
                    } else {
                        if (Tools::isLarge(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                            // 当本车速度大于最高速度
                            if (!Tools::isSmall(test_acceleration, 0.0)) {
                                // 当前车速大于最大车速，加速度必须小于0
                                LOG(INFO) << "当前车速大于最大车速，而选中状态加速度不小于0";
                                return false;
                            } else {
                                // 计算当前速度是否可能减速到最低速度
                                if (Tools::isSmall((subvehicle_min_velocity * subvehicle_min_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 当前速度需要减速到最低速度
                                    // 计算到达交点的时间开销
                                    double subvehicle_reach_interaction_time_consume = (subvehicle_min_velocity - judge_state.getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_min_velocity * subvehicle_min_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_min_velocity;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = subvehicle_min_velocity;
                                    // 计算安全距离
                                    double safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                        continue;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出判断
                                        LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                        return false;
                                    }
                                } else {
                                    // 当前速度不需要减速到最低速度
                                    // 计算到达交点时的时间开销
                                    double subvehicle_reach_interaction_time_consume = (sqrt(judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state.getVehicleCurrentMovement().velocity_) / test_acceleration;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = judge_state.getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                        continue;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出判断
                                        LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                        return false;
                                    }
                                }
                            }
                        } else if (Tools::isSmall(judge_state.getVehicleCurrentMovement().velocity_, subvehicle_min_velocity)) {
                            // 当本车速度小于最低速度
                            if (!Tools::isLarge(test_acceleration, 0.0)) {
                                // 当前车速小于最低车速，加速度必须大于0
                                LOG(INFO) << "当前车速小于最低车速，而选中状态的加速度不满足大于0";
                                return false;
                            } else {
                                // 判断本车速度是否会加速到最大速度
                                if (Tools::isSmall((subvehicle_max_velocity * subvehicle_max_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 本车会加速到最大速度
                                    // 计算时间开销
                                    double subvehicle_reach_interaction_time_consume = (subvehicle_max_velocity - judge_state.getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_max_velocity * subvehicle_max_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_max_velocity;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = subvehicle_max_velocity;
                                    // 计算安全距离
                                    double safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                        continue;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出判断
                                        LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                        return false;
                                    }
                                } else {
                                    // 本车不会加速到最大速度
                                    // 计算时间开销
                                    double subvehicle_reach_interaction_time_consume = (sqrt(judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state.getVehicleCurrentMovement().velocity_) / test_acceleration;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = judge_state.getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                        continue;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出判断
                                        LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                        return false;
                                    }
                                }
                            }
                        } else {
                            // 当本车速度处于正常区间
                            // 判断本车是否会加速到最大速度，或减速到最小速度
                            if (Tools::isLarge(test_acceleration, 0.0) && Tools::isSmall((subvehicle_max_velocity * subvehicle_max_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                // 本车会加速到最大速度
                                // 计算时间开销
                                double subvehicle_reach_interaction_time_consume = (subvehicle_max_velocity - judge_state.getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_max_velocity * subvehicle_max_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_max_velocity;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = subvehicle_max_velocity;
                                // 计算安全距离
                                double safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                    // 障碍物离交点大于安全距离，此时安全
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                    continue;
                                } else {
                                    // 障碍物离交点小于安全距离，退出判断
                                    LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                    return false;
                                }
                            } else if (Tools::isSmall(test_acceleration, 0.0) && Tools::isSmall((subvehicle_min_velocity * subvehicle_min_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                // 本车会减速到最小速度
                                // 计算到达交点的时间开销
                                double subvehicle_reach_interaction_time_consume = (subvehicle_min_velocity - judge_state.getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_min_velocity * subvehicle_min_velocity - judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_min_velocity;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = subvehicle_min_velocity;
                                // 计算安全距离
                                double safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                    // 障碍物离交点大于安全距离，此时安全
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                    continue;
                                } else {
                                    // 障碍物离交点小于安全距离，退出判断
                                    LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                    return false;
                                }
                            } else {
                                // 本车既不会加速到最大速度，也不会减速到最小速度
                                // 计算时间开销
                                double subvehicle_reach_interaction_time_consume = (sqrt(judge_state.getVehicleCurrentMovement().velocity_ * judge_state.getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state.getVehicleCurrentMovement().velocity_) / test_acceleration;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = judge_state.getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                // 计算安全距离
                                double safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                    // 障碍物离交点大于安全距离，此时安全
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << safe_distance << "米";
                                    continue;
                                } else {
                                    // 障碍物离交点小于安全距离，退出判断
                                    LOG(INFO) << "不满足当本车到达交点时，障碍物离交点还有安全距离的条件";
                                    return false;
                                }
                            }
                        }
                    }
                } else {
                    // 如果不相交，继续下一次判断
                    continue;
                }
            } else {
                // 如果障碍物速度为0
                OccupationArea subvehicle_occupation_area = OccupationArea(judge_state);
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE);
                size_t subvehicle_interact_index, obstacle_interact_index;
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    // 如果相交，返回false
                    std::cout << "选中状态与静止障碍物相交" << std::endl;
                    LOG(INFO) << "选中状态与静止障碍物相交";
                    return false;
                } else {
                    // 如果不相交，继续下一次判断
                    continue;
                }
            }
        }
    }
    std::cout << "当前选中状态经过测试为安全" << std::endl;
    LOG(INFO) << "当前选中状态经过测试为安全";
    return true;
}

// 判断状态的路径是否违反交通规则
void DecisionMaking::RSS::trafficRuleCheck(StandardState *judge_state, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &obstacles) {
    // 首先判断状态是否可行
    if (judge_state->getCapability()) {
        // 得到本状态的行驶路线的占用区域
        OccupationArea subvehicle_occupation_area = OccupationArea(*judge_state, SUBVEHICLE_OCCUPANCY_SAMPLING_GAP_FOR_TRAFFIC_CHECK, SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_TRAFFIC_CHECK, SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_TRAFFIC_CHECK);
        // 遍历交通规则生成的空气墙障碍物
        for (size_t i = 0; i < obstacles.size(); i++) {
            // 得到障碍物墙的线体的占用区域
            OccupationArea traffic_rule_obstacle_occupation_area = OccupationArea(obstacles[i]);
            size_t subvehicle_interact_index, obstacle_interact_index;
            // 判断两占用区域是否相交
            if (occupationInteractionJudgement(subvehicle_occupation_area, traffic_rule_obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                // 在状态中记录此空气墙为有效空气墙
                if (subvehicle_interact_index > 0) {
                    // 如果相交，状态不安全
                    judge_state->setSafety(false);
                    LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "与空气墙体相交，违法交通规则，不安全";
                    return;
                } else {
                    // 如果已经碰撞判断是不是红绿灯，如果是还是要等
                    if (obstacles[i].mode == vec_map_cpp_msgs::VirtualObstacle::CONDITION_DECISION) {
                        judge_state->setSafety(false);
                        LOG(INFO) << "处于红灯上";
                        return;
                    }
                }

            } else {
                // nothing
            }
        }
    }
}

// 得到交通规则与路径碰撞的最近碰撞点在路径中的下标，返回值是true表示发生碰撞，false表示未发生碰撞
bool DecisionMaking::RSS::collisionPositionIndexInCurve(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_rule_obstacles, size_t *cut_index) {
    // 初始化返回值
    *cut_index = judge_curve.size();
    bool collision_result = false;
    // 首先得到当前状态的占用区域，由于不是一个状态，占用区域是通过曲线生成的
    OccupationArea subvehicle_occupation_area = OccupationArea(judge_curve, vehicle_width, vehicle_length);
    // 遍历交通障碍物，生成占用区
    for (size_t i = 0; i < traffic_rule_obstacles.size(); i++) {
        OccupationArea traffic_rule_obstacle_occupation_area = OccupationArea(traffic_rule_obstacles[i]);
        size_t subvehicle_interact_index, obstacle_interact_index;
        // 判断两占用区域是否相交
        if (occupationInteractionJudgement(subvehicle_occupation_area, traffic_rule_obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
            if (subvehicle_interact_index > 0) {
                // 如果这一次碰撞点比上一次更近，对碰撞点进行更新
                // 由于交通规则中都是固定空气墙，不是未来轨迹，就算碰撞点在车辆当前所在位置，也是发生碰撞的
                if (subvehicle_interact_index < *cut_index) {
                    *cut_index = subvehicle_interact_index;
                    collision_result = true;
                }
            } else {
                // 如果已经碰撞判断是不是红绿灯，如果是还是要等
                if (traffic_rule_obstacles[i].mode == vec_map_cpp_msgs::VirtualObstacle::CONDITION_DECISION) {
                    LOG(INFO) << "current is in traffic condition obstacle";
                    *cut_index = subvehicle_interact_index;
                    collision_result = true;
                }
            }
        }
    }
    return collision_result;
}

// 得到障碍物与路径碰撞的最近碰撞点在路径中的下标，返回值是true表示发生碰撞，false表示未发生碰撞
bool DecisionMaking::RSS::collisionPositionIndexInCurve(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, double vehicle_velocity, const std::vector<Obstacle> &obstacles, size_t *cut_index) {
    // 初始化返回值
    *cut_index = judge_curve.size();
    bool collision_result = false;
    // 首先得到当前状态的占用区域，由于不是一个状态，占用区域是通过曲线生成的
    OccupationArea subvehicle_occupation_area = OccupationArea(judge_curve, vehicle_width * SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_OBSTACLE_CHECK, vehicle_length * SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_OBSTACLE_CHECK);
    // 遍历障碍物
    for (size_t i = 0; i < obstacles.size(); i++) {
        Obstacle obstacle = obstacles[i];
        // 遍历障碍物的每一个预测路径
        for (size_t lane_index = 0; lane_index < obstacle.getPredictedTrajectoryNumber(); lane_index++) {
            // 判断是静止障碍物还是动态障碍物
            if (Tools::isZero(obstacle.getObstacleVelocity())) {
                // 如果是静止障碍物
                // 构造障碍物占用区
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index, vehicle_velocity, 1);
                size_t subvehicle_interact_index, obstacle_interact_index;
                // 判断两占用区域是否相交
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    // 如果相交，且这一次碰撞点比当前更近，对碰撞点进行更新
                    // 由于是静态障碍物，不是未来轨迹，就算碰撞点在车辆当前所在位置，也是发生碰撞的
                    // if (GLOBAL_IS_IN_JIFEI_) {
                    //     if (subvehicle_interact_index == 0) {
                    //         continue;
                    //     }
                    // }
                    if (subvehicle_interact_index < *cut_index) {
                        *cut_index = subvehicle_interact_index;
                        collision_result = true;
                    }
                }
            } else {
            }
        }
    }
    return collision_result;
}

// 得到障碍物与路径碰撞的最近碰撞点在路径中的下标(带距离判断)，返回值是true表示发生碰撞，false表示未发生碰撞
bool DecisionMaking::RSS::collisionPositionIndexInCurve(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, double vehicle_velocity, const std::vector<Obstacle> &obstacles, size_t *cut_index, bool is_big) {
    // 初始化返回值
    *cut_index = judge_curve.size();
    bool collision_result = false;
    // 首先得到当前状态的占用区域，由于不是一个状态，占用区域是通过曲线生成的
    OccupationArea subvehicle_occupation_area = OccupationArea(judge_curve, vehicle_width * SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_OBSTACLE_CHECK, vehicle_length * SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_OBSTACLE_CHECK);
    // 遍历障碍物
    for (size_t i = 0; i < obstacles.size(); i++) {
        Obstacle obstacle = obstacles[i];
        // 遍历障碍物的每一个预测路径
        for (size_t lane_index = 0; lane_index < obstacle.getPredictedTrajectoryNumber(); lane_index++) {
            // 判断是静止障碍物还是动态障碍物
            if (Tools::isZero(obstacle.getObstacleVelocity())) {
                // 如果是静止障碍物
                // 构造障碍物占用区
                if (is_big) {
                    // 需要大膨胀障碍物,修改障碍物长和宽
                    obstacle.setObstacleShape(obstacle.getObstacleWidth() + BIGGER_OBSTACLE_EXTRA_EXPAND_CONST, obstacle.getObstacleLength() + BIGGER_OBSTACLE_EXTRA_EXPAND_CONST);
                }
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index, vehicle_velocity, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE);
                size_t subvehicle_interact_index, obstacle_interact_index;
                // 判断两占用区域是否相交
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    // 如果相交，且这一次碰撞点比当前更近，对碰撞点进行更新
                    // 由于是静态障碍物，不是未来轨迹，就算碰撞点在车辆当前所在位置，也是发生碰撞的
                    // if (GLOBAL_IS_IN_JIFEI_) {
                    //     if (subvehicle_interact_index == 0) {
                    //         continue;
                    //     }
                    // }
                    if (subvehicle_interact_index < *cut_index) {
                        *cut_index = subvehicle_interact_index;
                        collision_result = true;
                    }
                } else {
                    // 如果不碰撞
                }
            }
        }
    }
    return collision_result;
}

// 得到动态障碍物与路径碰撞的最近碰撞点在路径中的下标，返回值是true表示发生碰撞，false表示未发生碰撞
bool DecisionMaking::RSS::collisionWithDynamicObstacles(const PathPlanningUtilities::Curve &judge_curve, double velocity,double vehicle_width, double vehicle_length, const std::vector<Obstacle> &obstacles, size_t *cut_index) {
    // 初始化返回值
    *cut_index = judge_curve.size();
    bool collision_result = false;
    // 首先得到当前状态的占用区域，由于不是一个状态，占用区域是通过曲线生成的
    OccupationArea subvehicle_occupation_area = OccupationArea(judge_curve, vehicle_width * SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_OBSTACLE_CHECK, vehicle_length * SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_OBSTACLE_CHECK);
    // 遍历障碍物
    for (size_t i = 0; i < obstacles.size(); i++) {
        Obstacle obstacle = obstacles[i];
        // 遍历障碍物的每一个预测路径
        for (size_t lane_index = 0; lane_index < obstacle.getPredictedTrajectoryNumber(); lane_index++) {
            // 判断是静止障碍物还是动态障碍物
            if (Tools::isZero(obstacle.getObstacleVelocity())) {
                // 如果是静止障碍物，不进行处理
            } else {
                // 如果是运动障碍物
                // 构造障碍物占用区
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index, velocity, DYNAMIC_OBSTACLE_COLLISION_CHECK_SAMPLING_GAP);
                size_t subvehicle_interact_index, obstacle_interact_index;
                // 判断两占用区域是否相交
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    // 发生碰撞时，有两种情况可以不用考虑障碍物，一是本车先到碰撞点且障碍物还有风险距离，二是障碍物到碰撞点时本车还有风险距离
                    bool is_safe = false;
                    // 碰撞点是车所在位置为特殊情况
                    if (subvehicle_interact_index == 0 && obstacle_interact_index != 0) {
                        is_safe = true;
                    }
                    // 第一种情况
                    {
                        // 本车到达碰撞点所花时间
                        double traveled_time = static_cast<double>(subvehicle_interact_index) * LANE_GAP_DISTANCE / velocity;
                        // 风险距离
                        double safe_distance = Tools::getOvertakeSafeDistance(velocity, obstacle.getObstacleVelocity());
                        if (!Tools::isSmall(static_cast<double>(obstacle_interact_index) * OBSTACLE_MARGIN, traveled_time * obstacle.getObstacleVelocity() + safe_distance)) {
                            is_safe = true;                        
                        }
                    }
                    // 第二种情况
                    {
                        // 障碍物到达碰撞点所花时间
                        double traveled_time = static_cast<double>(obstacle_interact_index) * OBSTACLE_MARGIN / obstacle.getObstacleVelocity();
                        // 风险距离
                        double safe_distance = Tools::getWaitSafeDistance(velocity, obstacle.getObstacleVelocity());
                        if (!Tools::isSmall(static_cast<double>(subvehicle_interact_index) * LANE_GAP_DISTANCE, velocity * traveled_time + safe_distance)) {
                            is_safe = true;
                        }
                    }

                    if (!is_safe) {
                        if (subvehicle_interact_index < *cut_index) {
                            *cut_index = subvehicle_interact_index;
                            collision_result = true;
                        }
                    } else {
                        // 判断是否为逆向来车
                        double yaw_gap = std::abs(Tools::safeThetaTransform(obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].rotation_ - subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].rotation_));
                        if (Tools::isLarge(yaw_gap, 4.0 / 5.0 * PI)) {
                            LOG(INFO) << "逆向来车";
                            *cut_index = subvehicle_interact_index;
                            collision_result = true;
                        } else {
                            LOG(INFO) << "障碍物离碰撞点较远，无需考虑";
                        }
                    }
                }
            }
        }
    }
    return collision_result;
}

// 判断蔽障道路是否可以快速通过
bool DecisionMaking::RSS::isAvoidanceQuickPass(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, const std::vector<Obstacle> &obstacles) {
    OccupationArea subvehicle_occupation_area = OccupationArea(judge_curve, vehicle_width * SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_OBSTACLE_CHECK, vehicle_length * SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_OBSTACLE_CHECK);
    for (size_t i = 0; i < obstacles.size(); i++) {
        Obstacle obstacle = obstacles[i];
        // 遍历障碍物的每一个预测路径
        for (size_t lane_index = 0; lane_index < obstacle.getPredictedTrajectoryNumber(); lane_index++) {
            // 判断是静止障碍物还是动态障碍物
            if (Tools::isZero(obstacle.getObstacleVelocity())) {
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE, true);
                size_t subvehicle_interact_index, obstacle_interact_index;
                // 判断两占用区域是否相交
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    return false;
                }
            }
        }
    }
    return true;
}

// 获取障碍物到路径的距离
double DecisionMaking::RSS::getObstacleDistanceToCurve(const PathPlanningUtilities::Curve &curve, const Obstacle &obstacle) {
    double distance = OBSTACLE_TO_PATH_MAX_DISTANCE;
    for (size_t j = 0; j < curve.size(); j++) {
        // 进行坐标转化
        PathPlanningUtilities::Point2f local_position = Tools::calcNewCoordinationPosition(curve[j], obstacle.getObstaclePosition());
        if (Tools::isSmall(local_position.x_,0.0) && j > 0) {
            // 找到了对应的点
            distance = std::fabs(local_position.y_);
            return distance;
        }
    }
    return distance;
}

// 计算两个占用区域之间的距离
void DecisionMaking::RSS::occupationDistanceCalculation(const OccupationArea &subvehicle_occupation_area, const OccupationArea &obstacle_occupation_area, double distance_upper, std::vector<double>* result) {
    for (size_t i = 0; i < subvehicle_occupation_area.getSampledOccupationArea().size(); i++) {
        double min_distance = distance_upper;
        size_t subvehicle_true_index = subvehicle_occupation_area.getSampledOccupationAreaBijectionIndex(i);
        for (size_t j = 0; j < obstacle_occupation_area.getSampledOccupationArea().size(); j++) {
            double distance = Tools::rectangleShortestDistanceWithUpper(subvehicle_occupation_area.getSampledOccupationArea()[i], obstacle_occupation_area.getSampledOccupationArea()[j], distance_upper);
            if (Tools::isSmall(distance, min_distance)) {
                min_distance = distance;
            }
            // 如果最小距离是0,要精确化碰撞点
            if (Tools::isZero(min_distance)) {
                // 首先记录当前点距离为0.0
                assert(subvehicle_true_index < (*result).size());
                (*result)[subvehicle_true_index] = min_distance;
                // 进行精细化
                if (i == 0) {
                    // 如果相交点就在车辆当前所在位置，不需要进行精细化
                    goto record;
                } else {
                    // 如果相交点不在车辆当前所在位置
                    size_t subvehicle_last_interact_index = subvehicle_occupation_area.getSampledOccupationAreaBijectionIndex(i - 1);
                    for (size_t k = subvehicle_last_interact_index; k <= subvehicle_true_index; k++) {
                        if (Tools::isZero(Tools::rectangleShortestDistance(subvehicle_occupation_area.getOccupationArea()[k], obstacle_occupation_area.getSampledOccupationArea()[j]))) {
                            subvehicle_true_index = k;
                            goto record;
                        }
                    }
                }
            }
        }
        record:
        // 更新距离保存结果
        assert(subvehicle_true_index < (*result).size());
        if (Tools::isLarge((*result)[subvehicle_true_index], min_distance)) {
            (*result)[subvehicle_true_index] = min_distance;
        }
    }
}

// 计算障碍物与路径之间的距离
void DecisionMaking::RSS::obstacleDistanceJudgment(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, double vehicle_velocity, const std::vector<Obstacle> &obstacles, std::vector<double>* result) {
    // 首先得到当前状态的占用区域，由于不是一个状态，占用区域是通过曲线生成的
    OccupationArea subvehicle_occupation_area = OccupationArea(judge_curve, SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_OBSTACLE_CHECK * vehicle_width, SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_OBSTACLE_CHECK * vehicle_length, 1);
    // 初始化障碍物与路径距离上限
    double distance_upper = MAX_DISTANCE_RATIO * vehicle_width;
    // 初始化result
    (*result) = std::vector<double>(judge_curve.size(), distance_upper);
    // 遍历障碍物
    for (size_t i = 0; i < obstacles.size(); i++) {
        Obstacle obstacle = obstacles[i];
        // 遍历障碍物的每一个预测路径
        for (size_t lane_index = 0; lane_index < obstacle.getPredictedTrajectoryNumber(); lane_index++) {
            // 判断是静止障碍物还是动态障碍物
            if (Tools::isZero(obstacle.getObstacleVelocity())) {
                // 如果是静止障碍物
                // 构造障碍物占用区
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index, vehicle_velocity, 1);
                // 计算两个占用区域的距离
                occupationDistanceCalculation(subvehicle_occupation_area, obstacle_occupation_area, distance_upper, result);
            }
        }
    }
}

// 计算交通障碍物与路径之间的距离
void DecisionMaking::RSS::obstacleDistanceJudgment(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_rule_obstacles, std::vector<double>* result) {
    // 首先得到当前状态的占用区域，由于不是一个状态，占用区域是通过曲线生成的
    OccupationArea subvehicle_occupation_area = OccupationArea(judge_curve, EXPAND_RATIO * vehicle_width, EXPAND_RATIO * vehicle_length, 1);
    // 初始化障碍物与路径距离上限
    double distance_upper = MAX_DISTANCE_RATIO * vehicle_width;
    // 初始化result
    (*result) = std::vector<double>(judge_curve.size(), distance_upper);
    // 遍历交通障碍物，生成占用区
    for (size_t i = 0; i < traffic_rule_obstacles.size(); i++) {
        OccupationArea traffic_rule_obstacle_occupation_area = OccupationArea(traffic_rule_obstacles[i]);
        // 计算两个占用区域的距离
        occupationDistanceCalculation(subvehicle_occupation_area, traffic_rule_obstacle_occupation_area, distance_upper, result);
    }
}

// 判断道路是否被障碍物所占据
void DecisionMaking::SubVehicle::judgeStateCorrespondingLaneBeingOccupied(StandardState *judge_state, const std::vector<Obstacle> &obstacles) {
    if (!judge_state->getCapability()) {
        return;
    }
    // 初始化结果
    bool is_occupied = false;
    // 得到判断的起点和终点
    size_t start_index, end_index;
    // 得到车辆当前定位
    PathPlanningUtilities::CurvePoint current_position = judge_state->getVehicleCurrentPosition();
    start_index = Tools::findNearestPositionIndexInCoordination(judge_state->getRespondingLane().getLaneCoordnation(), current_position.position_);
    end_index = judge_state->getRespondingLane().getLaneCoordnation().size();
    
    // 遍历障碍物
    for (auto obstacle: obstacles) {
        for (auto predict_path: obstacle.getPredictedTrajectorySet()) {
            if (judge_state->getRespondingLane().judgePoint2fInLane(predict_path.back().position_, start_index, end_index, 3.0)) {
                // 被占据
                is_occupied = true;
                break;
            }
        }
        if (is_occupied) {
            break;
        }
    }
    if (is_occupied) {
        LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "被占据";
    } else {
        LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "不被占据";
    }
    judge_state->setLaneBeingOccupiedByObstacle(is_occupied);
    return;
}
