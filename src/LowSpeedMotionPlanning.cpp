#include "Common.hpp"

// 生成低速机动状态（包括避障和停车）
void DecisionMaking::SubVehicle::generateLowVelocityState() {
    // 只有在三大状态都不可行时，才可能进入低速机动状态
    // 低速机动状态包括了避障和停车两种状态
    // 如果当前速度为高速状态，则进行刹车，进入低速状态
    // 如果当前速度为低速状态，则进行避障
    // 如果避障路径均不可行，进入停车状态

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
    // 得到当前曲率
    this->current_vehicle_kappa_mutex_.lock();
    double current_point_kappa = this->current_vehicle_kappa_;
    current_point_in_world.kappa_ = current_point_kappa;
    this->current_vehicle_kappa_mutex_.unlock();
    // 得到当前障碍物情况
    this->obstacle_mutex_.lock();
    std::vector<Obstacle> obstacles = this->obstacles_;
    this->obstacle_mutex_.unlock();
    //

    // 1. 第一步，判断当前速度是高速还是低速状态(低速状态为道路限速的一半以下)
    // 得到速度限制
    double lane_velocity_expectation = this->states_set_[StateNames::FORWARD].getExpectedVelocityCurrent();
    double velocity_limitation = std::min(VELOCITY_THRESHOLD, lane_velocity_expectation);
    LOG(INFO) << "当前限速为" << lane_velocity_expectation;
    // 判断当前是否处于低速状态
    if (Tools::isLarge(current_movement_state.velocity_, VELOCITY_THRESHOLD)) {
        // 如果当前速度大于限速，此时为高速状态
        LOG(INFO) << "当前处于高速状态，需要先减速进入低速状态再进行避障";
        std::cout << "当前处于高速状态，需要先减速进入低速状态再进行避障" << std::endl;
        // 判断上一次状态是否完成
        PathPlanningUtilities::Curve uncompleted_total_curve, uncompleted_curve, uncompleted_extended_curve;
        bool is_uncompleted_curve_collision = false;
        bool is_uncompleted_curve_traffic_influence = false;
        size_t uncompleted_curve_cut_index = 0;
        if (this->current_state_.getStateCompleted() == false && this->current_state_.getVehicleCurrentPositionIndexInTrajectory() < this->current_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory()) {
            // 如果上一次状态未完成
            LOG(INFO) << "上一次状态未能完成就开始重新规划";
            std::cout << "上一次状态未能完成就开始重新规划" << std::endl;
            // 得到上一次规划未能走完的路径
            PathPlanningUtilities::Curve total_curve, unfinished_total_curve, unfinished_curve, unfinished_extended_curve;
            total_curve.assign(this->current_state_.getTrajectory()[this->current_state_.getChoosedTrajectoryIndex()].begin(), this->current_state_.getTrajectory()[this->current_state_.getChoosedTrajectoryIndex()].end());
            assert(this->choosed_state_.getExtendedTrajectory().size() > 0 && this->choosed_state_.getExtendedTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].size() > 0);
            total_curve.insert(total_curve.end(), this->current_state_.getExtendedTrajectory()[this->current_state_.getChoosedTrajectoryIndex()].begin(), this->current_state_.getExtendedTrajectory()[this->current_state_.getChoosedTrajectoryIndex()].end());

            unfinished_curve.assign(total_curve.begin() + this->choosed_state_.getVehicleCurrentPositionIndexInTrajectory(), total_curve.begin() + this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory());

            unfinished_extended_curve.assign(total_curve.begin() + this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory(), total_curve.end());

            unfinished_total_curve.assign(unfinished_curve.begin(), unfinished_curve.end());
            unfinished_total_curve.insert(unfinished_total_curve.end(), unfinished_extended_curve.begin(), unfinished_extended_curve.end());
            // 计算未完成状态的碰撞
            // 碰撞的路径截断点
            size_t cut_index = unfinished_total_curve.size() - 1;
            // 碰撞发生标志位
            bool is_collision_occur = false;
            bool is_traffic_influence;
            // 进行交通规则碰撞判定
            size_t traffic_rule_cut_index;
            if (RSS::collisionPositionIndexInCurve(unfinished_total_curve, this->vehicle_width_, this->vehicle_length_, this->traffic_rule_obstacles_, &traffic_rule_cut_index)) {
                // 如果发生碰撞，记录碰撞点
                if (traffic_rule_cut_index <= cut_index) {
                    cut_index = traffic_rule_cut_index;
                    is_traffic_influence = true;
                }
                is_collision_occur = true;
                LOG(INFO) << "空气墙障碍物发生碰撞";
            } else {
                // 如果没有发生碰撞
                // do nothing
            }
            // 进行障碍物碰撞判定
            size_t obstacle_cut_index;
            if (RSS::collisionPositionIndexInCurve(unfinished_total_curve, this->vehicle_width_, this->vehicle_length_, current_movement_state.velocity_, obstacles, &obstacle_cut_index)) {
                // 如果发生碰撞，记录碰撞点
                if (obstacle_cut_index <= cut_index) {
                    cut_index = obstacle_cut_index;
                    is_traffic_influence = false;
                }
                is_collision_occur = true;
                LOG(INFO) << "静态障碍物发生碰撞";
            } else {
                // 如果没有发生碰撞
                // do nothing
            }
            // 进行动态障碍物修正
            size_t dynamic_obstacle_block_index;
            if (RSS::collisionWithDynamicObstacles(unfinished_total_curve, current_movement_state.velocity_, this->vehicle_width_, this->vehicle_length_, obstacles, &dynamic_obstacle_block_index)) {
                // 如果发生碰撞，修正碰撞点
                if (dynamic_obstacle_block_index <= cut_index) {
                    cut_index = dynamic_obstacle_block_index;
                    is_traffic_influence = false;
                }
                is_collision_occur = true;
                LOG(INFO) << "动态障碍物发生碰撞";
            }

            // 记录上一个状态的信息
            uncompleted_total_curve = unfinished_total_curve;
            uncompleted_curve = unfinished_curve;
            uncompleted_extended_curve = unfinished_extended_curve;
            uncompleted_curve_cut_index = unfinished_total_curve.size() - 1;
            is_uncompleted_curve_collision = is_collision_occur;
            if (is_collision_occur) {
                uncompleted_curve_cut_index = cut_index;
                is_uncompleted_curve_traffic_influence = is_traffic_influence;
            }
            LOG(INFO) << "旧路径长度" << uncompleted_curve.size() << "，旧路径延伸段长度" << uncompleted_extended_curve.size();
            LOG(INFO) << "旧路径高速到低速切换路径发生碰撞的结果为 " << is_uncompleted_curve_collision << "，碰撞点在路径上的第" << uncompleted_curve_cut_index;
            std::cout << "旧路径高速到低速切换路径发生碰撞的结果为 " << is_uncompleted_curve_collision << "，碰撞点在路径上的第" << uncompleted_curve_cut_index << std::endl;
        } else {
            // 如果上一次状态已经完成
            // do nothing
            LOG(INFO) << "上一次状态已经完成才开始重新规划";
            std::cout << "上一次状态已经完成才开始重新规划" << std::endl;
            this->current_state_.setStateCompleted(true);
        }

        // 开始规划新的路径
        // 加载地图信息
        assert(this->center_lane_.getLaneExistance());
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = this->center_lane_.getLaneCoordnation();
        std::vector<TransMatrix> trans_matrixes = this->center_lane_.getLaneTransMatrix();
        // 得到frenet坐标系起点
        size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, current_point_in_world.position_);
        TransMatrix trans_matrix = trans_matrixes[start_index_of_lane];
        Eigen::Vector2d current_point_in_world_v2d(current_point_in_world.position_.x_, current_point_in_world.position_.y_);
        Eigen::Vector2d start_point_position_in_frenet;
        Tools::transferPointCoordinateSystem(trans_matrix, current_point_in_world_v2d, &start_point_position_in_frenet);
        double start_point_theta_in_frenet;
        Tools::transferThetaCoordinateSystem(trans_matrix, current_point_in_world.theta_, &start_point_theta_in_frenet);
        double start_point_kappa_in_frenet;
        start_point_kappa_in_frenet = current_point_kappa - cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*1.0/(1.0/lane_coordination[start_index_of_lane].worldpos_.kappa_ - start_point_position_in_frenet(1));
        PathPlanningUtilities::VehicleState start_point_in_frenet;
        start_point_in_frenet.position_.x_ = start_point_position_in_frenet(0);
        start_point_in_frenet.position_.y_ = start_point_position_in_frenet(1);
        start_point_in_frenet.theta_ = start_point_theta_in_frenet;
        start_point_in_frenet.kappa_ = start_point_kappa_in_frenet;
        // 得到目标点的d轴偏移
        double goal_point_y_in_frenet = start_point_in_frenet.position_.y_ + sin(start_point_in_frenet.theta_) * 0.5 * this->vehicle_length_ + 0.5 * this->vehicle_width_ * (cos(start_point_in_frenet.theta_) - 1);
        // 如果偏移量较小就不管了
        if (Tools::isSmall(goal_point_y_in_frenet, MIN_LATERAL_OFFSET_TO_BE_CONSIDERED)) {
            goal_point_y_in_frenet = 0.0;
        }
        LOG(INFO) << "蔽障路径横向偏移为" << goal_point_y_in_frenet;
        // 得到frenet坐标系下的终点组
        double motion_planning_length = Tools::normalMotionDistance(current_movement_state.velocity_, SUBVEHICLE_COMMON_DECCELERATION, CONSTANT_DISTANCE);
        size_t motion_planning_index_length = static_cast<int>(motion_planning_length / LANE_GAP_DISTANCE);
        size_t goal_index_of_lane = start_index_of_lane + motion_planning_index_length;
        // 从起点到终点每间隔固定长度进行采样
        size_t sample_length = static_cast<size_t>(SAMPLE_LENGTH / LANE_GAP_DISTANCE);
        size_t sampled_goal_index_of_lane = start_index_of_lane + static_cast<size_t>(std::max(static_cast<int>(motion_planning_index_length * LONGITUDE_SAMPLING_RATIO_FOR_LOW_SPEED_STATE) - static_cast<int>(MIN_LONGITUDE_PLANNING_LENGTH_FOR_LOW_SPEED_STATE / LANE_GAP_DISTANCE), static_cast<int>(0)));
        assert(sampled_goal_index_of_lane + sample_length <= goal_index_of_lane);
        // 3. 开始规划本状态的路径组
        std::vector<PathPlanningUtilities::Curve> curves;
        std::vector<PathPlanningUtilities::Curve> frenet_curves;
        std::vector<PathPlanningUtilities::Curve> extended_curves;
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
            goal_point_in_frenet.position_.x_ = this->center_lane_.getLaneCenterPathInFrenet()[sampled_goal_index_of_lane].x_;
            goal_point_in_frenet.position_.y_ = goal_point_y_in_frenet;
            goal_point_in_frenet.theta_ = 0.0;
            goal_point_in_frenet.kappa_ = 0.0;
            // 用quintic polynomial方法进行路径生成(distance 是调整参数 TODO)
            PathPlanningUtilities::Curve trajectory_curve_in_frenet;
            PathPlanningUtilities::QParameters params;
            double distance = PathPlanningUtilities::calcDistance(start_point_in_frenet.position_, goal_point_in_frenet.position_);
            PathPlanningUtilities::QuinticSplinePlanning::getParametersByMileageConstraint(start_point_in_frenet, goal_point_in_frenet, distance, 5, params);
            int point_number = sampled_goal_index_of_lane - start_index_of_lane + 1;;
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
                PathPlanningUtilities::CurvePoint point_in_world;
                PathPlanningUtilities::Point2f point_in_frenet;
                point_in_frenet.x_ = lane_coordination[i].station_;
                point_in_frenet.y_ = goal_point_y_in_frenet;
                Eigen::Vector2d vector_frenet(point_in_frenet.x_, point_in_frenet.y_);
                Eigen::Vector2d vector_world;
                vector_world = trans_matrixes[i].rotation_.inverse() * (vector_frenet - trans_matrixes[i].trans_);
                point_in_world.position_.x_ = vector_world(0);
                point_in_world.position_.y_ = vector_world(1);
                point_in_world.theta_ = lane_coordination[i].worldpos_.theta_;
                point_in_world.kappa_ = lane_coordination[i].worldpos_.kappa_;
                trajectory_curve_in_world.push_back(point_in_world);
            }
            // 进行赋值
            curves.push_back(trajectory_curve_in_world);
            frenet_curves.push_back(trajectory_curve_in_frenet);
        }
        // 生成延伸路径
        PathPlanningUtilities::Curve extended_trajectory_curve_in_world;
        for (size_t i = goal_index_of_lane + 1; i < lane_coordination.size(); i++) {
            PathPlanningUtilities::Point2f point_in_frenet;
            point_in_frenet.x_ = lane_coordination[i].station_;
            point_in_frenet.y_ = goal_point_y_in_frenet;
            PathPlanningUtilities::CurvePoint point_in_world;
            Eigen::Vector2d vector_frenet(point_in_frenet.x_, point_in_frenet.y_);
            Eigen::Vector2d vector_world;
            vector_world = trans_matrixes[i].rotation_.inverse() * (vector_frenet - trans_matrixes[i].trans_);
            point_in_world.position_.x_ = vector_world(0);
            point_in_world.position_.y_ = vector_world(1);
            point_in_world.theta_ = lane_coordination[i].worldpos_.theta_;
            point_in_world.kappa_ = lane_coordination[i].worldpos_.kappa_;
            extended_trajectory_curve_in_world.push_back(point_in_world);
        }
        for (size_t i = 0; i < curves.size(); i++) {
            extended_curves.push_back(extended_trajectory_curve_in_world);
        }
        // 确定路径组中的选中路径
        // 选出最优路径（角度小，偏移小）
        double min_score = MAX_VALUE;
        size_t choosed_index = 0;
        for (size_t i = 0; i < frenet_curves.size(); i++) {
            double average_theta = 0.0, average_offset = 0.0;
            for (size_t j = 0; j < frenet_curves[i].size(); j++) {
                // 计算轨迹在frenet坐标系下的平均角度和平均偏移
                average_theta = average_theta + std::fabs(frenet_curves[i][j].theta_);
                average_offset = average_offset + std::fabs(frenet_curves[i][j].position_.y_ - frenet_curves[i][0].position_.y_) + std::fabs(frenet_curves[i][j].position_.y_ - frenet_curves[i][frenet_curves[i].size() - 1].position_.y_);
            }
            average_theta = average_theta / frenet_curves[i].size();
            average_offset = average_offset / frenet_curves[i].size();
            // LOG(INFO) << "index " << i << " average_offset " << average_offset;
            // LOG(INFO) << "index " << i << " average_theta " << average_theta;
            double score = average_offset + LONGITUDE_PATH_SELECTION_YAW_WEIGHT_FOR_LOW_SPEED_STATE * average_theta;
            if (Tools::isSmall(score, min_score)) {
                min_score = score;
                choosed_index = i;
            }
        }
        LOG(INFO) << "choosed index is " << choosed_index;

        // 从状态中挖出这一距离的路径作为减速的路径
        // 高速状态要先减速成为低速状态
        double aimed_velocity = LESS_THAN_BOUNDARY_RATION * velocity_limitation;
        // 2. 第二步，补充相应状态
        // 设置高速状态到低速状态的平均减速度（TOFIX）
        double aimed_acceleration = EXPECTED_DECCELERATION_FOR_LOW_SPEED;
        // 计算减速所需距离
        double traveled_distance = std::max((aimed_velocity * aimed_velocity - current_movement_state.velocity_ * current_movement_state.velocity_) / (2.0 * aimed_acceleration), AVOIDANCE_MIN_DISTANCE);
        // 路径起始点
        size_t start_index = 0;
        // 路径终点
        size_t end_index = start_index + static_cast<size_t>(traveled_distance / LANE_GAP_DISTANCE);
        // 路径延伸段终点
        double extended_distance = EXTENDED_DISTANCE_FOR_LOW_SPEED;
        size_t extend_end_index = end_index + static_cast<size_t>(extended_distance/ LANE_GAP_DISTANCE);
        assert(extend_end_index < curves[choosed_index].size() + extended_curves[choosed_index].size());

        PathPlanningUtilities::Curve aimed_curve, extended_aimed_curve, total_aimed_curve, total_forward_curve;
        total_forward_curve.assign(curves[choosed_index].begin(), curves[choosed_index].end());
        total_forward_curve.insert(total_forward_curve.end(), extended_curves[choosed_index].begin(), extended_curves[choosed_index].end());
        // 得到行驶路径
        aimed_curve.assign(total_forward_curve.begin() + start_index, total_forward_curve.begin() + end_index);
        // 得到行驶路径的延伸路径
        extended_aimed_curve.assign(total_forward_curve.begin() + end_index, total_forward_curve.begin() + extend_end_index);
        // 得到总路径
        total_aimed_curve.assign(aimed_curve.begin(), aimed_curve.end());
        total_aimed_curve.insert(total_aimed_curve.end(), extended_aimed_curve.begin(), extended_aimed_curve.end());

        // 碰撞的路径截断点
        size_t cut_index = total_aimed_curve.size() - 1;
        // 碰撞发生标志位
        bool is_collision_occur = false;
        bool is_traffic_influence;
        // 进行交通规则碰撞判定
        size_t traffic_rule_cut_index;
        if (RSS::collisionPositionIndexInCurve(total_aimed_curve, this->vehicle_width_, this->vehicle_length_, this->traffic_rule_obstacles_, &traffic_rule_cut_index)) {
            // 如果发生碰撞，记录碰撞点
            if (traffic_rule_cut_index <= cut_index) {
                cut_index = traffic_rule_cut_index;
                is_traffic_influence = true;
            }
            is_collision_occur = true;
            LOG(INFO) << "空气墙发生碰撞";
        } else {
            // 如果没有发生碰撞
            // do nothing
        }
        // 进行障碍物碰撞判定
        size_t obstacle_cut_index;
        if (RSS::collisionPositionIndexInCurve(total_aimed_curve, this->vehicle_width_, this->vehicle_length_, current_movement_state.velocity_, obstacles, &obstacle_cut_index)) {
            // 如果发生碰撞，记录碰撞点
            if (obstacle_cut_index <= cut_index) {
                cut_index = obstacle_cut_index;
                is_traffic_influence = false;
            }
            is_collision_occur = true;
            LOG(INFO) << "静态障碍物发生碰撞";
        } else {
            // 如果没有发生碰撞
            // do nothing
        }
        // 进行动态障碍物修正
        size_t dynamic_obstacle_block_index;
        if (RSS::collisionWithDynamicObstacles(total_aimed_curve, current_movement_state.velocity_, this->vehicle_width_, this->vehicle_length_, obstacles, &dynamic_obstacle_block_index)) {
            // 如果发生碰撞，修正碰撞点
            if (dynamic_obstacle_block_index <= cut_index) {
                cut_index = dynamic_obstacle_block_index;
                is_traffic_influence = false;
            }
            is_collision_occur = true;
            LOG(INFO) << "动态障碍物发生碰撞";
        }

        std::cout << "新路径长度" << aimed_curve.size() << "，新路径延伸段长度" << extended_aimed_curve.size() << std::endl;
        LOG(INFO) << "新路径长度" << aimed_curve.size() << "，新路径延伸段长度" << extended_aimed_curve.size();

        LOG(INFO) << "新路径高速到低速切换路径发生碰撞的结果为 " << is_collision_occur << "，碰撞点在路径上的第" << cut_index << "个点";
        std::cout << "新路径高速到低速切换路径发生碰撞的结果为 " << is_collision_occur << "，碰撞点在路径上的第" << cut_index << "个点" << std::endl;

        // 判断是选用新路径还是旧路径
        bool is_choosing_new_curve;
        if (this->current_state_.getStateCompleted() == true) {
            // 如果上一个状态已经完成，那么直接用新的路径
            is_choosing_new_curve = true;
            LOG(INFO) << "上一个状态已完成，不需要用旧路径";
        } else {
            // 如果上一个状态未能完成，对比状态
            // 首先判断上一个状态是否有碰撞，如果没有直接上一个状态
            if (!is_uncompleted_curve_collision) {
                // 如果上一个状态没撞，选上一个状态
                is_choosing_new_curve = false;
                LOG(INFO) << "上一个状态没有碰撞，不需要用新路径";
            } else if (!is_collision_occur) {
                // 如果上一个状态撞了，新路径没撞。
                // 判断旧路径的碰撞点是否远于新路径总长度，如果远，还是用旧路径
                if (uncompleted_curve_cut_index >= total_aimed_curve.size()) {
                    is_choosing_new_curve = false;
                    LOG(INFO) << "新路径没碰撞，旧路径碰撞，但是旧路径碰撞点远于新路径长度，还是用旧路径";
                } else {
                    is_choosing_new_curve = true;
                    LOG(INFO) << "新路径没碰撞，旧路径碰撞，且旧路径碰撞点近于新路径长度，用新路径";
                }
            } else {
                // 两个都撞了
                // 判断哪个更远
                if (cut_index > uncompleted_curve_cut_index + static_cast<int>(TO_CHOOSE_PATH_LONGER_DISTANCE / LANE_GAP_DISTANCE)) {
                    // 如果新路径比上一个状态碰撞点远6米以上，选新路径
                    is_choosing_new_curve = true;
                    LOG(INFO) << "新路径碰撞点更远，要用新路径";
                } else {
                    // 否则选旧路径
                    is_choosing_new_curve = false;
                    LOG(INFO) << "新路径碰撞点更近，要用旧路径";
                }
            }
        }

        if (is_choosing_new_curve) {
            // 选中新路径
            LOG(INFO) << "选择新路径进行减速";
            std::cout << "选择新路径进行减速" << std::endl;
            PathPlanningUtilities::Curve final_aimed_curve, final_aimed_extended_curve;
            if (is_collision_occur) {
                // 如果发生碰撞，找出存不存在更大的加速度可以不碰撞
                double final_aimed_acceleration = EXPECTED_DECCELERATION_FOR_LOW_SPEED;
                for (; !Tools::isSmall(final_aimed_acceleration, -COMMON_DECCELERATION); final_aimed_acceleration += -AIM_ACCELERATION_SAMPLING_GAP_FOR_LOW_SPEED_STATE) {
                    double final_traveled_distance = std::max((aimed_velocity * aimed_velocity - current_movement_state.velocity_ * current_movement_state.velocity_) / (2.0 * final_aimed_acceleration), AVOIDANCE_MIN_DISTANCE);
                    if (Tools::isSmall(final_traveled_distance + EXTENDED_DISTANCE_FOR_LOW_SPEED, cut_index * LANE_GAP_DISTANCE)) {
                        // 可以成功减速
                        is_collision_occur = false;
                        break;
                    }
                }
                // 如果不存在
                if (is_collision_occur) {
                    final_aimed_curve = aimed_curve;
                    final_aimed_extended_curve = extended_aimed_curve;
                    LOG(INFO) << "最大减速度都无法进行减速";
                } else {
                    // 如果存在
                    size_t final_end_index = static_cast<size_t>(std::max((aimed_velocity * aimed_velocity - current_movement_state.velocity_ * current_movement_state.velocity_) / (2.0 * final_aimed_acceleration), AVOIDANCE_MIN_DISTANCE) / LANE_GAP_DISTANCE);
                    size_t final_extended_end_index = final_end_index + static_cast<size_t>((LESS_THAN_BOUNDARY_RATION * EXTENDED_DISTANCE_FOR_LOW_SPEED) / LANE_GAP_DISTANCE);
                    final_aimed_curve.assign(total_aimed_curve.begin(), total_aimed_curve.begin() + final_end_index);
                    final_aimed_extended_curve.assign(total_aimed_curve.begin() + final_end_index, total_aimed_curve.begin() + final_extended_end_index);
                    std::cout << "可以以更大加速度" << final_aimed_acceleration << "进行减速" << std::endl;
                    LOG(INFO) << "可以以更大加速度" << final_aimed_acceleration << "进行减速";
                }
            } else {
                final_aimed_curve = aimed_curve;
                final_aimed_extended_curve = extended_aimed_curve;
            }

            // 进行状态赋值
            // 确定状态是停车还是避障
            if (is_collision_occur) {
                // 由于高速到低速切换过程中，存在碰撞风险，因此直接补充并选择停车状态。
                // 停车状态为有效状态
                this->states_set_[StateNames::STOP].enable();
                // 提供车辆形状信息
                this->states_set_[StateNames::STOP].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
                // 提供起始点信息
                this->states_set_[StateNames::STOP].setVehicleStartState(current_point_in_world);
                this->states_set_[StateNames::STOP].setVehicleStartMovement(current_movement_state);
                // 确定速度上下限制,速度固定为0(无效值)
                this->states_set_[StateNames::STOP].setVelocityLimitation(current_movement_state.velocity_, 0.0);
                // 确定加速度上下限制,固定为0(无效值)
                this->states_set_[StateNames::STOP].setAccelerationLimitationMax(0.0);
                this->states_set_[StateNames::STOP].setAccelerationLimitationMin(0.0);
                // 确定速度期望值(无效值)
                this->states_set_[StateNames::STOP].setExpectedVelocity(0.0, 0.0);
                // 设置安全性(无效值)
                this->states_set_[StateNames::STOP].setSafety(true);
                // 设置优先级(TOFIX)(无效值)
                this->states_set_[StateNames::STOP].setPriority(0.0);
                // 设置路径信息(选取直行道路作为停车的路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(final_aimed_curve);
                this->states_set_[StateNames::STOP].setTrajectory(curve_set);
                // 设置延伸路径信息(选取直行道路作为停车的延伸路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> extended_curve_set;
                extended_curve_set.push_back(final_aimed_extended_curve);
                this->states_set_[StateNames::STOP].setExtendedTrajectory(extended_curve_set);
                // 提供车辆当前速度信息
                this->states_set_[StateNames::STOP].setVehicleCurrentMovement(current_movement_state);
                // 提供车辆当前点在路径中的下标
                this->states_set_[StateNames::STOP].setVehicleCurrentPositionIndexInTrajectory(0);
                // 提供车辆目标点在路径中的下标(设置停车位置，是停车状态中最重要的环节)
                // 停车位置为碰撞点减去车辆半长，再减去余量（3米）
                if (this->is_single_lane_ && is_traffic_influence) {
                    cut_index = std::max(static_cast<int>(0), static_cast<int>(cut_index) - static_cast<int>(GAP_MAINTAIN / LANE_GAP_DISTANCE));
                } else if (is_traffic_influence) {
                    cut_index = std::max(static_cast<int>(0), static_cast<int>(cut_index) - static_cast<int>(GAP_MAINTAIN / LANE_GAP_DISTANCE));
                } else {
                    cut_index = std::max(static_cast<int>(0), static_cast<int>(cut_index) - static_cast<int>(GAP_MAINTAIN / LANE_GAP_DISTANCE));
                }
                this->states_set_[StateNames::STOP].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(cut_index);
                // 提供车辆目标点速度
                this->states_set_[StateNames::STOP].setGoalVelocity(0.0);
                // 最大速度限制
                this->states_set_[StateNames::STOP].setAllowMaxVelocity(aimed_velocity);
                // 设置期望保持的加速度值（无效值）
                this->states_set_[StateNames::STOP].setVehicleDynamicPlanningExpectedAcceleration(0.0);

                LOG(INFO) << "停车状态补充完毕，刹车距离为" << static_cast<double>(cut_index * LANE_GAP_DISTANCE) << "米，起始速度为" << current_movement_state.velocity_ << "米/秒";
                // 选中状态为停车状态
                this->choosed_state_ = this->states_set_[StateNames::STOP];

            } else {
                // 进行高速到低速的切换，这一过程属于避障状态。
                // 避障状态为有效状态
                this->states_set_[StateNames::AVOIDANCE].enable();
                // 提供车辆形状信息
                this->states_set_[StateNames::AVOIDANCE].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
                // 提供起始点信息
                this->states_set_[StateNames::AVOIDANCE].setVehicleStartState(current_point_in_world);
                this->states_set_[StateNames::AVOIDANCE].setVehicleStartMovement(current_movement_state);
                // 确定速度上下限制(无效值)
                this->states_set_[StateNames::AVOIDANCE].setVelocityLimitation(current_movement_state.velocity_, 0.0);
                // 确定加速度上下限制,固定为0(无效值)
                this->states_set_[StateNames::AVOIDANCE].setAccelerationLimitationMax(0.0);
                this->states_set_[StateNames::AVOIDANCE].setAccelerationLimitationMin(0.0);
                // 确定速度期望值(无效值)
                this->states_set_[StateNames::AVOIDANCE].setExpectedVelocity(0.0, 0.0);
                // 设置安全性(无效值)
                this->states_set_[StateNames::AVOIDANCE].setSafety(true);
                // 设置优先级(TOFIX)(无效值)
                this->states_set_[StateNames::AVOIDANCE].setPriority(0.0);
                // 设置路径信息(选取直行道路作为减速的路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(final_aimed_curve);
                this->states_set_[StateNames::AVOIDANCE].setTrajectory(curve_set);
                // 设置延伸路径信息(选取直行道路作为减速的延伸路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> extended_curve_set;
                extended_curve_set.push_back(final_aimed_extended_curve);
                this->states_set_[StateNames::AVOIDANCE].setExtendedTrajectory(extended_curve_set);
                // 设置选中轨迹
                this->states_set_[StateNames::AVOIDANCE].setChoosedTrajectoryIndex(0);
                // 提供车辆当前速度信息
                this->states_set_[StateNames::AVOIDANCE].setVehicleCurrentMovement(current_movement_state);
                // 提供车辆当前点在路径中的下标
                this->states_set_[StateNames::AVOIDANCE].setVehicleCurrentPositionIndexInTrajectory(0);
                // 提供车辆目标点在路径中的下标（就是路径的长度）
                this->states_set_[StateNames::AVOIDANCE].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->states_set_[StateNames::AVOIDANCE].getTrajectoryLength());
                // 提供车辆目标点速度
                this->states_set_[StateNames::AVOIDANCE].setGoalVelocity(aimed_velocity);
                // 提供允许的最大速度
                this->states_set_[StateNames::AVOIDANCE].setAllowMaxVelocity(aimed_velocity);
                // 设置期望保持的加速度值（无效值）
                this->states_set_[StateNames::AVOIDANCE].setVehicleDynamicPlanningExpectedAcceleration(0.0);

                LOG(INFO) << "减速状态补充完毕，减速距离为" << static_cast<double>(this->states_set_[StateNames::AVOIDANCE].getTrajectoryLength() * LANE_GAP_DISTANCE) << "米，起始速度为" << current_movement_state.velocity_ << "米/秒，目标速度为" << aimed_velocity << "米/秒";
                // 选中状态为避障状态
                this->choosed_state_ = this->states_set_[StateNames::AVOIDANCE];
            }

        } else {
            // 选中旧路径
            LOG(INFO) << "选择旧路径进行减速";
            PathPlanningUtilities::Curve final_aimed_curve, final_aimed_extended_curve;
            if (is_uncompleted_curve_collision) {
                // 如果发生碰撞，找出存不存在更大的加速度可以不碰撞
                double final_aimed_acceleration = EXPECTED_DECCELERATION_FOR_LOW_SPEED;
                for (; !Tools::isSmall(final_aimed_acceleration, -COMMON_DECCELERATION); final_aimed_acceleration += -AIM_ACCELERATION_SAMPLING_GAP_FOR_LOW_SPEED_STATE) {
                    double final_traveled_distance = std::max((aimed_velocity * aimed_velocity - current_movement_state.velocity_ * current_movement_state.velocity_) / (2.0 * final_aimed_acceleration), AVOIDANCE_MIN_DISTANCE);
                    if (Tools::isSmall(final_traveled_distance + EXTENDED_DISTANCE_FOR_LOW_SPEED, uncompleted_curve_cut_index * LANE_GAP_DISTANCE)) {
                        // 可以成功减速
                        is_uncompleted_curve_collision = false;
                        break;
                    }
                }
                // 如果不存在
                if (is_uncompleted_curve_collision) {
                    final_aimed_curve = uncompleted_curve;
                    final_aimed_extended_curve = uncompleted_extended_curve;
                    LOG(INFO) << "最大减速度都无法进行减速";
                } else {
                    // 如果存在
                    size_t final_end_index = static_cast<size_t>(std::max((aimed_velocity * aimed_velocity - current_movement_state.velocity_ * current_movement_state.velocity_) / (2.0 * final_aimed_acceleration), AVOIDANCE_MIN_DISTANCE) / LANE_GAP_DISTANCE);
                    size_t final_extended_end_index = final_end_index + static_cast<size_t>((EXTENDED_DISTANCE_FOR_LOW_SPEED * LESS_THAN_BOUNDARY_RATION) / LANE_GAP_DISTANCE);
                    final_aimed_curve.assign(uncompleted_total_curve.begin(), uncompleted_total_curve.begin() + final_end_index);
                    final_aimed_extended_curve.assign(uncompleted_total_curve.begin() + final_end_index, uncompleted_total_curve.begin() + final_extended_end_index);
                    std::cout << "可以以更大加速度" << final_aimed_acceleration << "进行减速" << std::endl;
                    LOG(INFO) << "可以以更大加速度" << final_aimed_acceleration << "进行减速";
                }
            } else {
                // 如果没有发生碰撞
                final_aimed_curve = uncompleted_curve;
                final_aimed_extended_curve = uncompleted_extended_curve;
            }

            // 进行状态赋值
            // 确定状态是停车还是避障
            if (is_uncompleted_curve_collision) {
                // 由于高速到低速切换过程中，存在碰撞风险，因此直接补充并选择停车状态。
                // 停车状态为有效状态
                this->states_set_[StateNames::STOP].enable();
                // 提供车辆形状信息
                this->states_set_[StateNames::STOP].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
                // 提供起始点信息
                this->states_set_[StateNames::STOP].setVehicleStartState(current_point_in_world);
                this->states_set_[StateNames::STOP].setVehicleStartMovement(current_movement_state);
                // 确定速度上下限制,速度固定为0(无效值)
                this->states_set_[StateNames::STOP].setVelocityLimitation(current_movement_state.velocity_, 0.0);
                // 确定加速度上下限制,固定为0(无效值)
                this->states_set_[StateNames::STOP].setAccelerationLimitationMax(0.0);
                this->states_set_[StateNames::STOP].setAccelerationLimitationMin(0.0);
                // 确定速度期望值(无效值)
                this->states_set_[StateNames::STOP].setExpectedVelocity(0.0, 0.0);
                // 设置安全性(无效值)
                this->states_set_[StateNames::STOP].setSafety(true);
                // 设置优先级(TOFIX)(无效值)
                this->states_set_[StateNames::STOP].setPriority(0.0);
                // 设置路径信息(选取直行道路作为停车的路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(final_aimed_curve);
                this->states_set_[StateNames::STOP].setTrajectory(curve_set);
                // 设置延伸路径信息(选取直行道路作为停车的延伸路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> extended_curve_set;
                extended_curve_set.push_back(final_aimed_extended_curve);
                this->states_set_[StateNames::STOP].setExtendedTrajectory(extended_curve_set);
                // 提供车辆当前速度信息
                this->states_set_[StateNames::STOP].setVehicleCurrentMovement(current_movement_state);
                // 提供车辆当前点在路径中的下标
                this->states_set_[StateNames::STOP].setVehicleCurrentPositionIndexInTrajectory(0);
                // 提供车辆目标点在路径中的下标(设置停车位置，是停车状态中最重要的环节)
                // 停车位置为碰撞点减去车辆半长，再减去余量（3米）
                if (this->is_single_lane_ && is_uncompleted_curve_traffic_influence) {
                    uncompleted_curve_cut_index = std::max(static_cast<int>(0), static_cast<int>(uncompleted_curve_cut_index) - static_cast<int>(GAP_MAINTAIN / LANE_GAP_DISTANCE));
                } else if (is_uncompleted_curve_traffic_influence) {
                    uncompleted_curve_cut_index = std::max(static_cast<int>(0), static_cast<int>(uncompleted_curve_cut_index) - static_cast<int>(GAP_MAINTAIN / LANE_GAP_DISTANCE));
                } else {
                    uncompleted_curve_cut_index = std::max(static_cast<int>(0), static_cast<int>(uncompleted_curve_cut_index) - static_cast<int>(GAP_MAINTAIN / LANE_GAP_DISTANCE));
                }

                this->states_set_[StateNames::STOP].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(uncompleted_curve_cut_index);
                // 提供车辆目标点速度
                this->states_set_[StateNames::STOP].setGoalVelocity(0.0);
                // 设置期望保持的加速度值（无效值）
                this->states_set_[StateNames::STOP].setVehicleDynamicPlanningExpectedAcceleration(0.0);

                // 是否为接着之前的路径
                this->states_set_[StateNames::STOP].isStateContinue(true);

                LOG(INFO) << "停车状态补充完毕，刹车距离为" << static_cast<double>(uncompleted_curve_cut_index * LANE_GAP_DISTANCE) << "米，起始速度为" << current_movement_state.velocity_ << "米/秒";
                // 选中状态为停车状态
                this->choosed_state_ = this->states_set_[StateNames::STOP];

            } else {
                // 进行高速到低速的切换，这一过程属于避障状态。
                // 避障状态为有效状态
                this->states_set_[StateNames::AVOIDANCE].enable();
                // 提供车辆形状信息
                this->states_set_[StateNames::AVOIDANCE].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
                // 提供起始点信息
                this->states_set_[StateNames::AVOIDANCE].setVehicleStartState(current_point_in_world);
                this->states_set_[StateNames::AVOIDANCE].setVehicleStartMovement(current_movement_state);
                // 确定速度上下限制(无效值)
                this->states_set_[StateNames::AVOIDANCE].setVelocityLimitation(LESS_THAN_BOUNDARY_RATION * VELOCITY_THRESHOLD, 0.0);
                // 确定速度期望值(无效值)
                this->states_set_[StateNames::AVOIDANCE].setExpectedVelocity(0.0, 0.0);
                // 确定加速度上下限制,固定为0(无效值)
                this->states_set_[StateNames::AVOIDANCE].setAccelerationLimitationMax(0.0);
                this->states_set_[StateNames::AVOIDANCE].setAccelerationLimitationMin(0.0);
                // 设置安全性(无效值)
                this->states_set_[StateNames::AVOIDANCE].setSafety(true);
                // 设置优先级(TOFIX)(无效值)
                this->states_set_[StateNames::AVOIDANCE].setPriority(0.0);
                // 设置路径信息(选取直行道路作为减速的路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(final_aimed_curve);
                this->states_set_[StateNames::AVOIDANCE].setTrajectory(curve_set);
                // 设置延伸路径信息(选取直行道路作为减速的延伸路径)(TOFIX)
                std::vector<PathPlanningUtilities::Curve> extended_curve_set;
                extended_curve_set.push_back(final_aimed_extended_curve);
                this->states_set_[StateNames::AVOIDANCE].setExtendedTrajectory(extended_curve_set);
                // 设置选中轨迹
                this->states_set_[StateNames::AVOIDANCE].setChoosedTrajectoryIndex(0);
                // 提供车辆当前速度信息
                this->states_set_[StateNames::AVOIDANCE].setVehicleCurrentMovement(current_movement_state);
                // 提供车辆当前点在路径中的下标
                this->states_set_[StateNames::AVOIDANCE].setVehicleCurrentPositionIndexInTrajectory(0);
                // 提供车辆目标点在路径中的下标（就是路径的长度）
                this->states_set_[StateNames::AVOIDANCE].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->states_set_[StateNames::AVOIDANCE].getTrajectoryLength());
                // 提供车辆目标点速度
                this->states_set_[StateNames::AVOIDANCE].setGoalVelocity(aimed_velocity);
                // 设置最大允许速度
                this->states_set_[StateNames::AVOIDANCE].setAllowMaxVelocity(aimed_velocity);
                // 设置期望保持的加速度值（无效值）
                this->states_set_[StateNames::AVOIDANCE].setVehicleDynamicPlanningExpectedAcceleration(0.0);

                // 是否为接着之前的路径
                this->states_set_[StateNames::AVOIDANCE].isStateContinue(true);

                LOG(INFO) << "减速状态补充完毕，减速距离为" << static_cast<double>(this->states_set_[StateNames::AVOIDANCE].getTrajectoryLength() * LANE_GAP_DISTANCE) << "米，起始速度为" << current_movement_state.velocity_ << "米/秒，目标速度为" << aimed_velocity << "米/秒";
                // 选中状态为避障状态
                this->choosed_state_ = this->states_set_[StateNames::AVOIDANCE];
            }
        }

    } else {
        // 如果当前速度小于等于限速，此时为低速状态
        double max_allowed_velocity = LESS_THAN_BOUNDARY_RATION * velocity_limitation;
        // 低速状态直接开始避障逻辑
        // 首先确定避障边界范围（以左侧为正，右侧为负）
        // 避障范围的确立，第一步判断道路是否存在，第二步判断道路是否允许避障，如果允许则以道路中心向两侧生成多条曲线，如果不允许则只生成中心线，第三步判断生成线的法向加速度和加加速度，去除大于阈值的曲线，第四步判断每一条线的截断点（碰撞点），如果有曲线没有碰撞点则选其作为避障路径，如果所有曲线都有碰撞点，取最远的作为停车路径，如果都差不多远，选中心线作为停车路径。
        LOG(INFO) << "当前处于低速状态，直接可以进入避障状态";
        std::cout << "当前处于低速状态，直接可以进入避障状态" << std::endl;
        // 保存每一条道路上的最优避障轨迹
        std::vector<DecisionMaking::AvoidancePath> each_lane_avoidance_path_set;
        // // judge if maintain last path
        // if ((this->current_state_.getStateName() == AVOIDANCE || this->current_state_.getStateName() == STOP) && this->current_state_.)

        // 生成道路的避障路径
        clock_t start_planning_time = clock();
        // 利用多线程进行路径生成
        std::vector<DecisionMaking::AvoidancePath> center_avoidance_paths, left_avoidance_paths, right_avoidance_paths;
        std::shared_ptr<std::thread> center_avoidance_path_generator_ptr, left_avoidance_path_generator_ptr, right_avoidance_path_generator_ptr;
        // 创建线程
        if (this->center_lane_.getLaneExistance()) {
            LOG(INFO) << "中间车道存在";
            center_avoidance_path_generator_ptr = std::make_shared<std::thread>(&DecisionMaking::SubVehicle::AvoidancePathGenerator, this, std::ref(this->center_lane_), std::ref(center_avoidance_paths));
            // std::vector<DecisionMaking::AvoidancePath> avoidance_curves = this->AvoidancePathGenerator(this->center_lane_);
            // each_lane_avoidance_path_set.insert(each_lane_avoidance_path_set.end(), avoidance_curves.begin(), avoidance_curves.end());
        }

        // // 生成左侧道的避障路径
        if (this->left_lane_.getLaneExistance()) {
            LOG(INFO) << "左车道存在";
            left_avoidance_path_generator_ptr = std::make_shared<std::thread>(&DecisionMaking::SubVehicle::AvoidancePathGenerator, this, std::ref(this->left_lane_), std::ref(left_avoidance_paths));
            // std::vector<DecisionMaking::AvoidancePath> avoidance_curves = this->AvoidancePathGenerator(this->left_lane_);
            // each_lane_avoidance_path_set.insert(each_lane_avoidance_path_set.end(), avoidance_curves.begin(), avoidance_curves.end());
        }

        // 生成右侧道的避障路径
        if (this->right_lane_.getLaneExistance()) {
            LOG(INFO) << "右车道存在";
            right_avoidance_path_generator_ptr = std::make_shared<std::thread>(&DecisionMaking::SubVehicle::AvoidancePathGenerator, this, std::ref(this->right_lane_), std::ref(right_avoidance_paths));
            // std::vector<DecisionMaking::AvoidancePath> avoidance_curves = this->AvoidancePathGenerator(this->right_lane_);
            // each_lane_avoidance_path_set.insert(each_lane_avoidance_path_set.end(), avoidance_curves.begin(), avoidance_curves.end());
        }
        // 启动线程
        if (this->center_lane_.getLaneExistance()) {
            center_avoidance_path_generator_ptr->join();
        }
        if (this->left_lane_.getLaneExistance()) {
            left_avoidance_path_generator_ptr->join();
        }
        if (this->right_lane_.getLaneExistance()) {
            right_avoidance_path_generator_ptr->join();
        }

        // 保存生成的路径
        each_lane_avoidance_path_set.insert(each_lane_avoidance_path_set.end(), center_avoidance_paths.begin(), center_avoidance_paths.end());
        each_lane_avoidance_path_set.insert(each_lane_avoidance_path_set.end(), left_avoidance_paths.begin(), left_avoidance_paths.end());
        each_lane_avoidance_path_set.insert(each_lane_avoidance_path_set.end(), right_avoidance_paths.begin(), right_avoidance_paths.end());

        clock_t end_planning_time = clock();
        LOG(INFO) << "avoidance paths generation finish, time consuming is " << static_cast<double>(end_planning_time - start_planning_time) * 1000.0 / CLOCKS_PER_SEC << "ms";
        LOG(INFO) << "avoidance path number is " << each_lane_avoidance_path_set.size();
        std::cout << "avoidance path number is " << each_lane_avoidance_path_set.size() << std::endl;
        assert(each_lane_avoidance_path_set.size() != 0);

        // 对全部的避障路径进行可视化(TODO)
        // 可视化全部的蔽障路径
        // 清空之前的可视化
        visualization_msgs::MarkerArray delete_marker_array;
        delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
        this->vis_multi_curve_pub_.publish(delete_marker_array);
        visualization_msgs::MarkerArray marker_array;
        // 可视化全部路径
        for(size_t i = 0; i < each_lane_avoidance_path_set.size(); i++) {
            PathPlanningUtilities::Curve center_trajectory = each_lane_avoidance_path_set[i].getTrajectory();
            std_msgs::ColorRGBA color;
            color.r = 138.0 / 255.0;
            color.g = 43.0 / 255.0;
            color.b = 226.0 / 255.0;
            color.a = 1.0;
            marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(center_trajectory, color, i));
        }
        this->vis_multi_curve_pub_.publish(marker_array);

        // 对避障路径进行选择
        clock_t start_choose_time = clock();
        size_t best_path_index = selectAvoidancePath(each_lane_avoidance_path_set);
        clock_t end_choose_time = clock();
        LOG(INFO) << "best path get, time consuming is " << static_cast<double>(end_choose_time - start_choose_time) * 1000.0 / CLOCKS_PER_SEC << "ms";
        std::cout << "best path get, time consuming is " << static_cast<double>(end_choose_time - start_choose_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;

        // 得到最优路径
        AvoidancePath best_avoidance_path = each_lane_avoidance_path_set[best_path_index];

        // 得到最优路径后，进行动态障碍物修正
        // 进行动态障碍物修正
        size_t final_cut_index, dynamic_obstacle_block_index;
        bool is_dynamic_influence = false;  // 受动态影响是否严重
        if (RSS::collisionWithDynamicObstacles(best_avoidance_path.getTrajectory(), current_movement_state.velocity_, this->vehicle_width_, this->vehicle_length_, obstacles, &dynamic_obstacle_block_index)) {
            final_cut_index = std::min(dynamic_obstacle_block_index, static_cast<size_t>(best_avoidance_path.getCutIndex()));
            is_dynamic_influence = true;
        } else {
            final_cut_index = best_avoidance_path.getCutIndex();
            is_dynamic_influence = false;
        }

        // 已经得到避障轨迹，开始状态填充
        if (!(final_cut_index == best_avoidance_path.getTrajectory().size() - 1)) {
            LOG(INFO) << "最终避障轨迹与障碍物相交，交点位置为" << final_cut_index;
            // 填充停车状态
            // 停车状态为有效状态
            this->states_set_[StateNames::STOP].enable();
            // 提供车辆形状信息
            this->states_set_[StateNames::STOP].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
            // 提供起始点信息
            this->states_set_[StateNames::STOP].setVehicleStartState(current_point_in_world);
            this->states_set_[StateNames::STOP].setVehicleStartMovement(current_movement_state);
            // 确定速度上下限制,速度固定为0(无效值)
            this->states_set_[StateNames::STOP].setVelocityLimitation(current_movement_state.velocity_, 0.0);
            // 确定加速度上下限制,固定为0(无效值)
            this->states_set_[StateNames::STOP].setAccelerationLimitationMax(0.0);
            this->states_set_[StateNames::STOP].setAccelerationLimitationMin(0.0);
            // 确定速度期望值(无效值)
            this->states_set_[StateNames::STOP].setExpectedVelocity(0.0, 0.0);
            // 设置安全性(无效值)
            this->states_set_[StateNames::STOP].setSafety(true);
            // 设置优先级(TOFIX)(无效值)
            this->states_set_[StateNames::STOP].setPriority(0.0);
            // 设置路径信息(选取直行道路作为停车的路径)(TOFIX)
            std::vector<PathPlanningUtilities::Curve> curve_set;
            curve_set.push_back(best_avoidance_path.getTrajectory());
            this->states_set_[StateNames::STOP].setTrajectory(curve_set);
            // 设置延伸路径信息(选取直行道路作为停车的延伸路径)(无延伸路径)
            std::vector<PathPlanningUtilities::Curve> extended_curve_set;
            extended_curve_set.push_back(PathPlanningUtilities::Curve());
            this->states_set_[StateNames::STOP].setExtendedTrajectory(extended_curve_set);
            // 提供车辆当前速度信息
            this->states_set_[StateNames::STOP].setVehicleCurrentMovement(current_movement_state);
            // 提供车辆当前点在路径中的下标
            this->states_set_[StateNames::STOP].setVehicleCurrentPositionIndexInTrajectory(0);
            // 提供车辆目标点在路径中的下标(设置停车位置，是停车状态中最重要的环节)
            // 停车位置为碰撞点减去车辆半长，再减去余量（3米）
            // 判断离终点距离
            size_t distance_to_goal_index = static_cast<size_t>(std::max(this->distance_to_goal_, 0.0) / LANE_GAP_DISTANCE);
            // LOG(INFO) << "离终点的下标数" << distance_to_goal_index;
            size_t cut_index;
            if (final_cut_index < distance_to_goal_index){
                if (is_dynamic_influence) {
                    // 动态影响
                    cut_index = std::max(static_cast<int>(0), static_cast<int>(final_cut_index) - static_cast<int>(GAP_MAINTAIN / LANE_GAP_DISTANCE));
                    LOG(INFO) << "距离类型3，动态障碍物碰撞";
                } else {
                    if (this->is_single_lane_ && best_avoidance_path.getInfluenceType() == 1) {
                        cut_index = std::max(static_cast<int>(0), static_cast<int>(final_cut_index) - static_cast<int>(0.65 * GAP_MAINTAIN / LANE_GAP_DISTANCE));
                        LOG(INFO) << "距离类型1，单路径，交通碰撞";
                    } else if (best_avoidance_path.getInfluenceType() == 1) {
                        cut_index = std::max(static_cast<int>(0), static_cast<int>(final_cut_index) - static_cast<int>(0.65 * GAP_MAINTAIN / LANE_GAP_DISTANCE));
                        LOG(INFO) << "距离类型2，交通碰撞";
                    } else {
                        // 非动态影响
                        cut_index = std::max(static_cast<int>(0), static_cast<int>(final_cut_index) - static_cast<int>(0.65 * GAP_MAINTAIN / LANE_GAP_DISTANCE));
                        LOG(INFO) << "距离类型3，静态障碍物碰撞";
                    }

                }
                
            } else {
                cut_index = distance_to_goal_index;
                // PathPlanningUtilities::Point2f destination = {this->destination_pose_.pose.position.x, this->destination_pose_.pose.position.y};
                LOG(INFO) << "到终点前不会发生碰撞，直接以终点作为目标点" << best_avoidance_path.getTrajectory()[cut_index].position_;
                // LOG(INFO) << "真实目标点坐标" << destination;

                LOG(INFO) << "地图给出到终点距离" << this->distance_to_goal_;
                // double true_disance = PathPlanningUtilities::calcDistance(destination, current_point_in_world.position_);
                // LOG(INFO) << "实际计算出的距离" << true_disance;

                // if (true_disance > 2.0) {
                //     // 显示路径直接的差
                //     for (size_t ll = 0; ll < best_avoidance_path.getTrajectory().size() - 1; ll++) {
                //         double dis = PathPlanningUtilities::calcDistance(best_avoidance_path.getTrajectory()[ll].position_, best_avoidance_path.getTrajectory()[ll + 1].position_);
                //         LOG(INFO) << ll << "'s distance " << dis;
                //     }
                // }

                // LOG(INFO) << "目标点和终点的误差距离为" << PathPlanningUtilities::calcDistance(best_avoidance_path.getTrajectory()[cut_index].position_, destination);
            }
            // // 特殊情况
            // GLOBAL_IN_GROUND_MUTEX_.lock();
            // if (GLOBAL_IS_IN_GROUND_) {
            //     // 在园区内，距离为3.5米
            //     cut_index = std::max(static_cast<int>(0), static_cast<int>(final_cut_index) - static_cast<int>(3.5 / LANE_GAP_DISTANCE));
            // }
            // GLOBAL_IN_GROUND_MUTEX_.unlock();
            // if (GLOBAL_IS_IN_CHECK_) {
            //     // 临检4米
            //     cut_index = std::max(static_cast<int>(0), static_cast<int>(final_cut_index) - static_cast<int>(4.0 / LANE_GAP_DISTANCE));
            // }
            std::cout << "cut index is " << cut_index << std::endl;
            this->states_set_[StateNames::STOP].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(cut_index);
            // 提供车辆目标点速度
            this->states_set_[StateNames::STOP].setGoalVelocity(0.0);
            // 计算最大允许速度
            // 首先是初始值
            double max_allowable_velocity = max_allowed_velocity;
            if (cut_index > 0) {
                // 得到路径片段
                PathPlanningUtilities::Curve path_segment;
                path_segment.assign(best_avoidance_path.getTrajectory().begin(), best_avoidance_path.getTrajectory().begin() + cut_index);
                // 其次是曲率限制
                double max_curvature_before_cut_index = path_segment[Tools::calcMaxKappaIndexForCurve(path_segment)].kappa_;
                double curvature_velocity_limitation = Tools::calcVelocityForMaxNormalAcceleration(max_curvature_before_cut_index);

                // 曲率变化率限制
                double max_curvature_change_rate_befor_cut_index = Tools::getCurveMaxCurvatureChangeRate(path_segment);
                double curvature_change_rate_velocity_limitation = Tools::calcVelocityForMaxNormalJerk(max_curvature_change_rate_befor_cut_index);

                // 之后是障碍物限制
                double min_distance_to_obs = MAX_VALUE;
                for (size_t i = 0; i < cut_index; i++) {
                    double distance_to_obs = best_avoidance_path.getObsCollisionInfo()[i];
                    if (Tools::isSmall(distance_to_obs, min_distance_to_obs)) {
                        min_distance_to_obs = distance_to_obs;
                    }
                }
                double obstacle_velocity_limitation = AVOIDANCE_VELOCITY + std::min(1.0, min_distance_to_obs / (MAX_DISTANCE_RATIO * this->vehicle_width_)) * (0.9 * VELOCITY_THRESHOLD - AVOIDANCE_VELOCITY);

                max_allowable_velocity = std::min(std::min(std::min(max_allowable_velocity, curvature_velocity_limitation), obstacle_velocity_limitation), curvature_change_rate_velocity_limitation);

                LOG(INFO) << "曲线的最大曲率为" << max_curvature_before_cut_index << "，离障碍物最小距离为" << min_distance_to_obs;

                LOG(INFO) << "曲率对于速度的限制为 " << curvature_velocity_limitation << ", 曲率变化率对于速度的限制为" << curvature_change_rate_velocity_limitation << ", 障碍物对速度的限制为 " << obstacle_velocity_limitation;
            }

            // 判断是否为脱困模式
            if (best_avoidance_path.getOutofTrapping()) {
                // 脱困模式最大速度为0.5
                max_allowable_velocity = 0.5;
            }

            this->states_set_[StateNames::STOP].setAllowMaxVelocity(max_allowable_velocity);

            // 设置期望保持的加速度值（无效值）
            this->states_set_[StateNames::STOP].setVehicleDynamicPlanningExpectedAcceleration(0.0);
            // 设置是否进行保持
            this->states_set_[StateNames::STOP].setStateMaintain(false);
            // 设置是否为脱困模式
            this->states_set_[StateNames::STOP].setOutofTrapping(best_avoidance_path.getOutofTrapping());

            LOG(INFO) << "停车状态补充完毕，刹车距离为" << static_cast<double>(cut_index * LANE_GAP_DISTANCE) << "米，起始速度为" << current_movement_state.velocity_ << "米/秒";
            // 选中状态为停车状态
            this->choosed_state_ = this->states_set_[StateNames::STOP];
            std::cout << "state stop finished" << std::endl;
        } else {
            LOG(INFO) << "最终避障轨迹没有与障碍物相交";
            // 避障状态为有效状态
            this->states_set_[StateNames::AVOIDANCE].enable();
            // 提供车辆形状信息
            this->states_set_[StateNames::AVOIDANCE].setVehicleSize(this->vehicle_length_, this->vehicle_width_);
            // 提供起始点信息
            this->states_set_[StateNames::AVOIDANCE].setVehicleStartState(current_point_in_world);
            this->states_set_[StateNames::AVOIDANCE].setVehicleStartMovement(current_movement_state);
            // 确定速度上下限制(无效值)
            this->states_set_[StateNames::AVOIDANCE].setVelocityLimitation(0.9 * VELOCITY_THRESHOLD, 0.0);
            // 确定加速度上下限制,固定为0(无效值)
            this->states_set_[StateNames::AVOIDANCE].setAccelerationLimitationMax(0.0);
            this->states_set_[StateNames::AVOIDANCE].setAccelerationLimitationMin(0.0);
            // 确定速度期望值(无效值)
            this->states_set_[StateNames::AVOIDANCE].setExpectedVelocity(0.0, 0.0);
            // 设置安全性(无效值)
            this->states_set_[StateNames::AVOIDANCE].setSafety(true);
            // 设置优先级(TOFIX)(无效值)
            this->states_set_[StateNames::AVOIDANCE].setPriority(0.0);
            // 设置路径信息(选取直行道路作为减速的路径)(TOFIX)
            std::vector<PathPlanningUtilities::Curve> curve_set;
            curve_set.push_back(best_avoidance_path.getTrajectory());
            this->states_set_[StateNames::AVOIDANCE].setTrajectory(curve_set);
            // 设置延伸路径信息(选取直行道路作为减速的延伸路径)(TOFIX)
            std::vector<PathPlanningUtilities::Curve> extended_curve_set;
            extended_curve_set.push_back(PathPlanningUtilities::Curve());
            this->states_set_[StateNames::AVOIDANCE].setExtendedTrajectory(extended_curve_set);
            // 提供车辆当前速度信息
            this->states_set_[StateNames::AVOIDANCE].setVehicleCurrentMovement(current_movement_state);
            // 提供车辆当前点在路径中的下标
            this->states_set_[StateNames::AVOIDANCE].setVehicleCurrentPositionIndexInTrajectory(0);
            // 提供车辆目标点在路径中的下标（就是路径的长度）
            this->states_set_[StateNames::AVOIDANCE].setVehicleDynamicPlanningGoalPositionIndexInTrajectory(this->states_set_[StateNames::AVOIDANCE].getTrajectoryLength() - 1);
            // 提供车辆目标点速度
            // 获取最大曲率
            double curve_max_kappa = std::abs(best_avoidance_path.getMaxKappa());
            // 获取最大曲率变化率
            double curvature_max_change_rate = Tools::getCurveMaxCurvatureChangeRate(best_avoidance_path.getTrajectory());
            // 获得离障碍物最小
            double min_distance_to_obs = best_avoidance_path.getMinDistanceToObstacle();
            LOG(INFO) << "曲线的最大曲率为" << curve_max_kappa << "，离障碍物最小距离为" << min_distance_to_obs;

            // 首先是曲率限制
            double curvature_velocity_limitation = Tools::calcVelocityForMaxNormalAcceleration(curve_max_kappa);
            // 曲率变化率限制
            double curvature_change_rate_velocity_limitation = Tools::calcVelocityForMaxNormalJerk(curvature_max_change_rate);
            // 之后是障碍物限制
            double obstacle_velocity_limitation = AVOIDANCE_VELOCITY + std::min(1.0, min_distance_to_obs / (MAX_DISTANCE_RATIO * this->vehicle_width_)) * (0.9 * VELOCITY_THRESHOLD - AVOIDANCE_VELOCITY);

            LOG(INFO) << "曲率对于速度的限制为 " << curvature_velocity_limitation << ", 曲率变化率对于速度的限制为" << curvature_change_rate_velocity_limitation << ", 障碍物对速度的限制为 " << obstacle_velocity_limitation;

            double goal_velocity = std::min(std::min(max_allowed_velocity, std::min(curvature_velocity_limitation, obstacle_velocity_limitation)), curvature_change_rate_velocity_limitation);

            // 判断是否为脱困模式
            if (best_avoidance_path.getOutofTrapping()) {
                // 脱困模式最大速度为0.5
                goal_velocity = 0.5;
            }

            this->states_set_[StateNames::AVOIDANCE].setGoalVelocity(goal_velocity);
            // 设置最大速度限制
            this->states_set_[StateNames::AVOIDANCE].setAllowMaxVelocity(goal_velocity);
            // 设置期望保持的加速度值（无效值）
            this->states_set_[StateNames::AVOIDANCE].setVehicleDynamicPlanningExpectedAcceleration(0.0);
            // 设置是否进行保持
            this->states_set_[StateNames::AVOIDANCE].setStateMaintain(false);
            // 设置是否为脱困模式
            this->states_set_[StateNames::AVOIDANCE].setOutofTrapping(best_avoidance_path.getOutofTrapping());
            LOG(INFO) << "避障状态补充完毕，规划距离为" << static_cast<double>(this->states_set_[StateNames::AVOIDANCE].getTrajectoryLength() * LANE_GAP_DISTANCE) << "米，目标速度为" << goal_velocity << "米/秒";
            // 选中状态为避障状态
            this->choosed_state_ = this->states_set_[StateNames::AVOIDANCE];
            std::cout << "state avoidance finished" << std::endl;
        }
    }
}

// 生成低速避障路径
void DecisionMaking::SubVehicle::AvoidancePathGenerator(const Lane &lane, std::vector<DecisionMaking::AvoidancePath> &avoidance_curves) {
    // 生成中心避障轨迹和拓展避障轨迹
    std::vector<PathPlanningUtilities::Curve> curves;
    std::vector<double> frenet_offsets;
    std::vector<double> frenet_lateral_movements;
    // 首先得到参考线
    std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = lane.getLaneCoordnation();
    std::vector<TransMatrix> trans_matrixes = lane.getLaneTransMatrix();
    // 获得当前定位
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    // 得到当前曲率
    this->current_vehicle_kappa_mutex_.lock();
    current_vehicle_world_position.kappa_ = this->current_vehicle_kappa_;
    this->current_vehicle_kappa_mutex_.unlock();
    // 得到当前速度
    this->current_vehicle_movement_mutex_.lock();
    PathPlanningUtilities::VehicleMovementState current_vehicle_movement = this->current_vehicle_movement_;
    this->current_vehicle_movement_mutex_.unlock();

    LOG(INFO) << "低速规划初始坐标为" << current_vehicle_world_position.position_ << "," << current_vehicle_world_position.theta_ << "," << current_vehicle_world_position.kappa_;

    // 判断当前速度是否为0
    bool is_outof_trapping_mode_available = false;
    if (Tools::isZero(current_vehicle_movement.velocity_) && this->stop_count_recorder_ >= MAX_STOP_COUNT && this->is_avoidance_capable_) {
        // 进入脱困模式
        LOG(INFO) << "可以进入脱困模式";
        is_outof_trapping_mode_available = true;
    } else if (Tools::isZero(current_vehicle_movement.velocity_) && this->ROTATE_AND_REVERSE_ENABLE_FLAG_) {
        // 特殊情况,可以进行脱困模式
        LOG(INFO) << "可以进入脱困模式";
    }

    std::vector<double> init_curvatures;
    if (is_outof_trapping_mode_available) {
        // 脱困模式
        init_curvatures = {-0.9 * MAX_CURVATURE, -0.5 * MAX_CURVATURE, 0.0, current_vehicle_world_position.kappa_, 0.5 * MAX_CURVATURE, 0.9 * MAX_CURVATURE};
    } else {
        // 正常模式
        init_curvatures = {current_vehicle_world_position.kappa_};
    }

    for (auto init_curvature: init_curvatures) {
        bool is_outof_trapping_mode = false;
        if (!Tools::isZero(std::abs(init_curvature - current_vehicle_world_position.kappa_))) {
            // 当前处于脱困模式
            is_outof_trapping_mode = true;
            // 重新计算当前曲率和朝向
            // 首先给出曲率对应的新朝向角
            double rear_yaw = Tools::centerYawToRearYaw(current_vehicle_world_position.theta_, current_vehicle_world_position.kappa_, DISTANCE_FROM_REAR_TO_CENTER);
            // 计算对应的后轴中心曲率
            double rear_curvature = Tools::centerToRearCurvature(init_curvature, DISTANCE_FROM_REAR_TO_CENTER);
            // 计算新曲率对应的几何中心角
            double new_center_yaw = Tools::rearYawToCenterYaw(rear_yaw, rear_curvature, DISTANCE_FROM_REAR_TO_CENTER);
            LOG(INFO) << "目标曲率" << init_curvature << ", 后轴朝向角" << rear_yaw << ", 后轴曲率" << rear_curvature << ", 几何中心朝向角" << new_center_yaw;
            // 设置新的朝向和曲率
            current_vehicle_world_position.theta_ = new_center_yaw; 
            current_vehicle_world_position.kappa_ = init_curvature;
            LOG(INFO) << "脱困模式";
            LOG(INFO) << "低速规划起点为" << current_vehicle_world_position.position_ << "," << current_vehicle_world_position.theta_;
        } else {
            // 处于正常模式
            // 读取历史路径
            this->history_curve_mutex_.lock();
            PathPlanningUtilities::Curve history_curve = this->history_curve_;
            this->history_curve_mutex_.unlock();
            if (history_curve.size() > 0) {
                size_t nearest_idx = Tools::findNearestPositionIndexInCurve(history_curve, current_vehicle_world_position.position_);
                double distance_error_to_history = PathPlanningUtilities::calcDistance(current_vehicle_world_position.position_, history_curve[nearest_idx].position_);
                double yaw_error_to_history = std::abs(current_vehicle_world_position.theta_ - history_curve[nearest_idx].theta_);
                LOG(INFO) << "低速状态与历史路径的距离差为" << distance_error_to_history << ", 角度差为" << yaw_error_to_history;
                if (Tools::isSmall(distance_error_to_history, 0.2) && Tools::isSmall(yaw_error_to_history, 0.2)) {
                    current_vehicle_world_position.position_ = history_curve[nearest_idx].position_;
                    current_vehicle_world_position.theta_ = history_curve[nearest_idx].theta_;
                    current_vehicle_world_position.kappa_ = history_curve[nearest_idx].kappa_;
                }
            }
            LOG(INFO) << "低速规划起点为" << current_vehicle_world_position.position_ << "," << current_vehicle_world_position.theta_;
        }

        // 得到当前定位在frenet坐标系起点
        size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, current_vehicle_world_position.position_);
        Eigen::Vector2d start_point_in_world_v2d(current_vehicle_world_position.position_.x_, current_vehicle_world_position.position_.y_);
        // LOG(INFO) << "世界坐标系的朝向" << current_vehicle_world_position.theta_ << ", 对应基线的朝向" << lane_coordination[start_index_of_lane].worldpos_.theta_;

        Eigen::Vector2d start_point_position_in_frenet;
        Tools::transferPointCoordinateSystem(trans_matrixes[start_index_of_lane], start_point_in_world_v2d, &start_point_position_in_frenet);
        double start_point_theta_in_frenet;
        Tools::transferThetaCoordinateSystem(trans_matrixes[start_index_of_lane], current_vehicle_world_position.theta_, &start_point_theta_in_frenet);
        // LOG(INFO) << "转变后的朝向" << start_point_theta_in_frenet;
        double start_point_kappa_in_frenet;
        start_point_kappa_in_frenet = current_vehicle_world_position.kappa_ - cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*cos(start_point_theta_in_frenet)*1.0/(1.0/lane_coordination[start_index_of_lane].worldpos_.kappa_ - start_point_position_in_frenet(1));
        PathPlanningUtilities::VehicleState start_point_in_frenet;
        start_point_in_frenet.position_.x_ = start_point_position_in_frenet(0);
        start_point_in_frenet.position_.y_ = start_point_position_in_frenet(1);
        start_point_in_frenet.theta_ = start_point_theta_in_frenet;
        start_point_in_frenet.kappa_ = start_point_kappa_in_frenet;

        // 得到frenet坐标系下的终点集合
        double aim_motion_planning_length;
        // 首先得到一个初始的规划距离
        if (this->stop_count_recorder_ <= MAX_STOP_COUNT) {
            aim_motion_planning_length = Tools::normalMotionDistance(current_vehicle_movement.velocity_, SUBVEHICLE_COMMON_DECCELERATION, CONSTANT_DISTANCE * 0.7);
        } else {
            // 一直处于停车状态之中,存在问题,对规划长度进行调整
            aim_motion_planning_length = Tools::normalMotionDistance(current_vehicle_movement.velocity_, SUBVEHICLE_COMMON_DECCELERATION, CONSTANT_DISTANCE * 0.7);
            aim_motion_planning_length = aim_motion_planning_length * (0.8 + 0.3 * (2 * MAX_STOP_COUNT - this->stop_count_recorder_) / MAX_STOP_COUNT);
        }
        LOG(INFO) << "avoidance path max planning distance: " << aim_motion_planning_length;

        // 之后开始纵向拓展，拓展从0.3到1.2，每隔0.3一个
        double lateral_gap = EXTEND_FACTOR_GAP;
        if (!this->is_avoidance_capable_) {
            lateral_gap = MAX_CURVE_EXTEND_FACTOR * 0.9;
        }
        if (is_outof_trapping_mode) {
            lateral_gap = 0.2;
        }
        for (double factor = lateral_gap; !Tools::isLarge(factor, MAX_CURVE_EXTEND_FACTOR); factor += lateral_gap) {
            // 得到最终的规划距离
            double motion_planning_length = aim_motion_planning_length * factor;
            assert(start_index_of_lane + static_cast<int>(motion_planning_length/LANE_GAP_DISTANCE) < lane_coordination.size() - 1);
            // LOG(INFO) << "规划长度" << motion_planning_length;
            // 得到对应下标
            size_t goal_index_of_lane = start_index_of_lane + static_cast<int>(motion_planning_length/LANE_GAP_DISTANCE);
            // 纵向距离已经确定，开始确定横向距离
            PathPlanningUtilities::BoundedCurvePoint goal_with_range;
            goal_with_range.center_point_.position_.x_ = lane.getLaneCenterPathInFrenet()[goal_index_of_lane].x_;
            goal_with_range.center_point_.position_.y_ = 0.0;
            goal_with_range.center_point_.theta_ = 0.0;
            goal_with_range.center_point_.kappa_ = 0.0;
            // 判断是否允许蔽障
            if (this->is_avoidance_capable_) {
                goal_with_range.left_distance_ = std::fabs(lane_coordination[goal_index_of_lane].max_height_);
                goal_with_range.right_distance_ = std::fabs(lane_coordination[goal_index_of_lane].min_height_);
            } else {
                goal_with_range.left_distance_ = 0.1;
                goal_with_range.right_distance_ = 0.1;
            }

            // 生成多种横向偏移的目标点
            std::vector<PathPlanningUtilities::CurvePoint> goal_set_frenet;
            Tools::goalSetGenerate(goal_with_range, AVOIDANCE_DEVIATION_GAP, static_cast<int>(MAX_BIAS / AVOIDANCE_DEVIATION_GAP), &goal_set_frenet);
            // 用quintic polynomial方法进行路径组生成并完成world系投影
            for (size_t k = 0; k < goal_set_frenet.size(); k++) {
                // 首先生成目标点
                // 得到当前位置对应的映射
                PathPlanningUtilities::CurvePoint goal_point = goal_set_frenet[k];

                // quintic polynomial方法路径生成
                PathPlanningUtilities::Curve trajectory;
                PathPlanningUtilities::QParameters params;
                double distance = PathPlanningUtilities::calcDistance(start_point_in_frenet.position_, goal_point.position_);
                PathPlanningUtilities::QuinticSplinePlanning::getParametersByMileageConstraint(start_point_in_frenet, goal_point, distance, 5, params);
                int point_number = goal_index_of_lane - start_index_of_lane + 1;
                // LOG(INFO) << "规划起点" << start_point_in_frenet.position_ << "," << start_point_in_frenet.theta_ << "," << start_point_in_frenet.kappa_;
                // LOG(INFO) << "规划终点" << goal_point;
                // LOG(INFO) << "起点下标" << start_index_of_lane << ", 终点下标" << goal_index_of_lane;
                // LOG(INFO) << "距离" << distance;
                PathPlanningUtilities::pathGenerator* pg_ = new PathPlanningUtilities::pathGenerator(point_number);
                pg_->calcCurve(params, trajectory);

                // 以路径最后一个点作为延伸段路径的起始点
                PathPlanningUtilities::VehicleState trajectory_last_point;
                trajectory_last_point.position_ = trajectory[trajectory.size() - 1].position_;
                trajectory_last_point.theta_ = trajectory[trajectory.size() - 1].theta_;
                trajectory_last_point.kappa_ = trajectory[trajectory.size() - 1].kappa_;
                // 得到延伸段路径最后一个点
                // LOG(INFO) << "路径的实际长度" << motion_planning_length << ", 补全长度" << static_cast<int>(std::max(MAX_CURVE_EXTEND_FACTOR * aim_motion_planning_length - motion_planning_length, 0.0) / LANE_GAP_DISTANCE) << "，拓展长度" << static_cast<int>(EXTENDED_DISTANCE_FOR_LOW_SPEED / LANE_GAP_DISTANCE);
                size_t extended_curve_latex_point_index = goal_index_of_lane + static_cast<int>(std::max(MAX_CURVE_EXTEND_FACTOR * aim_motion_planning_length - motion_planning_length, 0.0) / LANE_GAP_DISTANCE);
                assert(extended_curve_latex_point_index < lane_coordination.size());
                PathPlanningUtilities::Curve extended_trajectory;
                // 判断拓展长度是否足够长
                if (Tools::isLarge((extended_curve_latex_point_index - goal_index_of_lane) * LANE_GAP_DISTANCE, EXTEND_FACTOR_GAP * aim_motion_planning_length * 0.9)) {
                    // 得到最后一个点坐标
                    PathPlanningUtilities::CurvePoint extended_goal_point;
                    extended_goal_point.position_.x_ = lane_coordination[extended_curve_latex_point_index].station_;
                    extended_goal_point.position_.y_ = goal_set_frenet[k].position_.y_;
                    extended_goal_point.theta_ = 0.0;
                    extended_goal_point.kappa_ = 0.0;
                    // 生成路径
                    distance = PathPlanningUtilities::calcDistance(trajectory_last_point.position_, extended_goal_point.position_);
                    // 判断如果距离过小,过滤
                    if (Tools::isSmall(distance, 0.6 * static_cast<double>(extended_curve_latex_point_index - goal_index_of_lane) * LANE_GAP_DISTANCE)) {
                        continue;
                    }
                    PathPlanningUtilities::QuinticSplinePlanning::getParametersByMileageConstraint(trajectory_last_point, extended_goal_point, distance, 5, params);
                    point_number = extended_curve_latex_point_index - goal_index_of_lane + 1;
                    pg_ = new PathPlanningUtilities::pathGenerator(point_number);
                    pg_->calcCurve(params, extended_trajectory);
                }
                // 得到最终路径(路径和延伸的拼接)
                if (extended_trajectory.size() > 0) {
                    trajectory.insert(trajectory.end(), extended_trajectory.begin() + 1, extended_trajectory.end());
                }
                // 判断路径两点之间的间距(DEGUB)
                // for (size_t ll = 0; ll < trajectory.size() - 1; ll++) {
                //     double dis = PathPlanningUtilities::calcDistance(trajectory[ll].position_, trajectory[ll + 1].position_);
                //     if (dis < 0.05) {
                //         LOG(INFO) << ll << "'s distance is small: " << dis;
                //     }
                // }

                // 将路径转化到world坐标系下
                PathPlanningUtilities::Curve trajectory_world;
                for (size_t i = 0; i < trajectory.size(); i++) {
                    PathPlanningUtilities::CurvePoint point_in_frenet = trajectory[i];
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
                    trajectory_world.push_back(point_in_world);
                }
                // 保存路径
                curves.push_back(trajectory_world);
                // 保存离中心偏移量
                frenet_offsets.push_back(goal_set_frenet[k].position_.y_);
                // 保存横向移动距离
                frenet_lateral_movements.push_back(std::abs(start_point_in_frenet.position_.y_ - goal_set_frenet[k].position_.y_));
            }
        }
        // 得到当前障碍物情况
        this->obstacle_mutex_.lock();
        std::vector<Obstacle> obstacles = this->obstacles_;
        this->obstacle_mutex_.unlock();
        // 计算最大可行曲率
        double allowable_max_curvature = MAX_CURVATURE;
        // according to velocity mini max curvature
        double allowable_curvature_for_velocity = MAX_NORMAL_ACCELERATION / std::max(EPS, (current_vehicle_movement.velocity_ * current_vehicle_movement.velocity_));
        allowable_max_curvature = std::min(allowable_max_curvature, allowable_curvature_for_velocity);

        double total_curves_min_curvature = MAX_VALUE;
        // 计算全部路径最大曲率的最小值
        for (size_t i = 0; i < curves.size(); i++) {
            // 计算路径的最大曲率
            double max_curvature = curves[i][Tools::calcMaxKappaIndexForCurve(curves[i])].kappa_;
            if (Tools::isSmall(std::abs(max_curvature), std::abs(total_curves_min_curvature))) {
                total_curves_min_curvature = max_curvature;
            }
        }
        LOG(INFO) << "全体路径最小曲率" << total_curves_min_curvature;

        // 计算横向偏移为0时的最小曲率路径
        size_t min_max_curvature_index = 0;
        double min_max_curvature = MAX_VALUE;
        for (size_t i = 0; i < curves.size(); i++) {
            if (Tools::isZero(frenet_offsets[i])) {
                double max_curvature = curves[i][Tools::calcMaxKappaIndexForCurve(curves[i])].kappa_;
                if (Tools::isSmall(std::abs(max_curvature), std::abs(min_max_curvature))) {
                    min_max_curvature = max_curvature;
                    min_max_curvature_index = i;
                }
            }
        }

        LOG(INFO) << "无横向偏移路径最小曲率" << min_max_curvature;

        bool is_emergency = false;
        // 判断最小曲率是否大于曲率限制
        if (!Tools::isSmall(std::abs(total_curves_min_curvature), allowable_max_curvature)) {
            is_emergency = true;
        }

        // 完成路径生成后,开始进行路径补全
        for (size_t i = 0; i < curves.size(); i++) {
            // 计算路径的最大曲率
            double max_curvature = curves[i][Tools::calcMaxKappaIndexForCurve(curves[i])].kappa_;

            if (Tools::isLarge(fabs(max_curvature), allowable_max_curvature) && this->is_avoidance_capable_) {
                // 大于最大曲率,不进行考虑
                if (!(is_emergency && i == min_max_curvature_index)) {
                    continue;
                }
            }
            // 用于障碍物距离计算的采样间隔
            int sample_gap;
            if (is_outof_trapping_mode) {
                sample_gap = static_cast<int>(0.05 * (this->vehicle_length_ / LANE_GAP_DISTANCE));
            } else {
                sample_gap = static_cast<int>(0.25 * (this->vehicle_length_ / LANE_GAP_DISTANCE));
            }
            
            // 初始化蔽障路径
            DecisionMaking::AvoidancePath avoidance_path = DecisionMaking::AvoidancePath(curves[i], frenet_offsets[i], frenet_lateral_movements[i], PRIORITY_INIT_VALUE_MAX - lane.getLanePriority(), sample_gap);
            // 设置是否为脱困模式
            avoidance_path.setOutofTrapping(is_outof_trapping_mode);
            // 得到用于障碍物距离计算的路径
            PathPlanningUtilities::Curve collision_risk_sampled_curve = avoidance_path.getSampledTrajectory();
            // 得到采样下标
            std::vector<size_t> sample_indexes = avoidance_path.getSampleIndexes();
            // 开始进行与障碍物的距离计算(只考虑静态障碍物)
            std::vector<double> sampled_obstacle_collision_risk;
            RSS::obstacleDistanceJudgment(collision_risk_sampled_curve, this->vehicle_width_, this->vehicle_length_, current_vehicle_movement.velocity_, obstacles, &sampled_obstacle_collision_risk);
            assert(sampled_obstacle_collision_risk.size() == sample_indexes.size());
            // 记录碰撞风险
            std::vector<double> obstacle_collision_risk(curves[i].size(), 0.0);
            assert(obstacle_collision_risk.size() - 1 == sample_indexes[sample_indexes.size() - 1]);
            for (size_t j = 0; j < sample_indexes.size() - 1; j++) {
                if (Tools::isZero(sampled_obstacle_collision_risk[j])) {
                    break;
                }
                for (size_t k = sample_indexes[j]; k < sample_indexes[j + 1]; k++) {
                    obstacle_collision_risk[k] = sampled_obstacle_collision_risk[j];
                }
            }
            obstacle_collision_risk[obstacle_collision_risk.size() - 1] = sampled_obstacle_collision_risk[sampled_obstacle_collision_risk.size() - 1];

            // 对碰撞点需要进行精细化处理
            auto min_obs_collision_risk = std::min_element(obstacle_collision_risk.begin(), obstacle_collision_risk.end());
            if (Tools::isZero(*min_obs_collision_risk)) {
                // 表明发生碰撞
                int zero_index = std::distance(obstacle_collision_risk.begin(), min_obs_collision_risk);
                // 得到初始点
                int start_index, end_index;
                start_index = std::max(0, zero_index - sample_gap);
                end_index = std::min(static_cast<int>(obstacle_collision_risk.size()) - 1, zero_index + sample_gap);
                assert(end_index > start_index);
                // 得到路径片段
                PathPlanningUtilities::Curve segment;
                segment.assign(avoidance_path.getTrajectory().begin() + start_index, avoidance_path.getTrajectory().begin() + end_index);
                // 进行计算
                size_t segment_index = 0;
                RSS::collisionPositionIndexInCurve(segment, this->vehicle_width_, this->vehicle_length_, current_vehicle_movement.velocity_, obstacles, &segment_index);
                size_t truely_index = segment_index + start_index;
                for (size_t j = truely_index; j < obstacle_collision_risk.size(); j++) {
                    obstacle_collision_risk[j] = 0.0;
                }
            }

            // 开始与交通规则的距离计算
            std::vector<double> sampled_traffic_collision_risk;
            RSS::obstacleDistanceJudgment(collision_risk_sampled_curve, this->vehicle_width_, this->vehicle_length_, this->traffic_rule_obstacles_, &sampled_traffic_collision_risk);
            // 记录碰撞风险
            std::vector<double> traffic_collision_risk(curves[i].size(), 0.0);
            assert(traffic_collision_risk.size() - 1 == sample_indexes[sample_indexes.size() - 1]);
            for (size_t j = 0; j < sample_indexes.size() - 1; j++) {
                if (Tools::isZero(sampled_traffic_collision_risk[j])) {
                    break;
                }
                for (size_t k = sample_indexes[j]; k < sample_indexes[j + 1]; k++) {
                    traffic_collision_risk[k] = sampled_traffic_collision_risk[j];
                }
            }
            traffic_collision_risk[traffic_collision_risk.size() - 1] = sampled_traffic_collision_risk[sampled_traffic_collision_risk.size() - 1];

            // 对碰撞点需要进行精细化处理
            auto min_tra_collision_risk = std::min_element(traffic_collision_risk.begin(), traffic_collision_risk.end());
            if (Tools::isZero(*min_tra_collision_risk)) {
                // 表明发生碰撞
                int zero_index = std::distance(traffic_collision_risk.begin(), min_tra_collision_risk);
                // 得到初始点
                int start_index, end_index;
                start_index = std::max(0, zero_index - sample_gap);
                end_index = std::min(static_cast<int>(traffic_collision_risk.size()) - 1, zero_index + sample_gap);
                assert(end_index > start_index);
                // 得到路径片段
                PathPlanningUtilities::Curve segment;
                segment.assign(avoidance_path.getTrajectory().begin() + start_index, avoidance_path.getTrajectory().begin() + end_index);
                // 进行计算
                size_t segment_index = 0;
                RSS::collisionPositionIndexInCurve(segment, this->vehicle_width_, this->vehicle_length_, this->traffic_rule_obstacles_, &segment_index);
                size_t truely_index = segment_index + start_index;
                for (size_t j = truely_index; j < traffic_collision_risk.size(); j++) {
                    traffic_collision_risk[j] = 0.0;
                }
            }

            avoidance_path.saveCollisionInfo(obstacle_collision_risk, traffic_collision_risk);
            avoidance_curves.push_back(avoidance_path);
        }
    }
}

// 进行路径选择
size_t DecisionMaking::SubVehicle::selectAvoidancePath(const std::vector<DecisionMaking::AvoidancePath> &avoidance_paths) {
    // 选择由以下几个方面进行:
    // 安全性>通过性>偏移量>曲率
    size_t best_path_index = 0;
    if (avoidance_paths.size() == 1) {
        return best_path_index;
    }

    // 计算每一条路径的不同损失
    std::vector<double> safety_costs, length_costs, offset_costs, movement_costs, priority_costs, curvature_costs;
    safety_costs.resize(avoidance_paths.size());
    length_costs.resize(avoidance_paths.size());
    offset_costs.resize(avoidance_paths.size());
    movement_costs.resize(avoidance_paths.size());
    priority_costs.resize(avoidance_paths.size());
    curvature_costs.resize(avoidance_paths.size());

    // 首先计算安全损失
    double M = 8.0 / LANE_GAP_DISTANCE / log(10);  // 安全损失距离参量
    double N = 1.0 / log(10);
    for (size_t i = 0; i < avoidance_paths.size(); i++) {
        double collision_risk_cost = 0.0;
        // 先判断障碍物的碰撞风险
        for (size_t j = 0; j < avoidance_paths[i].getTrajectory().size(); j++) {
            // collision_risk_cost += exp(- static_cast<double>(j) / M) * pow(std::max(1.0 - avoidance_paths[i].getObsCollisionInfo()[j] / (1.0 * MAX_DISTANCE_RATIO * this->vehicle_width_), 0.0), 2.0);
            double cost = 0.0;
            if (Tools::isZero(avoidance_paths[i].getObsCollisionInfo()[j])) {
                cost = 1.0;
            } else if (Tools::isSmall(avoidance_paths[i].getObsCollisionInfo()[j], 0.8)) {
                cost = exp(- avoidance_paths[i].getObsCollisionInfo()[j] / N);
            } else {
                cost = 0.0;
            }
            collision_risk_cost += exp(- static_cast<double>(j) / M) * cost;
        }

        // 再判断交通规则的碰撞风险
        for (size_t j = 0; j < avoidance_paths[i].getTrajectory().size(); j++) {
            // collision_risk_cost += exp(- static_cast<double>(j) / M) * pow(std::max(1.0 - avoidance_paths[i].getTraCollisionInfo()[j] / (0.5 * MAX_DISTANCE_RATIO * this->vehicle_width_), 0.0), 2.0);
            double cost = 0.0;
            if (Tools::isZero(avoidance_paths[i].getTraCollisionInfo()[j])) {
                cost = 1.0;
            } else if (Tools::isSmall(avoidance_paths[i].getTraCollisionInfo()[j], 0.2)) {
                cost = exp(- avoidance_paths[i].getTraCollisionInfo()[j] / N);
            } else {
                cost = 0.0;
            }
            collision_risk_cost += exp(- static_cast<double>(j) / M) * cost;
        }

        // for (size_t j = 0; j < avoidance_paths[i].getTrajectory().size(); j++) {
        //     double cost = 0.0;
        //     if (Tools::isZero(avoidance_paths[i].getObsCollisionInfo()[j])) {
        //         cost = 1.0;
        //     } else if (Tools::isSmall(avoidance_paths[i].getObsCollisionInfo()[j], 0.8)) {
        //         cost = exp(- avoidance_paths[i].getObsCollisionInfo()[j] / N);
        //     } else {
        //         cost = 0.0;
        //     }
        //     collision_risk_cost += cost;
        // }
        // for (size_t j = 0; j < avoidance_paths[i].getTrajectory().size(); j++) {
        //     // collision_risk_cost += exp(- static_cast<double>(j) / M) * pow(std::max(1.0 - avoidance_paths[i].getTraCollisionInfo()[j] / (0.5 * MAX_DISTANCE_RATIO * this->vehicle_width_), 0.0), 2.0);
        //     double cost = 0.0;
        //     if (Tools::isZero(avoidance_paths[i].getTraCollisionInfo()[j])) {
        //         cost = 1.0;
        //     } else if (Tools::isSmall(avoidance_paths[i].getTraCollisionInfo()[j], 0.2)) {
        //         cost = exp(- avoidance_paths[i].getTraCollisionInfo()[j] / N);
        //     } else {
        //         cost = 0.0;
        //     }
        //     collision_risk_cost += cost;
        // }

        collision_risk_cost = collision_risk_cost / static_cast<double>(avoidance_paths[i].getTrajectory().size());
        safety_costs[i] = collision_risk_cost;
    }

    // 计算长度损失（首先对长度进行分级，长度差异小于2米的同样的）
    

    for (size_t i = 0; i < avoidance_paths.size(); i++) {
        double length_cost = 0.0;
        // if (Tools::isSmall(static_cast<double>(avoidance_paths[i].getCutIndex() * LANE_GAP_DISTANCE), 16.0)) {
        //     length_cost = 1.0 - static_cast<double>(avoidance_paths[i].getCutIndex()) / static_cast<double>(avoidance_paths[i].getTrajectory().size() - 1);
        // } else {
        //     length_cost = 0.0;
        // }
        length_cost = std::floor((1.0 - static_cast<double>(avoidance_paths[i].getCutIndex()) / static_cast<double>(avoidance_paths[i].getTrajectory().size() - 1)) * 5.0)/ 5.0;
        length_costs[i] = length_cost;
    }

    // 计算偏移量损失
    for (size_t i = 0; i < avoidance_paths.size(); i++) {
        double offset_cost = avoidance_paths[i].getLateralOffset() * avoidance_paths[i].getLateralOffset();
        offset_costs[i] = offset_cost;
    }

    // 计算移动损失
    for (size_t i = 0; i < avoidance_paths.size(); i++) {
        double movement_cost = avoidance_paths[i].getLateralMovement() * avoidance_paths[i].getLateralMovement();
        movement_costs[i] = movement_cost;
    }

    // 计算曲率损失
    for (size_t i = 0; i < avoidance_paths.size(); i++) {
        double cuvature_cost = avoidance_paths[i].getMaxKappa() * avoidance_paths[i].getMaxKappa();
        curvature_costs[i] = cuvature_cost;
    }

    // 计算优先级损失
    for (size_t i = 0; i < avoidance_paths.size(); i++) {
        double priority_cost = avoidance_paths[i].getPriority();
        priority_costs[i] = priority_cost;
    }
    // 完成损失计算后,对每种损失进行聚类分析,将其分为好,中,差三种类别
    std::vector<std::vector<size_t>> safety_evaluations, length_evaluations, offset_evaluations, movement_evaluations, priority_evaluations, curvature_evaluations;

    // 聚类数量
    int k_means = 3;

    std::vector<double> safety_centers = Tools::kMeans(safety_costs, k_means, safety_evaluations);
    std::vector<double> length_centers = Tools::kMeans(length_costs, k_means, length_evaluations);
    std::vector<double> offset_centers = Tools::kMeans(offset_costs, k_means, offset_evaluations);
    std::vector<double> movement_centers = Tools::kMeans(movement_costs, k_means, movement_evaluations);
    std::vector<double> priority_centers = Tools::kMeans(priority_costs, k_means, priority_evaluations);
    std::vector<double> curvature_centers = Tools::kMeans(curvature_costs, k_means, curvature_evaluations);

    // 调试信息
    for (int i = 0; i < k_means; i++) {
        LOG(INFO) << "safety centers " << i << ": " << safety_centers[i] << ", size is " << safety_evaluations[i].size();
        LOG(INFO) << "length centers " << i << ": " << length_centers[i] << ", size is " << length_evaluations[i].size();
        LOG(INFO) << "offset centers " << i << ": " << offset_centers[i] << ", size is " << offset_evaluations[i].size();
        LOG(INFO) << "movement centers " << i << ": " << movement_centers[i] << ", size is " << movement_evaluations[i].size();
        LOG(INFO) << "priority centers " << i << ": " << priority_centers[i] << ", size is " << priority_evaluations[i].size();
        LOG(INFO) << "curvature centers " << i << ": " << curvature_centers[i] << ", size is " << curvature_evaluations[i].size();
    }

    // 开始对每一条路径进行评估
    std::vector<double> final_evaluations(avoidance_paths.size(), 0.0);

    // 安全性评价
    size_t total_length = 0;
    for (size_t i = 0; i < safety_evaluations.size(); i++) {
        total_length += safety_evaluations[i].size();
        for (size_t j = 0; j < safety_evaluations[i].size(); j++) {
            final_evaluations[safety_evaluations[i][j]] += 1000.0 * i;            
        }
    }
    assert(total_length == avoidance_paths.size());

    // 长度评价
    total_length = 0;
    for (size_t i = 0; i < length_evaluations.size(); i++) {
        total_length += length_evaluations[i].size();
        for (size_t j = 0; j < length_evaluations[i].size(); j++) {
            final_evaluations[length_evaluations[i][j]] += 100.0 * i;
        }
    }
    assert(total_length == avoidance_paths.size());

    // 偏移量评价
    total_length = 0;
    for (size_t i = 0; i < offset_evaluations.size(); i++) {
        total_length += offset_evaluations[i].size();
        for (size_t j = 0; j < offset_evaluations[i].size(); j++) {
            final_evaluations[offset_evaluations[i][j]] += 1.0 * i;
        }
    }
    assert(total_length == avoidance_paths.size());

    // 移动量评价
    total_length = 0;
    for (size_t i = 0; i < movement_evaluations.size(); i++) {
        total_length += movement_evaluations[i].size();
        for (size_t j = 0; j < movement_evaluations[i].size(); j++) {
            final_evaluations[movement_evaluations[i][j]] += 1.0 * i;
        }
    }
    assert(total_length == avoidance_paths.size());

    // 优先级评价
    total_length = 0;
    for (size_t i = 0; i < priority_evaluations.size(); i++) {
        total_length += priority_evaluations[i].size();
        for (size_t j = 0; j < priority_evaluations[i].size(); j++) {
            final_evaluations[priority_evaluations[i][j]] += 10.0 * i;
        }
    }
    assert(total_length == avoidance_paths.size());

    // 曲率评价
    total_length = 0;
    for (size_t i = 0; i < curvature_evaluations.size(); i++) {
        total_length += curvature_evaluations[i].size();
        for (size_t j = 0; j < curvature_evaluations[i].size(); j++) {
            final_evaluations[curvature_evaluations[i][j]] += 0.1 * i;
        }
    }
    assert(total_length == avoidance_paths.size());

    // 再次根据偏移量和曲率对路径权重进行精细化选择
    for (size_t i = 0; i < final_evaluations.size(); i++) {
        final_evaluations[i] += 0.01 * safety_costs[i] + 0.001 * length_costs[i] + 0.0001 * offset_costs[i] + 0.0001 * movement_costs[i] + 0.00001 * curvature_costs[i];
    }


    // 得到最优路径
    auto min_cost_iter = std::min_element(final_evaluations.begin(), final_evaluations.end());
    best_path_index = std::distance(std::begin(final_evaluations), min_cost_iter);

    // 判断是否存在无碰撞路径
    for (auto avoidance_path: avoidance_paths) {
        if (avoidance_path.isCollision() == false) {
            LOG(INFO) << "存在无碰撞路径";
            break;
        }
    }

    // 记录最优路径信息
    LOG(INFO) << "best avoidance path index: " << best_path_index;
    LOG(INFO) << "is best avoidance path collision: " << avoidance_paths[best_path_index].isCollision();
    LOG(INFO) << "best avoidance path influence type" << avoidance_paths[best_path_index].getInfluenceType();
    LOG(INFO) << "best avoidance path total index length: " << avoidance_paths[best_path_index].getTrajectory().size();
    LOG(INFO) << "best avoidance path static cut index: " << avoidance_paths[best_path_index].getCutIndex();
    LOG(INFO) << "best avoidance path min distance to obs: " << avoidance_paths[best_path_index].getMinDistanceToObstacle();
    LOG(INFO) << "best avoidance path lateral offset: " << avoidance_paths[best_path_index].getLateralOffset();
    LOG(INFO) << "best avoidance path lateral movement: " << avoidance_paths[best_path_index].getLateralMovement();
    LOG(INFO) << "best avoidance path priority: " << avoidance_paths[best_path_index].getPriority();
    LOG(INFO) << "best avoidance path max curvature: " << avoidance_paths[best_path_index].getMaxKappa();
    LOG(INFO) << "best avoidance path safety cost: " << safety_costs[best_path_index];
    LOG(INFO) << "best avoidance path length cost: " << length_costs[best_path_index];
    LOG(INFO) << "best avoidance path evaluation: " << final_evaluations[best_path_index];

    return best_path_index;
}