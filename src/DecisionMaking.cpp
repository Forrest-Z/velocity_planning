/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

#define random(x) (rand()%x)
// Debug 可视化碰撞点
extern std::vector<Rectangle> collision_rectangles;

// 判断状态的安全性
// 根据优先级高低查询障碍物对路径影响、加入观察距离内是否存在静态障碍物、最高速度约束、速度变化约束等，计算出可行速度,如果没有可行速度则放弃
// 静态障碍物出现在路径上则为换道直接放弃、高速变低速、低速变蔽障。
// 详解：如果换道最优先，则判断其路径以及观测距离内有无静态障碍物，再判断是否存在一定速度区间实现安全换道；
// 如果直行最优先，判断路径及观测距离内有无静态障碍物，如果有，高速进入低速模式，如果是再路径内则低速变为蔽障模式。
void DecisionMaking::SubVehicle::checkStates() {

    // 具体实现
    // 0.障碍物信息补全（位置、速度、加速度、朝向、大小、属性、类别从传感器中读取出来，预测轨迹从地图中获取）
    this->updateObstacleInformation();
    std::cout << "OBSTACLE INFOMATION UPDATED" << std::endl;
    LOG(INFO) << "OBSTACLE INFOMATION UPDATED";
    // 1. 开启三个线程进行不同车道的处理，每个线程对不同车道进行速度动态规划
    // 1-a.先建立中间道速度规划线程(TOFIX)
    this->obstacle_mutex_.lock();
    std::vector<Obstacle> obstacles = this->obstacles_;
    this->obstacle_mutex_.unlock();

    // 判断每个状态对应道路是否被障碍物占据
    this->judgeStateCorrespondingLaneBeingOccupied(&(this->states_set_[StateNames::FORWARD]), obstacles);
    this->judgeStateCorrespondingLaneBeingOccupied(&(this->states_set_[StateNames::TURN_LEFT]), obstacles);
    this->judgeStateCorrespondingLaneBeingOccupied(&(this->states_set_[StateNames::TURN_RIGHT]), obstacles);
    // std::thread center_lane_velocity_planning_thread(DecisionMaking::RSS::velocityPlanningForState, &this->states_set_[StateNames::FORWARD], obstacles);

    // // 1-b.再建立左道速度规划线程(TOFIX)
    // std::thread left_lane_velocity_planning_thread(DecisionMaking::RSS::velocityPlanningForState, &this->states_set_[StateNames::TURN_LEFT], obstacles);

    // // 1-c.最后建立右道速度规划线程(TOFIX)
    // std::thread right_lane_velocity_planning_thread(DecisionMaking::RSS::velocityPlanningForState, &this->states_set_[StateNames::TURN_RIGHT], obstacles);

    // // 阻塞父线程直到上述三个线程完全结束
    // center_lane_velocity_planning_thread.join();
    // left_lane_velocity_planning_thread.join();
    // right_lane_velocity_planning_thread.join();

    // // debug不使用线程
    // this->velocityPlanningForState(&(this->states_set_[StateNames::FORWARD]), obstacles, true);
    // this->velocityPlanningForState(&(this->states_set_[StateNames::TURN_LEFT]), obstacles, true);
    // this->velocityPlanningForState(&(this->states_set_[StateNames::TURN_RIGHT]), obstacles, true);




    // DEBUG
    // std::cout << "DEBUG DEBUG DEBUG" << std::endl;
    // std::cout << "DEBUG SIZE: " << states_set_[StateNames::FORWARD].last_planned_curve_.size() << std::endl;
    // std::cout << "DEBUG s size: " << states_set_[StateNames::FORWARD].s_.size() << std::endl;
    // std::cout << "DEBUG v size: " << states_set_[StateNames::FORWARD].v_.size() << std::endl;
    // std::cout << "DEBUG a size: " << states_set_[StateNames::FORWARD].a_.size() << std::endl;
    // Record time consumption
    clock_t start_time = clock();
    VelocityPlanning::VelocityPlanner* v_planner_forward = new VelocityPlanning::VelocityPlanner(&(states_set_[StateNames::FORWARD]));
    v_planner_forward->runOnce(obstacles);

    VelocityPlanning::VelocityPlanner* v_planner_left = new VelocityPlanning::VelocityPlanner(&(states_set_[StateNames::TURN_LEFT]));
    v_planner_left->runOnce(obstacles);

    VelocityPlanning::VelocityPlanner* v_planner_right = new VelocityPlanning::VelocityPlanner(&(states_set_[StateNames::TURN_RIGHT]));
    v_planner_right->runOnce(obstacles);

    clock_t end_time = clock();
    double time_consumption = static_cast<double>((end_time - start_time)) / CLOCKS_PER_SEC;
    std::cout << "Time consumption: " << time_consumption << std::endl;


    // END DEBUG

    // 设置三大状态都不可行(for debug)
    // this->states_set_[StateNames::FORWARD].disable();
    // this->states_set_[StateNames::TURN_LEFT].disable();
    // this->states_set_[StateNames::TURN_RIGHT].disable();

    // 判断是否由障碍物导致全部路径不安全
    if (this->states_set_[StateNames::FORWARD].getCapability() && !this->states_set_[StateNames::FORWARD].getSafety()) {
        this->IS_DANGEROUS_FLAG_ = true;
    } else if (this->states_set_[StateNames::TURN_LEFT].getCapability() && !this->states_set_[StateNames::TURN_LEFT].getSafety()) {
        this->IS_DANGEROUS_FLAG_ = true;
    } else if (this->states_set_[StateNames::TURN_RIGHT].getCapability() && !this->states_set_[StateNames::TURN_RIGHT].getSafety()) {
        this->IS_DANGEROUS_FLAG_ = true;
    } else {
        this->IS_DANGEROUS_FLAG_ = false;
    }

    LOG(INFO) << "是否有障碍物导致路径不安全:" << this->IS_DANGEROUS_FLAG_;

    // 根据交通规则进行路径可行性判断，如果路径与永久交通障碍物相交说明路径不可行，如果与非永久交通规则障碍物相交则说明路径不安全
    this->updateValidateTrafficRuleInformation();
    DecisionMaking::RSS::trafficRuleCheck(&(this->states_set_[StateNames::FORWARD]), this->traffic_rule_obstacles_);
    DecisionMaking::RSS::trafficRuleCheck(&(this->states_set_[StateNames::TURN_LEFT]), this->traffic_rule_obstacles_);
    DecisionMaking::RSS::trafficRuleCheck(&(this->states_set_[StateNames::TURN_RIGHT]), this->traffic_rule_obstacles_);

    if (this->IS_SURROUND_RADAR_ENABLE_FLAG_) {
        // 根据毫米波预警判断是否能够进行换道
        this->left_alert_mutex_.lock();
        bool is_left_alert = this->left_alert_;
        this->left_alert_mutex_.unlock();
        if (is_left_alert) {
            this->states_set_[StateNames::TURN_LEFT].setSafety(false);
            LOG(INFO) << "左侧毫米波报警，禁止换道";
        }
        this->right_alert_mutex_.lock();
        bool is_right_alert = this->right_alert_;
        this->right_alert_mutex_.unlock();
        if (is_right_alert) {
            this->states_set_[StateNames::TURN_RIGHT].setSafety(false);
            LOG(INFO) << "右侧毫米波报警，禁止换道";
        }
    }

    // check capability of states
    // 判断上一个状态是否完成，如果未能完成，将三大状态全部设置为不可行
    if (this->current_state_.getStateCompleted() == false) {
        LOG(INFO) << "上一个状态未能完成，无法进行重新规划";
        this->states_set_[StateNames::FORWARD].disable();
        this->states_set_[StateNames::TURN_LEFT].disable();
        this->states_set_[StateNames::TURN_RIGHT].disable();
    }

    // 判断状态是否为上一个状态的邻居，如果不是，也设置为不可行
    if (!(this->current_state_.getStateName() == StateNames::TURN_RIGHT || Tools::searchNeighborStates(this->current_state_.getNeighborStates(), StateNames::TURN_RIGHT))) {
        LOG(INFO) << "不是当前状态的邻居，拒绝右换道";
        this->states_set_[StateNames::TURN_RIGHT].disable();
    }
    if (!(this->current_state_.getStateName() == StateNames::TURN_LEFT || Tools::searchNeighborStates(this->current_state_.getNeighborStates(), StateNames::TURN_LEFT))) {
        LOG(INFO) << "不是当前状态的邻居，拒绝左换道";
        this->states_set_[StateNames::TURN_LEFT].disable();
    }

    std::cout << "VELOCITY PLANNING COMPLETE" << std::endl;
    std::cout << "forward state capability is " << this->states_set_[StateNames::FORWARD].getCapability() << ", and safety is " << this->states_set_[StateNames::FORWARD].getSafety() << ", and priority is " << this->states_set_[StateNames::FORWARD].getPriority() << std::endl;
    std::cout << "left state capability is " << this->states_set_[StateNames::TURN_LEFT].getCapability() << ", and safety is " << this->states_set_[StateNames::TURN_LEFT].getSafety() << ", and priority is " << this->states_set_[StateNames::TURN_LEFT].getPriority() << std::endl;
    std::cout << "right state capability is " << this->states_set_[StateNames::TURN_RIGHT].getCapability() << ", and safety is " << this->states_set_[StateNames::TURN_RIGHT].getSafety() << ", and priority is " << this->states_set_[StateNames::TURN_RIGHT].getPriority() << std::endl;

    LOG(INFO) << "VELOCITY PLANNING COMPLETE";
    LOG(INFO) << "forward state capability is " << this->states_set_[StateNames::FORWARD].getCapability() << ", and safety is " << this->states_set_[StateNames::FORWARD].getSafety() << ", and priority is " << this->states_set_[StateNames::FORWARD].getPriority();
    LOG(INFO) << "left state capability is " << this->states_set_[StateNames::TURN_LEFT].getCapability() << ", and safety is " << this->states_set_[StateNames::TURN_LEFT].getSafety() << ", and priority is " << this->states_set_[StateNames::TURN_LEFT].getPriority();
    LOG(INFO) << "right state capability is " << this->states_set_[StateNames::TURN_RIGHT].getCapability() << ", and safety is " << this->states_set_[StateNames::TURN_RIGHT].getSafety() << ", and priority is " << this->states_set_[StateNames::TURN_RIGHT].getPriority();
}

// 进行状态选择
// 如果优先度最高的状态可行、选最高的（同时判断是否需要超车）、如果优先度最高状态不可行、进行判别,如果状态都不行则进入停车状态。
void DecisionMaking::SubVehicle::chooseStates() {
    // 0. 如果三个状态中有可行并安全的状态，选其中优先级最高的。
    // 1. 如果三个状态都不可行，且蔽障满足条件，选蔽障状态。
    // 2. 以上都不可行，选停车状态。
    // 判断是否需要进行超车，如果需要进行超车，重新根据期望加速度更新权重
    if (this->IS_OVERTAKE_ENABLE_FLAG_) {
        if (this->states_set_[StateNames::FORWARD].getCapability() && this->states_set_[StateNames::FORWARD].getSafety()) {
            bool is_forward_decceleration;
            if (Tools::isSmall(this->states_set_[StateNames::FORWARD].getStateMaxAvailableAcceleration(), STATE_CHOOSING_ACCELERATION_THRESHOLD)) {
                is_forward_decceleration = true;
            } else {
                is_forward_decceleration = false;
            }
            
            // 确定优先级增加量
            if ((this->is_length_enough_ || (this->guidance_type_ != Lane::GuidanceType::CHANGE_LEFT && this->guidance_type_ != Lane::GuidanceType::CHANGE_RIGHT))) {
                double priority_offset;
                if (!is_forward_decceleration) {
                    // 加速不阻碍
                    priority_offset = PRIORITY_INCREMENT_VALUE;
                } else {
                    // 加速被阻碍
                    priority_offset = this->states_set_[StateNames::FORWARD].getVehicleCurrentMovement().velocity_ / this->states_set_[StateNames::FORWARD].getExpectedVelocityCurrent() * PRIORITY_INCREMENT_VALUE;
                    // 判断状态对应道路是否被占据
                    if (!this->states_set_[StateNames::FORWARD].getLaneBeingOccupiedByObstacle()) {
                        // 没被占据,优先级上升
                        priority_offset += (1 - this->states_set_[StateNames::FORWARD].getVehicleCurrentMovement().velocity_ / this->states_set_[StateNames::FORWARD].getExpectedVelocityCurrent()) * PRIORITY_INCREMENT_VALUE;
                        LOG(INFO) << "中间道路咩有被障碍物占据";
                    }
                }
                
                LOG(INFO) << "中间道路优先级进行调整，原来为" << this->states_set_[StateNames::FORWARD].getPriority() << "增量为" << priority_offset;
                this->states_set_[StateNames::FORWARD].setPriority(this->states_set_[StateNames::FORWARD].getPriority() + priority_offset);
            }
            // // 加速度偏移量
            // double acceleration_priority_offset = this->states_set_[StateNames::FORWARD].getVehicleDynamicPlanningExpectedAcceleration() * 40.0;
            // this->states_set_[StateNames::FORWARD].setPriority(this->states_set_[StateNames::FORWARD].getPriority() + acceleration_priority_offset);
            // LOG(INFO) << "中间道最终优先级为" << acceleration_priority_offset;
        }
    }
    if (this->IS_OVERTAKE_ENABLE_FLAG_) {
        if (this->states_set_[StateNames::TURN_LEFT].getCapability() && this->states_set_[StateNames::TURN_LEFT].getSafety()) {
            bool is_turn_left_decceleration;
            if (Tools::isSmall(this->states_set_[StateNames::TURN_LEFT].getStateMaxAvailableAcceleration(), STATE_CHOOSING_ACCELERATION_THRESHOLD)) {
                is_turn_left_decceleration = true;
            } else {
                is_turn_left_decceleration = false;
            }        
            // 确定优先级增加量
            if ((this->is_length_enough_ || (this->guidance_type_ == Lane::GuidanceType::CHANGE_LEFT || this->guidance_type_ == Lane::GuidanceType::CENTER_LEFT || this->guidance_type_ == Lane::GuidanceType::ALL_AVAILABLE))) {
                double priority_offset;
                if (!is_turn_left_decceleration) {
                    priority_offset = PRIORITY_INCREMENT_VALUE;
                } else {
                    priority_offset = this->states_set_[StateNames::TURN_LEFT].getVehicleCurrentMovement().velocity_ / this->states_set_[StateNames::TURN_LEFT].getExpectedVelocityCurrent() * PRIORITY_INCREMENT_VALUE;
                    // 判断状态对应道路是否被占据
                    if (!this->states_set_[StateNames::TURN_LEFT].getLaneBeingOccupiedByObstacle()) {
                        // 没被占据,优先级上升
                        priority_offset += (1 - this->states_set_[StateNames::TURN_LEFT].getVehicleCurrentMovement().velocity_ / this->states_set_[StateNames::TURN_LEFT].getExpectedVelocityCurrent()) * PRIORITY_INCREMENT_VALUE;
                        LOG(INFO) << "左侧道路咩有被障碍物占据";
                    }
                }
                 
                LOG(INFO) << "左侧道路优先级进行调整，原来为" << this->states_set_[StateNames::TURN_LEFT].getPriority() << "增量为" << priority_offset;
                this->states_set_[StateNames::TURN_LEFT].setPriority(this->states_set_[StateNames::TURN_LEFT].getPriority() + priority_offset);
            }
            // // 加速度偏移量
            // double acceleration_priority_offset = this->states_set_[StateNames::TURN_LEFT].getVehicleDynamicPlanningExpectedAcceleration() * 40.0;
            // this->states_set_[StateNames::TURN_LEFT].setPriority(this->states_set_[StateNames::TURN_LEFT].getPriority() + acceleration_priority_offset);
            // LOG(INFO) << "左侧道最终优先级为" << acceleration_priority_offset;
        }
    }
    if (this->IS_OVERTAKE_ENABLE_FLAG_) {
        if (this->states_set_[StateNames::TURN_RIGHT].getCapability() && this->states_set_[StateNames::TURN_RIGHT].getSafety()) {
            bool is_turn_right_decceleration;
            if (Tools::isSmall(this->states_set_[StateNames::TURN_RIGHT].getStateMaxAvailableAcceleration(), STATE_CHOOSING_ACCELERATION_THRESHOLD)) {
                is_turn_right_decceleration = true;
            } else {
                is_turn_right_decceleration = false;
            }          
            // 确定优先级增加量
            if ((this->is_length_enough_ || this->guidance_type_ == Lane::GuidanceType::CHANGE_RIGHT || this->guidance_type_ == Lane::GuidanceType::CENTER_RIGHT || this->guidance_type_ == Lane::GuidanceType::ALL_AVAILABLE)) {
                double priority_offset;
                if (!is_turn_right_decceleration) {
                    priority_offset = PRIORITY_INCREMENT_VALUE_LOWER;
                } else {
                    priority_offset = this->states_set_[StateNames::TURN_RIGHT].getVehicleCurrentMovement().velocity_ / this->states_set_[StateNames::TURN_RIGHT].getExpectedVelocityCurrent() * PRIORITY_INCREMENT_VALUE_LOWER;
                    // 判断状态对应道路是否被占据
                    if (!this->states_set_[StateNames::TURN_RIGHT].getLaneBeingOccupiedByObstacle()) {
                        // 没被占据,优先级上升
                        priority_offset += (1 - this->states_set_[StateNames::TURN_RIGHT].getVehicleCurrentMovement().velocity_ / this->states_set_[StateNames::TURN_RIGHT].getExpectedVelocityCurrent()) * PRIORITY_INCREMENT_VALUE;
                        LOG(INFO) << "右侧道路咩有被障碍物占据";
                    }
                }
                
                LOG(INFO) << "右侧道路优先级进行调整，原来为" << this->states_set_[StateNames::TURN_RIGHT].getPriority() << "增量为" << priority_offset;
                this->states_set_[StateNames::TURN_RIGHT].setPriority(this->states_set_[StateNames::TURN_RIGHT].getPriority() + priority_offset);
            }
            // // 加速度偏移量
            // double acceleration_priority_offset = this->states_set_[StateNames::TURN_RIGHT].getVehicleDynamicPlanningExpectedAcceleration() * 40.0;
            // this->states_set_[StateNames::TURN_RIGHT].setPriority(this->states_set_[StateNames::TURN_RIGHT].getPriority() + acceleration_priority_offset);
            // LOG(INFO) << "右侧道最终优先级为" << acceleration_priority_offset;
        }
    }

    // 构建可选状态列表，包含三大状态
    std::vector<DecisionMaking::StandardState> availiable_states_set;
    availiable_states_set.push_back(this->states_set_[StateNames::FORWARD]);
    availiable_states_set.push_back(this->states_set_[StateNames::TURN_LEFT]);
    availiable_states_set.push_back(this->states_set_[StateNames::TURN_RIGHT]);
    // 开始选取优先级最高的可行安全状态
    if (!this->sortStatesPriority(&availiable_states_set)) {
        // 如果所有状态都不可行或不安全，选中直行状态（TOFIX）
        std::cout << "no state is capable and safety, enter low velocity states" << std::endl;
        LOG(INFO) << "no state is capable and safety, enter low velocity states";
        // 如果所有状态都不可行或不安全，进入低速机动模式(包括蔽障和停车)
        this->generateLowVelocityState();
        // 临时将期望状态指定为选中状态以便调试（TOFIX）
        this->expected_state_ = this->states_set_[StateNames::FORWARD];
    } else {
        // 存在可行且安全的状态
        for (size_t i = availiable_states_set.size() - 1; i >= 0; i--) {
            if (availiable_states_set[i].getCapability() && availiable_states_set[i].getSafety()) {
                this->choosed_state_ = availiable_states_set[i];
                break;
            }
        }
        // 得到期望状态
        this->expected_state_ = availiable_states_set[availiable_states_set.size() - 1];
    }

    // 开启转向灯
    if (this->choosed_state_.getStateName() == StateNames::FORWARD || this->choosed_state_.getStateName() == StateNames::TURN_LEFT || this->choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
        // if (this->choosed_state_.getRespondingLane().getTurn() == vec_map_cpp_msgs::LocalLane::LEFT) {
        //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
        //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::LEFT;
        //     this->turn_signal_pub_.publish(turn_signal_cmd);
        //     LOG(INFO) << "left turn light";
        // } else if (this->choosed_state_.getRespondingLane().getTurn() == vec_map_cpp_msgs::LocalLane::RIGHT) {
        //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
        //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::RIGHT;
        //     this->turn_signal_pub_.publish(turn_signal_cmd);
        //     LOG(INFO) << "right turn light";
        // } else if (this->choosed_state_.getRespondingLane().getTurn() == vec_map_cpp_msgs::LocalLane::THROUGH){
        if (this->choosed_state_.getStateName() == StateNames::TURN_LEFT) {
            dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::LEFT;
            this->turn_signal_pub_.publish(turn_signal_cmd);
            LOG(INFO) << "through left turn light";
        }else if (this->choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
            dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::RIGHT;
            this->turn_signal_pub_.publish(turn_signal_cmd);
            LOG(INFO) << "through right turn light";
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
            //     // } else if (Tools::isLarge(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()][Tools::calcMaxKappaIndexForCurve(this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()])].kappa_, 0.1)) {
            //     //     dbw_mkz_msgs::TurnSignalCmd turn_signal_cmd;
            //     //     turn_signal_cmd.cmd.value = dbw_mkz_msgs::TurnSignal::LEFT;
            //     //     this->turn_signal_pub_.publish(turn_signal_cmd);
            //     // }
            // }
        }
        // }
    }

    // 可视化
    this->Visualization();

    std::cout << "CHOOSE STATE COMPLETE" << std::endl;
    std::cout << "choosed state name: " << DIC_STATE_NAME[this->choosed_state_.getStateName()] << std::endl;
    std::cout << "choosed start point: " << this->choosed_state_.getVehicleStartState().position_.x_ << "||" << this->choosed_state_.getVehicleStartState().position_.y_ << "||" << this->choosed_state_.getVehicleStartState().theta_ << "||"<< this->choosed_state_.getVehicleStartState().kappa_ << "||"<< std::endl;
    std::cout << "choosed state trajectory number: " << this->choosed_state_.getTrajectory().size() << std::endl;
    if (this->choosed_state_.getTrajectory().size() > 0) {
        std::cout << "choosed state trajectory length: " << this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].size() << std::endl;
    } else {
        ROS_ERROR("CHOOSED STATE NO TRAJECTORY");
    }
    std::cout << "expected state name: " << DIC_STATE_NAME[this->expected_state_.getStateName()] << std::endl;

    LOG(INFO) << "CHOOSE STATE COMPLETE";
    LOG(INFO) << "choosed state name: " << DIC_STATE_NAME[this->choosed_state_.getStateName()];
    LOG(INFO) << "choosed start point: " << this->choosed_state_.getVehicleStartState().position_.x_ << "||" << this->choosed_state_.getVehicleStartState().position_.y_ << "||" << this->choosed_state_.getVehicleStartState().theta_ << "||"<< this->choosed_state_.getVehicleStartState().kappa_ << "||";
    LOG(INFO) << "choosed state trajectory number: " << this->choosed_state_.getTrajectory().size();
    if (this->choosed_state_.getTrajectory().size() > 0) {
        LOG(INFO) << "choosed state trajectory length: " << this->choosed_state_.getTrajectory()[this->choosed_state_.getChoosedTrajectoryIndex()].size();
    } else {
        ROS_ERROR("CHOOSED STATE NO TRAJECTORY");
    }
    LOG(INFO) << "expected state name: " << DIC_STATE_NAME[this->expected_state_.getStateName()];
    // END DEBUG

    // 发布新版路径
    if (this->choosed_state_.getStateName() == StateNames::FORWARD || this->choosed_state_.getStateName() == StateNames::TURN_LEFT || this->choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
        // 如果选中状态是三大状态时，保持加速度模式
        this->choosed_state_.publishCurveMsgVelocityMaintain(this->motion_planning_curve_pub_);

        // Load last planned curve to state machine
        if (choosed_state_.getStateName() == StateNames::FORWARD) {
            (&states_set_[StateNames::FORWARD])->last_planned_curve_ = choosed_state_.last_planned_curve_;
        } else if (choosed_state_.getStateName() == StateNames::TURN_LEFT) {
            (&states_set_[StateNames::TURN_LEFT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
        } else if (choosed_state_.getStateName() == StateNames::TURN_RIGHT) {
            (&states_set_[StateNames::TURN_RIGHT])->last_planned_curve_ = choosed_state_.last_planned_curve_;
        } else {
            assert(false);
        }

    
    } else if (this->choosed_state_.getStateName() == StateNames::AVOIDANCE){
        // 如果选中状态不是三大状态时，追点模式
        // 判断是否进行脱困
        if (this->choosed_state_.getOutofTrapping()) {
            // 需要进行脱困
            // 发布目标角度
            {
                // 将车辆转到特定角度
                path_planning_msgs::MotionPlanningCurve rotate_msgs;
                rotate_msgs.header.frame_id = "world";
                rotate_msgs.header.stamp = ros::Time::now();
                rotate_msgs.mode = path_planning_msgs::MotionPlanningCurve::ROTATE;
                rotate_msgs.tracking_mode = path_planning_msgs::MotionPlanningCurve::CENTER;
                rotate_msgs.aim_curvature = this->choosed_state_.getTotalTrajectory()[0].kappa_;
                this->motion_planning_curve_pub_.publish(rotate_msgs);
                ros::Rate wait_rate(1);
                // 判断回正是否完成,完成则结束此状态
                while (true) {
                    // 计算当前曲率
                    this->current_vehicle_kappa_mutex_.lock();
                    double current_curvature = this->current_vehicle_kappa_;
                    this->current_vehicle_kappa_mutex_.unlock();
                    LOG(INFO) << "曲率差仍存在" << std::abs(current_curvature - this->choosed_state_.getTotalTrajectory()[0].kappa_);
                    // 计算当前角度
                    this->current_vehicle_world_position_mutex_.lock();
                    double current_yaw = this->current_vehicle_world_position_.theta_;
                    LOG(INFO) << "当前几何中心位置为" << this->current_vehicle_world_position_.position_ << "," << this->current_vehicle_world_position_.theta_ << "," << this->current_vehicle_world_position_.kappa_;
                    this->current_vehicle_world_position_mutex_.unlock();
                    LOG(INFO) << "目标位置为" << this->choosed_state_.getTotalTrajectory()[0];
                    LOG(INFO) << "角度差仍存在" << std::abs(current_yaw - this->choosed_state_.getTotalTrajectory()[0].theta_);
                    if (Tools::isSmall(std::abs(current_curvature - this->choosed_state_.getTotalTrajectory()[0].kappa_), 0.01)) {
                        LOG(INFO) << "转向完成";
                        std::cout << "转向完成" << std::endl;
                        break;
                    }
                    wait_rate.sleep();
                }
            }
        }
        this->choosed_state_.publishCurveMsgPointReach(this->motion_planning_curve_pub_);
    } else if (this->choosed_state_.getStateName() == StateNames::STOP) {
        // 如果是停车状态，轨迹长度小于1.6米，且当前速度为0，不发布路径
        if (this->choosed_state_.getVehicleDynamicPlanningGoalPositionIndexInTrajectory() <= static_cast<int>(SHORTEST_DISTANCE_TO_PUBLISH_WHEN_STOP / LANE_GAP_DISTANCE) && Tools::isZero(this->choosed_state_.getVehicleStartMovement().velocity_)) {
            // do nothing
            LOG(INFO) << "无意义停车状态，不发布消息";
            // 判断是否可以进行转向
            int rotate_result = this->generateRotateState();
            if (rotate_result > -1) {
                // 可以进行转向
                std::cout << "进行转向" << std::endl;
                return;
            }
            // 判断是否可以进行倒车
            int reverse_result = this->generateReverseState();
            if (reverse_result > -1) {
                // 可以进行倒车
                std::cout << "进行倒车" << std::endl;
                return;
            }
        } else {
            // 判断是否进行脱困
            if (this->choosed_state_.getOutofTrapping()) {
                // 需要进行脱困
                // 发布目标角度
                {
                    // 将车辆转到特定角度
                    path_planning_msgs::MotionPlanningCurve rotate_msgs;
                    rotate_msgs.header.frame_id = "world";
                    rotate_msgs.header.stamp = ros::Time::now();
                    rotate_msgs.mode = path_planning_msgs::MotionPlanningCurve::ROTATE;
                    rotate_msgs.tracking_mode = path_planning_msgs::MotionPlanningCurve::CENTER;
                    rotate_msgs.aim_curvature = this->choosed_state_.getTotalTrajectory()[0].kappa_;
                    this->motion_planning_curve_pub_.publish(rotate_msgs);
                    ros::Rate wait_rate(1);
                    // 判断回正是否完成,完成则结束此状态
                    while (true) {
                        // 计算当前曲率
                        this->current_vehicle_kappa_mutex_.lock();
                        double current_curvature = this->current_vehicle_kappa_;
                        this->current_vehicle_kappa_mutex_.unlock();
                        std::cout << "曲率差仍存在" << std::abs(current_curvature - this->choosed_state_.getTotalTrajectory()[0].kappa_) << std::endl;
                        LOG(INFO) << "曲率差仍存在" << std::abs(current_curvature - this->choosed_state_.getTotalTrajectory()[0].kappa_);
                        // 计算当前角度
                        this->current_vehicle_world_position_mutex_.lock();
                        double current_yaw = this->current_vehicle_world_position_.theta_;
                        LOG(INFO) << "当前几何中心位置为" << this->current_vehicle_world_position_.position_ << "," << this->current_vehicle_world_position_.theta_ << "," << this->current_vehicle_world_position_.kappa_;
                        this->current_vehicle_world_position_mutex_.unlock();
                        LOG(INFO) << "目标位置为" << this->choosed_state_.getTotalTrajectory()[0] << std::endl;
                        LOG(INFO) << "角度差仍存在" << std::abs(current_yaw - this->choosed_state_.getTotalTrajectory()[0].theta_);
                        if (Tools::isSmall(std::abs(current_curvature - this->choosed_state_.getTotalTrajectory()[0].kappa_), 0.01)) {
                            LOG(INFO) << "转向完成";
                            std::cout << "转向完成" << std::endl;
                            break;
                        }
                        wait_rate.sleep();
                    }
                }
            }
            this->choosed_state_.publishCurveMsgPointReach(this->motion_planning_curve_pub_);
        }
    }
}

// 进行可视化
void DecisionMaking::SubVehicle::Visualization() {
    // 清空之前的可视化
    visualization_msgs::MarkerArray delete_marker_array;
    delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
    this->visualization_pub_.publish(delete_marker_array);

    // 1. 可视化三大状态的路径
    visualization_msgs::MarkerArray curves_marker_array;
    std_msgs::ColorRGBA curve_visualization_color;
    if (this->states_set_[StateNames::FORWARD].getCapability()) {
        if (this->states_set_[StateNames::FORWARD].getSafety()) {
            curve_visualization_color.r = 1;
            curve_visualization_color.g = 1;
            curve_visualization_color.b = 1;
            curve_visualization_color.a = 1;
        } else {
            curve_visualization_color.r = 0.5;
            curve_visualization_color.g = 0.5;
            curve_visualization_color.b = 0.5;
            curve_visualization_color.a = 0.5;
        }
        curves_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(this->states_set_[StateNames::FORWARD].getTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()], curve_visualization_color, VisualizationMethods::VisualizationID::FOWARD_ID));
    }
    if (this->states_set_[StateNames::TURN_LEFT].getCapability()) {
        if (this->states_set_[StateNames::TURN_LEFT].getSafety()) {
            curve_visualization_color.r = 1;
            curve_visualization_color.g = 1;
            curve_visualization_color.b = 1;
            curve_visualization_color.a = 1;
        } else {
            curve_visualization_color.r = 0.5;
            curve_visualization_color.g = 0.5;
            curve_visualization_color.b = 0.5;
            curve_visualization_color.a = 0.5;
        }
        curves_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(this->states_set_[StateNames::TURN_LEFT].getTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()], curve_visualization_color, VisualizationMethods::VisualizationID::TURN_LEFT_ID));
    }
    if (this->states_set_[StateNames::TURN_RIGHT].getCapability()) {
        if (this->states_set_[StateNames::TURN_RIGHT].getSafety()) {
            curve_visualization_color.r = 1;
            curve_visualization_color.g = 1;
            curve_visualization_color.b = 1;
            curve_visualization_color.a = 1;
        } else {
            curve_visualization_color.r = 0.5;
            curve_visualization_color.g = 0.5;
            curve_visualization_color.b = 0.5;
            curve_visualization_color.a = 0.5;
        }
        curves_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(this->states_set_[StateNames::TURN_RIGHT].getTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()], curve_visualization_color, VisualizationMethods::VisualizationID::TURN_RIGHT_ID));
    }
    this->visualization_pub_.publish(curves_marker_array);

    // DEBUG 可视化三大状态的延伸路径
    visualization_msgs::MarkerArray extended_curves_marker_array;
    std_msgs::ColorRGBA extended_curve_visualization_color;
    if (this->states_set_[StateNames::FORWARD].getCapability()) {
        if (this->states_set_[StateNames::FORWARD].getSafety()) {
            extended_curve_visualization_color.r = 1;
            extended_curve_visualization_color.g = 1;
            extended_curve_visualization_color.b = 1;
            extended_curve_visualization_color.a = 0.5;
        } else {
            extended_curve_visualization_color.r = 0.75;
            extended_curve_visualization_color.g = 0.75;
            extended_curve_visualization_color.b = 0.75;
            extended_curve_visualization_color.a = 0.3;
        }
        extended_curves_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(this->states_set_[StateNames::FORWARD].getExtendedTrajectory()[this->states_set_[StateNames::FORWARD].getChoosedTrajectoryIndex()], extended_curve_visualization_color, VisualizationMethods::VisualizationID::EXTENDED_FOWARD_ID));
    }
    if (this->states_set_[StateNames::TURN_LEFT].getCapability()) {
        if (this->states_set_[StateNames::TURN_LEFT].getSafety()) {
            extended_curve_visualization_color.r = 1;
            extended_curve_visualization_color.g = 1;
            extended_curve_visualization_color.b = 1;
            extended_curve_visualization_color.a = 0.5;
        } else {
            extended_curve_visualization_color.r = 0.75;
            extended_curve_visualization_color.g = 0.75;
            extended_curve_visualization_color.b = 0.75;
            extended_curve_visualization_color.a = 0.3;
        }
        extended_curves_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(this->states_set_[StateNames::TURN_LEFT].getExtendedTrajectory()[this->states_set_[StateNames::TURN_LEFT].getChoosedTrajectoryIndex()], extended_curve_visualization_color, VisualizationMethods::VisualizationID::EXTENDED_TURN_LEFT_ID));
    }
    if (this->states_set_[StateNames::TURN_RIGHT].getCapability()) {
        if (this->states_set_[StateNames::TURN_RIGHT].getSafety()) {
            extended_curve_visualization_color.r = 1;
            extended_curve_visualization_color.g = 1;
            extended_curve_visualization_color.b = 1;
            extended_curve_visualization_color.a = 0.5;
        } else {
            extended_curve_visualization_color.r = 0.75;
            extended_curve_visualization_color.g = 0.75;
            extended_curve_visualization_color.b = 0.75;
            extended_curve_visualization_color.a = 0.3;
        }
        extended_curves_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(this->states_set_[StateNames::TURN_RIGHT].getExtendedTrajectory()[this->states_set_[StateNames::TURN_RIGHT].getChoosedTrajectoryIndex()], extended_curve_visualization_color, VisualizationMethods::VisualizationID::EXTENDED_TURN_RIGHT_ID));
    }
    this->visualization_pub_.publish(extended_curves_marker_array);
    // END DEBUG
    // 1.可视化停车状态的路径
    if (this->states_set_[StateNames::STOP].getCapability()) {
        visualization_msgs::MarkerArray stop_curve_marker_array;
        std_msgs::ColorRGBA stop_curve_visualization_color;
        stop_curve_visualization_color.r = 1.0;
        stop_curve_visualization_color.g = 0.38;
        stop_curve_visualization_color.b = 0.0;
        stop_curve_visualization_color.a = 1.0;
        PathPlanningUtilities::Curve raw_curve, curve;
        raw_curve = this->states_set_[StateNames::STOP].getTrajectory()[this->states_set_[StateNames::STOP].getChoosedTrajectoryIndex()];
        raw_curve.insert(raw_curve.end(), this->states_set_[StateNames::STOP].getExtendedTrajectory()[this->states_set_[StateNames::STOP].getChoosedTrajectoryIndex()].begin(), this->states_set_[StateNames::STOP].getExtendedTrajectory()[this->states_set_[StateNames::STOP].getChoosedTrajectoryIndex()].end());
        curve.assign(raw_curve.begin() + this->states_set_[StateNames::STOP].getVehicleCurrentPositionIndexInTrajectory(), raw_curve.begin() + this->states_set_[StateNames::STOP].getVehicleDynamicPlanningGoalPositionIndexInTrajectory());
        stop_curve_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(curve, stop_curve_visualization_color, VisualizationMethods::VisualizationID::STOP_ID));
        this->visualization_pub_.publish(stop_curve_marker_array);
    }

    // 1.可视化蔽障状态的路径
    if (this->states_set_[StateNames::AVOIDANCE].getCapability()) {
        visualization_msgs::MarkerArray travel_curve_marker_array, remain_curve_marker_array;
        std_msgs::ColorRGBA travel_curve_visualization_color, remain_curve_visualization_color;
        travel_curve_visualization_color.r = 0.1;
        travel_curve_visualization_color.g = 0.79;
        travel_curve_visualization_color.b = 0.68;
        travel_curve_visualization_color.a = 1.0;
        remain_curve_visualization_color.r = 0.1;
        remain_curve_visualization_color.g = 0.79;
        remain_curve_visualization_color.b = 0.68;
        remain_curve_visualization_color.a = 0.5;
        PathPlanningUtilities::Curve travel_curve, remain_curve;
        travel_curve.assign(this->states_set_[StateNames::AVOIDANCE].getTrajectory()[this->states_set_[StateNames::AVOIDANCE].getChoosedTrajectoryIndex()].begin(), this->states_set_[StateNames::AVOIDANCE].getTrajectory()[this->states_set_[StateNames::AVOIDANCE].getChoosedTrajectoryIndex()].end());
        remain_curve.assign(this->states_set_[StateNames::AVOIDANCE].getExtendedTrajectory()[this->states_set_[StateNames::AVOIDANCE].getChoosedTrajectoryIndex()].begin(), this->states_set_[StateNames::AVOIDANCE].getExtendedTrajectory()[this->states_set_[StateNames::AVOIDANCE].getChoosedTrajectoryIndex()].end());
        travel_curve_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(travel_curve, travel_curve_visualization_color, VisualizationMethods::VisualizationID::AVOIDANCE_START_ID));
        remain_curve_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(remain_curve, remain_curve_visualization_color, VisualizationMethods::VisualizationID::AVOIDANCE_EXTEND_ID));
        this->visualization_pub_.publish(travel_curve_marker_array);
        this->visualization_pub_.publish(remain_curve_marker_array);
    }

    // 1.可视化倒车状态的路径
    if (this->states_set_[StateNames::REVERSE].getCapability()) {
    }

    // 2.可视化选中状态的路径
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

    // ３. 可视化道路
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

    // ４. 可视化状态机
    double position_x = this->choosed_state_.getVehicleCurrentPosition().position_.x_;
    double position_y = this->choosed_state_.getVehicleCurrentPosition().position_.y_;
    VisualizationMethods::visualizeStates(this->states_set_, this->choosed_state_, position_x, position_y, VisualizationMethods::VisualizationID::STATES_START_ID, this->visualization_pub_);

    // 4.5可视化是否为脱困模式
    if (this->choosed_state_.getOutofTrapping()) {
        // 脱困模式
        std_msgs::ColorRGBA out_of_trapping_color;
        out_of_trapping_color.r = 0.6;
        out_of_trapping_color.g = 0.6;
        out_of_trapping_color.b = 0.0;
        out_of_trapping_color.a = 1.0;
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(VisualizationMethods::visualizeStringToMarker("OUTOF TRAPPING", position_x, position_y, out_of_trapping_color, VisualizationMethods::VisualizationID::OUTOFTRAP_ID));
        this->visualization_pub_.publish(marker_array);
    }

    // 5. 可视化交通规则
    VisualizationMethods::visualizeTrafficRules(this->traffic_rule_obstacles_raw_, this->visualization_pub_);

    // 6. 可视化本车占用区域
    // 清空之前的可视化
    if (this->choosed_state_.getStateName() != StateNames::AVOIDANCE && this->choosed_state_.getStateName() != StateNames::STOP){
        visualization_msgs::MarkerArray delete_occupation_marker_array;
        delete_occupation_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(VisualizationMethods::VisualizationID::OCCUPATION_AREA_START_ID));
        this->vis_occupation_pub_.publish(delete_occupation_marker_array);

        VisualizationMethods::visualizeSubvehicleOccupationArea(this->choosed_state_, this->vehicle_rear_axis_center_scale_, this->vis_occupation_pub_);
    }

    // 可视化碰撞点
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
}

// 根据上一次状态和此次选中的状态判断规划模块是否已经无效
bool DecisionMaking::SubVehicle::motionPlanningUncapableJudgement() {
    bool result = false;
    //  得到当前速度
    this->current_vehicle_movement_mutex_.lock();
    double current_velocity = this->current_vehicle_movement_.velocity_;
    this->current_vehicle_movement_mutex_.unlock();
    // 首先判断选中状态是否是停车状态
    if (this->choosed_state_.getStateName() == StateNames::STOP) {
        if (this->current_state_.getStateName() == StateNames::STOP && Tools::isZero(current_velocity)) {
            // 如果上一个状态也是停车并且当前速度为0，停车计数器加一
            this->stop_count_recorder_++;
            usleep(1000000);
            LOG(INFO) << "停车计数器增加" << this->stop_count_recorder_;
            std::cout << "停车计数器增加" << this->stop_count_recorder_ << std::endl;
        } else {
            this->stop_count_recorder_ = 0;
        }
    } else {
        this->stop_count_recorder_ = 0;
    }

    if (this->stop_count_recorder_ >= MAX_STOP_COUNT * 2 - 1) {
        this->low_frequency_stop_count_recorder_++;
    }

    if (this->low_frequency_stop_count_recorder_ > 2) {
        if (!this->IS_TOTAL_AUTONOMOUS_FLAG_) {
            // 非全自动模式，不自己处理无法规划问题
            std::cout << "计数器值满，报告无法规划" << std::endl;
            LOG(INFO) << "计数器值满，报告无法规划";
            // 调用报告无法进行规划服务
            std_srvs::Trigger trigger;
            this->motion_planning_failed_client_.call(trigger);
            if (trigger.response.success) {
                // 报告成功
                this->mission_start_mutex_.lock();
                this->MISSION_START_FLAG_ = false;
                this->mission_start_mutex_.unlock();
                result = true;
                LOG(INFO) << "报告无法规划成功";
                // 计数器归零
                this->stop_count_recorder_ = 0;
                this->low_frequency_stop_count_recorder_ = 0;
            } else {
                LOG(INFO) << "报告无法规划失败";
                // 什么都不做
            }
        } else {
            // 全自动模式
            std::cout << "计数器值满，请求调头" << std::endl;
            LOG(INFO) << "计数器值满，请求调头";
            // 如果停车计数大于5（对应时间长度应为5秒），说明规划模块已经失效，请求调头
            mission_msgs::RequireTurnAround require_turn_around_service;
            geometry_msgs::PoseStamped current_pose;
            current_pose.header.frame_id = "world";
            current_pose.header.stamp = ros::Time::now();
            this->current_vehicle_world_position_mutex_.lock();
            current_pose.pose.position.x = this->current_vehicle_world_position_.position_.x_;
            current_pose.pose.position.y = this->current_vehicle_world_position_.position_.y_;
            current_pose.pose.orientation.x = 0.0;
            current_pose.pose.orientation.y = 0.0;
            current_pose.pose.orientation.z = sin(this->current_vehicle_world_position_.theta_/2.0);
            current_pose.pose.orientation.w = cos(this->current_vehicle_world_position_.theta_/2.0);
            this->current_vehicle_world_position_mutex_.unlock();
            current_pose.pose.position.z = 0;
            require_turn_around_service.request.current_pose = current_pose;
            this->motion_planning_uncapable_client_.call(require_turn_around_service);
            if (require_turn_around_service.response.success == true) {
                // 开始调用A星程序结束
                this->mission_start_mutex_.lock();
                this->MISSION_START_FLAG_ = false;
                this->mission_start_mutex_.unlock();
                result = true;
                std::cout << "调头开始" << std::endl;
                LOG(INFO) << "调头开始";
                // 计数器归零
                this->stop_count_recorder_ = 0;
                this->low_frequency_stop_count_recorder_ = 0;
            } else {
                // 什么都不做
                std::cout << "无法调头" << std::endl;
                LOG(INFO) << "无法调头";
            }
            LOG(INFO) << "发布规划无法继续行驶消息";
        }
    }
    // the_end:
    if (this->stop_count_recorder_ >= 2 * MAX_STOP_COUNT) {
        // 计数器归零
        this->stop_count_recorder_ = 0;
    }

    if (this->low_frequency_stop_count_recorder_ > 2) {
        this->low_frequency_stop_count_recorder_ = 0;
    }

    return result;
}

// 计算状态的曲线中最大曲率
double DecisionMaking::SubVehicle::calcMaxKappaForState(const StandardState &state, size_t length) {
    double max_kappa = 0.0;
    size_t start_index = state.getVehicleCurrentPositionIndexInTrajectory();
    // assert(start_index + length < state.getTrajectoryLength() + state.getExtendedTrajectoryLength());
    if (start_index + length >= state.getTrajectoryLength() + state.getExtendedTrajectoryLength()) {
        length = state.getTrajectoryLength() + state.getExtendedTrajectoryLength() - start_index - 1;
    }
    for (size_t i = start_index; i < start_index + length; i++) {
        double kappa;
        if (i < state.getTrajectoryLength()) {
            kappa = state.getTrajectory()[state.getChoosedTrajectoryIndex()][i].kappa_;
        } else {
            kappa = state.getExtendedTrajectory()[state.getChoosedTrajectoryIndex()][i - state.getTrajectoryLength()].kappa_;
        }
        if (!Tools::isSmall(std::fabs(kappa), std::fabs(max_kappa))) {
            max_kappa = kappa;
        }
    }
    return max_kappa;
}

// 计算状态的曲线中的最大曲率变化率
double DecisionMaking::SubVehicle::calcMaxCurvatureChangeRateForState(const StandardState &state, size_t length) {
    size_t start_index = state.getVehicleCurrentPositionIndexInTrajectory();
    if (start_index + length >= state.getTrajectoryLength() + state.getExtendedTrajectoryLength()) {
        length = state.getTrajectoryLength() + state.getExtendedTrajectoryLength() - start_index - 1;
    }
    size_t end_index = start_index + length;
    PathPlanningUtilities::Curve total_curve = state.getTotalTrajectory();
    assert(end_index < total_curve.size());
    PathPlanningUtilities::Curve curve_segment;
    curve_segment.assign(total_curve.begin() + start_index, total_curve.begin() + end_index);
    return Tools::getCurveMaxCurvatureChangeRate(curve_segment);
}

// 选取状态机可行状态中优先度最高的状态
bool DecisionMaking::SubVehicle::sortStatesPriority(std::vector<DecisionMaking::StandardState> *states_set) {
    // 首先根据优先级进行排序
    for (size_t i = 0; i < states_set->size() - 1; i++) {
        for (size_t j = 0; j < states_set->size() - i - 1; j++) {
            if (Tools::isLarge((*states_set)[j].getPriority(), (*states_set)[j + 1].getPriority())) {
                DecisionMaking::StandardState tmp_state;
                tmp_state = (*states_set)[j + 1];
                (*states_set)[j + 1] = (*states_set)[j];
                (*states_set)[j] = tmp_state;
            }
        }
    }
    // 然后判断是否存在可行状态
    int count = 0;
    bool result = false;
    for (size_t i = 0; i < states_set->size(); i++) {
        if ((*states_set)[i].getCapability() && (*states_set)[i].getSafety()) {
            count++;
        }
    }

    if (count > 0) {
        result = true;
    }
    return result;
}
