#pragma once

#include <iostream>
#include <fstream>
#include <path_planning_msgs/BoundedCurve.h>
#include <path_planning_msgs/Curve.h>
#include <path_planning_msgs/CurvePoint.h>
#include <path_planning_msgs/MotionPlanningCurve.h>
#include <path_planning_msgs/Path.h>
#include <path_planning_msgs/PathPoint.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <mission_msgs/StartMain.h>
#include <mission_msgs/RequireTurnAround.h>
#include <control_msgs/CoreReport.h>
#include <glog/logging.h>
#include "Point.hpp"
#include "Path.hpp"
#include "PathGenerator.h"
#include "Lane.hpp"
#include "Rectangle.hpp"
#include "Const.hpp"
#include "Obstacle.hpp"
#include "InfluenceObstacle.hpp"

namespace DecisionMaking {



// 定义状态名称
enum StateNames {
    STOP = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    FORWARD = 3,
    AVOIDANCE = 4,
    REVERSE = 5,
    ROTATE = 6,
    STATE_SIZE = 7
};

// 定义特殊状态名称
enum SpecialStateNames {
    FOLLOW_PATH = 10,
    REACH_POINT = 11,
    SPECIAL_STATE_SIZE = 2
};

// 定义不同的状态、包括了状态的名称、状态的相邻状态、起始点和终止点、起始速度和终止速度、路径、优先级。
// 实际上状态为车辆运动和规划信息变量的集合
class StandardState {
 public:
    // 构造函数，定义状态的名字和邻居，不可更改
    StandardState() {}

    StandardState(size_t state_name,
                    std::vector<size_t> neighbor_states) {
        this->state_name_ = state_name;
        this->neighbor_states_ = neighbor_states;
    }

    ~StandardState() {}

    // 确定状态是否可行
    void enable() {
        this->capability_ = true;
    }

    void disable() {
        this->capability_ = false;
    }

    bool getCapability() const {
        return this->capability_;
    }

    // 确定是否安全
    void setSafety(bool safety) {
        this->safety_ = safety;
    }

    bool getSafety() const {
        return this->safety_;
    }

    // 确定优先级(值越大，优先级越高)
    void setPriority(double priority) {
        this->priority_ = priority;
    }

    double getPriority() const {
        return this->priority_;
    }

    // 获取状态名称
    size_t getStateName() const {
        return this->state_name_;
    }

    // 获取状态邻居状态
    const std::vector<size_t> &getNeighborStates() const {
        return this->neighbor_states_;
    }

    // 确定车辆长宽
    void setVehicleSize(double length, double width) {
        this->vehicle_length_ = length;
        this->vehicle_width_ = width;
    }

    double getVehicleLength() const {
        return this->vehicle_length_;
    }

    double getVehicleWidth() const {
        return this->vehicle_width_;
    }

    // 确定起始点位置
    void setVehicleStartState(const PathPlanningUtilities::VehicleState &start_point) {
        this->start_point_ = start_point;
    }

    const PathPlanningUtilities::VehicleState &getVehicleStartState() const {
        return this->start_point_;
    }

    // 确定车辆当前点在路径上的位置下标
    void setVehicleCurrentPositionIndexInTrajectory(size_t vehicle_current_position_index) {
        this->vehicle_current_position_index_ = vehicle_current_position_index;
    }

    size_t getVehicleCurrentPositionIndexInTrajectory() const {
        return this->vehicle_current_position_index_;
    }

    // 获取车辆当前点的位置信息
    PathPlanningUtilities::CurvePoint getVehicleCurrentPosition() const {
        if (this->vehicle_current_position_index_ < this->trajectory_curves_[this->choosed_trajectory_index_].size()) {
            return this->trajectory_curves_[this->choosed_trajectory_index_][this->vehicle_current_position_index_];
        } else {
            return this->extended_curves_[this->choosed_trajectory_index_][this->vehicle_current_position_index_ - this->trajectory_curves_[this->choosed_trajectory_index_].size()];
        }
    }

    // 确定车辆动态规划目标点在路径上的位置下标
    void setVehicleDynamicPlanningGoalPositionIndexInTrajectory(size_t vehicle_goal_position_index) {
        this->vehicle_goal_position_index_ = vehicle_goal_position_index;
    }

    size_t getVehicleDynamicPlanningGoalPositionIndexInTrajectory() const {
        return this->vehicle_goal_position_index_;
    }

    // 确定车辆动态规划期望保持的加速度
    void setVehicleDynamicPlanningExpectedAcceleration(double vehicle_goal_expected_average_acceleration) {
        this->vehicle_goal_expected_average_acceleration_ = vehicle_goal_expected_average_acceleration;
    }

    double getVehicleDynamicPlanningExpectedAcceleration() const {
        return this->vehicle_goal_expected_average_acceleration_;
    }

    // 确定起始点速度，加速度
    void setVehicleStartMovement(const PathPlanningUtilities::VehicleMovementState &start_movement) {
        this->start_movement_ = start_movement;
    }

    const PathPlanningUtilities::VehicleMovementState &getVehicleStartMovement() const {
        return this->start_movement_;
    }

    // 确定当前速度、加速度
    void setVehicleCurrentMovement(const PathPlanningUtilities::VehicleMovementState &current_movement) {
        this->current_movement_ = current_movement;
    }

    const PathPlanningUtilities::VehicleMovementState &getVehicleCurrentMovement() const {
        return this->current_movement_;
    }

    // 设置车辆速度上下界
    void setVelocityLimitation(double velocity_limitation_max, double velocity_limitation_min) {
        this->velocity_limitation_max_ = velocity_limitation_max;
        this->velocity_limitation_min_ = velocity_limitation_min;
    }

    double getVelocityLimitationMax() const {
        return this->velocity_limitation_max_;
    }

    double getVelocityLimitationMin() const {
        return this->velocity_limitation_min_;
    }

    // 设置加速度上下限
    void setAccelerationLimitationMax(double acceleration_limitation_max) {
        this->acceleration_limitation_max_ = acceleration_limitation_max;
    }

    void setAccelerationLimitationMin(double acceleration_limitation_min) {
        this->acceleration_limitation_min_ = acceleration_limitation_min;
    }

    double getAccelerationLimitationMax() const {
        return this->acceleration_limitation_max_;
    }

    double getAccelerationLimitationMin() const {
        return this->acceleration_limitation_min_;
    }

    // 设置道路速度期望
    void setExpectedVelocity(double velocity_expectation_current, double velocity_expectation_limitation) {
        this->velocity_expectation_current_ = velocity_expectation_current;
        this->velocity_expectation_limitation_ = velocity_expectation_limitation;
    }

    double getExpectedVelocityCurrent() const {
        return this->velocity_expectation_current_;
    }

    double getExpectedVelocityLimitation() const {
        return this->velocity_expectation_limitation_;
    }

    // 设置目标点速度
    void setGoalVelocity(double goal_velocity) {
        this->goal_velocity_ = goal_velocity;
    }

    double getGoalVelocity() const {
        return this->goal_velocity_;
    }

    // 确定路径
    void setTrajectory(const std::vector<PathPlanningUtilities::Curve> &trajectory_curves) {
        this->trajectory_curves_ = trajectory_curves;
    }

    const std::vector<PathPlanningUtilities::Curve>& getTrajectory() const {
        return this->trajectory_curves_;
    }

    const size_t getTrajectoryLength() const {
        return this->trajectory_curves_[this->choosed_trajectory_index_].size();
    }

    // 确定路径在frenet坐标系下坐标
    void setFrenetTrajectory(const std::vector<PathPlanningUtilities::Curve> &frenet_curves) {
        this->frenet_curves_ = frenet_curves;
    }

    std::vector<PathPlanningUtilities::Curve> getFrenetTrajectory() const {
        return this->frenet_curves_;
    }

    // 确定路径的衍生段
    void setExtendedTrajectory(const std::vector<PathPlanningUtilities::Curve> &extended_curves) {
        this->extended_curves_ = extended_curves;
    }

    const std::vector<PathPlanningUtilities::Curve> &getExtendedTrajectory() const {
        return this->extended_curves_;
    }

    size_t getExtendedTrajectoryLength() const {
        return this->extended_curves_[this->choosed_trajectory_index_].size();
    }

    // 确定全路径
    PathPlanningUtilities::Curve getTotalTrajectory() const {
        PathPlanningUtilities::Curve total_curve;
        total_curve.assign(this->trajectory_curves_[this->choosed_trajectory_index_].begin(), this->trajectory_curves_[this->choosed_trajectory_index_].end());
        total_curve.insert(total_curve.end(), this->extended_curves_[this->choosed_trajectory_index_].begin(), this->extended_curves_[this->choosed_trajectory_index_].end());
        return total_curve;
    }

    // 确定选择的路径
    void setChoosedTrajectoryIndex(size_t choosed_trajectory_index) {
        this->choosed_trajectory_index_ = choosed_trajectory_index;
    }

    size_t getChoosedTrajectoryIndex() const {
        return this->choosed_trajectory_index_;
    }

    // 状态的有效障碍物信息
    void addInfluenceObstacle(const InfluenceObstacle &influence_obstacle) {
        this->influence_obstacles_.push_back(influence_obstacle);
    }

    const std::vector<InfluenceObstacle> &getInfluenceObstacle() const {
        return this->influence_obstacles_;
    }

    void clearInfluenceObstacle() {
        std::vector<InfluenceObstacle>().swap(this->influence_obstacles_);
    }

    void loadStProfile(const std::vector<double>& s, const std::vector<double>& v, const std::vector<double>& a) {
        s_ = s;
        v_ = v;
        a_ = a;
    }

    // 发布路径（保持速度模式）
    void publishCurveMsgVelocityMaintain(const ros::Publisher &publisher) {

        // // Clear
        // last_planned_curve_.clear();

        if (this->state_name_ != StateNames::TURN_LEFT && this->state_name_ != StateNames::TURN_RIGHT && this->state_name_ != StateNames::FORWARD) {
            std::cout << "[Error] publish error curve message" << std::endl;
            LOG(INFO) << "[Error] publish error curve message";
            exit(0);
        }
        path_planning_msgs::MotionPlanningCurve final_curve;
        // 填充必备信息
        // 填充消息头
        final_curve.header.frame_id = "world";
        final_curve.header.stamp = ros::Time::now();
        // 填充路径点
        // 计算速度上限
        double max_velocity = this->velocity_expectation_limitation_;
        max_velocity = std::min(max_velocity, this->velocity_limitation_max_);
        assert(Tools::isLarge(max_velocity, this->velocity_limitation_min_));
        // 得到全路径
        PathPlanningUtilities::Curve total_curve;
        total_curve.assign(this->trajectory_curves_[this->choosed_trajectory_index_].begin(), this->trajectory_curves_[this->choosed_trajectory_index_].end());
        total_curve.insert(total_curve.end(), this->extended_curves_[this->choosed_trajectory_index_].begin(), this->extended_curves_[this->choosed_trajectory_index_].end());
        final_curve.points.resize(total_curve.size());
        // 进行赋值
        for (size_t i = 0; i < total_curve.size(); i++) {
            path_planning_msgs::CurvePoint point;
            // 填充位置点的空间信息
            point.x = total_curve[i].position_.x_;
            point.y = total_curve[i].position_.y_;
            point.theta = total_curve[i].theta_;
            point.kappa = total_curve[i].kappa_;
             // 填充位置点的速度信息
            if (i <= this->vehicle_current_position_index_) {
                // 在当前位置前的点速度都为当前速度
                point.velocity = this->current_movement_.velocity_;
            } else {
                if (velocity_planning_from_previous_version_) {
                    double distance = LANE_GAP_DISTANCE * (i - this->vehicle_current_position_index_);
                    if (Tools::isLarge(this->current_movement_.velocity_, max_velocity)) {
                        // 如果当前车速大于最大车速
                        point.velocity = std::max(sqrt(std::max(this->current_movement_.velocity_ * this->current_movement_.velocity_ + 2.0 * this->vehicle_goal_expected_average_acceleration_ * distance, 0.0)), this->velocity_limitation_min_);
                    } else if (Tools::isSmall(this->current_movement_.velocity_, this->velocity_limitation_min_)) {
                        // 如果当前车速小于最小车速
                        point.velocity = std::min(sqrt(std::max(this->current_movement_.velocity_ * this->current_movement_.velocity_ + 2.0 * this->vehicle_goal_expected_average_acceleration_ * distance, 0.0)), max_velocity);
                    } else {
                        // 如果当前车速大于最小车速，小于最大车速
                        point.velocity = std::max(std::min(sqrt(std::max(this->current_movement_.velocity_ * this->current_movement_.velocity_ + 2.0 * this->vehicle_goal_expected_average_acceleration_ * distance, 0.0)), max_velocity), this->velocity_limitation_min_);
                    }
                } else {
                    double s = LANE_GAP_DISTANCE * (i - vehicle_current_position_index_);
                    int lower_index = std::lower_bound(s_.begin(), s_.end(), s) - s_.begin();
                    if (lower_index >= s_.size()) {
                        // printf("[VelocityPlanning] current s excesses the upper bound.\n");
                        lower_index = s_.size() - 1;
                    }
                    double velocity = v_[lower_index];

                    // DEBUG
                    std::cout << "s: " << s << ", velocity: " << velocity << std::endl;
                    // END DEBUG

                    point.velocity = velocity;
                }










            }
            final_curve.points[i] = point;
        }

        // Update last planned curve
        last_planned_curve_.assign(total_curve.begin() + vehicle_current_position_index_ + 1, total_curve.end());

        // // DEBUG
        // std::cout << "debug debug" << std::endl;
        // std::cout << last_planned_curve_.size() << std::endl;
        // // END DEBUG

        // // DEBUG
        // assert(s_.size() == v_.size());
        // for (int i = 0; i < s_.size(); i++) {
        //     std::cout << s_[i] << ", " << v_[i] << std::endl;
        // }
        // // END DEBUG

        // 填充路径点之间的间隔（单位为米）
        final_curve.point_margin = LANE_GAP_DISTANCE;
        // 填充路径模式
        final_curve.mode = path_planning_msgs::MotionPlanningCurve::MAINTAIN_VELOCITY;
        // 填充车辆中心点当前处于路径上对应的位置
        final_curve.vehicle_position_index = this->vehicle_current_position_index_;
        // 填充是否允许倒车
        final_curve.reverse_allowing = false;
        // 标志位
        final_curve.tracking_mode = path_planning_msgs::MotionPlanningCurve::CENTER;

        // 填充模式MAINTAIN_ACCELERATION下的特定内容
        // 填充期望保持的加速度
        final_curve.expected_acceleration = this->vehicle_goal_expected_average_acceleration_;
        // 填充能够到达的最高速度(可以超速1.2倍)
        final_curve.velocity_limitation_max = max_velocity;
        // 填充能够到达的最低速度
        final_curve.velocity_limitation_min = this->velocity_limitation_min_;

        // 消息填充完毕，开始发布
        publisher.publish(final_curve);

        LOG(INFO) << "轨迹发布: 轨迹类型为保持速度, 轨迹长度为" << final_curve.points.size() << ", 车辆当前处于轨迹中第" << final_curve.vehicle_position_index << "个点, 轨迹更新点为第" << this->trajectory_curves_[this->choosed_trajectory_index_].size() << "个点. 轨迹的特征包括轨迹的加速度和速度上下限, 其中加速度为" << final_curve.expected_acceleration << ", 速度上限为" << final_curve.velocity_limitation_max << "米/秒, 速度下限为" << final_curve.velocity_limitation_min << "米/秒.";
        std::cout << "轨迹发布: 轨迹类型为保持速度, 轨迹长度为" << final_curve.points.size() << ", 车辆当前处于轨迹中第" << final_curve.vehicle_position_index << "个点, 轨迹更新点为第" << this->trajectory_curves_[this->choosed_trajectory_index_].size() << "个点. 轨迹的特征包括轨迹的加速度和速度上下限, 其中加速度为" << final_curve.expected_acceleration << ", 速度上限为" << final_curve.velocity_limitation_max << "米/秒, 速度下限为" << final_curve.velocity_limitation_min << "米/秒." << std::endl;

        #ifndef NDEBUG
        // 首先本工程的目录
        std::string root_path = ros::package::getPath("motion_planning");
        std::string log_file_path = "/curve_record/" + Tools::returnCurrentTimeAndDate() + ".csv";
        log_file_path = root_path + log_file_path;
        std::ofstream file(log_file_path);
        if (file) {
            for (auto point: final_curve.points) {
                file << std::setprecision(14) << point.x << "," << point.y << "," << point.theta << "," << point.kappa << "\n";
            }
        }
        file.close();
        #endif
    }

    // 发布路径（追踪目标点模式）
    void publishCurveMsgPointReach(const ros::Publisher &publisher) {
        if (this->state_name_ == StateNames::TURN_LEFT || this->state_name_ == StateNames::TURN_RIGHT || this->state_name_ == StateNames::FORWARD) {
            std::cout << "[Error] publish error curve message" << std::endl;
            LOG(INFO) << "[Error] publish error curve message";
            exit(0);
        }
        path_planning_msgs::MotionPlanningCurve final_curve;
        // 填充必备信息
        // 填充消息头
        final_curve.header.frame_id = "world";
        final_curve.header.stamp = ros::Time::now();
        // 填充路径点
        for (size_t i = 0; i < this->trajectory_curves_[this->choosed_trajectory_index_].size(); i++) {
            path_planning_msgs::CurvePoint point;
            point.x = this->trajectory_curves_[this->choosed_trajectory_index_][i].position_.x_;
            point.y = this->trajectory_curves_[this->choosed_trajectory_index_][i].position_.y_;
            point.theta = this->trajectory_curves_[this->choosed_trajectory_index_][i].theta_;
            point.kappa = this->trajectory_curves_[this->choosed_trajectory_index_][i].kappa_;
            final_curve.points.push_back(point);
        }
        for (size_t i = 0; i < this->extended_curves_[this->choosed_trajectory_index_].size(); i++) {
            path_planning_msgs::CurvePoint point;
            point.x = this->extended_curves_[this->choosed_trajectory_index_][i].position_.x_;
            point.y = this->extended_curves_[this->choosed_trajectory_index_][i].position_.y_;
            point.theta = this->extended_curves_[this->choosed_trajectory_index_][i].theta_;
            point.kappa = this->extended_curves_[this->choosed_trajectory_index_][i].kappa_;
            final_curve.points.push_back(point);
        }
        // 填充路径点之间的间隔（单位为米）
        if (this->state_name_ == StateNames::REVERSE) {
            final_curve.point_margin = LANE_GAP_DISTANCE * 0.1;
        } else {
            final_curve.point_margin = LANE_GAP_DISTANCE;
        }
        // 填充路径模式
        if (this->state_name_ == StateNames::ROTATE) {
            final_curve.mode = path_planning_msgs::MotionPlanningCurve::ROTATE;
            final_curve.aim_curvature = final_curve.points[0].kappa;
        } else {
            final_curve.mode = path_planning_msgs::MotionPlanningCurve::REACH_POINT;
        }
        // 填充车辆中心点当前处于路径上对应的位置
        final_curve.vehicle_position_index = this->vehicle_current_position_index_;
        // 填充是否允许倒车
        if (this->state_name_ == StateNames::REVERSE) {
            final_curve.reverse_allowing = true;
        } else {
            final_curve.reverse_allowing = false;
        }
        // 标志位
        final_curve.tracking_mode = path_planning_msgs::MotionPlanningCurve::CENTER;

        // 填充模式REACH_POINT下的特定内容
        // 填充目标点的速度
        final_curve.goal_velocity = this->goal_velocity_;
        // 填充目标点在路径中的下标
        final_curve.goal_index = this->vehicle_goal_position_index_;
        // 填充速度上限
        final_curve.allowable_max_velocity = this->allowable_max_velocity_;
        
        // 消息填充完毕，开始发布
        publisher.publish(final_curve);

        LOG(INFO) << "轨迹发布: 轨迹类型为追踪目标点, 轨迹长度为" << final_curve.points.size() << ", 车辆当前处于轨迹中第" << final_curve.vehicle_position_index << "个点. 轨迹的特征包括轨迹的目标点和目标点速度, 其中目标点处于轨迹第" << final_curve.goal_index << "个点, 目标点速度为" << final_curve.goal_velocity << "米/秒." << ", 目标点坐标为" << final_curve.points[final_curve.goal_index].x << ", " << final_curve.points[final_curve.goal_index].y;
        std::cout << "轨迹发布: 轨迹类型为追踪目标点, 轨迹长度为" << final_curve.points.size() << ", 车辆当前处于轨迹中第" << final_curve.vehicle_position_index << "个点. 轨迹的特征包括轨迹的目标点和目标点速度, 其中目标点处于轨迹第" << final_curve.goal_index << "个点, 目标点速度为" << final_curve.goal_velocity << "米/秒." << std::endl;

        #ifndef NDEBUG
        // 首先本工程的目录
        std::string root_path = ros::package::getPath("motion_planning");
        std::string log_file_path = "/curve_record/" + Tools::returnCurrentTimeAndDate() + ".csv";
        log_file_path = root_path + log_file_path;
        std::ofstream file(log_file_path);
        if (file) {
            for (auto point: final_curve.points) {
                file << std::setprecision(14) << point.x << "," << point.y << "," << point.theta << "," << point.kappa << "\n";
            }
        }
        file.close();
        #endif
    }

    // ---------------------------------非通用的特殊位--------------------------------
        
    // 设置状态是否完成（只有current_state才用得到这一项）
    void setStateCompleted(bool is_completed) {
        this->is_completed_ = is_completed;
    }

    const bool getStateCompleted() const {
        return this->is_completed_;
    }

    // 状态可以达到的最大加速度
    void setStateMaxAvailableAcceleration(double max_available_acceleration) {
        this->max_available_acceleration_ = max_available_acceleration;
    }

    double getStateMaxAvailableAcceleration() const {
        return this->max_available_acceleration_;
    }

    // 状态的可行加速度区间
    void setStateAccelerationSection(const SectionSet &acceleration_section) {
        this->acceleration_section_ = acceleration_section;
    }
    const SectionSet &getStateAccelerationSection() const {
        return this->acceleration_section_;
    }

    // 是否接着之前未完成状态
    void isStateContinue(bool is_continue) {
        this->is_continue_ = is_continue;
    }

    bool getStateContinue() const {
        return this->is_continue_;
    }

    void setRespondingLane(const Lane &lane) {
        this->responding_lane_ = lane;
    }

    const Lane &getRespondingLane() const {
        return this->responding_lane_;
    }

    // 是否进行状态保持
    void setStateMaintain(bool state_maintain) {
        this->state_maintain_ = state_maintain;
    }

    bool getStateMaintain() const {
        return this->state_maintain_;
    }

    // 设置道路是否被障碍物占据
    void setLaneBeingOccupiedByObstacle(bool is_lane_being_occupied_by_obstacle) {
        this->is_lane_being_occupied_by_obstacle_ = is_lane_being_occupied_by_obstacle;
    }

    // 获取道路是否被障碍物占据
    bool getLaneBeingOccupiedByObstacle() const {
        return this->is_lane_being_occupied_by_obstacle_;
    }

    void setAllowMaxVelocity(double allowable_max_velocity) {
        this->allowable_max_velocity_ = allowable_max_velocity;
    }

    // 设置为脱困模式
    void setOutofTrapping(bool is_outof_trapping) {
        this->is_outof_trapping_ = is_outof_trapping;
    }

    // 获取是否为脱困模式
    bool getOutofTrapping() const {
        return this->is_outof_trapping_;
    }


    size_t state_name_;  // 名称
    std::vector<size_t> neighbor_states_;  // 邻居
    bool capability_ = false;  // 可行性
    bool safety_ = false;  //安全性
    bool is_completed_ = true;  // 状态是否完成(默认为状态完成)
    double priority_ = -1.0;  // 优先级(值越大，优先级越高，区间在0.0~10.0)
    double velocity_limitation_max_;  //状态的速度上限（随时间变化，自动更新）
    double velocity_limitation_min_;  //状态的速度下限（随时间变化，自动更新）
    double acceleration_limitation_max_;  // 加速度上限（随时间变化，自动更新）
    double acceleration_limitation_min_;  // 减速度上限（随时间变化，自动更新）
    double velocity_expectation_current_;  // 当前道路期望速度（随时间变化，自动更新）
    double velocity_expectation_limitation_;  // 未来道路期望速度（随时间变化，自动更新）
    double vehicle_width_;  // 车辆宽度
    double vehicle_length_;  // 车辆长度
    PathPlanningUtilities::VehicleState start_point_;  // 起始点位置状态
    PathPlanningUtilities::VehicleMovementState start_movement_;  // 起始点速度,加速度
    size_t vehicle_current_position_index_;  // 车辆当前位置在路径中的下标，如果路径不存在，此也不存在（随时间变化，自动更新）
    PathPlanningUtilities::VehicleMovementState current_movement_;  // 当前速度、加速度（随时间变化，自动更新）
    size_t vehicle_goal_position_index_;  // 车辆动态速度规划目标点在路径中的下标（随时间变化，速度规划更新）
    double goal_velocity_;  // 目标点速度（随时间变化，速度规划更新）
    double vehicle_goal_expected_average_acceleration_;  // 车辆在动态速度规划中期望保持的加速度(随时间变化，速度规划更新)
    std::vector<PathPlanningUtilities::Curve> trajectory_curves_;  // 生成轨迹
    std::vector<PathPlanningUtilities::Curve> frenet_curves_;  // 生成轨迹在frenet坐标系下坐标
    std::vector<PathPlanningUtilities::Curve> extended_curves_;  // 生成轨迹延道路方向的衍生段
    size_t choosed_trajectory_index_ = 0;  //选择的轨迹
    std::vector<InfluenceObstacle> influence_obstacles_;  // 状态对应的有效障碍物信息(随时间变化，自动更新)
    double max_available_acceleration_;  // 状态可以达到的最大加速度
    SectionSet acceleration_section_;  // 状态的可行加速度区间
    bool is_continue_ = false;  // 是否接着没走完的路走
    Lane responding_lane_;  // 对应道路
    bool state_maintain_ = true;  // 是否进行状态保持 
    bool is_lane_being_occupied_by_obstacle_ = false;  // 判断对应道路是不是没有被障碍物占据
    double allowable_max_velocity_;  // 最大允许速度
    bool is_outof_trapping_ = false;  // 是否为脱困模式

    // Velocity planning
    std::vector<double> s_;
    std::vector<double> v_;
    std::vector<double> a_;
    PathPlanningUtilities::Curve last_planned_curve_;
    bool velocity_profile_generation_state_{false};
    bool velocity_planning_from_previous_version_{false};
};

}