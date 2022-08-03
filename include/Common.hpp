/*
    Copyright [2019] Jian ZhiQiang
*/

#ifndef SRC_MULTI_PATH_GENERATOR_INCLUDE_COMMON_HPP_
#define SRC_MULTI_PATH_GENERATOR_INCLUDE_COMMON_HPP_

#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <memory>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <self_test/self_test.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/SurroundReport.h>
#include <dbw_mkz_msgs/TurnSignal.h>
#include <dbw_mkz_msgs/TurnSignalCmd.h>
#include <vec_map_cpp_msgs/LocalLane.h>
#include <vec_map_cpp_msgs/VirtualObstacle.h>
#include <vec_map_cpp_msgs/GetGuidedCurves.h>
#include <vec_map_cpp_msgs/GetPredictedTrajectory.h>
#include <path_planning_msgs/BoundedCurve.h>
#include <path_planning_msgs/Curve.h>
#include <path_planning_msgs/CurvePoint.h>
#include <path_planning_msgs/MotionPlanningCurve.h>
#include <path_planning_msgs/Path.h>
#include <path_planning_msgs/PathPoint.h>
#include <traffic_light_msgs/traffic_lights.h>
#include <ibeo_lidar_msgs/object_filter_data.h>
#include <ibeo_lidar_msgs/object_filter.h>
#include <ibeo_lidar_msgs/Contour_box.h>
#include <mission_msgs/StartMain.h>
#include <mission_msgs/RequireTurnAround.h>
#include <control_msgs/CoreReport.h>
#include <glog/logging.h>
#include <thread>
#include <mutex>
#include "Point.hpp"
#include "Path.hpp"
#include "PathGenerator.h"
#include "PathUtilities.h"
#include "MessageConverter.h"
#include "Const.hpp"
#include "LineSegment.hpp"
#include "Rectangle.hpp"
#include "Lane.hpp"
#include "Tools.hpp"
#include "InfluenceObstacle.hpp"
#include "Obstacle.hpp"
#include "AvoidancePath.hpp"

// extern bool GLOBAL_IS_IN_GROUND_;
// extern std::mutex GLOBAL_IN_GROUND_MUTEX_;
// extern Rectangle GLOBAL_GROUND_AREA;
// extern bool GLOBAL_IS_IN_JIFEI_;
// extern std::mutex GLOBAL_IN_JIFEI_MUTEX_;
// extern Rectangle GLOBAL_JIFEI_AREA;
// extern bool GLOBAL_IS_IN_CHECK_;
// extern bool GLOBAL_IS_IN_SLOW_;
// extern bool GLOBAL_IS_IN_OVERTAKE_;
// extern int GLOBAL_IS_IN_CHECK_COUNT_FLAG_;

//===================================================== 决策过程内容 ======================================================

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

    // 发布路径（保持速度模式）
    void publishCurveMsgVelocityMaintain(const ros::Publisher &publisher) const {
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
            }
            final_curve.points[i] = point;
        }
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

 private:
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
};

// // 障碍物与自身的关系枚举
// enum ObstacleRelationship {
//     COMPETITION = 0,
//     INTERACTION = 1,
//     BLOCK = 3,
//     NO_INFLUENCE = 4
// };

// 0.初始化车辆和道路信息 1.对状态进行更新（起终点、规划路径、优先级、可行性） 2.判断每一个状态的安全性（加入感知障碍物约束）并补全最终速度
// 3.判断是否进行超车、确定状态 4.保持状态并监视 5.在此基础上不断更新车辆道路信息。
class SubVehicle{
 public:
    // 构造函数和析构函数
    explicit SubVehicle(const ros::NodeHandle &nh) {
        // 获取ros句柄
        this->nh_ = nh;
        // 初始化车辆的状态机
        this->initVehicleStates();
        // 初始化当前状态为停车状态
        this->current_state_ = this->states_set_[StateNames::STOP];
        // 初始化车辆信息
        double vehicle_width, vehicle_length, vehicle_rear_axis_center_scale;
        this->nh_.getParam("vehicle_width", vehicle_width);
        this->nh_.getParam("vehicle_length", vehicle_length);
        this->nh_.getParam("vehicle_rear_axis_center_scale", vehicle_rear_axis_center_scale);
        this->vehicle_width_ = vehicle_width;
        this->vehicle_length_ = vehicle_length;
        this->vehicle_rear_axis_center_scale_ = vehicle_rear_axis_center_scale;

        // 初始化全局变量
        // 获取是否允许超车标志位
        this->nh_.getParam("is_overtake_enable", this->IS_OVERTAKE_ENABLE_FLAG_);
        // 获取环绕雷达是否使用标志位
        this->nh_.getParam("is_surround_radar_enable", this->IS_SURROUND_RADAR_ENABLE_FLAG_);
        // 获取交通灯使用标志位
        this->nh_.getParam("traffic_light_usage_flag", this->TRAFFIC_LIGHT_USAGE_FLAG_);
        // 获取临时空气墙是否使用标志位
        this->nh_.getParam("not_permanent_traffic_rule_usage_flag", this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_);
        // 获取是否为全自动模式标志位
        this->nh_.getParam("is_total_autonomous", this->IS_TOTAL_AUTONOMOUS_FLAG_);
        // 获取是否允许倒车和原地转向
        this->nh_.getParam("rotate_and_reverse_enable", this->ROTATE_AND_REVERSE_ENABLE_FLAG_);

        // 初始化ros相关节点和服务
        this->rosInit();

        ROS_INFO("INITAL SUCCESS");
        std::cout << "IS_OVERTAKE_ENABLE_FLAG: " << this->IS_OVERTAKE_ENABLE_FLAG_ << std::endl;
        std::cout << "IS_SURROUND_RADAR_ENABLE_FLAG: " << this->IS_SURROUND_RADAR_ENABLE_FLAG_ << std::endl;
        std::cout << "IS_TRAFFIC_LIGHT_USAGE_FLAG: " << this->TRAFFIC_LIGHT_USAGE_FLAG_ << std::endl;
        std::cout << "NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG: " << this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_ << std::endl;
        std::cout << "IS_TOTAL_AUTONOMOUS_FLAG: " << this->IS_TOTAL_AUTONOMOUS_FLAG_ << std::endl;
        std::cout << "ROTATE_AND_REVERSE_ENABLE_FLAG: " << this->ROTATE_AND_REVERSE_ENABLE_FLAG_ << std::endl;
    }

    ~SubVehicle() {}

    // 初始化并启动线程
    void runMotionPlanning();

 private:
    // 状态机初始化
    void initVehicleStates();

    // 初始化ros节点
    void rosInit();

    // 启动ros订阅线程,50hz
    void listenRosMSGThread();

    // 更新车辆位置信息，ros节点
    void updateVehiclePosition(const nav_msgs::Odometry::ConstPtr odometry_msg);

    // 更新车辆速度和速度朝向
    void updateVehicleMovement(const std_msgs::Float64::ConstPtr velocity_msg);

    // 更新车辆曲率
    void updateVehicleCurvature(const std_msgs::Float64::ConstPtr curvature_msg);

    // 更新控制报告信息
    void updateControlReport(const control_msgs::CoreReport::ConstPtr control_report_msg);
    
    // 更新转向标志位
    void updateCurveTurn(const std_msgs::Empty::ConstPtr curve_turn_msg);

    // 更新毫米波雷达信息
    void updateSurroundRadarInfo(const dbw_mkz_msgs::SurroundReport::ConstPtr radar_msgs);

    // 开始任务
    bool startMission(mission_msgs::StartMainRequest &request ,mission_msgs::StartMainResponse &response);

    // 强制停车
    bool forcedStop(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);

    // 判断任务结束
    bool missionFinishJudgement();

    // 规划和决策线程,30hz
    void motionPlanningThread();

    // 更新地图信息，ros服务(TODO)
    void updateMapInformation();

    // 更新状态
    // 1. 根据当前状态判断可行状态（停车恒定可行，避障在速度大于阈值时恒定不可行）
    // 2. 给每一个可行状态赋值起终点、起点速度和规划出的局部路径(不包含停车和避障)
    // 3. 给每一个可行状态优先级
    void updateStates();

    // 规划出某个状态在对应道路上的行驶轨迹,为更新状态服务
    void updateLaneTrajectoryforStates(StandardState *standard_state, const Lane &lane, const PathPlanningUtilities::VehicleState &start_point_in_world, const PathPlanningUtilities::VehicleMovementState &start_point_movement, double start_point_steering_angle);

    // 辅助状态更新
    // 更新停车状态，并选择停车状态
    void generateStopState(PathPlanningUtilities::Point2f goal_point, std::string lane_type);

    // 辅助状态更新
    // 更新避障状态，并选择避障状态
    void generateObstacleAvoidanceState(PathPlanningUtilities::Point2f goal_point);

    // 障碍物信息callback函数，ros节点
    void getObstacles(const ibeo_lidar_msgs::object_filter_data::ConstPtr &msg);

    // 历史路径callback函数,ros节点
    void getHistoryCurve(const path_planning_msgs::MotionPlanningCurve &msg);

    // 补全障碍物全部信息(与感知部分对接)
    void updateObstacleInformation();

    // 补全障碍物的状态信息，其中acceleration和class_name为无用值，只要给占位符即可
    void updateObstacleState(Obstacle* obstacle, const PathPlanningUtilities::Point2f position, double width, double length, double orientation, double velocity, double velocity_direction, double acceleration, size_t class_name);

    // 补全障碍物的预测信息，包括轨迹和位置
    void updateObstaclePredictedInformation(Obstacle* obstacle);

    // 得到有效的交通障碍物列表
    void updateValidateTrafficRuleInformation();

    // 判断状态的安全性（不包含避障和停车）
    // 根据优先级高低查询障碍物对路径影响、加入观察距离内是否存在静态障碍物、最高速度约束、速度变化约束等，计算出可行速度,如果没有可行速度则放弃
    // 静态障碍物出现在路径上则为换道直接放弃、高速变低速、低速变避障。
    // 详解：如果换道最优先，则判断其路径以及观测距离内有无静态障碍物，再判断是否存在一定速度区间实现安全换道；
    // 如果直行最优先，判断路径及观测距离内有无静态障碍物，如果有，高速进入低速模式，如果是再路径内则低速变为避障模式。
    void checkStates();

    // 生成低速机动状态（包括避障和停车）
    void generateLowVelocityState();

    // 生成低速避障路径
    void AvoidancePathGenerator(const Lane &lane, std::vector<DecisionMaking::AvoidancePath> &avoidance_curves);

    // 进行低速避障路径选择
    size_t selectAvoidancePath(const std::vector<DecisionMaking::AvoidancePath> &avoidance_paths);

    // // 判断是否有需要填充低优先级状态的信息，如果有，进行填充(可能不需要)
    // void generateLowPriorityStates();

    // 填充特殊状态的信息
    void generateSpecialState();

    // 生成倒车状态
    int generateReverseState();

    // 生成转向状态
    int generateRotateState();

    // 进行状态选择
    // 如果优先度最高的状态可行、选最高的（同时判断是否需要超车）、如果优先度最高状态不可行、进行判别,如果状态都不行则进入停车状态。
    void chooseStates();

    // 进行可视化
    void Visualization();

    // 根据上一次状态和此次选中的状态判断规划模块是否已经无效
    bool motionPlanningUncapableJudgement();

    // 进行车辆和障碍物状态更新，判断当前状态是否安全，判断期望状态是否可行，进行动态速度规划。
    void maintainStates();

    // 判断停车状态是否需要继续，还是重新进行规划
    void stopStateMaintain();

    // 判断避障状态是否需要继续，还是重新进行规划
    void avoidanceStateMaintain();

    // 特殊状态保持
    void specialStateMaintain();

    // 速度规划（为了进行多线程操作才创建的函数）(要注意线程安全性)
    void velocityPlanningForState(StandardState *judge_state, const std::vector<Obstacle> &obstacles, bool is_aggressive);

    // 自检测函数
    void selfTestForProgress(diagnostic_updater::DiagnosticStatusWrapper& status);

    // 判断道路是否被障碍物所占据
    void judgeStateCorrespondingLaneBeingOccupied(StandardState *judge_state, const std::vector<Obstacle> &obstacles);

    // 计算状态的曲线中最大曲率点
    double calcMaxKappaForState(const StandardState &state, size_t length);

    // 计算状态的曲线中的最大曲率变化率
    double calcMaxCurvatureChangeRateForState(const StandardState &state, size_t length);

    // 对状态机可行状态进行排序
    bool sortStatesPriority(std::vector<DecisionMaking::StandardState> *states_set);

    // ros相关变量
    ros::NodeHandle nh_;  // ros句柄
    tf::TransformListener* tf_listener_ptr_;  // tf监听器
    tf::StampedTransform tf_transformer_;  // tf变换矩阵
    ros::ServiceClient map_service_client_;  // 地图服务
    ros::ServiceClient obstacle_trajectory_prediction_service_client_;  // 障碍物轨迹预测服务
    ros::ServiceClient traffic_light_service_client_;  // 交通灯服务
    ros::ServiceClient destination_reached_service_client_;  // 到达目标点消息服务
    ros::ServiceClient motion_planning_uncapable_client_;  // A星掉头服务
    ros::ServiceClient motion_planning_failed_client_;  // 报告无法规划服务
    ros::Publisher visualization_pub_;  // 可视化发布节点
    ros::Publisher vis_vehicle_pub_;  // 可视化车辆发布节点
    ros::Publisher vis_obstacle_pub_;  // 可视化障碍物发布节点
    ros::Publisher vis_occupation_pub_;  // 可视化车辆占用区节点
    ros::Publisher vis_collision_pub_;  // 可视化碰撞位置发布节点
    ros::Publisher vis_multi_curve_pub_;  // 可视化生成的全部路径节点
    ros::Publisher vis_influence_obstacle_pub_;  // 可视化有效障碍物信息节点
    ros::Publisher motion_planning_curve_pub_;  // 最终轨迹发布节点
    ros::Publisher turn_signal_pub_;  // 转向灯发布节点
    ros::Publisher emergency_break_pub_;  // 紧急停车布节点
    ros::Subscriber history_curve_sub_;  //历史轨迹更新
    ros::Subscriber odom_sub_;  // 位置更新节点
    ros::Subscriber movement_sub_;  // 速度更新节点
    ros::Subscriber curvature_sub_;  // 曲率更新节点
    ros::Subscriber control_report_sub_;  // 控制报告节点
    ros::Subscriber obstacle_sub_;  // 障碍物信息更新节点
    ros::Subscriber surround_radar_sub_;  // 毫米波雷达预警节点
    ros::ServiceServer mission_start_service_server_;  // 任务开始服务
    ros::ServiceServer forced_stop_service_server_;  // 强制停车服务
    self_test::TestRunner self_test_;  // 自检测对象

    //状态机相关变量
    std::vector<StandardState> states_set_;  // 状态机
    StandardState current_state_;  // 当前的状态
    StandardState choosed_state_;  // 选择的状态
    StandardState expected_state_;  // 期望的状态
    StandardState special_state_;  // 特殊状态

    // 地图信息变量
    // 左车道信息
    // 右车道信息
    // 直行车道信息(包含车道中心线、车道宽度、停止线位置、路口白线位置，道路特殊标记点，路口是否存在红绿灯，最高限速等)
    // 当前位置的道路最高限速
    // 是否为单车道
    Lane left_lane_;
    Lane right_lane_;
    Lane center_lane_;
    size_t guidance_type_;
    double expected_velocity_upper_bound_;
    bool is_avoidance_capable_;
    bool is_single_lane_;
    double remain_distance_;
    double distance_to_goal_;

    // 全部障碍物信息
    std::vector<ibeo_lidar_msgs::object_filter> perception_objects_;  // 感知的物体信息 
    std::vector<Obstacle> obstacles_;   // 障碍物列表
    std::vector<vec_map_cpp_msgs::VirtualObstacle> traffic_rule_obstacles_raw_;  // 由原始交通规则生成的障碍物
    std::vector<vec_map_cpp_msgs::VirtualObstacle> traffic_rule_obstacles_;  // 由上者得到的有效交通规则障碍物

    // 车辆信息变量
    double vehicle_width_;  // 车辆宽度
    double vehicle_length_;  // 车辆长度
    double vehicle_rear_axis_center_scale_;  // 车辆后轴中心占车长比例
    PathPlanningUtilities::VehicleState current_vehicle_world_position_;  // 车辆位置信息、朝向信息(世界坐标系下)
    PathPlanningUtilities::VehicleMovementState current_vehicle_movement_;  // 车辆的速度信息
    double current_vehicle_kappa_;  // 车辆曲率信息
    geometry_msgs::PoseStamped destination_pose_;  // 行驶的目的地
    PathPlanningUtilities::Curve history_curve_;  // 历史行驶轨迹

    // 标志位变量
    bool VEHICLE_POSITION_READY_FLAG_ = false;  // 车辆位置加载成功
    bool VEHICLE_MOVEMENT_READY_FLAG_ = false;  // 车辆运动信息加载成功
    bool VEHICLE_CURVATURE_READY_FLAG_ = false;  // 车辆曲率信息加载成功
    bool VEHICLE_SURROUND_RADAR_READY_FLAG_ = false;  // 车辆毫米波信息加载成功
    bool MISSION_START_FLAG_ = false;  // 开始任务
    bool TRAFFIC_LIGHT_USAGE_FLAG_ = false;  // 交通灯是否使用
    bool NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_ = false;  // 临时空气墙是否使用
    bool IS_DANGEROUS_FLAG_ = false;  // 是否有障碍物导致状态不安全
    bool IS_OVERTAKE_ENABLE_FLAG_ = false;  // 是否允许高速主动换道
    bool IS_SURROUND_RADAR_ENABLE_FLAG_ = false;  // 是否使用毫米波雷达
    bool IS_EMERGENCY_BREAK_FLAG_ = false;  // 紧急停车标志
    bool IS_TOTAL_AUTONOMOUS_FLAG_ = false;  // 是否为全自动模式
    bool CONTROL_FINISHED_FLAG_ = false;  // 控制完成信号
    bool ROTATE_AND_REVERSE_ENABLE_FLAG_ = false;  // 允许进行倒车或转向

    // 用于自检的标志位变量
    bool SELF_TEST_INIT_ = false;  // 目前处在初始化阶段
    bool SELF_TEST_PLANNING_ = false;  // 目前处在规划阶段
    bool SELF_TEST_DECISION_MAKING_ = false;  // 目前处于决策阶段
    bool SELF_TEST_STATE_MAINTAINING_ = false;  // 目前处在状态保持阶段

    // 多重标志位变量
    size_t special_state_type_;  // 特殊状态类型(TOFIX)
    int stop_count_recorder_ = 0;  // 障碍物停车次数计数器
    int low_frequency_stop_count_recorder_ = 0;  // 低频障碍物停车计数器
    bool is_length_enough_ = false;  // 是否有足够的距离进行换道
    bool right_alert_ = false;  // 右侧毫米波报警
    bool left_alert_ = false;  // 左侧毫米波报警
    

    //变量互斥锁
    std::mutex vehicle_position_ready_flag_mutex_;  // 车辆位置加载成功锁
    std::mutex vehicle_movement_ready_flag_mutex_;  // 车辆运动信息加载成功锁
    std::mutex vehicle_curvature_ready_flag_mutex_;  // 车辆曲率信息加载成功锁
    std::mutex current_vehicle_world_position_mutex_;  // 车辆世界坐标系位置锁
    std::mutex current_vehicle_movement_mutex_;  //车辆速度信息锁
    std::mutex current_vehicle_kappa_mutex_;  // 车辆曲率锁
    std::mutex destination_mutex_;  // 目的地信息锁
    std::mutex mission_start_mutex_;  // 任务开始标志锁
    std::mutex obstacle_mutex_;  // 障碍物锁
    std::mutex perception_object_mutex_;  // 感知物体锁
    std::mutex right_alert_mutex_;  // 右报警锁
    std::mutex left_alert_mutex_;  // 左报警锁
    std::mutex vehicle_surround_radar_ready_flag_mutex_;  // 毫米波信息锁
    std::mutex emergency_break_flag_mutex_;  // 紧急停车锁
    std::mutex history_curve_mutex_;  // 历史路径锁
    std::mutex control_finished_flag_mutex_;  // 控制完成标志位锁
};

namespace RSS {

// 交通规则判定
void trafficRuleCheck(StandardState *judge_state, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &obstacles);

// 判断状态的路径是否与静态障碍物重合
bool checkStateBeingBlocked(StandardState *judge_state, const Obstacle &obstacle, size_t lane_index);

// 得到期望的加速度区间
SectionSet getAccelerationSectionSet(StandardState *judge_state, const std::vector<Obstacle> &obstacles, bool is_aggressive);

// 调整车辆状态来满足安全模型
SectionSet checkSaftyModel(StandardState *judge_state, const std::vector<Obstacle> &obstacles, bool is_aggressive);

// 判断当前状态是否安全
bool stateSafetyJudgement(const StandardState &judge_state, const std::vector<Obstacle> &obstacles);

// 得到交通规则与路径碰撞的最近碰撞点在路径中的下标，返回值是true表示发生碰撞，false表示未发生碰撞
bool collisionPositionIndexInCurve(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_rule_obstacles, size_t *cut_index);

// 得到障碍物与路径碰撞的最近碰撞点在路径中的下标（不带距离判断），返回值是true表示发生碰撞，false表示未发生碰撞
bool collisionPositionIndexInCurve(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, double vehicle_velocity, const std::vector<Obstacle> &obstacles, size_t *cut_index);

// 得到障碍物与路径碰撞的最近碰撞点在路径中的下标(带距离判断)，返回值是true表示发生碰撞，false表示未发生碰撞
bool collisionPositionIndexInCurve(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, double vehicle_velocity, const std::vector<Obstacle> &obstacles, size_t *cut_index, bool is_big);

bool collisionWithDynamicObstacles(const PathPlanningUtilities::Curve &judge_curve, double velocity, double vehicle_width, double vehicle_length, const std::vector<Obstacle> &obstacles, size_t *cut_index);

// 判断蔽障道路是否可以快速通过
bool isAvoidanceQuickPass(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, const std::vector<Obstacle> &obstacles);

// 占用区域类（障碍物或本车占用的区域）
class OccupationArea {
 public:
    OccupationArea() {}

    ~OccupationArea() {}

    // 构造障碍物类型的占用区，占用区为障碍物在第lane_index条路径上的占用区
    explicit OccupationArea(const Obstacle &obstacle, size_t lane_index, int sample_gap = 3, bool as_occupation = true);

    // 构造障碍物类型的占用区，占用区为障碍物在第lane_index条路径上的占用区
    explicit OccupationArea(const Obstacle &obstacle, size_t lane_index, double velocity, int sample_gap = 3);

    // 构造车道类型的占用区
    explicit OccupationArea(const StandardState &judge_state, int sample_gap = 30, double width_expanded_factor = 1.2, double length_expanded_factor = 1.1);

    // 构造空气墙类型的占用区
    explicit OccupationArea(const vec_map_cpp_msgs::VirtualObstacle &obstacle, int sample_gap = 1);

    // 构造曲线类型的占用区
    explicit OccupationArea(const PathPlanningUtilities::Curve &curve, double area_width, double area_length, int sample_gap = 20);
    
    // 获取占用区域
    const std::vector<Rectangle> &getOccupationArea() const {
        return this->occupation_area_;
    }

    // 获取下采样后的占用区域
    const std::vector<Rectangle> &getSampledOccupationArea() const {
        return this->sampled_occupation_area_;
    }

    // 获取下采样后占用区域每一个矩形对应原来路径的下标
    const std::vector<size_t> &getSampledOccupationAreaBijectionIndex() const {
        return this->sampled_occupation_area_bijection_indexes_;
    }

    size_t getSampledOccupationAreaBijectionIndex(size_t index) const {
        return this->sampled_occupation_area_bijection_indexes_[index];
    }

    // 设置和获取优先权
    void setPriority(bool is_priority) {
        this->is_priority = is_priority;
    }

    bool getPriority() const {
        return this->is_priority;
    }

 private:
    // 采样占用区域
    void samplingOccupationArea(int sampling_gap);

    bool is_priority;  // 是否具有优先权
    std::vector<Rectangle> occupation_area_;  // 占用区域
    std::vector<Rectangle> sampled_occupation_area_;  // 降采样后的占用区域
    std::vector<size_t> sampled_occupation_area_bijection_indexes_;  // 降采样后每一个占用区矩形对应的原来路径的下标
};

// 判断两个占用区域是否产生相交, true表示相交，false表示不相交
bool occupationInteractionJudgement(const OccupationArea &subvehicle_occupation_area, const OccupationArea &obstacle_occupation_area, size_t *subvehicle_interact_index, size_t *obstacle_interact_index);

// 获取障碍物到路径的距离
double getObstacleDistanceToCurve(const PathPlanningUtilities::Curve &curve, const Obstacle &obstacle);

// 计算两个占用区域之间的距离
void occupationDistanceCalculation(const OccupationArea &subvehicle_occupation_area, const OccupationArea &obstacle_occupation_area, double distance_upper, std::vector<double>* result);

// 计算障碍物与路径之间的距离
void obstacleDistanceJudgment(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, double vehicle_velocity, const std::vector<Obstacle> &obstacles, std::vector<double>* result);

// 计算交通障碍物与路径之间的距离
void obstacleDistanceJudgment(const PathPlanningUtilities::Curve &judge_curve, double vehicle_width, double vehicle_length, const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_rule_obstacles, std::vector<double>* result);

};  // namespace RSS

}  // namespace DecisionMaking

//==================================================== 可视化函数 =========================================================

// 可视化函数集合(将不同类型转为marker)
namespace VisualizationMethods {
// 可视化起始id
enum VisualizationID {
    STATES_START_ID = 0,
    STATES_END_ID = 10,
    TURN_LEFT_ID = 100,
    TURN_RIGHT_ID = 101,
    FOWARD_ID = 102,
    EXTENDED_TURN_LEFT_ID = 103,  // DEBUG
    EXTENDED_TURN_RIGHT_ID = 104,  // DEBUG
    EXTENDED_FOWARD_ID = 105,  // DEBUG
    AVOIDANCE_START_ID = 200,
    AVOIDANCE_EXTEND_ID = 250,
    STOP_ID = 300,
    OUTOFTRAP_ID = 350,
    REVERSE_ID = 400,
    CHOOSED_CURVE_START_ID = 500,
    CHOOSED_CURVE_END_ID = 3000,
    LEFT_LANE_ID = 3001,
    RIGHT_LANE_ID = 3002,
    CENTER_LANE_ID = 3003,
    VEHICLE_INFO_VEL = 4000,
    VEHICLE_INFO_ANGLE = 4001,
    VEHICLE_INFO_SAFETY = 4002,
    TRAFFIC_RULE_START_ID = 5000,
    OCCUPATION_AREA_START_ID = 6000,
    OBSTACLE_SHAPE_START_ID = 10000,
    OBSTACLE_TRAJECTORY_START_ID = 20000,
    OBSTACLE_VELOCITY_START_ID = 30000,
    OBSTACLE_VELOCITY_DIRECTION_START_ID = 40000,
    VEHICLE_ODOM = 60000
};

// 将道路转化为marker
visualization_msgs::Marker visualizeLaneToMarker(const Lane &lane, const std_msgs::ColorRGBA &color, int id);

// 将路径转化为marker
visualization_msgs::Marker visualizeCurvesToMarker(const PathPlanningUtilities::Curve &curve, const std_msgs::ColorRGBA &color, int id);

// 将矩形框转为marker
visualization_msgs::Marker visualizeRectToMarker(double position_x, double position_y, double theta, double width, double length, double center_scale, const std_msgs::ColorRGBA &color, int id);

// 将文本转为为marker
visualization_msgs::Marker visualizeStringToMarker(const std::string &text, double position_x, double position_y, const std_msgs::ColorRGBA &color, int id);

// 删除可视化marker
visualization_msgs::Marker visualizedeleteMarker(int id);

// 删除从起始id开始的全部可视化marker
visualization_msgs::Marker visualizedeleteAllMarker(int start_id);

// 将状态机可视化在车辆规划位置附近
void visualizeStates(const std::vector<DecisionMaking::StandardState> &state_set, const DecisionMaking::StandardState &choosed_state, double position_x, double position_y, int start_id, const ros::Publisher &publisher);

// 可视化选中的路径及车辆延路径行驶的边界
void visualizeChoosedCurveWithBoundary(const PathPlanningUtilities::Curve &choosed_curve, double width, double length, double center_scale, int start_id, int gap, const ros::Publisher &publisher);

// 可视化车辆状态信息
void visualizeVehicleState(double position_x, double position_y, const std::string &velocity, const std::string &wheel_angle, const std::string &safety, const ros::Publisher &publisher);

// 可视化交通规则
void visualizeTrafficRules(const std::vector<vec_map_cpp_msgs::VirtualObstacle> &traffic_obstacles, const ros::Publisher &publisher);

// 可视化本车占用区域
void visualizeSubvehicleOccupationArea(const DecisionMaking::StandardState &choosed_state, double center_scale, const ros::Publisher &publisher, size_t gap = 20);

// 将障碍物形状转化为marker
visualization_msgs::Marker visualizeObstacleShape(const DecisionMaking::Obstacle &obstacle, int id);

// 将障碍物预测轨迹转化为marker组
std::vector<visualization_msgs::Marker> visualizeObstacleTrajectory(const DecisionMaking::Obstacle &obstacle, int start_id);

// 将障碍物速度转化为marker
visualization_msgs::Marker visualizeObstacleVelocity(const DecisionMaking::Obstacle &obstacle, int id);

// 将障碍物速度方向转化为marker
visualization_msgs::Marker visualizeObstacleVelocityDirection(const DecisionMaking::Obstacle &obstacle);

// 可视化障碍物
void visualizeObstacles(const std::vector<DecisionMaking::Obstacle> &obstacles, const ros::Publisher &publisher);

// 本车碰撞点可视化
void visualizeInteractionPosition(std::vector<Rectangle> *rectangles, const ros::Publisher &publisher);

// 多条轨迹可视化
void visualizeMultiCurves(const std::vector<PathPlanningUtilities::Curve> &curves, const ros::Publisher &publisher);

// 在特定位置生成圆柱marker
visualization_msgs::Marker visualizePosition(double position_x, double position_y, std_msgs::ColorRGBA color, int id);

// 可视化有效障碍物
void visualizeInfluenceObstacles(const std::vector<InfluenceObstacle> &influence_obstacles, const ros::Publisher &publisher);

}  // namespace VisualizationMethods

#endif  // SRC_MULTI_PATH_GENERATOR_INCLUDE_COMMON_HPP_
