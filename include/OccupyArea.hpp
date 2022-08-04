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
#include <vec_map_cpp_msgs/LocalLane.h>
#include <vec_map_cpp_msgs/VirtualObstacle.h>
#include <vec_map_cpp_msgs/GetGuidedCurves.h>
#include <vec_map_cpp_msgs/GetPredictedTrajectory.h>

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
#include "StandardState.hpp"

namespace DecisionMaking {

namespace RSS{




// 占用区域类（障碍物或本车占用的区域）
class OccupationArea {
 public:
    OccupationArea() {}

    ~OccupationArea() {}

    // 构造障碍物类型的占用区，占用区为障碍物在第lane_index条路径上的占用区
    OccupationArea(const Obstacle &obstacle, size_t lane_index, int sample_gap = 3, bool as_occupation = true);

    // 构造障碍物类型的占用区，占用区为障碍物在第lane_index条路径上的占用区
    OccupationArea(const Obstacle &obstacle, size_t lane_index, double velocity, int sample_gap = 3);

    // 构造车道类型的占用区
    OccupationArea(const StandardState &judge_state, int sample_gap = 30, double width_expanded_factor = 1.2, double length_expanded_factor = 1.1);

    // 构造空气墙类型的占用区
    OccupationArea(const vec_map_cpp_msgs::VirtualObstacle &obstacle, int sample_gap = 1);

    // 构造曲线类型的占用区
    OccupationArea(const PathPlanningUtilities::Curve &curve, double area_width, double area_length, int sample_gap = 20);
    
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
}
}