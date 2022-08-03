/*
    Copyright [2020] Jian ZhiQiang
*/

#ifndef AVOIDANCE_PATH_INCLUDE_COMMON_HPP_
#define AVOIDANCE_PATH_INCLUDE_COMMON_HPP_

#include "Point.hpp"
#include "Path.hpp"
#include "Tools.hpp"
#include "Compare.hpp"

namespace DecisionMaking {
// 避障道路类
class AvoidancePath {
 public:
    // 构造函数
    AvoidancePath(const PathPlanningUtilities::Curve &curve, double lateral_offset, double lateral_movement, double priority, int sample_gap = 1) {
        this->curve_ = curve;
        this->lateral_offset_ = lateral_offset;
        this->lateral_movement_ = lateral_movement;
        this->priority_ = priority;
        this->max_collision_risk_ = -1.0;
        this->max_kappa_ = curve[Tools::calcMaxKappaIndexForCurve(curve)].kappa_;
        this->obstacle_cut_index_ = curve.size() - 1;
        this->traffic_cut_index_ = curve.size() - 1;
        this->initSampleIndex(sample_gap);
    }

    AvoidancePath() {}

    // 析构函数
    ~AvoidancePath() {}

    // 保存障碍物碰撞信息
    void saveCollisionInfo(const std::vector<double>& collision_risk_to_obs, const std::vector<double>& collision_risk_to_traffic_rule) {
        this->collision_risk_to_obs_ = collision_risk_to_obs;
        this->collision_risk_to_traffic_rule_ = collision_risk_to_traffic_rule;
        // 计算碰撞点信息
        auto obs_smallest = std::min_element(std::begin(this->collision_risk_to_obs_), std::end(this->collision_risk_to_obs_));
        auto tra_smallest = std::min_element(std::begin(this->collision_risk_to_traffic_rule_), std::end(this->collision_risk_to_traffic_rule_));
        this->max_collision_risk_ = *obs_smallest;
        if (Tools::isZero(*obs_smallest)) {
            this->obstacle_cut_index_ = std::distance(std::begin(this->collision_risk_to_obs_), obs_smallest);
        }
        if (Tools::isZero(*tra_smallest)) {
            this->traffic_cut_index_ = std::distance(std::begin(this->collision_risk_to_traffic_rule_), tra_smallest);
        }
    }

    // 获取离障碍物的最小距离
    double getMinDistanceToObstacle() const {
        return this->max_collision_risk_;
    }

    // 获取避障路径
    const PathPlanningUtilities::Curve &getTrajectory() const {
        return this->curve_;
    }

    // 得到采样路径
    PathPlanningUtilities::Curve getSampledTrajectory() const {
        PathPlanningUtilities::Curve curve;
        for (auto i: this->sampled_index_) {
            curve.push_back(this->curve_[i]);
        }
        return curve;
    }

    // 得到采样点
    const std::vector<size_t> &getSampleIndexes() const {
        return this->sampled_index_;
    }

    // 获取路径最大曲率
    double getMaxKappa() const {
        return this->max_kappa_;
    }

    const std::vector<double>& getTraCollisionInfo() const {
        return this->collision_risk_to_traffic_rule_;
    }

    const std::vector<double>& getObsCollisionInfo() const {
        return this->collision_risk_to_obs_;
    }

   // 获得截断点中最近的一个
    int getCutIndex() const {
        return std::min(this->obstacle_cut_index_, this->traffic_cut_index_);
    }

    // 判断是否碰撞
    bool isCollision() const {
        // 只要存在较小截断点，就存在碰撞
        if (this->getCutIndex() == static_cast<int>(this->curve_.size() - 1)) {
            // 无截断点
            return false;
        } else {
            // 有截断点
            return true;
        }
    }

    // 得到最近碰撞点的影响类型
    int getInfluenceType() const {
        if (this->traffic_cut_index_ == static_cast<int>(this->curve_.size() - 1) && this->obstacle_cut_index_ == static_cast<int>(this->curve_.size() - 1)) {
            // 完全没有发生碰撞
            return -1;
        } else {
            int min_cut_index = std::min(this->traffic_cut_index_, this->obstacle_cut_index_);
            if (min_cut_index == this->obstacle_cut_index_) {
                // 障碍物碰撞
                return 0;
            } else {
                // 交通障碍物碰撞
                return 1;
            }
        }
    }

    // 获取横向偏移
    double getLateralOffset() const {
        return this->lateral_offset_;
    }

    // 获得横向移动距离
    double getLateralMovement() const {
        return this->lateral_movement_;
    }

    // 获得优先级
    double getPriority() const {
        return this->priority_;
    }

    // 设置脱困模式
    void setOutofTrapping(bool is_outof_trapping) {
        this->is_outof_trapping_ = is_outof_trapping;
    }

    // 获取是否为脱困模式
    bool getOutofTrapping() const {
        return this->is_outof_trapping_;
    }

 private:

    // 初始化采样下标
    void initSampleIndex(int sample_gap) {
        for (size_t i = 0; i < this->curve_.size(); i += sample_gap) {
            this->sampled_index_.push_back(i);
        }
        if (this->sampled_index_[this->sampled_index_.size() - 1] != this->curve_.size() - 1) {
            this->sampled_index_.push_back(this->curve_.size() - 1);
        }
    }

    PathPlanningUtilities::Curve curve_;  // 避障路径
    double max_kappa_;  // 路径的最大曲率
    int obstacle_cut_index_;  // 障碍物截断点
    int traffic_cut_index_;  // 交通规则截断点
    double lateral_offset_;  // 路径的横向偏移量
    double lateral_movement_;  // 需要进行的横向移动距离
    double max_collision_risk_;  // 最大碰撞风险
    std::vector<double> collision_risk_to_obs_;  // 路径与障碍物的碰撞风险
    std::vector<double> collision_risk_to_traffic_rule_;  // 路径与道路边界的碰撞风险
    std::vector<size_t> sampled_index_;  // 用于碰撞检测的采样下标
    double priority_;  // 优先级
    bool is_outof_trapping_ = false;  // 是否为脱困模式

};

};

#endif