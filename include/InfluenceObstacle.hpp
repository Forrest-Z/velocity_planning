/*
    Copyright [2020] Jian ZhiQiang
*/

#ifndef INFLUENCE_OBSTACLE_INCLUDE_COMMON_HPP_
#define INFLUENCE_OBSTACLE_INCLUDE_COMMON_HPP_

#include "Point.hpp"
#include "Path.hpp"

// 有影响的障碍物类，用于记录对状态造成影响的障碍物和虚拟障碍物
class InfluenceObstacle {
 public:
    enum InfluenceObstacleType{
        VIRTUAL_OBSTACLE = 0,
        STATIC_OBSTACLE = 1,
        DYNAMIC_OBSTACLE = 2
    };

    std::vector<std::string> dic_type = {"VIRTUAL_OBSTACLE", "STATIC_OBSTACLE", "DYNAMIC_OBSTACLE"};

    enum InfluenceObstacleInfluence {
        ACCELERATION = 0,
        DECCELERATION = 1,
        ACCELERATION_AND_DECCELERATION = 2,
        UNSAFE = 3,
        NONE = 4
    };

    std::vector<std::string> dic_influence = {"ACCELERATION", "DECCELERATION", "ACCELERATION_AND_DECCELERATION", "UNSAFE", "NONE"};

    enum InfluenceObstacleRelationship {
        INTERACTION = 0,
        COMPETATION =1
    };

    std::vector<std::string> dic_relationship = {"INTERACTION", "COMPETATION"};

    // 构造函数
    InfluenceObstacle(){}

    // 析构函数
    ~InfluenceObstacle(){}

    // 设置类型
    void setInfluenceObstacleType(InfluenceObstacleType type) {
        this->type_ = type;
    }

    InfluenceObstacleType getInfluenceObstacleType() const {
        return this->type_;
    }

    // 设置中心坐标
    void setInfluenceObstacleCenterPosition(PathPlanningUtilities::Point2f center_position) {
        this->center_position_ = center_position;
    }

    PathPlanningUtilities::Point2f getInfluenceObstacleCenterPosition() const {
        return this->center_position_;
    }

    // 设置速度
    void setInfluenceObstacleVelocity(double velocity) {
        this->velocity_ = velocity;
    }

    double getInfluenceObstacleVelocity() const {
        return this->velocity_;
    }

    // 设置障碍物碰撞点坐标
    void setInfluenceObstacleCollisionPosition(PathPlanningUtilities::Point2f obstalce_collision_position) {
        this->obstalce_collision_position_ = obstalce_collision_position;
    }

    PathPlanningUtilities::Point2f getInfluenceObstacleCollisionPosition() const {
        return this->obstalce_collision_position_;
    }

    // 设置本车碰撞点坐标
    void setInfluenceSubvehicleCollisionPosition(PathPlanningUtilities::Point2f subvehicle_collision_position) {
        this->subvehicle_collision_position_ = subvehicle_collision_position;
    }

    PathPlanningUtilities::Point2f getInfluenceSubvehicleCollisionPosition() const {
        return this->subvehicle_collision_position_;
    }

    // 设置障碍物离碰撞距离
    void setInfluenceObstacleCollisionDistance(double obstacle_distance_to_collision) {
        this->obstacle_distance_to_collision_ = obstacle_distance_to_collision;
    }

    double getInfluenceObstacleCollisionDistance() const {
        return this->obstacle_distance_to_collision_;
    }

    // 设置本车到障碍物距离
    void setInfluenceSubvehicleCollisionDistance(double subvehicle_distance_to_collision) {
        this->subvehicle_distance_to_collision_ = subvehicle_distance_to_collision;
    }

    double getInfluenceSubvehicleCollisionDistance() const {
        return this->subvehicle_distance_to_collision_;
    }

    // 设置障碍物对本车的影响
    void setInfluenceObstacleInfluence(InfluenceObstacleInfluence influence) {
        this->influence_ = influence;
    }

    InfluenceObstacleInfluence getInfluenceObstacleInfluence() const {
        return this->influence_;
    }

    // 设置障碍物与本车之间的关系
    void setInfluenceObstacleRelationship(InfluenceObstacleRelationship relationship) {
        this->relationship_ = relationship;
    }

    InfluenceObstacleRelationship getInfluenceObstacleRelationship() const {
        return this->relationship_;
    }

    // 设置障碍物轨迹预测
    void setInfluenceObstaclePredictedPath (const PathPlanningUtilities::Curve &predicted_path) {
        this->predicted_path_ = predicted_path;
    }

    const PathPlanningUtilities::Curve &getInfluenceObstaclePredictedPath() const{
        return this->predicted_path_;
    }
    
    // 描述信息
    std::string getInfuenceObstacleDescription() const{
        std::stringstream description;
        description << "类型" << dic_type[this->type_] << std::endl;
        description << "中心坐标" << this->center_position_.x_ << "||" << this->center_position_.y_ << std::endl;
        description << "速度" << this->velocity_ << std::endl;
        description << "障碍物离碰撞点距离" << this->obstacle_distance_to_collision_ << "米" << std::endl;
        description << "本车离碰撞点距离" << this->subvehicle_distance_to_collision_ << "米" << std::endl;
        description << "对本车的影响" << dic_influence[this->influence_] << std::endl;
        if (this->type_ == InfluenceObstacleType::DYNAMIC_OBSTACLE){
            description << "与本车的关系" << dic_relationship[this->relationship_] << std::endl;
        }
        return description.str();
    }


 private:
    InfluenceObstacleType type_;  // 障碍物的种类
    PathPlanningUtilities::Point2f center_position_;  // 障碍物中心坐标
    double velocity_;  // 障碍物速度
    PathPlanningUtilities::Point2f subvehicle_collision_position_;  // 本车碰撞点坐标
    PathPlanningUtilities::Point2f obstalce_collision_position_;  // 障碍物碰撞点坐标
    double obstacle_distance_to_collision_;  // 障碍物离碰撞点的距离
    double subvehicle_distance_to_collision_;  // 本车离碰撞点的距离
    InfluenceObstacleInfluence influence_;  // 障碍物对本车的影响（需要本车加速还是减速来避免危险）
    // --------------------- 动态障碍物特有属性----------------------
    InfluenceObstacleRelationship relationship_;  // 障碍物与本车之间的关系（包括交错和竞争）
    PathPlanningUtilities::Curve predicted_path_;  // 障碍物轨迹的预测
    // --------------------- 静态障碍物特有属性 -----------------

};

#endif