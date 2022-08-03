/*
    Copyright [2019] Jian ZhiQiang
*/

#include "Common.hpp"

// 填充特殊状态的信息(目前没有用)
// 特殊状态只有在触发了某些条件的情况下才会执行，并且是立刻执行
void DecisionMaking::SubVehicle::generateSpecialState() {
    if (this->special_state_type_ == SpecialStateNames::FOLLOW_PATH) {
        // 这种情况是为了能够让车追随一个已定义好的路径而不是生成路径
        // 初始化特殊状态，没有邻居
        std::vector<size_t> neighbor_states;
        this->special_state_ = StandardState(SpecialStateNames::FOLLOW_PATH, neighbor_states);
    } else if (this->special_state_type_ == SpecialStateNames::REACH_POINT) {
        // 这种情况是为了让车能够生成到达固定点的路径
        // 初始化特殊状态，没有邻居
        std::vector<size_t> neighbor_states;
        this->special_state_ = StandardState(SpecialStateNames::REACH_POINT, neighbor_states);

        // 获取车辆当前速度信息
        this->current_vehicle_movement_mutex_.lock();
        PathPlanningUtilities::VehicleMovementState vehicle_current_movement = this->current_vehicle_movement_;
        this->current_vehicle_movement_mutex_.unlock();
        // 获取车辆当前在世界坐标系下的位置
        this->current_vehicle_world_position_mutex_.lock();
        PathPlanningUtilities::VehicleState current_vehicle_position_in_world = this->current_vehicle_world_position_;
        this->current_vehicle_world_position_mutex_.unlock();
        this->current_vehicle_kappa_mutex_.lock();
        double current_vehicle_kappa = this->current_vehicle_kappa_;
        current_vehicle_position_in_world.kappa_ = current_vehicle_kappa;
        this->current_vehicle_kappa_mutex_.unlock();
        // // 更新地图信息
        // this->updateMapInformation();

        // 利用中道进行到达目标点的规划
        // 特殊状态为有效状态
        this->special_state_.enable();
        // 提供车辆形状信息
        this->special_state_.setVehicleSize(this->vehicle_length_, this->vehicle_width_);
        // 提供起始点信息
        this->special_state_.setVehicleStartState(current_vehicle_position_in_world);
        this->special_state_.setVehicleStartMovement(vehicle_current_movement);
        // 确定速度上下限制,速度固定为0(无效值)
        this->special_state_.setVelocityLimitation(0.0, 0.0);
        // 确定加速度上下限制,固定为0(无效值)
        this->special_state_.setAccelerationLimitationMax(0.0);
        this->special_state_.setAccelerationLimitationMin(0.0);
        // 设置安全性（无效值）
        this->special_state_.setSafety(true);
        // 设置优先级(TOFIX)(无效值)
        this->special_state_.setPriority(0.0);
                
                
        // 设置路径信息(选取直行道路作为停车的路径)(TOFIX)
        this->states_set_[StateNames::STOP].setTrajectory(this->states_set_[StateNames::FORWARD].getTrajectory());
        // 设置延伸路径信息(选取直行道路作为停车的延伸路径)(TOFIX)
        this->states_set_[StateNames::STOP].setExtendedTrajectory(this->states_set_[StateNames::FORWARD].getExtendedTrajectory());
        // 提供车辆当前速度信息
        this->special_state_.setVehicleCurrentMovement(vehicle_current_movement);
        // 提供车辆当前点在路径中的下标
        this->special_state_.setVehicleCurrentPositionIndexInTrajectory(0);
        // 提供车辆目标点在路径中的下标

        // 提供车辆目标点速度
        this->special_state_.setGoalVelocity(0.0);
        // 设置期望保持的加速度值（无效值）
        this->special_state_.setVehicleDynamicPlanningExpectedAcceleration(0.0);

    } else {
        std::cout << "[Error] error special state type" << std::endl;
        LOG(INFO) << "[Error] error special state type";
        exit(0);
    }
}
