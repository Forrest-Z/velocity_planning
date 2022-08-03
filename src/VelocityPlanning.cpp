#include "Common.hpp"

// Debug 可视化碰撞点
std::vector<Rectangle> collision_rectangles;

// 判断状态的路径是否与静态障碍物重合
bool DecisionMaking::RSS::checkStateBeingBlocked(StandardState *judge_state, const Obstacle &obstacle, size_t lane_index) {
    if (!Tools::isZero(obstacle.getObstacleVelocity())) {
        // 动态障碍物不处理
        return false;
    } else {
        // 0.如果障碍物物速度是0(此时障碍物没有预测轨迹和未来位置)
        // 1.判断车辆轨迹是否与障碍物相交，如果不，则全部加速度可行
        // 1.如果相交则全部加速度不可行
        // 2.构建障碍物和车辆的占用区域
        LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----障碍物为静止障碍物";
        OccupationArea subvehicle_occupation_area = OccupationArea(*judge_state);
        OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index, OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE);
        size_t subvehicle_interact_index, obstacle_interact_index;
        if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
            LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----静止障碍物与路径相交，障碍物交点为" << obstacle_interact_index << "本车交点为" << subvehicle_interact_index;
            LOG(INFO) << "Collided obstacle id: " << obstacle.getID();

            // 保存为碰撞点
            collision_rectangles.push_back(subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index]);

            // 返回为真
            return true;
        } else {
            // 如果不相交，返回为假
            LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----静止障碍物与路径不相交";
            return false;
        }
    }
}

// 判断车辆以什么加速度区间行驶时可以保障安全性
SectionSet DecisionMaking::RSS::getAccelerationSectionSet(StandardState *judge_state, const std::vector<Obstacle> &obstacles, bool is_aggressive) {
    SectionSet final_result_section_set;
    // 初始化最大最小速度和最大最小加速度
    double subvehicle_max_acceleration = judge_state->getAccelerationLimitationMax();
    double subvehicle_min_acceleration = judge_state->getAccelerationLimitationMin(); // 负数
    LOG(INFO) << "加速度可行区间为" << subvehicle_min_acceleration << "到" << subvehicle_max_acceleration;
    // 获取本车行驶的最高速度和最低速度
    double subvehicle_min_velocity = judge_state->getVelocityLimitationMin();
    // 最高速度由道路期望速度与状态速度限制共同决定
    double subvehicle_max_velocity = judge_state->getVelocityLimitationMax();
    LOG(INFO) << "本车当前速度" << judge_state->getVehicleCurrentMovement().velocity_ << "，最大速度可到" << subvehicle_max_velocity << "，最小速度可到" << subvehicle_min_velocity;
    // 初始化区间
    Section init_section = {subvehicle_min_acceleration, subvehicle_max_acceleration};
    final_result_section_set.push_back(init_section);

    // 首先遍历障碍物
    for (size_t obstacle_index = 0; obstacle_index < obstacles.size(); obstacle_index++) {
        // 遍历此时刻这个障碍物的每一预测轨迹
        Obstacle obstacle = obstacles[obstacle_index];
        LOG(INFO) << "对于障碍物" << obstacle.getID();
        for (size_t lane_index = 0; lane_index < obstacle.getPredictedTrajectoryNumber(); lane_index++) {
            if (!Tools::isZero(obstacle.getObstacleVelocity())) {
                LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----障碍物为动态障碍物";
                // 0. 如果障碍物速度不是0
                // 1. 构建障碍物和车辆的占用区域
                OccupationArea subvehicle_occupation_area = OccupationArea(*judge_state);
                OccupationArea obstacle_occupation_area = OccupationArea(obstacle, lane_index);
                // 2. 判断两个占用区域之间是否相交，如果不相交则安全
                size_t subvehicle_interact_index, obstacle_interact_index;
                if (occupationInteractionJudgement(subvehicle_occupation_area, obstacle_occupation_area, &subvehicle_interact_index, &obstacle_interact_index)) {
                    // 3. 如果两个占用区域之间相交，判断相交点，得到相交点后分为两种情况进行讨论。
                    // 打印交点信息
                    LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----障碍物与路径相交";
                    LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----障碍物交点位置信息 " << obstacle_interact_index << "||" << obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].center_x_ << "||" << obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].center_y_ << "||" << obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].width_ << "||" << obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].length_ << "||" << obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].rotation_ << std::endl;
                    LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----本车交点位置信息 " << subvehicle_interact_index << "||" << subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].center_x_ << "||" << subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].center_y_ << "||" << subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].width_ << "||" << subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].length_ << "||" << subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].rotation_ << std::endl;
                    LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----障碍物离交点距离" << obstacle_interact_index * OBSTACLE_MARGIN << "米";
                    LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----本车离交点距离" << subvehicle_interact_index * LANE_GAP_DISTANCE << "米";

                    // 保存为碰撞点
                    collision_rectangles.push_back(subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index]);

                    // 如果交点是本车所在位置时，不需要进行考虑，直接全部可行
                    if (subvehicle_interact_index == 0) {
                        // 判断是否为逆行
                        double yaw_gap = std::abs(Tools::safeThetaTransform(obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].rotation_ - subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].rotation_));
                        LOG(INFO) << "车辆夹角为" << yaw_gap;
                        if (Tools::isLarge(yaw_gap, 4.0 / 5.0 * PI)) {
                            LOG(INFO) << "Current vehicle orientation: " << subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].rotation_;
                            LOG(INFO) << "Coming vehicle orientation: " << obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].rotation_;
                            LOG(INFO) << "逆向来车";
                            // 清空可行
                            SectionSet().swap(final_result_section_set);
                            return final_result_section_set;
                        } else {
                            // 非逆向来车
                            LOG(INFO) << "本车处在路径交点位置，可以以任何加速度行驶";
                            continue;
                        }

                    }

                    // 首先初始化加速度区间
                    SectionSet tmp_section_set;
                    
                    // ---------------------------------- markhere wait safe ---------------------------------------
                    // 首先计算等待安全区间
                    // 第一种，等待安全区间。当障碍物到达交点时，本车离交点还有安全距离。
                    // 采样最大加速度区间，判断在不同加速度情况下，车辆是否可以和障碍物形成上述安全关系，如果可以则当前加速度为可行加速度
                    LOG(INFO) << "第一种等待安全情况：当障碍物到达交点时，本车离交点还有安全距离";
                    double max_acceleration_for_wait = subvehicle_min_acceleration;
                    for (double test_acceleration = subvehicle_max_acceleration; Tools::isLarge(test_acceleration,subvehicle_min_acceleration); test_acceleration -= SAFE_DISTANCE_CALCULATION_ACCELERATION_SAMPLING_GAP) {
                        // double类型保留小数点后一位
                        test_acceleration = std::floor(test_acceleration * 10.0 + 0.5) / 10.0;
                        if (obstacle_interact_index != 0) {
                            LOG(INFO) << "等待的障碍物还未进入交点";
                            // 当障碍物到达交点时，本车离交点还有安全距离
                            // 加速度为0单独考虑
                            if (Tools::isZero(test_acceleration)) {
                                // 当前车速如果大于最大车速或小于最小车速，加速度不能为0.0
                                if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity) || Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                                    // 当前车速大于最大车速或小于最小车速，加速度不能为0;
                                    continue;
                                } else {
                                    double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity();
                                    double wait_safe_distance = Tools::getWaitSafeDistance(judge_state->getVehicleCurrentMovement().velocity_, obstacle.getObstacleVelocity());
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车经过此时间走过的距离为" << obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ << "米，离交点的距离还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ << "米，而安全距离为" << wait_safe_distance << "米";
                                    if (Tools::isSmall(obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决满足条件0";
                                        break;
                                    } else {
                                        LOG(INFO) << "判决不满足条件10";
                                        continue;
                                    }
                                }
                            }
                            // 计算障碍物到达交点所花时间
                            double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity();
                            // 在此时间内，本车进行匀加速/减速运动
                            // 判断本车速度与最高/最低速度的关系
                            if (Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                                // 如果当前车速大于最高车速，那没法进行加速，只能减速，减到最大速度
                                if (!Tools::isSmall(test_acceleration, 0.0)) {
                                    // 加速度大于等于0直接不可以
                                    continue;
                                } else {
                                    // 速度大于最大速度时，加速度必须小于0.0
                                    // 这段时间内经过加速度，速度变为了如下值
                                    double subvehicle_interaction_velocity = std::max(judge_state->getVehicleCurrentMovement().velocity_ + obstacle_reach_interaction_time_consume * test_acceleration, subvehicle_min_velocity);
                                    if (Tools::isLarge(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                        // 经过加速度后，速度大于最小速度
                                        // 计算本车走过的距离vt+0.5at^2
                                        double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                        // 根据当前车速计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件1";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件1";
                                            continue;
                                        }

                                    } else {
                                        // 经过加速度后，速度可以减到最小速度
                                        // 计算走过的距离
                                        double subvehicle_traveled_distance = subvehicle_min_velocity * obstacle_reach_interaction_time_consume - (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) * (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) / (2.0 * test_acceleration);
                                        // 计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件2";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件2";
                                            continue;
                                        }
                                    }
                                }
                            } else if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity)) {
                                // 如果当前车速小于最高车速，那没法进行减速，只能加速，加到最大速度
                                if (!Tools::isLarge(test_acceleration, 0.0)) {
                                    // 加速度小于等于0直接不可以
                                    continue;
                                } else {
                                    // 速度小于最小车速时，加速度必须大于0
                                    // 计算经过此加速度后，本车的速度(大于0，小于最大速度)
                                    double subvehicle_interaction_velocity = std::min(judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_max_velocity);
                                    if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                        // 经过此加速度，速度加到了最大速度
                                        // 计算走过的距离
                                        double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                        // 计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件3";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件3";
                                            continue;
                                        }
                                    } else {
                                        // 经过此加速度，速度在正常区间内(大于0，小于最大)
                                        // 计算走过的距离
                                        double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                        // 计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件4";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件4";
                                            continue;
                                        }
                                    }
                                }
                            } else {
                                // 如果当前车速小于最高车速，大于最低车速，加速减速均可
                                // 计算经过加速度后，车辆的速度
                                double subvehicle_interaction_velocity = std::min(std::max(judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_min_velocity), subvehicle_max_velocity);
                                if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                    // 如果车辆加速到最大速度
                                    // 计算经过的距离
                                    double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                    // 计算安全距离
                                    double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决满足条件5";
                                        break;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "判决不满足条件5";
                                        continue;
                                    }
                                } else if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                    // 如果车辆减速到最小速度
                                    // 计算经过的距离
                                    double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) * (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) / (2.0 * test_acceleration);
                                    // 计算安全距离
                                    double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                    if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决不满足条件6";
                                        break;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "判决不满足条件6";
                                        continue;
                                    }
                                } else {
                                    // 最终车速在最大最小之间
                                    // 计算经过的距离
                                    double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << subvehicle_interact_index * LANE_GAP_DISTANCE - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                    if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决满足条件7";
                                        break;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "判决不满足条件7";
                                        continue;
                                    }
                                }
                            }
                            // 更新等待安全区间
                        } else {
                            LOG(INFO) << "等待的障碍物已经在交点上";
                            // 计算反应时间
                            double response_time = ALOWED_RESPONSE_TIME_WHEN_SAFE_DISTANCE_NOT_SATISFIED;
                            // 计算交点处的速度朝向差
                            double theta_gap = std::fabs(subvehicle_occupation_area.getOccupationArea()[subvehicle_interact_index].rotation_ - obstacle_occupation_area.getOccupationArea()[obstacle_interact_index].rotation_);
                            // 当障碍物到达交点时，本车离交点还有安全距离
                            // 加速度为0单独考虑
                            if (Tools::isZero(test_acceleration)) {
                                // 当前车速如果大于最大车速或小于最小车速，加速度不能为0.0
                                if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity) || Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                                    // 当前车速大于最大车速或小于最小车速，加速度不能为0;
                                    continue;
                                } else {
                                    double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity() + response_time;
                                    double wait_safe_distance = Tools::getWaitSafeDistance(judge_state->getVehicleCurrentMovement().velocity_, obstacle.getObstacleVelocity());
                                    // 计算可用距离
                                    double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车经过此时间走过的距离为" << obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ << "米，剩余距离为" << available_distance - obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ << "米，而安全距离为" << wait_safe_distance << "米";
                                    if (Tools::isSmall(obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ + wait_safe_distance, available_distance) || Tools::isEqual(obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ + wait_safe_distance, available_distance)) {
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决满足条件0";
                                        break;
                                    } else {
                                        LOG(INFO) << "判决不满足条件10";
                                        continue;
                                    }
                                }
                            }
                            // 计算障碍物到达交点所花时间
                            double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity() + response_time;
                            // 在此时间内，本车进行匀加速/减速运动
                            // 判断本车速度与最高/最低速度的关系
                            if (Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                                // 如果当前车速大于最高车速，那没法进行加速，只能减速，减到最大速度
                                if (!Tools::isSmall(test_acceleration, 0.0)) {
                                    // 加速度大于等于0直接不可以
                                    continue;
                                } else {
                                    // 速度大于最大速度时，加速度必须小于0.0
                                    // 这段时间内经过加速度，速度变为了如下值
                                    double subvehicle_interaction_velocity = std::max(judge_state->getVehicleCurrentMovement().velocity_ + obstacle_reach_interaction_time_consume * test_acceleration, subvehicle_min_velocity);
                                    if (Tools::isLarge(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                        // 经过加速度后，速度大于最小速度
                                        // 计算本车走过的距离vt+0.5at^2
                                        double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                        // 根据当前车速计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        // 计算可用距离
                                        double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，剩余距离为" << available_distance - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, available_distance) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, available_distance)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件1";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件1";
                                            continue;
                                        }

                                    } else {
                                        // 经过加速度后，速度可以减到最小速度
                                        // 计算走过的距离
                                        double subvehicle_traveled_distance = subvehicle_min_velocity * obstacle_reach_interaction_time_consume - (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) * (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) / (2.0 * test_acceleration);
                                        // 计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        // 计算可用距离
                                        double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，剩余距离为" << available_distance - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, available_distance) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, available_distance)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件2";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件2";
                                            continue;
                                        }
                                    }
                                }
                            } else if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity)) {
                                // 如果当前车速小于最高车速，那没法进行减速，只能加速，加到最大速度
                                if (!Tools::isLarge(test_acceleration, 0.0)) {
                                    // 加速度小于等于0直接不可以
                                    continue;
                                } else {
                                    // 速度小于最小车速时，加速度必须大于0
                                    // 计算经过此加速度后，本车的速度(大于0，小于最大速度)
                                    double subvehicle_interaction_velocity = std::min(judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_max_velocity);
                                    if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                        // 经过此加速度，速度加到了最大速度
                                        // 计算走过的距离
                                        double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                        // 计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        // 计算可用距离
                                        double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，剩余距离" << available_distance - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, available_distance) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, available_distance)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件3";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件3";
                                            continue;
                                        }
                                    } else {
                                        // 经过此加速度，速度在正常区间内(大于0，小于最大)
                                        // 计算走过的距离
                                        double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                        // 计算安全距离
                                        double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                        // 计算可用距离
                                        double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                        // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                        LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，剩余距离" << available_distance - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                        if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, available_distance) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, available_distance)) {
                                            // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                            max_acceleration_for_wait = test_acceleration;
                                            LOG(INFO) << "判决满足条件4";
                                            break;
                                        } else {
                                            // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                            LOG(INFO) << "判决不满足条件4";
                                            continue;
                                        }
                                    }
                                }
                            } else {
                                // 如果当前车速小于最高车速，大于最低车速，加速减速均可
                                // 计算经过加速度后，车辆的速度
                                double subvehicle_interaction_velocity = std::min(std::max(judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_min_velocity), subvehicle_max_velocity);
                                if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                    // 如果车辆加速到最大速度
                                    // 计算经过的距离
                                    double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                    // 计算安全距离
                                    double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 计算可用距离
                                    double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，剩余可用距离" << available_distance - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, available_distance) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, available_distance)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决满足条件5";
                                        break;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "判决不满足条件5";
                                        continue;
                                    }
                                } else if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                    // 如果车辆减速到最小速度
                                    // 计算经过的距离
                                    double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) * (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) / (2.0 * test_acceleration);
                                    // 计算安全距离
                                    double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 计算可用距离
                                    double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << available_distance - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                    if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, available_distance) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, available_distance)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决不满足条件6";
                                        break;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "判决不满足条件6";
                                        continue;
                                    }
                                } else {
                                    // 最终车速在最大最小之间
                                    // 计算经过的距离
                                    double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double wait_safe_distance = Tools::getWaitSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 计算可用距离
                                    double available_distance = subvehicle_interact_index * LANE_GAP_DISTANCE + obstacle.getObstacleVelocity() * response_time * cos(theta_gap);
                                    // 判断障碍物到达交点时，本车是否离交点还有安全距离
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，走过了" << subvehicle_traveled_distance << "米，离交点还有" << available_distance - subvehicle_traveled_distance << "米，而安全距离为" << wait_safe_distance << "米";
                                    if (Tools::isSmall(subvehicle_traveled_distance + wait_safe_distance, available_distance) || Tools::isEqual(subvehicle_traveled_distance + wait_safe_distance, available_distance)) {
                                        // 如果本车离交点还有安全距离，说明此加速度可以保障安全
                                        max_acceleration_for_wait = test_acceleration;
                                        LOG(INFO) << "判决满足条件7";
                                        break;
                                    } else {
                                        // 如果本车离交点没安全距离，说明此加速度不安全，退出此次循环
                                        LOG(INFO) << "判决不满足条件7";
                                        continue;
                                    }
                                }
                            }
                            // 更新等待安全区间
                        }
                    }
                    LOG(INFO) << "满足当障碍物达到交点时，本车离交点还存在安全距离的最大加速度为:" << max_acceleration_for_wait;


                    // ---------------------------------- markhere wait expected --------------------------------
                    // 当本车到达交点时，障碍物已经超过交点安全距离
                    LOG(INFO) << "第二种等待期望：当本车到达交点时，障碍物已经超过交点安全距离";
                    double another_max_acceleration_for_wait = ANOTHER_MAX_ACCELERATION_FOR_WAIT_MIN_VALUE;
                    for (double test_acceleration = subvehicle_max_acceleration; Tools::isLarge(test_acceleration, ANOTHER_MAX_ACCELERATION_FOR_WAIT_MIN_VALUE); test_acceleration -= SAFE_DISTANCE_CALCULATION_ACCELERATION_SAMPLING_GAP) {
                        // double类型保留小数点后一位
                        test_acceleration = std::floor(test_acceleration * 10.0 + 0.5) / 10.0;
                        // 加速度为0单独考虑
                        if (Tools::isZero(test_acceleration)) {
                            // 当前车速如果大于最大车速或小于最小车速，加速度不能为0.0
                            if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity) || Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                                // 当前车速大于最大车速或小于最小车速，加速度不能为0;
                                continue;
                            } else {
                                double subvehicle_reach_interaction_time_consume = subvehicle_interact_index * LANE_GAP_DISTANCE / judge_state->getVehicleCurrentMovement().velocity_;
                                double wait_expected_distance = Tools::getWaitExpectedDistance(judge_state->getVehicleCurrentMovement().velocity_, obstacle.getObstacleVelocity());
                                LOG(INFO) << "本车以匀速" << judge_state->getVehicleCurrentMovement().velocity_ << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米1";
                                if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                    another_max_acceleration_for_wait = test_acceleration;
                                    break;
                                } else {
                                    continue;
                                }
                            }
                        }
                        if (Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                            // 当前车速大于最大车速
                            if (!Tools::isSmall(test_acceleration, 0.0)) {
                                // 当车速大于最大车速时，加速度不能大于0
                                continue;
                            } else {
                                // 当车速大于最大车速时，加速度必须小于0
                                // 计算当前速度是否可能减速到最低速度
                                if (Tools::isSmall((subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 当前速度会减速到最低速度
                                    // 计算到达交点时的时间开销
                                    double subvehicle_reach_interaction_time_consume = (subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_min_velocity;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = subvehicle_min_velocity;
                                    // 计算安全距离
                                    double wait_expected_distance = Tools::getWaitExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断在时间开销内，障碍物是否能够达到超越交点安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米2";
                                    if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                        // 障碍物能够达到超越交点安全距离，得到此加速可行，退出循环
                                        another_max_acceleration_for_wait = test_acceleration;
                                        break;
                                    } else {
                                        // 障碍物不能达到超越交点安全距离，退出本次循环
                                        continue;
                                    }
                                } else {
                                    // 当前速度不会减速到最低速度
                                    // 计算到达交点时的时间开销
                                    double subvehicle_reach_interaction_time_consume = (sqrt(judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double wait_expected_distance = Tools::getWaitExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断在时间开销内，障碍物是否能够达到超越交点安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米3";
                                    if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                        // 障碍物能够达到超越交点安全距离，得到此加速可行，退出循环
                                        another_max_acceleration_for_wait = test_acceleration;
                                        
                                        break;
                                    } else {
                                        // 障碍物不能达到超越交点安全距离，退出本次循环
                                        continue;
                                    }
                                }
                            }
                        } else if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity)) {
                            // 当前车速小于最小车速
                            if (!Tools::isLarge(test_acceleration, 0.0)) {
                                // 当前车速小于最小车速时，加速度不能小于0
                                continue;
                            } else {
                                // 当前车速小于最小车速时，加速度必须大于0
                                // 计算当前加速度是否可以加速到最大速度，或减速到0
                                if (Tools::isSmall((subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 可以加速到最大速度
                                    // 计算时间开销
                                    double subvehicle_reach_interaction_time_consume = (subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_max_velocity;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = subvehicle_max_velocity;
                                    // 计算安全距离
                                    double wait_expected_distance = Tools::getWaitExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断在时间开销内，障碍物是否能够达到超越交点安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米4";
                                    if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                        // 障碍物能够达到超越交点安全距离，得到此加速可行，退出循环
                                        another_max_acceleration_for_wait = test_acceleration;
                                        
                                        break;
                                    } else {
                                        // 障碍物不能达到超越交点安全距离，退出本次循环
                                        continue;
                                    }
                                } else {
                                    // 不能加速到最大速度
                                    // 计算时间开销
                                    double subvehicle_reach_interaction_time_consume = (sqrt(judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double wait_expected_distance = Tools::getWaitExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断在时间开销内，障碍物是否能够达到超越交点安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米5";
                                    if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                        // 障碍物能够达到超越交点安全距离，得到此加速可行，退出循环
                                        another_max_acceleration_for_wait = test_acceleration;
                                        
                                        break;
                                    } else {
                                        // 障碍物不能达到超越交点安全距离，退出本次循环
                                        continue;
                                    }
                                }
                            }
                        } else {
                            // 本车速度大于最小速度，小于最大速度，此时可以加速也可以减速
                            // 判断经过加速度，本车速度是否可以达到最大速度或最小速度
                            if (Tools::isLarge(test_acceleration, 0.0) && Tools::isSmall((subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                // 本车速度达到最大速度
                                // 计算时间开销
                                double subvehicle_reach_interaction_time_consume = (subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_max_velocity;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = subvehicle_max_velocity;
                                // 计算安全距离
                                double wait_expected_distance = Tools::getWaitExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断在时间开销内，障碍物是否能够达到超越交点安全距离
                                LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米6";
                                if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                    // 障碍物能够达到超越交点安全距离，得到此加速可行，退出循环
                                    another_max_acceleration_for_wait = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 障碍物不能达到超越交点安全距离，退出本次循环
                                    continue;
                                }
                            } else if (Tools::isSmall(test_acceleration, 0.0) && Tools::isSmall((subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                // 本车速度达到最小速度
                                // 计算到达交点时的时间开销
                                double subvehicle_reach_interaction_time_consume = (subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_min_velocity;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = subvehicle_min_velocity;
                                // 计算安全距离
                                double wait_expected_distance = Tools::getWaitExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断在时间开销内，障碍物是否能够达到超越交点安全距离
                                LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米7";
                                if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                    // 障碍物能够达到超越交点安全距离，得到此加速可行，退出循环
                                    another_max_acceleration_for_wait = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 障碍物不能达到超越交点安全距离，退出本次循环
                                    continue;
                                }
                            } else {
                                // 本车既不能达到最大速度也不能达到最小速度
                                // 计算到交点的时间开销
                                double subvehicle_reach_interaction_time_consume = (sqrt(judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration;
                                // 计算到达交点的本车速度
                                double subvehicle_interaction_velocity = judge_state->getVehicleCurrentMovement().velocity_ + subvehicle_reach_interaction_time_consume * test_acceleration;
                                // 计算安全距离
                                double wait_expected_distance = Tools::getWaitExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断在时间开销内，障碍物是否能够达到超越交点安全距离
                                LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒到达交点，所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时，障碍物超过交点的距离为" << subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() - obstacle_interact_index * OBSTACLE_MARGIN << "米，而安全距离为" << wait_expected_distance << "米8";
                                if (Tools::isLarge(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity(), obstacle_interact_index * OBSTACLE_MARGIN + wait_expected_distance)) {
                                    // 障碍物能够达到超越交点安全距离，得到此加速可行，退出循环
                                    another_max_acceleration_for_wait = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 障碍物不能达到超越交点安全距离，退出本次循环
                                    continue;
                                }
                            }
                        }
                    }
                    LOG(INFO) << "满足当本车到达交点时，障碍物已经超过交点安全距离的最大加速度为:" << another_max_acceleration_for_wait;
                    
                    if (!is_aggressive) {
                        // 判断是否同时满足两个区间
                        if (Tools::isLarge(max_acceleration_for_wait, subvehicle_min_acceleration) && Tools::isLarge(another_max_acceleration_for_wait, ANOTHER_MAX_ACCELERATION_FOR_WAIT_MIN_VALUE)) {
                            max_acceleration_for_wait = std::min(max_acceleration_for_wait, another_max_acceleration_for_wait);
                            Section section;
                            section.min_ = subvehicle_min_acceleration;
                            section.max_ = max_acceleration_for_wait;
                            LOG(INFO) << "此路径得到的等待加速度区间为" << section.min_ << "到" << section.max_;
                            tmp_section_set.push_back(section);
                        }
                    } else {
                        // 只要满足安全区间即可
                        if (Tools::isLarge(max_acceleration_for_wait, subvehicle_min_acceleration)) {
                            max_acceleration_for_wait = std::min(max_acceleration_for_wait, another_max_acceleration_for_wait);
                            Section section;
                            section.min_ = subvehicle_min_acceleration;
                            section.max_ = max_acceleration_for_wait;
                            LOG(INFO) << "此路径得到的等待加速度区间为" << section.min_ << "到" << section.max_;
                            tmp_section_set.push_back(section);
                        }
                    }
                    

                    // ---------------------------- markhere overtake safe --------------------------------
                    // 第二种，超越。当本车到达交点时，障碍物离交点还存在安全距离；并且当障碍物到达交点时，本车已超越交点安全距离。以上两种情况只要都满足，就是安全的。
                    LOG(INFO) << "第一种超越安全情况：当本车到达交点时，障碍物离交点还存在安全距离的情况";
                    double min_acceleration_for_overtake = subvehicle_max_acceleration;
                    for (double test_acceleration = subvehicle_min_acceleration; Tools::isSmall(test_acceleration, subvehicle_max_acceleration); test_acceleration += SAFE_DISTANCE_CALCULATION_ACCELERATION_SAMPLING_GAP) {
                        // 当本车到达交点时，障碍物离交点还存在安全距离的情况。
                        // double类型保留小数点后一位
                        test_acceleration = std::floor(test_acceleration * 10.0 + 0.5) / 10.0;
                        // 加速度为0单独考虑
                        if (Tools::isZero(test_acceleration)) {
                            // 当前车速如果大于最大车速或小于最小车速，加速度不能为0.0
                            if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity) || Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                                // 当前车速大于最大车速或小于最小车速，加速度不能为0;
                                continue;
                            } else {
                                double subvehicle_reach_interaction_time_consume = subvehicle_interact_index * LANE_GAP_DISTANCE / judge_state->getVehicleCurrentMovement().velocity_;
                                double overtake_safe_distance = Tools::getOvertakeSafeDistance(judge_state->getVehicleCurrentMovement().velocity_, obstacle.getObstacleVelocity());
                                LOG(INFO) << "本车以匀速" << judge_state->getVehicleCurrentMovement().velocity_ << "到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米9";
                                if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                    min_acceleration_for_overtake = test_acceleration;
                                    
                                    break;
                                } else {
                                    continue;
                                }
                            }
                        }
                        if (Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                            // 当本车速度大于最高速度
                            if (!Tools::isSmall(test_acceleration, 0.0)) {
                                // 当前车速大于最大车速时，加速度不能大于等于0
                                continue;
                            } else {
                                // 计算当前速度是否可能减速到最低速度
                                if (Tools::isSmall((subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 当前速度需要减速到最低速度
                                    // 计算到达交点的时间开销
                                    double subvehicle_reach_interaction_time_consume = (subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_min_velocity;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = subvehicle_min_velocity;
                                    // 计算安全距离
                                    double overtake_safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米10";
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        min_acceleration_for_overtake = test_acceleration;
                                        
                                        break;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出循环
                                        continue;
                                    }
                                } else {
                                    // 当前速度不需要减速到最低速度
                                    // 计算到达交点时的时间开销
                                    double subvehicle_reach_interaction_time_consume = (sqrt(judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double overtake_safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米11";
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        min_acceleration_for_overtake = test_acceleration;
                                        break;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出循环
                                        continue;
                                    }
                                }
                            }
                        } else if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity)) {
                            // 当本车速度小于最低速度
                            if (!Tools::isLarge(test_acceleration, 0.0)) {
                                // 当前车速小于最低车速时，加速度不能小于0
                                continue;
                            } else {
                                // 判断本车速度是否会加速到最大速度或减速到0.0
                                if (Tools::isSmall((subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 本车会加速到最大速度
                                    // 计算时间开销
                                    double subvehicle_reach_interaction_time_consume = (subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_max_velocity;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = subvehicle_max_velocity;
                                    // 计算安全距离
                                    double overtake_safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米12";
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        min_acceleration_for_overtake = test_acceleration;
                                        break;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出循环
                                        continue;
                                    }
                                } else {
                                    // 本车不会加速到最大速度
                                    // 计算时间开销
                                    double subvehicle_reach_interaction_time_consume = (sqrt(judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration;
                                    // 计算到达交点时，本车速度
                                    double subvehicle_interaction_velocity = judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                    // 计算安全距离
                                    double overtake_safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                    LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米13";
                                    if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                        // 障碍物离交点大于安全距离，此时安全
                                        min_acceleration_for_overtake = test_acceleration;
                                        break;
                                    } else {
                                        // 障碍物离交点小于安全距离，退出循环
                                        continue;
                                    }
                                }
                            }
                        } else {
                            // 当本车速度处于正常区间，加速度可以大于0也可以小于0
                            // 判断本车是否会加速到最大速度，或减速到最小速度
                            if (Tools::isLarge(test_acceleration, 0.0) && Tools::isSmall((subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                // 本车会加速到最大速度
                                // 计算时间开销
                                double subvehicle_reach_interaction_time_consume = (subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_max_velocity * subvehicle_max_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_max_velocity;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = subvehicle_max_velocity;
                                // 计算安全距离
                                double overtake_safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米14";
                                if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                    // 障碍物离交点大于安全距离，此时安全
                                    min_acceleration_for_overtake = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 障碍物离交点小于安全距离，退出循环
                                    continue;
                                }
                            } else if (Tools::isSmall(test_acceleration, 0.0) && Tools::isSmall((subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration), subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                // 本车会减速到最小速度
                                // 计算到达交点的时间开销
                                double subvehicle_reach_interaction_time_consume = (subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration + (subvehicle_interact_index * LANE_GAP_DISTANCE - (subvehicle_min_velocity * subvehicle_min_velocity - judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration)) / subvehicle_min_velocity;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = subvehicle_min_velocity;
                                // 计算安全距离
                                double overtake_safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米15";
                                if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                    // 障碍物离交点大于安全距离，此时安全
                                    min_acceleration_for_overtake = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 障碍物离交点小于安全距离，退出循环
                                    continue;
                                }
                            } else {
                                // 本车既不会加速到最大速度，也不会减速到最小速度
                                // 计算时间开销
                                double subvehicle_reach_interaction_time_consume = (sqrt(judge_state->getVehicleCurrentMovement().velocity_ * judge_state->getVehicleCurrentMovement().velocity_ + 2.0 * test_acceleration * subvehicle_interact_index * LANE_GAP_DISTANCE) - judge_state->getVehicleCurrentMovement().velocity_) / test_acceleration;
                                // 计算到达交点时，本车速度
                                double subvehicle_interaction_velocity = judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * subvehicle_reach_interaction_time_consume;
                                // 计算安全距离
                                double overtake_safe_distance = Tools::getOvertakeSafeDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断本车达到交点时，障碍物离交点是否还存在安全距离
                                LOG(INFO) << "本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒并到达交点所花时间为" << subvehicle_reach_interaction_time_consume << "秒，此时障碍物离交点还有" << obstacle_interact_index * OBSTACLE_MARGIN - subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() << "米，而安全距离为" << overtake_safe_distance << "米16";
                                if (Tools::isSmall(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN) || Tools::isEqual(subvehicle_reach_interaction_time_consume * obstacle.getObstacleVelocity() + overtake_safe_distance, obstacle_interact_index * OBSTACLE_MARGIN)) {
                                    // 障碍物离交点大于安全距离，此时安全
                                    min_acceleration_for_overtake = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 障碍物离交点小于安全距离，退出循环
                                    continue;
                                }
                            }
                        }
                    }
                    LOG(INFO) << "满足当本车到达交点时，障碍物离交点还存在安全距离的最小加速度为:" << min_acceleration_for_overtake;

                    // --------------------------markhere overtake expected --------------------------
                    LOG(INFO) << "第二种超越期望情况：当障碍物到达交点时，本车已超越交点安全距离的情况";
                    double another_min_acceleration_for_overtake = subvehicle_max_acceleration;
                    for (double test_acceleration = subvehicle_min_acceleration; Tools::isSmall(test_acceleration, subvehicle_max_acceleration); test_acceleration += SAFE_DISTANCE_CALCULATION_ACCELERATION_SAMPLING_GAP) {
                        // 当障碍物到达交点时，本车已超越交点安全距离的情况。
                        // double类型保留小数点后一位
                        test_acceleration = std::floor(test_acceleration * 10.0 + 0.5) / 10.0;
                        // 加速度为0单独考虑
                        if (Tools::isZero(test_acceleration)) {
                            // 当前车速如果大于最大车速或小于最小车速，加速度不能为0.0
                            if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity) || Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                                // 当前车速大于最大车速或小于最小车速，加速度不能为0;
                                continue;
                            } else {
                                double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity();
                                double overtake_expected_distance = Tools::getOvertakeExpectedDistance(judge_state->getVehicleCurrentMovement().velocity_, obstacle.getObstacleVelocity());
                                LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以匀速" << judge_state->getVehicleCurrentMovement().velocity_ << "超越了交点" << obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米17";
                                if (Tools::isLarge(obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_, subvehicle_interact_index * LANE_GAP_DISTANCE + overtake_expected_distance) || Tools::isEqual(obstacle_reach_interaction_time_consume * judge_state->getVehicleCurrentMovement().velocity_, subvehicle_interact_index * LANE_GAP_DISTANCE + overtake_expected_distance)) {
                                    another_min_acceleration_for_overtake = test_acceleration;
                                    break;
                                } else {
                                    continue;
                                }
                            }
                        }
                        // 计算障碍物到达交点所花时间
                        double obstacle_reach_interaction_time_consume = obstacle_interact_index * OBSTACLE_MARGIN / obstacle.getObstacleVelocity();
                        // 判断当前速度与最大最小速度之间的关系
                        if (Tools::isLarge(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_max_velocity)) {
                            // 当前速度大于最大速度
                            if (!Tools::isSmall(test_acceleration, 0.0)) {
                                // 当前车速大于最大车速时，加速度不能大于等于0
                                continue;
                            } else {
                                // 计算经过此加速度后本车的最终速度
                                double subvehicle_interaction_velocity = std::max(judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_min_velocity);
                                // 判断本车是否减速到最小速度
                                if (Tools::isLarge(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                    // 本车车速大于最小速度
                                    // 计算本车走过的距离vt+0.5at^2
                                    double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                    // 根据当前车速计算安全距离
                                    double overtake_expected_distance = Tools::getOvertakeExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断障碍物到达交点时，本车是否超越交点安全距离
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，并超越了交点" << subvehicle_traveled_distance - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米19";
                                    if (Tools::isLarge(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车超越交点安全距离，说明此加速度可以保障安全
                                        another_min_acceleration_for_overtake = test_acceleration;
                                        break;
                                    } else {
                                        // 如果本车超越交点没安全距离，说明此加速度不安全，退出此次循环
                                        continue;
                                    }
                                } else {
                                    // 本车车速等于最小速度
                                    // 计算走过的距离
                                    double subvehicle_traveled_distance = subvehicle_min_velocity * obstacle_reach_interaction_time_consume - (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) * (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_min_velocity) / (2.0 * test_acceleration);
                                    // 根据当前车速计算安全距离
                                    double overtake_expected_distance = Tools::getOvertakeExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断障碍物到达交点时，本车是否超越交点安全距离
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，并超越了交点" << subvehicle_traveled_distance - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米19";
                                    if (Tools::isLarge(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车超越交点安全距离，说明此加速度可以保障安全
                                        another_min_acceleration_for_overtake = test_acceleration;
                                        
                                        break;
                                    } else {
                                        // 如果本车超越交点没安全距离，说明此加速度不安全，退出此次循环
                                        continue;
                                    }
                                }
                            }
                        } else if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, subvehicle_min_velocity)) {
                            // 当前车速小于最低车速
                            if (!Tools::isLarge(test_acceleration, 0.0)) {
                                // 当前车速小于最低车速，加速度不能小于0.0
                                continue;
                            } else {
                                // 计算经过此加速度后，本车的速度
                                double subvehicle_interaction_velocity = std::min(judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_max_velocity);
                                if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                    // 经过此加速度，速度加到了最大速度
                                    // 计算走过的距离
                                    double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                    // 根据当前车速计算安全距离
                                    double overtake_expected_distance = Tools::getOvertakeExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，并超越了交点" << subvehicle_traveled_distance - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米20";
                                    // 判断障碍物到达交点时，本车是否超越交点安全距离
                                    if (Tools::isLarge(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车超越交点安全距离，说明此加速度可以保障安全
                                        another_min_acceleration_for_overtake = test_acceleration;
                                        
                                        break;
                                    } else {
                                        // 如果本车超越交点没安全距离，说明此加速度不安全，退出此次循环
                                        continue;
                                    }

                                } else {
                                    // 经过此加速度，速度小于最大速度
                                    // 计算走过的距离
                                    double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                    // 根据当前车速计算安全距离
                                    double overtake_expected_distance = Tools::getOvertakeExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                    // 判断障碍物到达交点时，本车是否超越交点安全距离
                                    LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，并超越了交点" << subvehicle_traveled_distance - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米21";
                                    if (Tools::isLarge(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                        // 如果本车超越交点安全距离，说明此加速度可以保障安全
                                        another_min_acceleration_for_overtake = test_acceleration;
                                        
                                        break;
                                    } else {
                                        // 如果本车超越交点没安全距离，说明此加速度不安全，退出此次循环
                                        continue;
                                    }
                                }
                            }

                        } else {
                            // 当前车速处于正常区间内
                            // 计算经过加速度后，车辆的速度
                            double subvehicle_interaction_velocity = std::min(std::max(judge_state->getVehicleCurrentMovement().velocity_ + test_acceleration * obstacle_reach_interaction_time_consume, subvehicle_min_velocity), subvehicle_max_velocity);
                            if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_max_velocity)) {
                                // 如果车辆加速到最大速度
                                // 计算经过的距离
                                double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) * (subvehicle_interaction_velocity - judge_state->getVehicleCurrentMovement().velocity_) / (2.0 * test_acceleration);
                                // 根据当前车速计算安全距离
                                double overtake_expected_distance = Tools::getOvertakeExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断障碍物到达交点时，本车是否超越交点安全距离
                                LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，并超越了交点" << subvehicle_traveled_distance - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米22";
                                if (Tools::isLarge(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 如果本车超越交点安全距离，说明此加速度可以保障安全
                                    another_min_acceleration_for_overtake = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 如果本车超越交点没安全距离，说明此加速度不安全，退出此次循环
                                    continue;
                                }
                            } else if (Tools::isEqual(subvehicle_interaction_velocity, subvehicle_min_velocity)) {
                                // 如果车辆减速到最小速度
                                // 计算经过的距离
                                double subvehicle_traveled_distance = subvehicle_interaction_velocity * obstacle_reach_interaction_time_consume - (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) * (judge_state->getVehicleCurrentMovement().velocity_ - subvehicle_interaction_velocity) / (2.0 * test_acceleration);
                                // 根据当前车速计算安全距离
                                double overtake_expected_distance = Tools::getOvertakeExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断障碍物到达交点时，本车是否超越交点安全距离

                                LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，并超越了交点" << subvehicle_traveled_distance - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米23";
                                if (Tools::isLarge(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 如果本车超越交点安全距离，说明此加速度可以保障安全
                                    another_min_acceleration_for_overtake = test_acceleration;
                                    break;
                                } else {
                                    // 如果本车超越交点没安全距离，说明此加速度不安全，退出此次循环
                                    continue;
                                }
                            } else {
                                // 最终车速在最大最小之间
                                // 计算经过的距离
                                double subvehicle_traveled_distance = judge_state->getVehicleCurrentMovement().velocity_ * obstacle_reach_interaction_time_consume + 0.5 * test_acceleration * obstacle_reach_interaction_time_consume * obstacle_reach_interaction_time_consume;
                                // 根据当前车速计算安全距离
                                double overtake_expected_distance = Tools::getOvertakeExpectedDistance(subvehicle_interaction_velocity, obstacle.getObstacleVelocity());
                                // 判断障碍物到达交点时，本车是否超越交点安全距离
                                LOG(INFO) << "障碍物到达交点所花时间为" << obstacle_reach_interaction_time_consume << "秒，这段时间内本车以加速度" << test_acceleration << "加速到" << subvehicle_interaction_velocity << "米/秒，并超越了交点" << subvehicle_traveled_distance - subvehicle_interact_index * LANE_GAP_DISTANCE << "米，而安全距离为" << overtake_expected_distance << "米24";
                                if (Tools::isLarge(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE) || Tools::isEqual(subvehicle_traveled_distance, overtake_expected_distance + subvehicle_interact_index * LANE_GAP_DISTANCE)) {
                                    // 如果本车超越交点安全距离，说明此加速度可以保障安全
                                    another_min_acceleration_for_overtake = test_acceleration;
                                    
                                    break;
                                } else {
                                    // 如果本车超越交点没安全距离，说明此加速度不安全，退出此次循环
                                    continue;
                                }
                            }
                        }
                    }
                    LOG(INFO) << "满足当障碍物到达交点时，本车已超越交点安全距离的最小加速度为:" << another_min_acceleration_for_overtake;

                    // 得到最小超越加速度后求交集
                    min_acceleration_for_overtake = std::max(min_acceleration_for_overtake, another_min_acceleration_for_overtake);
                    // 判断得到加速度可行区间是否存在，如果存在则输出
                    if (Tools::isSmall(min_acceleration_for_overtake, subvehicle_max_acceleration)) {
                        Section section;
                        section.min_ = min_acceleration_for_overtake;
                        section.max_ = subvehicle_max_acceleration;
                        LOG(INFO) << "此路径得到的超越加速度区间为" << section.min_ << "到" << section.max_;
                        tmp_section_set.push_back(section);
                    } else {
                        LOG(INFO) << "此路径得到的超越加速度区间不存在";
                    }
                    // 判断是否能够合并区间
                    if (tmp_section_set.size() == 2) {
                        if (Tools::isLarge(tmp_section_set[0].max_, tmp_section_set[1].min_) && Tools::isLarge(tmp_section_set[1].max_, tmp_section_set[0].min_)) {
                            Section section;
                            section.min_ = tmp_section_set[0].min_;
                            section.max_ = tmp_section_set[1].max_;
                            tmp_section_set.resize(1);
                            tmp_section_set[0] = section;
                        }
                    }
                    // 进行区间求交集
                    if (tmp_section_set.size() == 0) {
                        return SectionSet();
                    } else {
                        final_result_section_set = Tools::getIntersection(final_result_section_set, tmp_section_set);
                    }
                } else {
                    // 不相交的情况
                    // 打印结果信息
                    LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "路径" << lane_index << "----障碍物与路径不相交";
                    // 什么都不做就好
                }
            }
        }
    }
    return final_result_section_set;
}

// 调整车辆状态来满足安全模型
SectionSet DecisionMaking::RSS::checkSaftyModel(StandardState *judge_state, const std::vector<Obstacle> &obstacles, bool is_aggressive) {
    // 初始化加速度可行区间
    SectionSet acceleration_section = SectionSet();
    // 首先第一步判断状态是否与静态障碍物相交，如果相交，返回空集
    bool is_collision_with_static_obstacles = false;
    // 遍历每一个障碍物
    for (size_t i = 0; i < obstacles.size(); i++) {
        // 遍历此时刻这个障碍物的每一预测轨迹
        for (size_t lane_index = 0; lane_index < obstacles[i].getPredictedTrajectoryNumber(); lane_index++) {
            if (checkStateBeingBlocked(judge_state, obstacles[i], lane_index)) {
                is_collision_with_static_obstacles = true;
                break;
            }
        }
    }

    if (is_collision_with_static_obstacles) {
        // 与静态障碍物相交
        LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "存在静态碰撞危险";
        return acceleration_section;
    }

    // 此时没有与静态障碍物碰撞
    // 下一步得到期望的加速度区间
    acceleration_section = getAccelerationSectionSet(judge_state, obstacles, is_aggressive);
    return acceleration_section;
}

// 速度规划（为了进行多线程操作才创建的函数）(要注意线程安全性)
void DecisionMaking::SubVehicle::velocityPlanningForState(StandardState *judge_state, const std::vector<Obstacle> &obstacles, bool is_aggressive) {
    if (judge_state->getCapability()) {
        // 判断如果状态初始不安全，直接退出
        if (judge_state->getSafety() == false) {
            return;
        }
        // 1-b 判断是否有逆行或静止障碍物，如果有，直接不安全并退出（也在checkSaftyModel中实现）
        // 1-c 进行安全性判断，得到安全的加速度区间（路径不与静止障碍物，逆行障碍物相交）
        SectionSet safety_acceleration_section_set = RSS::checkSaftyModel(judge_state, obstacles, is_aggressive);
        if (safety_acceleration_section_set.size() == 0) {
            LOG(INFO) << "状态的最终区间不存在";
        } else {
            std::stringstream ss;
            ss << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "最终区间为";
            for (size_t i = 0; i < safety_acceleration_section_set.size(); i++) {
                ss << safety_acceleration_section_set[i].min_ << "到" << safety_acceleration_section_set[i].max_ << ",";
            } 
            LOG(INFO) << ss.str();
        }
        if (safety_acceleration_section_set.size() == 0 || judge_state->getSafety() == false) {
            judge_state->setSafety(false);
            return;
        }
        // 赋值最大可行加速度
        judge_state->setStateMaxAvailableAcceleration(Tools::getSectionMaxValue(safety_acceleration_section_set));
        // 赋值加速度区间
        judge_state->setStateAccelerationSection(safety_acceleration_section_set);
        // 2. 得到安全性后，再进行舒适性判断，得到最合适的加速度。
        // 2-a. 首先计算正常情况下的加速度 (TOFIX)(TOFIX2020.8.19)
        // 计算轨迹限速
        double max_velocity_limitation_for_acceleration_calculation;
        max_velocity_limitation_for_acceleration_calculation = judge_state->getVelocityLimitationMax();
        // 得到期望加速度
        double normal_acceleration = 0.0;
        if (!Tools::isSmall(max_velocity_limitation_for_acceleration_calculation, judge_state->getVehicleCurrentMovement().velocity_)) {
            // 加速情况，如果是直行加速，如果是左转和右转速度保持就好
            if (judge_state->getStateName() == StateNames::FORWARD) {
                // 直行
                if ((this->guidance_type_ != Lane::GuidanceType::CHANGE_LEFT && this->guidance_type_ != Lane::GuidanceType::CHANGE_RIGHT) || this->is_length_enough_) {
                    // 如果直行是任务导向或距离足够
                    normal_acceleration = std::min(judge_state->getAccelerationLimitationMax() * EXPECTED_ACCELERATION_TO_MAX_ACCELERATION_RATIO, (max_velocity_limitation_for_acceleration_calculation - judge_state->getVehicleCurrentMovement().velocity_) / TIME_TO_EXPECTED_MAX_VELOCITY_FOLLOW_GUIDANCE);
                } else {
                    // 如果直行不是任务导向
                    normal_acceleration = std::min(MAX_ACCELERATION_FOR_NOT_FOLLOW_GUIDANCE, (judge_state->getExpectedVelocityCurrent() * VELOCITY_TO_EXPECTED_VELOCITY_RADIO_FOR_NOT_FOLLOW_GUIDANCE - judge_state->getVehicleCurrentMovement().velocity_) / TIME_TO_VELOCITY_FOR_NOT_FOLLOW_GUIDANCE);
                }

            } else if (judge_state->getStateName() == StateNames::TURN_LEFT) {
                // 非直行
                if ((this->guidance_type_ == Lane::GuidanceType::CHANGE_LEFT || this->guidance_type_ == Lane::GuidanceType::CENTER_LEFT || this->guidance_type_ == Lane::GuidanceType::ALL_AVAILABLE) || this->is_length_enough_) {
                    // 如果左转是任务导向或距离足够
                    // if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, std::min(1.6, max_velocity_limitation_for_acceleration_calculation))) {
                    //     // 如果速度小于阈值，则加速换道
                    //     normal_acceleration = 0.2;
                    // } else {
                    //     // 如果速度大于阈值，则匀速换道
                    //     normal_acceleration = 0.0;
                    // }
                    normal_acceleration = std::min(judge_state->getAccelerationLimitationMax() * EXPECTED_ACCELERATION_TO_MAX_ACCELERATION_RATIO, (max_velocity_limitation_for_acceleration_calculation - judge_state->getVehicleCurrentMovement().velocity_) / TIME_TO_EXPECTED_MAX_VELOCITY_FOLLOW_GUIDANCE);
                } else {
                    normal_acceleration = std::min(MAX_ACCELERATION_FOR_NOT_FOLLOW_GUIDANCE, (judge_state->getExpectedVelocityCurrent() * VELOCITY_TO_EXPECTED_VELOCITY_RADIO_FOR_NOT_FOLLOW_GUIDANCE - judge_state->getVehicleCurrentMovement().velocity_) / TIME_TO_VELOCITY_FOR_NOT_FOLLOW_GUIDANCE);
                }

            } else if (judge_state->getStateName() == StateNames::TURN_RIGHT) {
                // 非直行
                if ((this->guidance_type_ == Lane::GuidanceType::CHANGE_RIGHT || this->guidance_type_ == Lane::GuidanceType::CENTER_RIGHT || this->guidance_type_ == Lane::GuidanceType::ALL_AVAILABLE) || this->is_length_enough_) {
                    // if (Tools::isSmall(judge_state->getVehicleCurrentMovement().velocity_, std::min(1.6, max_velocity_limitation_for_acceleration_calculation))) {
                    //     // 如果速度小于阈值，则加速换道
                    //     normal_acceleration = 0.2;
                    // } else {
                    //     // 如果速度大于阈值，则匀速换道
                    //     normal_acceleration = 0.0;
                    // }
                    normal_acceleration = std::min(judge_state->getAccelerationLimitationMax() * EXPECTED_ACCELERATION_TO_MAX_ACCELERATION_RATIO, (max_velocity_limitation_for_acceleration_calculation - judge_state->getVehicleCurrentMovement().velocity_) / TIME_TO_EXPECTED_MAX_VELOCITY_FOLLOW_GUIDANCE);
                } else {
                    normal_acceleration = std::min(MAX_ACCELERATION_FOR_NOT_FOLLOW_GUIDANCE, (judge_state->getExpectedVelocityCurrent() * VELOCITY_TO_EXPECTED_VELOCITY_RADIO_FOR_NOT_FOLLOW_GUIDANCE - judge_state->getVehicleCurrentMovement().velocity_) / TIME_TO_VELOCITY_FOR_NOT_FOLLOW_GUIDANCE);
                }
            }
        } else {
            // 减速情况
            normal_acceleration = std::max((max_velocity_limitation_for_acceleration_calculation - judge_state->getVehicleCurrentMovement().velocity_) / TIME_TO_EXPECTED_VELOCITY_WHEN_DECCELERATION, -COMMON_DECCELERATION);
        }
        LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "目标速度为" << judge_state->getExpectedVelocityLimitation() << "，限制速度上限为" << judge_state->getVelocityLimitationMax() << "，而当前速度为" << judge_state->getVehicleCurrentMovement().velocity_ << "，因此正常情况下期望的加速度为" << normal_acceleration;

        // 2-b. 从加速度可行域中找出与此加速度最接近的加速度
        double expected_acceleration;
        double measure_distance_from_normal_acceleration = MAX_VALUE;
        Section shortest_distance_acceleration_section = Section();
        for (size_t i = 0; i < safety_acceleration_section_set.size(); i++) {
            Section test_section = safety_acceleration_section_set[i];
            double distance = Tools::distanceToSection(test_section, normal_acceleration);
            if (Tools::isSmall(distance, measure_distance_from_normal_acceleration) || Tools::isEqual(distance, measure_distance_from_normal_acceleration)) {
                measure_distance_from_normal_acceleration = distance;
                shortest_distance_acceleration_section = test_section;
            }
        }
        LOG(INFO) << "与期望加速度最近的可行加速度区间为" << shortest_distance_acceleration_section.min_ << "到" << shortest_distance_acceleration_section.max_;
        // 利用离正常加速度最近的可行区间得到期望加速度
        if ((Tools::isLarge(normal_acceleration, shortest_distance_acceleration_section.min_) || Tools::isEqual(normal_acceleration, shortest_distance_acceleration_section.min_)) && (Tools::isSmall(normal_acceleration, shortest_distance_acceleration_section.max_) || Tools::isEqual(normal_acceleration, shortest_distance_acceleration_section.max_))) {
            expected_acceleration = normal_acceleration;
        } else if (Tools::isSmall(normal_acceleration, shortest_distance_acceleration_section.min_)) {
            // 如果正常加速度小于区间下限
            expected_acceleration = std::min(shortest_distance_acceleration_section.min_ + SMALL_VALUE_FOR_AVOID_REACH_BOUNDARY, shortest_distance_acceleration_section.max_);
        } else {
            // 如果正常加速度大于区间上限
            expected_acceleration = std::max(shortest_distance_acceleration_section.min_, shortest_distance_acceleration_section.max_ - SMALL_VALUE_FOR_AVOID_REACH_BOUNDARY);
        }
        // 设置状态的期望平均加速度
        judge_state->setVehicleDynamicPlanningExpectedAcceleration(expected_acceleration);
        LOG(INFO) << "状态" << DIC_STATE_NAME[judge_state->getStateName()] << "得到的最终期望加速度为" << expected_acceleration;
    }
}
