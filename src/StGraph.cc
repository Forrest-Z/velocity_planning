/*
 * @Author: fujiawei0724
 * @Date: 2022-08-03 15:59:29
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-13 15:04:50
 * @Description: s-t graph for velocity planning.
 */
#include "Common.hpp"

namespace VelocityPlanning {



GridMap2D::GridMap2D(int x_max, int y_max) {
    mat_ = cv::Mat(y_max, x_max, CV_8UC1, cv::Scalar(255));
}

GridMap2D::~GridMap2D() = default;

std::vector<cv::Point> GridMap2D::eigenToCvPoint(const std::vector<Eigen::Vector2i>& points) {
    std::vector<cv::Point> cv_points;
    for (int i = 0; i < static_cast<int>(points.size()); i++) {
        cv_points.emplace_back(cv::Point(points[i](0), points[i](1)));
    }
    return cv_points;
}

void GridMap2D::print() {
    std::cout << mat_ << std::endl;
}

void GridMap2D::visualization() {
    cv::imshow("Grid map", mat_);
    cv::waitKey(0);
}

void GridMap2D::visualization(const std::vector<Cube2D<int>>& cubes) {
    addCubesVisualization(cubes);
    cv::imshow("Grid map", mat_);
    cv::waitKey();
}

void GridMap2D::visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths) {
    for (auto const& cubes : cube_paths) {
        addCubesVisualization(cubes);
    }
    cv::imshow("Grid map", mat_);
    cv::waitKey();
}

void GridMap2D::fillAccBannedArea(const std::vector<Eigen::Vector2i>& vertice) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> contour = eigenToCvPoint(vertice);
    contours.emplace_back(contour);
    cv::fillPoly(mat_, contours, cv::Scalar(ValType::OCCUPIED), 8);
}

void GridMap2D::fillObstacleBannedArea(const std::vector<Eigen::Vector2i>& vertice) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> contour = eigenToCvPoint(vertice);
    contours.emplace_back(contour);
    cv::fillPoly(mat_, contours, cv::Scalar(ValType::HALF_OCCUPIED), 8);
}

bool GridMap2D::expandSingleColumn(const int& grid_t_start, const int& grid_t_end, const int& grid_s_start, std::vector<Cube2D<int>>* cubes, int* real_s_start) {
    std::vector<Cube2D<int>> calculated_cubes;

    // Get grid s start
    int cur_s_start = grid_s_start;
    int max_s_grid = mat_.rows;
    for (; cur_s_start < max_s_grid; cur_s_start++) {
        ValType collision_type = getOccupiedState(grid_t_start, grid_t_end, cur_s_start);
        if (collision_type == ValType::OCCUPIED) continue;
        if (collision_type == ValType::HALF_OCCUPIED) continue;
        if (collision_type == ValType::FREE) break;
    }

    // Judge failed expansion
    if (cur_s_start >= max_s_grid) {

        // // DEBUG
        // std::cout << "cur s start: " << cur_s_start << ", max s grid: " << max_s_grid << std::endl;
        // // END DEBUG

        return false;
    }
    // Update real s start
    *real_s_start = cur_s_start;

    // Start expanding
    int cur_s_grid = cur_s_start;
    for (; cur_s_grid < max_s_grid; cur_s_grid++) {
        ValType collision_type = getOccupiedState(grid_t_start, grid_t_end, cur_s_grid);
        if (collision_type == ValType::OCCUPIED) {
            // Complete the last round
            if (cur_s_grid > cur_s_start) {
                Cube2D<int> cube = Cube2D<int>(grid_t_start, grid_t_end, cur_s_start, cur_s_grid);
                calculated_cubes.emplace_back(cube);
            }
            break;
        } else if (collision_type == ValType::HALF_OCCUPIED) {
            // Complete the medium round
            if (cur_s_grid > cur_s_start) {
                Cube2D<int> cube = Cube2D<int>(grid_t_start, grid_t_end, cur_s_start, cur_s_grid);
                calculated_cubes.emplace_back(cube);
            }

            // Update to next start index
            bool is_adjacent_to_occupied = false;
            for (; cur_s_grid < max_s_grid; cur_s_grid++) {
                ValType cur_collision_type = getOccupiedState(grid_t_start, grid_t_end, cur_s_grid);
                if (cur_collision_type == ValType::HALF_OCCUPIED) {
                    continue;
                } else if (cur_collision_type == ValType::OCCUPIED) {
                    is_adjacent_to_occupied = true;
                    break;
                } else if (cur_collision_type == ValType::FREE) {
                    // Get next start index
                    cur_s_start = cur_s_grid;
                    break;
                }
            }
            if (is_adjacent_to_occupied) {
                // No free space
                break;
            }

        } else if (collision_type == ValType::FREE) {
            continue;
        }
    }

    *cubes = calculated_cubes;
    return true;

    
}

GridMap2D::ValType GridMap2D::getOccupiedState(const int& grid_t_start, const int& grid_t_end, const int& grid_s) {
    // Iterate points in range
    ValType res_collision_type = ValType::FREE;
    for (int i = grid_t_start; i <= grid_t_end; i++) {
        uchar pixel_value = mat_.ptr<uchar>(grid_s)[i];
        ValType cur_collision_type = (ValType)static_cast<int>(pixel_value);
        if (cur_collision_type == ValType::OCCUPIED) {
            res_collision_type = cur_collision_type;
            break;
        } else if (cur_collision_type == ValType::HALF_OCCUPIED) {
            res_collision_type = cur_collision_type;
        } else if (cur_collision_type == ValType::FREE) {
            // Do nothing
            // Maybe some logic is need here
        }
    }

    return res_collision_type;

}

void GridMap2D::addCubeVisualization(const Cube2D<int>& grid_cube) {
    std::vector<cv::Point> contour = {cv::Point(grid_cube.t_start_, grid_cube.s_start_), cv::Point(grid_cube.t_start_, grid_cube.s_end_), cv::Point(grid_cube.t_end_, grid_cube.s_end_), cv::Point(grid_cube.t_end_, grid_cube.s_start_)};
    
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(contour);
    cv::polylines(mat_, contours, true, cv::Scalar(ValType::CUBE_BOUNDARY), 2);
}

void GridMap2D::addCubesVisualization(const std::vector<Cube2D<int>>& grid_cubes) {
    for (const auto& cube : grid_cubes) {
        addCubeVisualization(cube);
    }
}






StGraph::StGraph(const PathPlanningUtilities::Curve& path, const Param& param, const double& current_velocity) {
    // Load parameters
    start_velocity_ = current_velocity;
    param_ = param;
    path_ = path;
    ego_occupy_area_ = DecisionMaking::RSS::OccupationArea(path, param.occupied_width, param.occupied_length, 20);
    
    // Initialize grid map
    int x_max = std::round(param.t_max / param.t_resolution) + 1;
    int y_max = std::round(param.s_max / param.s_resolution) + 1;
    grid_map_2D_ = new GridMap2D(x_max, y_max);

    // // DEBUG
    // grid_map_2D_->visualization();
    // // END DEBUG


  
}

StGraph::~StGraph() = default;



Eigen::Vector2i StGraph::realValueToGridPos(const Eigen::Vector2d& real_position) {
    // The origin values of both s and t are 0.0, there is no offset
    int grid_x = std::round(real_position(0) / param_.t_resolution);
    int grid_y = std::round(real_position(1) / param_.s_resolution);
    Eigen::Vector2i grid_pos{grid_x, grid_y};
    return grid_pos;
}

std::vector<Eigen::Vector2i> StGraph::realValuesToGridPoss(const std::vector<Eigen::Vector2d>& real_positions) {
    int n = real_positions.size();
    std::vector<Eigen::Vector2i> grid_poss(n);
    for (int i = 0; i < n ; i++) {
        grid_poss[i] = realValueToGridPos(real_positions[i]);
    }
    return grid_poss;
}

Eigen::Vector2d StGraph::gridPosToRealValue(const Eigen::Vector2i& grid_position) {
    double real_t = grid_position(0) * param_.t_resolution;
    double real_s = grid_position(1) * param_.s_resolution;
    Eigen::Vector2d real_value{real_t, real_s};
    return real_value;
}

std::vector<Eigen::Vector2d> StGraph::gridPossToRealValues(const std::vector<Eigen::Vector2i>& grid_positions) {
    int n = grid_positions.size();
    std::vector<Eigen::Vector2d> real_values(n);
    for (int i = 0; i < n; i++) {
        real_values[i] = gridPosToRealValue(grid_positions[i]);
    }
    return real_values;
}

std::vector<Cube2D<double>> StGraph::gridCubesToRealCubes(const std::vector<Cube2D<int>>& grid_cubes) {
    std::vector<Cube2D<double>> cubes;
    for (int i = 0; i < grid_cubes.size(); i++) {
        cubes.emplace_back(gridCubeToRealCube(grid_cubes[i]));
    }
    return cubes;
}

Cube2D<double> StGraph::gridCubeToRealCube(const Cube2D<int>& grid_cube) {
    double t_start_real = grid_cube.t_start_ * param_.t_resolution;
    double t_end_real = grid_cube.t_end_ * param_.t_resolution;
    double s_start_real = grid_cube.s_start_ * param_.s_resolution;
    double s_end_real = grid_cube.s_end_ * param_.s_resolution;
    Cube2D<double> cube_real = Cube2D<double>(t_start_real, t_end_real, s_start_real, s_end_real);
    return cube_real;
}



std::vector<std::vector<Eigen::Vector2d>> StGraph::loadObstacle(const DecisionMaking::Obstacle& obstacle) {
    // if (obstacle.getObstacleVelocity() < 0.1) {
    //     // Ignore static obstacles, they will be processed by other logic
    //     // TODO: complete this logic
    //     return;
    // }

    // // DEBUG
    // std::cout << "load obstacle success" << std::endl;
    // // END DEBUG

    // // DEBUG
    // std::cout << "Obstacle trajectory number: " << obstacle.getPredictedTrajectoryNumber() << std::endl;
    // // END DEBUG

    std::vector<std::vector<Eigen::Vector2d>> real_vertex;

    for (int i = 0; i < obstacle.getPredictedTrajectoryNumber(); i++) {
        // Construct occupied area
        DecisionMaking::RSS::OccupationArea cur_obs_occupy_area = DecisionMaking::RSS::OccupationArea(obstacle, i, 1);
        
        // Check collision
        int ego_vehicle_start_collision_index = -1;
        int ego_vehicle_end_collision_index = -1;
        int cur_obs_start_collision_index = -1;
        int cur_obs_end_collision_index = -1;
        bool is_collision = DecisionMaking::RSS::occupationInteractionJudgement(ego_occupy_area_, cur_obs_occupy_area, &ego_vehicle_start_collision_index, &ego_vehicle_end_collision_index, &cur_obs_start_collision_index, &cur_obs_end_collision_index);

        if (!is_collision) {
            
            // // DEBUG
            // std::cout << "Without collision" << std::endl;
            // // END DEBUG

            continue;
        }

        if (ego_vehicle_start_collision_index == 0) {
            
            // // DEBUG
            // std::cout << "Rear vehicle, ignored" << std::endl;
            // // END DEBUG

            continue;
        }

        // Handle the situation where s_end is smaller than s_start
        if (obstacle.getObstacleVelocity() > 1.0 && fabs(obstacle.getObstacleVelocityDirection() - ego_occupy_area_.getOccupationArea()[ego_vehicle_start_collision_index].rotation_) > M_PI / 2.0) {
            swap(ego_vehicle_start_collision_index, ego_vehicle_end_collision_index);
        }

        // Calculate collision information
        double t_start = (ego_vehicle_start_collision_index * OBSTACLE_MARGIN) / obstacle.getObstacleVelocity();
        double t_end = (ego_vehicle_end_collision_index * OBSTACLE_MARGIN) / obstacle.getObstacleVelocity();
        double s_start = ego_vehicle_start_collision_index * LANE_GAP_DISTANCE;
        double s_end = ego_vehicle_end_collision_index * LANE_GAP_DISTANCE;

        // // DEBUG
        // std::cout << "t start: " << t_start << std::endl;
        // std::cout << "t end: " << t_end << std::endl;
        // std::cout << "s start: " << s_start << std::endl;
        // std::cout << "s end: " << s_end << std::endl;
        // // END DEBUG    

        // Calculate the projected length of the obstacle to the path of the ego lane
        double angle_diff = obstacle.getPredictedTrajectory(i)[cur_obs_start_collision_index].theta_ - path_[ego_vehicle_start_collision_index].theta_;
        double projected_length = std::max(obstacle.getObstacleLength() * fabs(cos(angle_diff)), obstacle.getObstacleWidth() * fabs(cos(angle_diff + M_PI * 0.5)));

        // // DEBUG
        // std::cout << "angle diff: " << angle_diff << std::endl;
        // std::cout << "projected length: " << projected_length << std::endl;
        // std::cout << obstacle.getObstacleLength() * fabs(tan(angle_diff)) << std::endl;
        // std::cout << obstacle.getObstacleWidth() * fabs(tan(angle_diff + M_PI * 0.5)) << std::endl;
        // std::cout << obstacle.getObstacleWidth() << std::endl;
        // std::cout << fabs(tan(angle_diff + M_PI * 0.5)) << std::endl;
        // // END DEBUG

        // Get four vertice
        std::vector<Eigen::Vector2d> real_vertice = {{t_start, s_start}, {t_start, s_start + projected_length}, {t_end, s_end}, {t_end, s_end - projected_length}};

        // Record
        real_vertex.emplace_back(real_vertice);

        // // DEBUG
        // for (int i = 0; i < 4; i++) {
        //     std::cout << real_vertice[i] << std::endl;
        // }
        // // END DEBUG

        // Convert
        std::vector<Eigen::Vector2i> grid_positions = realValuesToGridPoss(real_vertice);

        // DEBUG
        for (int i = 0; i < 4; i++) {
            std::cout << grid_positions[i] << std::endl;
        }
        // END DEBUG

        // Picture the banned area to grid map
        grid_map_2D_->fillObstacleBannedArea(grid_positions);

    }

    return real_vertex;
}

std::vector<std::vector<std::vector<Eigen::Vector2d>>> StGraph::loadObstacles(const std::vector<DecisionMaking::Obstacle>& obstacles) {
    // Record all the vertex to match the corresponding uncertainty
    std::vector<std::vector<std::vector<Eigen::Vector2d>>> obstalces_real_vertex;
    for (auto const& obs : obstacles) {
        std::vector<std::vector<Eigen::Vector2d>> obs_real_vertex = loadObstacle(obs);
        obstalces_real_vertex.emplace_back(obs_real_vertex);
    }

    return obstalces_real_vertex;
}


void StGraph::loadAccelerationLimitation() {
    // Load origin acceleration limitation to the graph
    // Several sampled point is selected to depict the irregular shape
    std::vector<double> sampled_ts(param_.acc_limit_t_sampled_points_num + 1);
    std::vector<double> sampled_lower_ss(param_.acc_limit_t_sampled_points_num + 1, 0.0);
    std::vector<double> sampled_upper_ss(param_.acc_limit_t_sampled_points_num + 1, param_.s_max);

    for (int i = 0; i <= param_.acc_limit_t_sampled_points_num; i++) {
        // Calculate the sampled time stamp
        double sampled_t = (static_cast<double>(i) / static_cast<double>(param_.acc_limit_t_sampled_points_num)) * param_.t_max;
        sampled_ts[i] = sampled_t;

        // // Calculate the lower boundary point
        // double sampled_cur_lower_s = start_velocity_ * sampled_t + 0.5 * param_.acc_min * pow(sampled_t, 2);
        // sampled_lower_ss[i] = std::max(0.0, sampled_cur_lower_s);
        // if (i > 0) {
        //     sampled_lower_ss[i] = std::max(sampled_lower_ss[i], sampled_lower_ss[i - 1]);
        // }
        sampled_lower_ss[i] = 0.0;
        

        // Calculate the upper boundary point
        double cur_pred_velocity = start_velocity_ + sampled_t * param_.acc_max;
        double sampled_cur_upper_s = 0.0;
        if (cur_pred_velocity >= param_.velocity_max && i != 0) {
            sampled_cur_upper_s = sampled_upper_ss[i - 1] + (sampled_ts[i] - sampled_ts[i - 1]) * param_.velocity_max;
        } else {
            sampled_cur_upper_s = start_velocity_ * sampled_t + 0.5 * param_.acc_max * pow(sampled_t, 2);
        }
        sampled_upper_ss[i] = std::min(param_.s_max, sampled_cur_upper_s);

        
    }

    // Generate boundary points
    std::vector<Eigen::Vector2d> lower_boundary_values;
    std::vector<Eigen::Vector2d> upper_boundary_values;
    for (int i = 0; i <= param_.acc_limit_t_sampled_points_num; i++) {
        lower_boundary_values.push_back({sampled_ts[i] - param_.t_resolution, sampled_lower_ss[i]});
        upper_boundary_values.push_back({sampled_ts[i] - param_.t_resolution, sampled_upper_ss[i]});
        if (i != param_.acc_limit_t_sampled_points_num) {
            upper_boundary_values.push_back({sampled_ts[i] - param_.t_resolution, sampled_upper_ss[i + 1]});
        }
    }

    // Supply the vertex to fill the polygon
    lower_boundary_values.push_back({param_.t_max, 0.0});
    upper_boundary_values.push_back({0.0, param_.s_max});

    // Convert to grid positions
    std::vector<Eigen::Vector2i> lower_boundary_grid_positions = realValuesToGridPoss(lower_boundary_values);
    std::vector<Eigen::Vector2i> upper_boundary_grid_positions = realValuesToGridPoss(upper_boundary_values);

    // Picture the banned area to grid map
    grid_map_2D_->fillAccBannedArea(lower_boundary_grid_positions);
    grid_map_2D_->fillAccBannedArea(upper_boundary_grid_positions);
}

bool StGraph::generateCubes(std::vector<std::vector<Cube2D<double>>>* cubes, std::vector<std::pair<double, double>>* last_s_range) {
    std::vector<std::vector<Cube2D<double>>> calculated_cubes;
    
    // DEBUG
    std::vector<std::vector<Cube2D<int>>> calculated_grid_cubes_columns;
    // END DEBUG
    
    // Calculate lateral cube width for each cube
    double t_lateral = param_.t_max / param_.lateral_segement_number;

    // Expand cubes
    // ~Stage I: special process for the first cube and second cube
    // std::vector<Cube2D<double>> first_cube;
    // double first_cube_s_max = start_velocity_ * t_lateral + 0.5 * param_.acc_max * pow(t_lateral, 2);
    // Cube2D<double> first_cur_cube = Cube2D<double>(0.0, t_lateral, 0.0, first_cube_s_max);

    // // DEBUG
    // std::cout << "First cube information" << std::endl;
    // first_cur_cube.print();
    // // END DEBUG

    // first_cube.emplace_back(first_cur_cube);
    // calculated_cubes.emplace_back(first_cube);

    // std::vector<Cube2D<double>> second_cube;
    // double second_cube_s_max = start_velocity_ * 2.0 * t_lateral + 0.5 * param_.acc_max * pow(2.0 * t_lateral, 2);
    // double second_cube_s_min = std::min(start_velocity_ * 2.0 * t_lateral + 0.5 * param_.acc_min * pow(2.0, t_lateral), 0.0);
    // Cube2D<double> second_cur_cube = Cube2D<double>(t_lateral, 2.0 * t_lateral, second_cube_s_min, second_cube_s_max);
    // second_cube.emplace_back(second_cur_cube);
    // calculated_cubes.emplace_back(second_cube);


    // ~Stage II: iterative expand the followed cubes
    // TODO: try multithreading here
    int s_start = 0;
    for (int i = 0; i < param_.lateral_segement_number; i++) {
        // Calculate the t boundary for this expansion
        double cur_t_start = i * t_lateral;
        double cur_t_end = (i + 1) * t_lateral;
        int cur_t_start_grid = static_cast<int>(cur_t_start / param_.t_resolution);
        int cur_t_end_grid = static_cast<int>(cur_t_end / param_.t_resolution);

        // // DEBUG
        // std::cout << "cur t start grid: " << cur_t_start_grid << std::endl;
        // std::cout << "cur t end grid: " << cur_t_end_grid << std::endl;
        // std::cout << "cur s start: " << s_start << std::endl; 
        // // END DEBUG

        // Get grid cubes in this column
        std::vector<Cube2D<int>> cur_cubes;
        bool is_expansion_success = grid_map_2D_->expandSingleColumn(cur_t_start_grid, cur_t_end_grid, s_start, &cur_cubes, &s_start);
        if (!is_expansion_success) {

            // // DEBUG
            // std::cout << "col num: " << i << std::endl;
            // // END DEBUG

            return false;
        }

        // // DEBUG
        // std::cout << "i: " << i << ", cubes size: " <<  cur_cubes.size() << std::endl;
        // // END DEBUG

        calculated_grid_cubes_columns.emplace_back(cur_cubes);

        calculated_cubes.emplace_back(gridCubesToRealCubes(cur_cubes));
    }

    // // Edit the second cube
    // calculated_cubes[1][0].s_start_ = calculated_cubes[2][0].s_start_;
    // calculated_cubes[1][0].s_end_ = (calculated_cubes[2].back().s_end_ + calculated_cubes[0][0].s_end_) / 2.0;

    // // Edit the first cube
    // calculated_cubes[0][0].s_end_ = calculated_cubes[1][0].s_end_;

    // // DEBNG
    // for (int i = 0; i < calculated_grid_cubes_columns.size(); i++) {
    //     for (int j = 0; j < calculated_grid_cubes_columns[i].size(); j++) {
    //         Cube2D<int> this_grid_cube = calculated_grid_cubes_columns[i][j];
    //         std::cout << "Cols: " << i << ", num: " << j << std::endl;
    //         this_grid_cube.print();
    //     }
    // }
    // // END DEBUG

    // // DEBUG
    // // Visualization 
    calculated_grid_cubes_columns_ = calculated_grid_cubes_columns;
    // visualization(calculated_grid_cubes_columns);
    // END DEBUG

    *cubes = calculated_cubes;

    // Calculate the last s range
    std::vector<Cube2D<double>> last_cubes_cols = calculated_cubes.back();
    std::vector<std::pair<double, double>> calculated_last_s_range;
    for (const auto& cube : last_cubes_cols) {
        calculated_last_s_range.emplace_back(std::make_pair(cube.s_start_, cube.s_end_));
    }

    *last_s_range = calculated_last_s_range;

    return true;
}

bool StGraph::isCubesConnected(const Cube2D<double>& cube_1, const Cube2D<double>& cube_2) {
    // DEBUG
    // END DEBUG

    if (cube_1.s_start_ > cube_2.s_end_ || cube_2.s_start_ > cube_1.s_end_) {
        return false;
    }
    return true;

}

bool StGraph::connectCubes(const std::vector<std::vector<Cube2D<double>>>& input_cubes, std::vector<std::vector<Cube2D<double>>>* output_cubes) {
    std::vector<Cube2D<double>> cube_path;
    dfsConnectCubes(input_cubes, 0, cube_path);

    // // DEBUG
    // std::cout << "dfs search complete" <<std::endl; 
    // // END DEBUG

    if (connected_cubes_.size() > 0) {
        *output_cubes = connected_cubes_;

        // DEBUG
        for (int i = 0; i < connected_cubes_.size(); i++) {
            std::cout << "path " << i << std::endl;
            for (int j = 0; j < connected_cubes_[i].size(); j++) {
                std::cout << "cube " << j << std::endl;
                connected_cubes_[i][j].print();
            }
        }
        // visualization(calculated_grid_cubes_columns_);
        // END DEBUG


        calculated_grid_cubes_columns_.clear();
        connected_cubes_.clear();
        return true;
    } else {

        // DEBUG
        std::cout << "Connection failed." << std::endl;
        // END DEBUG

        return false;
    }
    

    
}

void StGraph::dfsConnectCubes(const std::vector<std::vector<Cube2D<double>>>& input_cubes, int layer_index, std::vector<Cube2D<double>>& cube_path) {

    // DEBUG
    std::cout << "layer index: " << layer_index << std::endl;
    // END DEBUG

    if (layer_index == input_cubes.size()) {
        connected_cubes_.emplace_back(cube_path);
        return;
    }
    if (layer_index == 0 && cube_path.size() == 0) {
        cube_path.emplace_back(input_cubes[0][0]);
        dfsConnectCubes(input_cubes, layer_index + 1, cube_path);
        cube_path.pop_back();
    } else {
        Cube2D<double> last_cube = cube_path.back();
        for (int i = 0; i < input_cubes[layer_index].size(); i++) {
            if (isCubesConnected(last_cube, input_cubes[layer_index][i])) {
                cube_path.emplace_back(input_cubes[layer_index][i]);
                dfsConnectCubes(input_cubes, layer_index + 1, cube_path);
                cube_path.pop_back();
            }
        }
    }
}

bool StGraph::runOnce(const std::vector<DecisionMaking::Obstacle>& obstacles, std::vector<std::vector<Cube2D<double>>>* cube_paths, std::vector<std::pair<double, double>>* s_range) {
    // ~Stage I: add obstacles
    loadObstacles(obstacles);

    // ~Stage: II: add acc limitation
    loadAccelerationLimitation();

    // ~Stage II: generate cubes
    std::vector<std::vector<Cube2D<double>>> cubes;
    std::vector<std::pair<double, double>> last_s_range;
    bool is_generated_success = generateCubes(&cubes, &last_s_range);
    if (!is_generated_success) {
        return false;
    }

    // ~Stage III: generate cube paths
    std::vector<std::vector<Cube2D<double>>> connected_cube_paths;
    bool is_connected_success = connectCubes(cubes, &connected_cube_paths);
    if (!is_connected_success) {
        return false;
    }

    *cube_paths = connected_cube_paths;
    *s_range = last_s_range;
    return true;

}

void StGraph::visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths) {
    grid_map_2D_->visualization(cube_paths);
}


void StGraph::visualization() {
    grid_map_2D_->visualization();
}

void StGraph::print() {
    grid_map_2D_->print();
}

UncertaintyOccupiedArea::UncertaintyOccupiedArea() = default;

UncertaintyOccupiedArea::UncertaintyOccupiedArea(const std::vector<Eigen::Vector2d>& vertex, const Gaussian2D& gaussian_dis) {
    vertex_ = vertex;
    gaussian_dis_ = gaussian_dis;
}

UncertaintyOccupiedArea::~UncertaintyOccupiedArea() = default;

Gaussian2D UncertaintyOccupiedArea::toPointGaussianDis(Eigen::Vector2d& vertice) {
    Gaussian2D gaussian_dis;
    gaussian_dis.ave_values_ = vertice;
    gaussian_dis.covariance_ = gaussian_dis_.covariance_;
    return gaussian_dis;
}

UncertaintyObstacle::UncertaintyObstacle() = default;

UncertaintyObstacle::UncertaintyObstacle(const DecisionMaking::Obstacle& obs, const Gaussian2D& gaussian_dis) {
    obs_ = obs;
    gaussian_dis_ = gaussian_dis;
}

UncertaintyObstacle::~UncertaintyObstacle() = default;

void UncertaintyStGraph::loadObstacle(const UncertaintyObstacle& uncertainty_obs) {
    // Get the specific occupied area
    std::vector<std::vector<Eigen::Vector2d>> obs_traj_vertex = StGraph::loadObstacle(uncertainty_obs.obs_);

    // Transform uncertainty gaussian distribution
    // TODO: add this logic, trasform from world to s-t, include two steps: transform from world to frenet, then from frenet to s-t
    Gaussian2D obs_gaussian_dis;

    // Record
    for (const auto cur_traj_obs_vertex : obs_traj_vertex) {
        UncertaintyOccupiedArea uncer_occ_area = UncertaintyOccupiedArea(cur_traj_obs_vertex, obs_gaussian_dis);
        uncertainty_occupied_areas_.emplace_back(uncer_occ_area);
    }

}

void UncertaintyStGraph::loadObstacles(const std::vector<UncertaintyObstacle>& uncertainty_obstacles) {
    // Traverse obstacles
    for (const auto uncertainty_obs : uncertainty_obstacles) {
        loadObstacle(uncertainty_obs);
    }
}

std::vector<std::vector<Cube2D<double>>> UncertaintyStGraph::enhanceSafety(std::vector<std::vector<Cube2D<double>>>& initial_cube_paths) {

    assert(!initial_cube_paths.empty());

    // Initial results
    int m = initial_cube_paths.size();
    int n = initial_cube_paths[0].size();
    std::vector<std::vector<UncertaintyCube2D<double>>> enhanced_safety_uncertainty_cube_paths(m);

    // Add the uncertainty information to cube paths
    for (int i = 0; i < m; i++) {
        std::vector<UncertaintyCube2D<double>> uncertainty_cubes_path = transformCubesPathToUncertaintyCubesPath(initial_cube_paths[i]);
        enhanced_safety_uncertainty_cube_paths[i] = uncertainty_cubes_path;
    }

    // Execute safety enhancement for each cube
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {

        }
    }



}

std::vector<UncertaintyCube2D<double>> UncertaintyStGraph::transformCubesPathToUncertaintyCubesPath(const std::vector<Cube2D<double>>& cubes) {
    // Initialize
    int m = cubes.size();
    std::vector<UncertaintyCube2D<double>> uncertainty_cubes_path(m, UncertaintyCube2D<double>());

    // Calculate the accumulated gaussian distribution
    // Note that the covariance of the gaussian distribution is accumulated with the increasing of time stamp
    for (int i = 0; i < m; i++) {
        // Calculate the upper and lower bounds' gaussian distribution
        // TODO: add the calculation logic here
        Gaussian1D upper_gaussian_dis;
        Gaussian1D lower_gaussian_dis;

        // Supply data
        Cube2D<double> cur_cube = cubes[i];
        UncertaintyCube2D<double> uncertainty_cube = UncertaintyCube2D<double>(cur_cube, upper_gaussian_dis, lower_gaussian_dis);

        uncertainty_cubes_path[i] = uncertainty_cube;
    }

    return uncertainty_cubes_path;
    
}

bool UncertaintyStGraph::limitUncertaintyCube(UncertaintyCube2D<double>* uncertainty_cube) {
    
}


bool UncertaintyStGraph::limitSingleBound(const Gaussian1D& gaussian_dis, const double& t_start, const double& t_end, const BoundType& bound_type, double* limited_bound) {
    // Initialize buffer value
    double buffer_value = 0.0;

    

}










} // End of namespace VelocityPlanning