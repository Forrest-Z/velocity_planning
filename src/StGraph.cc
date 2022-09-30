/*
 * @Author: fujiawei0724
 * @Date: 2022-08-03 15:59:29
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-30 09:44:28
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

void GridMap2D::visualization(const std::string& name) {
    cv::Mat shown_mat;
    cv::flip(mat_, shown_mat, 0);
    cv::imshow(name, shown_mat);
    cv::waitKey(100);
}

void GridMap2D::visualization(const std::vector<Cube2D<int>>& cubes, const std::string& name) {
    cv::Mat drawn_mat = mat_.clone();
    addCubesVisualization(cubes, drawn_mat);
    cv::Mat shown_mat;
    cv::flip(drawn_mat, shown_mat, 0);
    cv::imshow(name, shown_mat);
    cv::waitKey(100);
}

void GridMap2D::visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths, const std::string& name) {
    cv::Mat drawn_mat = mat_.clone();
    for (auto const& cubes : cube_paths) {
        addCubesVisualization(cubes, drawn_mat);
    }
    cv::Mat shown_mat;
    cv::flip(drawn_mat, shown_mat, 0);
    cv::imshow(name, shown_mat);
    cv::waitKey(100);
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
                cube.upper_collision_type_ = CollisionType::ACCELERATION_BOUNDARY;
                if (calculated_cubes.empty()) {
                    cube.lower_collision_type_ = CollisionType::ACCELERATION_BOUNDARY;
                } else {
                    cube.lower_collision_type_ = CollisionType::OBSTACLE_BOUNDARY;
                }
                calculated_cubes.emplace_back(cube);
            }
            break;
        } else if (collision_type == ValType::HALF_OCCUPIED) {
            // Complete the medium round
            if (cur_s_grid > cur_s_start) {
                Cube2D<int> cube = Cube2D<int>(grid_t_start, grid_t_end, cur_s_start, cur_s_grid);
                cube.upper_collision_type_ = CollisionType::OBSTACLE_BOUNDARY;
                if (calculated_cubes.empty()) {
                    cube.lower_collision_type_ = CollisionType::ACCELERATION_BOUNDARY;
                } else {
                    cube.lower_collision_type_ = CollisionType::OBSTACLE_BOUNDARY;
                }
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

void GridMap2D::addCubeVisualization(const Cube2D<int>& grid_cube, cv::Mat& copied_mat) {
    std::vector<cv::Point> contour = {cv::Point(grid_cube.t_start_, grid_cube.s_start_), cv::Point(grid_cube.t_start_, grid_cube.s_end_), cv::Point(grid_cube.t_end_, grid_cube.s_end_), cv::Point(grid_cube.t_end_, grid_cube.s_start_)};
    
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(contour);
    cv::polylines(copied_mat, contours, true, cv::Scalar(ValType::CUBE_BOUNDARY), 2);
}

void GridMap2D::addCubesVisualization(const std::vector<Cube2D<int>>& grid_cubes, cv::Mat& copied_mat) {
    for (const auto& cube : grid_cubes) {
        addCubeVisualization(cube, copied_mat);
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
    Cube2D<double> cube_real = Cube2D<double>(t_start_real, t_end_real, s_start_real, s_end_real, grid_cube.upper_collision_type_, grid_cube.lower_collision_type_);
    return cube_real;
}

std::vector<std::vector<Cube2D<int>>> StGraph::realCubesPathsToGridCubesPaths(const std::vector<std::vector<Cube2D<double>>>& real_cubes_paths) {
    std::vector<std::vector<Cube2D<int>>> cubes_paths;
    for (int i = 0; i < real_cubes_paths.size(); i++) {
        cubes_paths.emplace_back(realCubesToGridCubes(real_cubes_paths[i]));
    }
    return cubes_paths;
}

std::vector<Cube2D<int>> StGraph::realCubesToGridCubes(const std::vector<Cube2D<double>>& real_cubes) {
    std::vector<Cube2D<int>> cubes;
    for (int i = 0; i < real_cubes.size(); i++) {
        cubes.emplace_back(realCubeToGridCube(real_cubes[i]));
    }
    return cubes;
}

Cube2D<int> StGraph::realCubeToGridCube(const Cube2D<double>& real_cube) {
    int t_start_grid = real_cube.t_start_ / param_.t_resolution;
    int t_end_grid = real_cube.t_end_ / param_.t_resolution;
    int s_start_grid = real_cube.s_start_ / param_.s_resolution;
    int s_end_grid = real_cube.s_end_ / param_.s_resolution;
    Cube2D<int> cube_grid = Cube2D<int>(t_start_grid, t_end_grid, s_start_grid, s_end_grid, real_cube.upper_collision_type_, real_cube.lower_collision_type_);
    return cube_grid;
}



std::vector<std::tuple<std::vector<Eigen::Vector2d>, double, double>> StGraph::loadObstacle(const DecisionMaking::Obstacle& obstacle) {
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

    std::vector<std::tuple<std::vector<Eigen::Vector2d>, double, double>> real_vertex_and_interaction_theta;

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

        // Get the interaction point's theta (for transformation coordination)
        double ego_vehicle_interaction_theta = ego_occupy_area_.getOccupationArea()[ego_vehicle_start_collision_index].rotation_;
        double obstacle_interaction_theta = cur_obs_occupy_area.getOccupationArea()[cur_obs_start_collision_index].rotation_;

        // Calculate collision information
        double t_start = 0.0;
        double t_end = 0.0;
        double s_start = 0.0;
        double s_end = 0.0;
        if (obstacle.getObstacleVelocity() >= 0.5) {
            t_start = std::max((cur_obs_start_collision_index * OBSTACLE_MARGIN) / obstacle.getObstacleVelocity(), 0.0);
            t_end = std::min((cur_obs_end_collision_index * OBSTACLE_MARGIN) / obstacle.getObstacleVelocity(), param_.t_max);
            s_start = ego_vehicle_start_collision_index * LANE_GAP_DISTANCE;
            s_end = ego_vehicle_end_collision_index * LANE_GAP_DISTANCE;
        } else {
            t_start = 0.0;
            t_end = std::min((cur_obs_end_collision_index * OBSTACLE_MARGIN) / obstacle.getObstacleVelocity(), param_.t_max);
            s_start = ego_vehicle_start_collision_index * LANE_GAP_DISTANCE;
            s_end = s_start;
        }


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
        real_vertex_and_interaction_theta.emplace_back(std::make_tuple(real_vertice, ego_vehicle_interaction_theta, obstacle_interaction_theta));

        // // DEBUG
        // for (int i = 0; i < 4; i++) {
        //     std::cout << "Vertice number: " << i << std::endl;
        //     std::cout << real_vertice[i] << std::endl;
        // }
        // // END DEBUG

        // Convert
        std::vector<Eigen::Vector2i> grid_positions = realValuesToGridPoss(real_vertice);

        // // DEBUG
        // for (int i = 0; i < 4; i++) {
        //     std::cout << grid_positions[i] << std::endl;
        // }
        // // END DEBUG

        // Picture the banned area to grid map
        grid_map_2D_->fillObstacleBannedArea(grid_positions);

    }

    return real_vertex_and_interaction_theta;
}

void StGraph::loadObstacles(const std::vector<DecisionMaking::Obstacle>& obstacles) {
    // Record all the vertex to match the corresponding uncertainty
    for (auto const& obs : obstacles) {
        loadObstacle(obs);
    }

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
        // if (sampled_t <= param_.acc_min_initial_valid_maximum) {
        //     sampled_lower_ss[i] = 0.0;
        // } else {
        //     sampled_lower_ss[i] = std::max(0.0, sampled_cur_lower_s);
        // }
        // if (i > 0) {
        //     sampled_lower_ss[i] = std::max(sampled_lower_ss[i], sampled_lower_ss[i - 1]);
        // }
        sampled_lower_ss[0] = 0.0;
        

        // Calculate the upper boundary point
        double required_acc = 0.0;
        if (sampled_t <= param_.acc_max_initial_valid_maximum) {
            required_acc = param_.acc_max_initial;
        } else {
            required_acc = param_.acc_max;
        }
        double cur_pred_velocity = start_velocity_ + sampled_t * required_acc;
        double sampled_cur_upper_s = 0.0;
        if (cur_pred_velocity >= param_.velocity_max && i != 0) {
            sampled_cur_upper_s = sampled_upper_ss[i - 1] + (sampled_ts[i] - sampled_ts[i - 1]) * param_.velocity_max;
        } else {
            sampled_cur_upper_s = start_velocity_ * sampled_t + 0.5 * required_acc * pow(sampled_t, 2);
        }
        sampled_upper_ss[i] = std::min(param_.s_max, sampled_cur_upper_s);

        
    }

    // // DEBUG
    // std::cout << "*********************************" << std::endl;
    // std::cout << "start velocity: " << start_velocity_ << std::endl;
    // std::cout << "max speed: " << param_.velocity_max << std::endl;
    // for (int i = 0; i < sampled_ts.size(); i++) {
    //     std::cout << "t: " << sampled_ts[i] << ", lower: " << sampled_lower_ss[i] << ", upper: " << sampled_upper_ss[i] << std::endl;
    // }
    // // END DEBUG

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
    upper_boundary_values.push_back({param_.t_max, param_.s_max});
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

    // DEBUG
    std::cout << "+++++++++++++++++++++++++++++++++ Initial cubes information +++++++++++++++++++++++++++++++++" << std::endl;
    // END DEBUG

    if (connected_cubes_.size() > 0) {
        *output_cubes = connected_cubes_;

        // // DEBUG
        // for (int i = 0; i < connected_cubes_.size(); i++) {
        //     std::cout << "path " << i << std::endl;
        //     for (int j = 0; j < connected_cubes_[i].size(); j++) {
        //         std::cout << "cube " << j << std::endl;
        //         connected_cubes_[i][j].print();
        //     }
        // }
        // visualization(calculated_grid_cubes_columns_, "Initial cubes paths");
        // // END DEBUG

        for (int i = 0; i < connected_cubes_.size(); i++) {
            printf("[StGraph] Path index: %d.\n", i);
            for (int j = 0; j < connected_cubes_[i].size(); j++) {
                printf("[StGraph] Cube index: %d, with information: ", j);
                connected_cubes_[i][j].print();
            }
        }

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

    // // DEBUG
    // std::cout << "layer index: " << layer_index << std::endl;
    // // END DEBUG

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

void StGraph::visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths, const std::string& name) {
    grid_map_2D_->visualization(cube_paths, name);
}


void StGraph::visualization(const std::string& name) {
    grid_map_2D_->visualization(name);
}

void StGraph::print() {
    grid_map_2D_->print();
}

UncertaintyOccupiedArea::UncertaintyOccupiedArea() = default;

UncertaintyOccupiedArea::UncertaintyOccupiedArea(const std::vector<Eigen::Vector2d>& vertex, const Gaussian2D& gaussian_dis) {
    vertex_ = Parallelogram(vertex);
    gaussian_dis_ = gaussian_dis;
}

UncertaintyOccupiedArea::~UncertaintyOccupiedArea() = default;

Gaussian2D UncertaintyOccupiedArea::toPointGaussianDis(Eigen::Vector2d& vertice) const {
    Gaussian2D gaussian_dis;
    gaussian_dis.ave_values_ = vertice;
    gaussian_dis.covariance_ = gaussian_dis_.covariance_;
    return gaussian_dis;
}

bool UncertaintyStGraph::generateInitialCubePath(const std::vector<DecisionMaking::Obstacle>& obstacles, std::vector<std::vector<Cube2D<double>>>* cube_paths, std::vector<std::pair<double, double>>* s_range) {
    // ~Stage I: add obstacles
    loadUncertaintyObstacles(obstacles);

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

void UncertaintyStGraph::loadUncertaintyObstacle(const DecisionMaking::Obstacle& uncertainty_obs) {
    // Get the specific occupied area
    std::vector<std::tuple<std::vector<Eigen::Vector2d>, double, double>> obs_traj_vertex_and_interaction_theta = StGraph::loadObstacle(uncertainty_obs);

    // Transform uncertainty gaussian distribution
    // Noth that in this context, we only calculate and transform the covariance sicne the average values are represented using the vertex

    // Record
    for (const auto& cur_traj_obs_vertex_and_interaction_theta : obs_traj_vertex_and_interaction_theta) {
        // Parse data
        std::vector<Eigen::Vector2d> cur_traj_obs_vertex = std::get<0>(cur_traj_obs_vertex_and_interaction_theta);
        double ego_vehicle_interaction_theta = std::get<1>(cur_traj_obs_vertex_and_interaction_theta);
        double obstacle_interaction_theta = std::get<2>(cur_traj_obs_vertex_and_interaction_theta);

        // Get corresponding t
        double corresponding_t = (cur_traj_obs_vertex[0](0) + cur_traj_obs_vertex[2](0)) / 2.0;

        // World variance is given by sensors
        // TODO: add a logic to get the initial logic for surrounding vehicles
        Eigen::Matrix<double, 2, 2> world_covariance;
        world_covariance << pow(corresponding_t, 2.0) * uncertainty_param_.obstacle_related_variance_coeff, 0.0, 
                            0.0, pow(corresponding_t, 2.0) * uncertainty_param_.obstacle_related_variance_coeff;
        Gaussian2D obs_world_gaussian_dis = Gaussian2D(world_covariance);

        // // DEBUG
        // std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        // std::cout << "World covariance: " << std::endl;
        // std::cout << world_covariance << std::endl;
        // std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        // // END DEBUG

        // Transform world to fake frenet
        // Get the rotation matrix
        Eigen::Matrix2d rotation_matrix = CoordinateUtils::getRotationMatrix(ego_vehicle_interaction_theta);
        Gaussian2D obs_frenet_gaussian_dis = GaussianUtils::transformGaussianDis(obs_world_gaussian_dis, rotation_matrix);

        // // DEBUG
        // std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        // std::cout << "Rotation matrix: " << std::endl;
        // std::cout << rotation_matrix << std::endl;
        // std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        // // END DBEUG

        // // DEBUG
        // std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        // std::cout << "Fake frenet covariance: " << std::endl;
        // std::cout << obs_frenet_gaussian_dis.covariance_ << std::endl;
        // std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        // // END DEBUG

        // Transform fake frenet to st
        Eigen::Matrix2d scale_matrix;
        double diff_angle = obstacle_interaction_theta - ego_vehicle_interaction_theta;
        if (fabs(diff_angle) < uncertainty_param_.front_vehicle_max_theta_diff) {
            scale_matrix = CoordinateUtils::getScaleMatrix(0.001, 1.0);
        } else {
            scale_matrix = CoordinateUtils::getScaleMatrix(1.0 / (uncertainty_obs.velocity_ * sin(diff_angle)), 1.0);
        }

        Gaussian2D obs_st_gaussian_dis = GaussianUtils::transformGaussianDis(obs_frenet_gaussian_dis, scale_matrix);

        // // DEBUG
        // std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        // std::cout << "Scale matrix: " << std::endl;
        // std::cout << scale_matrix << std::endl;
        // std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        // // END DBEUG

        // // DEBUG
        // std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        // std::cout << "st covariance: " << std::endl;
        // std::cout << obs_st_gaussian_dis.covariance_ << std::endl;
        // std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
        // // END DEBUG

        UncertaintyOccupiedArea uncer_occ_area = UncertaintyOccupiedArea(cur_traj_obs_vertex, obs_st_gaussian_dis);
        uncertainty_occupied_areas_.emplace_back(uncer_occ_area);
    }

}

void UncertaintyStGraph::loadUncertaintyObstacles(const std::vector<DecisionMaking::Obstacle>& uncertainty_obstacles) {
    // Traverse obstacles
    for (const auto& uncertainty_obs : uncertainty_obstacles) {
        loadUncertaintyObstacle(uncertainty_obs);
    }
}

bool UncertaintyStGraph::enhanceSafety(const std::vector<std::vector<Cube2D<double>>>& initial_cube_paths, std::vector<std::vector<Cube2D<double>>>* enhanced_cube_paths) {

    assert(!initial_cube_paths.empty());

    // Initial results
    int m = initial_cube_paths.size();
    int n = initial_cube_paths[0].size();
    std::vector<std::vector<UncertaintyCube2D<double>>> enhanced_safety_uncertainty_cube_paths(m);
    std::vector<std::vector<Cube2D<double>>> executed_cube_paths(m, std::vector<Cube2D<double>>(n, Cube2D<double>()));

    // Add the uncertainty information to cube paths
    for (int i = 0; i < m; i++) {
        std::vector<UncertaintyCube2D<double>> uncertainty_cubes_path = transformCubesPathToUncertaintyCubesPath(initial_cube_paths[i]);
        enhanced_safety_uncertainty_cube_paths[i] = uncertainty_cubes_path;
    }

    // Execute safety enhancement for each cube
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            if (j == 0) {
                executed_cube_paths[i][j] = enhanced_safety_uncertainty_cube_paths[i][j].initial_cube_;                
            } else {
                limitUncertaintyCube(&enhanced_safety_uncertainty_cube_paths[i][j]);
                executed_cube_paths[i][j] = enhanced_safety_uncertainty_cube_paths[i][j].enhanced_cube_;
            }

        }
    }


    // DEBUG
    // std::cout << "+++++++++++++++++++++++++++++++++ Enhanced safety cubes information +++++++++++++++++++++++++++++++++" << std::endl;
    // for (int i = 0; i < executed_cube_paths.size(); i++) {
    //     std::cout << "path " << i << std::endl;
    //     for (int j = 0; j < executed_cube_paths[i].size(); j++) {
    //         std::cout << "cube " << j << std::endl;
    //         executed_cube_paths[i][j].print();
    //     }
    // }
    // END DEBUG

    // // DEBUG
    // // Visualization
    // std::vector<std::vector<Cube2D<int>>> grid_cubes_paths = realCubesPathsToGridCubesPaths(executed_cube_paths);
    // visualization(grid_cubes_paths, "Enhanced cubes paths");
    // // END DEBUG

    std::cout << "+++++++++++++++++++++++++++++++++ Enhanced safety cubes information +++++++++++++++++++++++++++++++++" << std::endl;
    for (int i = 0; i < executed_cube_paths.size(); i++) {
        printf("[UncertaintyStGraph] Path index: %d.\n", i);
        for (int j = 0; j < executed_cube_paths[i].size(); j++) {
            printf("[UncertaintyStGraph] Cube index: %d, with information: ", j);
            executed_cube_paths[i][j].print();
        }
    }

    // Check continuity
    std::vector<std::vector<Cube2D<double>>> checked_connected_cube_paths;
    bool connected_paths_existence = checkCubesPathsContinuity(executed_cube_paths, &checked_connected_cube_paths);
    if (!connected_paths_existence) {
        return false;
    }

    *enhanced_cube_paths = checked_connected_cube_paths;

    return true;
}

std::vector<UncertaintyCube2D<double>> UncertaintyStGraph::transformCubesPathToUncertaintyCubesPath(const std::vector<Cube2D<double>>& cubes) {
    // Initialize
    int m = cubes.size();
    std::vector<UncertaintyCube2D<double>> uncertainty_cubes_path(m, UncertaintyCube2D<double>());

    // Calculate the accumulated gaussian distribution
    // Note that the covariance of the gaussian distribution is accumulated with the increasing of time stamp
    // TODO: refine this parameter
    // const double variance_coeff = 0.15;

    for (int i = 0; i < m; i++) {
        Cube2D<double> cur_cube = cubes[i];

        // Calculate the upper and lower bounds' gaussian distribution
        Eigen::Matrix<double, 1, 1> upper_bound_ave_matrix{cur_cube.s_end_};
        Eigen::Matrix<double, 1, 1> lower_bound_ave_matrix{cur_cube.s_start_};
        Eigen::Matrix<double, 1, 1> upper_bound_variance_matrix{uncertainty_param_.ego_vehicle_related_variance_coeff * pow(cur_cube.t_end_, 2.0)};
        Eigen::Matrix<double, 1, 1> lower_bound_variance_matrix{uncertainty_param_.ego_vehicle_related_variance_coeff * pow(cur_cube.t_end_, 2.0)};
        Gaussian1D upper_gaussian_dis = Gaussian1D(upper_bound_ave_matrix, upper_bound_variance_matrix);
        Gaussian1D lower_gaussian_dis = Gaussian1D(lower_bound_ave_matrix, lower_bound_variance_matrix);

        // Supply data
        UncertaintyCube2D<double> uncertainty_cube = UncertaintyCube2D<double>(cur_cube, upper_gaussian_dis, lower_gaussian_dis);

        uncertainty_cubes_path[i] = uncertainty_cube;
    }

    return uncertainty_cubes_path;
    
}

bool UncertaintyStGraph::checkCubesPathsContinuity(const std::vector<std::vector<Cube2D<double>>>& input_cubes_paths, std::vector<std::vector<Cube2D<double>>>* output_cubes_paths) {
    std::vector<std::vector<Cube2D<double>>> res;
    for (int i = 0; i < input_cubes_paths.size(); i++) {
        if (checkSingleCubesPathContinuity(input_cubes_paths[i])) {
            res.emplace_back(input_cubes_paths[i]);
        }
    }
    *output_cubes_paths = res;
    return !res.empty();
}

bool UncertaintyStGraph::checkSingleCubesPathContinuity(const std::vector<Cube2D<double>>& cubes_path) {
    for (int i = 0; i < cubes_path.size() - 1; i++) {
        if (!isCubesConnected(cubes_path[i], cubes_path[i + 1])) {
            return false;
        }
    }
    return true;
}

void UncertaintyStGraph::limitUncertaintyCube(UncertaintyCube2D<double>* uncertainty_cube) {
    double t_start = uncertainty_cube->initial_cube_.t_start_;
    double t_end = uncertainty_cube->initial_cube_.t_end_;
    
    // Calculate the limited bounds
    double limited_upper_bound;
    double limited_lower_bound;
    limitSingleBound(uncertainty_cube->upper_gaussian_dis_, t_start, t_end, BoundType::UPPER, uncertainty_cube->initial_cube_.upper_collision_type_, &limited_upper_bound);
    limitSingleBound(uncertainty_cube->lower_gaussian_dis_, t_start, t_end, BoundType::LOWER, uncertainty_cube->initial_cube_.lower_collision_type_, &limited_lower_bound);

    // Update uncertainty cube
    uncertainty_cube->upper_gaussian_dis_.ave_values_(0, 0) = limited_upper_bound;
    uncertainty_cube->lower_gaussian_dis_.ave_values_(0, 0) = limited_lower_bound;
    uncertainty_cube->enhanced_cube_.s_end_ = limited_upper_bound;
    uncertainty_cube->enhanced_cube_.s_start_ = limited_lower_bound;
        
}

void UncertaintyStGraph::limitSingleBound(const Gaussian1D& line_gaussian_dis, const double& t_start, const double& t_end, const BoundType& bound_type, const CollisionType& collision_type, double* limited_bound) {    
    // Initialize buffer value
    double buffer_value = 0.0;

    // Special situations
    if (collision_type == CollisionType::ACCELERATION_BOUNDARY) {
        *limited_bound = line_gaussian_dis.ave_values_(0, 0);
        return;
    }

    // Traverse all the uncertainty occupied areas
    for (const auto& cur_uncertainty_occ_area : uncertainty_occupied_areas_) {

        // // DEBUG
        // std::cout << "#################################################" << std::endl;
        // std::cout << "covariance: " << std::endl;
        // std::cout << cur_uncertainty_occ_area.gaussian_dis_.covariance_ << std::endl;
        // std::cout << "#################################################" << std::endl;
        // // END DEBUG

        // Calculate relative positions 
        double cur_nearest_t_in_line = 0.0;
        Eigen::Vector2d cur_nearest_vertice_in_polynomial;
        SRelativePositionType rel_pos_type = SRelativePositionType::UNKNOWN;
        TRelativePositionType t_rel_pos_type = TRelativePositionType::UNKNOWN;
        bool cur_state = ShapeUtils::judgeLineWithPolynomial(line_gaussian_dis.ave_values_(0, 0), t_start, t_end, cur_uncertainty_occ_area.vertex_, param_.s_resolution, &cur_nearest_t_in_line, cur_nearest_vertice_in_polynomial, &rel_pos_type, &t_rel_pos_type);
        
        if (!cur_state) {
            printf("[UncertaintyStGraph] Error collision!!!\n");
            assert(false);
        }

        // if (t_rel_pos_type != TRelativePositionType::OVERLAPPED) {
        //     continue;
        // }

        // Transform the gaussian distribution from an uncertainty area to a specific point
        Gaussian2D nearest_point_gaussian_dis = cur_uncertainty_occ_area.toPointGaussianDis(cur_nearest_vertice_in_polynomial);

        // Calulate location probability given a dimension and its range
        double dis_prob = GaussianUtils::calculateDistributionProbability(nearest_point_gaussian_dis, DimensionType::T, t_start, t_end);
        double current_required_confidence = 1.0 - (1.0 - uncertainty_param_.required_confidence) / dis_prob;

        // Get the two possible distribution of the 2D distribution
        Gaussian1D start_gaussian_dis;
        Gaussian1D end_gaussian_dis;
        GaussianUtils::transformGaussian2DTo1D(nearest_point_gaussian_dis, DimensionType::S, t_start, t_end, &start_gaussian_dis, &end_gaussian_dis);

        // Calculate difference gaussian distribution due to the type of bound
        Gaussian1D start_diff_gaussian_dis;
        Gaussian1D end_diff_gaussian_dis;
        if (bound_type == BoundType::UPPER) {
            start_diff_gaussian_dis = start_gaussian_dis - line_gaussian_dis;
            end_diff_gaussian_dis = end_gaussian_dis - line_gaussian_dis;
        } else if (bound_type == BoundType::LOWER) {
            start_diff_gaussian_dis = line_gaussian_dis - start_gaussian_dis;
            end_diff_gaussian_dis = line_gaussian_dis - end_gaussian_dis;
        } else {
            printf("[UncertaintyStGraph] Unknown bound type!!!\n");
            assert(false);
        }

        if (start_diff_gaussian_dis.ave_values_(0, 0) < 0.0 || end_diff_gaussian_dis.ave_values_(0, 0) < 0.0) {
            // TODO: check this logic, it is likely that there is a bug
            printf("[UncertaintyStGraph] Emergence situation!!!\n");
        }
        double start_diff_gaussian_res_buffer = LookUpTable::GaussianAverageValue::calculate(start_diff_gaussian_dis.covariance_(0, 0), current_required_confidence);
        double end_diff_gaussian_res_buffer = LookUpTable::GaussianAverageValue::calculate(end_diff_gaussian_dis.covariance_(0, 0), current_required_confidence);

        if ((rel_pos_type == SRelativePositionType::ABOVE && bound_type == BoundType::UPPER) || (rel_pos_type == SRelativePositionType::BELOW && bound_type == BoundType::LOWER)) {
            buffer_value = std::max({buffer_value, start_diff_gaussian_res_buffer, end_diff_gaussian_res_buffer});
        }

        // // DEBUG
        // std::cout << "++++++++++++++++++++++++++++++++++++" << std::endl;
        // std::cout << "t start: " << t_start << ", t end: " << t_end << std::endl;
        // std::cout << "Bound type: " << bound_type << std::endl;
        // std::cout << "Neareat point gaussian dis average value: " << nearest_point_gaussian_dis.ave_values_ << std::endl;
        // std::cout << "Nearest point gaussian dis variance: " << nearest_point_gaussian_dis.covariance_ << std::endl;
        // std::cout << "Distribution probability: " << dis_prob << std::endl;
        // std::cout << "Required confidence: " << current_required_confidence << std::endl;
        // std::cout << "Start diff gaussian buffer: " << start_diff_gaussian_res_buffer << std::endl;
        // std::cout << "End diff gaussian buffer: " << end_diff_gaussian_res_buffer << std::endl;
        // // END DEBUG


    }



    // Calculate the limited cube bound's position
    if (bound_type == BoundType::UPPER) {
        *limited_bound = line_gaussian_dis.ave_values_(0, 0) - buffer_value;
    } else if (bound_type == BoundType::LOWER) {
        *limited_bound = line_gaussian_dis.ave_values_(0, 0) + buffer_value;
    } else {
        printf("[UncertaintyStGraph] unknown bound type!!!\n");
        assert(false);
    }
    

}










} // End of namespace VelocityPlanning