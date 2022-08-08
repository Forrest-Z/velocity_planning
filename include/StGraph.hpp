/*
 * @Author: fujiawei0724
 * @Date: 2022-08-03 15:54:48
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-08-08 19:58:23
 * @Description: s-t graph
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <path_planning_msgs/BoundedCurve.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Obstacle.hpp"
#include "Point.hpp"
#include "Path.hpp"
#include "OccupyArea.hpp"


namespace VelocityPlanning {

template<typename T>
class  Cube2D {
 public:
    Cube2D() = default;
    Cube2D(const T& t_start, const T& t_end, const T& s_start, const T& s_end) {
        t_start_ = t_start;
        t_end_ = t_end;
        s_start_ = s_start;
        s_end_ = s_end;
    }
    ~Cube2D() = default;

    void print() {
        std::cout << "t start: " << t_start_ << ", t end: " << t_end_ << ", s start: " << s_start_ << ", s end: " << s_end_ << std::endl; 
    }

    T t_start_;
    T t_end_;
    T s_start_;
    T s_end_;
};

class GridMap2D {
 public:
    
    enum ValType {
        OCCUPIED = 0,
        HALF_OCCUPIED = 123,
        FREE = 255,
        CUBE_BOUNDARY = 66,
        UNKNOWN = 999,
    };

    GridMap2D(int x_max, int y_max);
    ~GridMap2D();

    void fillAccBannedArea(const std::vector<Eigen::Vector2i>& vertice);

    void fillObstacleBannedArea(const std::vector<Eigen::Vector2i>& vertice);

    bool expandSingleColumn(const int& grid_t_start, const int& grid_t_end, const int& grid_s_start, std::vector<Cube2D<int>>* cubes, int* real_s_start);

    std::vector<cv::Point> eigenToCvPoint(const std::vector<Eigen::Vector2i>& points);

    ValType getOccupiedState(const int& grid_t_start, const int& grid_t_end, const int& s);

    void addCubeVisualization(const Cube2D<int>& grid_cube);

    void addCubesVisualization(const std::vector<Cube2D<int>>& grid_cubes);

    // DEBUG
    void visualization();

    void visualization(const std::vector<Cube2D<int>>& cubes);

    void visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths);
    // END DEBUG

    cv::Mat mat_;
};

class StGraph {
 public:


    struct Param {
        double occupied_width = 2.0;
        double occupied_length = 5.0;
        double s_max = 50.0;
        double t_max = 10.0;
        double s_resolution = 0.1;
        double t_resolution = 0.1;
        double acc_max = 1.5;
        double acc_min = -2.0;
        int acc_limit_t_sampled_points_num = 10;
        int lateral_segement_number = 10;
        double velocity_max = 6.0;
    };

    /**
     * @description: Initialization of graph
     * @param {Curve&} path 
     * @param {Param&} param
     * @param {double&} current_velocity
     * TODO: the hypothesis is that the current velocity of the vehicle is with the same orientation of the first point of the path 
     */    
    StGraph(const PathPlanningUtilities::Curve& path, const Param& param, const double& current_velocity);
   
    ~StGraph();

    Eigen::Vector2i realValueToGridPos(const Eigen::Vector2d& real_position);

    std::vector<Eigen::Vector2i> realValuesToGridPoss(const std::vector<Eigen::Vector2d>& real_positions);

    Eigen::Vector2d gridPosToRealValue(const Eigen::Vector2i& grid_position);

    std::vector<Eigen::Vector2d> gridPossToRealValues(const std::vector<Eigen::Vector2i>& grid_positions);

    void loadObstacle(const DecisionMaking::Obstacle& obstacle);

    void loadObstacles(const std::vector<DecisionMaking::Obstacle>& obstacles);

    void loadAccelerationLimitation();

    bool generateCubes(std::vector<std::vector<Cube2D<double>>>* cubes, std::vector<std::pair<double, double>>* last_s_range);

    std::vector<Cube2D<double>> gridCubesToRealCubes(const std::vector<Cube2D<int>>& grid_cubes);

    Cube2D<double> gridCubeToRealCube(const Cube2D<int>& grid_cube);

    bool connectCubes(const std::vector<std::vector<Cube2D<double>>>& input_cubes, std::vector<std::vector<Cube2D<double>>>* output_cubes);

    bool isCubesConnected(const Cube2D<double>& cube_1, const Cube2D<double>& cube_2);

    void dfsConnectCubes(const std::vector<std::vector<Cube2D<double>>>& input_cubes, int layer_index, std::vector<Cube2D<double>>& cube_path);

    bool runOnce(const std::vector<DecisionMaking::Obstacle>& obstacles, std::vector<std::vector<Cube2D<double>>>* cube_paths);

    void visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths);

    void visualization();


    GridMap2D* grid_map_2D_{nullptr};
    PathPlanningUtilities::Curve path_;
    DecisionMaking::RSS::OccupationArea ego_occupy_area_;
    double start_velocity_{0.0};
    Param param_;

    std::vector<std::vector<Cube2D<double>>> connected_cubes_;
    std::vector<std::vector<Cube2D<int>>> calculated_grid_cubes_columns_;

};





} // End of namespace VelocityPlanning