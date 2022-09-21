/*
 * @Author: fujiawei0724
 * @Date: 2022-08-03 15:54:48
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-21 10:05:16
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
#include "GaussianDistribution.hpp"


namespace VelocityPlanning {

template<typename T>
class Cube2D {
 public:

    Cube2D() = default;

    Cube2D(const T& t_start, const T& t_end, const T& s_start, const T& s_end) {
        t_start_ = t_start;
        t_end_ = t_end;
        s_start_ = s_start;
        s_end_ = s_end;
    }

    Cube2D(const T& t_start, const T& t_end, const T& s_start, const T& s_end, const CollisionType& upper_t, const CollisionType& lower_t) {
        t_start_ = t_start;
        t_end_ = t_end;
        s_start_ = s_start;
        s_end_ = s_end;
        upper_collision_type_ = upper_t;
        lower_collision_type_ = lower_t;
    }

    ~Cube2D() = default;

    void print() {
        std::cout << "[Cube2D] "<< "t start: " << t_start_ << ", t end: " << t_end_ << ", s start: " << s_start_ << ", s end: " << s_end_ << std::endl; 
    }

    T t_start_;
    T t_end_;
    T s_start_;
    T s_end_;

    CollisionType upper_collision_type_{CollisionType::UNKNOWN};
    CollisionType lower_collision_type_{CollisionType::UNKNOWN};
};

template<typename T>
class UncertaintyCube2D {
 public:
    
    UncertaintyCube2D() = default;

    UncertaintyCube2D(const Cube2D<T>& cube, const Gaussian1D& upper_gaussian_dis, const Gaussian1D& lower_gaussian_dis) {
        initial_cube_ = cube;
        enhanced_cube_ = cube;
        upper_gaussian_dis_ = upper_gaussian_dis;
        lower_gaussian_dis_ = lower_gaussian_dis;
    }

    ~UncertaintyCube2D() = default;

    Cube2D<T> initial_cube_;
    Cube2D<T> enhanced_cube_;
    Gaussian1D upper_gaussian_dis_;
    Gaussian1D lower_gaussian_dis_; 
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

    void addCubeVisualization(const Cube2D<int>& grid_cube, cv::Mat& copied_mat);

    void addCubesVisualization(const std::vector<Cube2D<int>>& grid_cubes, cv::Mat& copied_mat);

    // DEBUG

    void print();

    void visualization(const std::string& name);

    void visualization(const std::vector<Cube2D<int>>& cubes, const std::string& name);

    void visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths, const std::string& name);
    // END DEBUG

    cv::Mat mat_;
};

class StGraph {
 public:


    struct Param {
        double occupied_width = 2.0;
        double occupied_length = 5.0;
        double s_max = 50.0;
        double t_max = 5.0;
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
     */    
    StGraph(const PathPlanningUtilities::Curve& path, const Param& param, const double& current_velocity);
   
    ~StGraph();

    Eigen::Vector2i realValueToGridPos(const Eigen::Vector2d& real_position);

    std::vector<Eigen::Vector2i> realValuesToGridPoss(const std::vector<Eigen::Vector2d>& real_positions);

    Eigen::Vector2d gridPosToRealValue(const Eigen::Vector2i& grid_position);

    std::vector<Eigen::Vector2d> gridPossToRealValues(const std::vector<Eigen::Vector2i>& grid_positions);

    std::vector<std::tuple<std::vector<Eigen::Vector2d>, double, double>> loadObstacle(const DecisionMaking::Obstacle& obstacle);

    void loadObstacles(const std::vector<DecisionMaking::Obstacle>& obstacles);

    void loadAccelerationLimitation();

    bool generateCubes(std::vector<std::vector<Cube2D<double>>>* cubes, std::vector<std::pair<double, double>>* last_s_range);

    std::vector<Cube2D<double>> gridCubesToRealCubes(const std::vector<Cube2D<int>>& grid_cubes);

    Cube2D<double> gridCubeToRealCube(const Cube2D<int>& grid_cube);

    std::vector<std::vector<Cube2D<int>>> realCubesPathsToGridCubesPaths(const std::vector<std::vector<Cube2D<double>>>& real_cubes_paths);

    std::vector<Cube2D<int>> realCubesToGridCubes(const std::vector<Cube2D<double>>& real_cubes);

    Cube2D<int> realCubeToGridCube(const Cube2D<double>& real_cube);

    bool connectCubes(const std::vector<std::vector<Cube2D<double>>>& input_cubes, std::vector<std::vector<Cube2D<double>>>* output_cubes);

    bool isCubesConnected(const Cube2D<double>& cube_1, const Cube2D<double>& cube_2);

    void dfsConnectCubes(const std::vector<std::vector<Cube2D<double>>>& input_cubes, int layer_index, std::vector<Cube2D<double>>& cube_path);

    bool runOnce(const std::vector<DecisionMaking::Obstacle>& obstacles, std::vector<std::vector<Cube2D<double>>>* cube_paths, std::vector<std::pair<double, double>>* s_range);

    void visualization(const std::vector<std::vector<Cube2D<int>>>& cube_paths, const std::string& name);

    void visualization(const std::string& name);

    void print();


    GridMap2D* grid_map_2D_{nullptr};
    PathPlanningUtilities::Curve path_;
    DecisionMaking::RSS::OccupationArea ego_occupy_area_;
    double start_velocity_{0.0};
    Param param_;

    std::vector<std::vector<Cube2D<double>>> connected_cubes_;
    std::vector<std::vector<Cube2D<int>>> calculated_grid_cubes_columns_;

};

// Uncertainty occupied area only exists given a s-t graph
class UncertaintyOccupiedArea {
 public: 

    UncertaintyOccupiedArea();

    UncertaintyOccupiedArea(const std::vector<Eigen::Vector2d>& vertex, const Gaussian2D& gaussian_dis);

    ~UncertaintyOccupiedArea();

    Gaussian2D toPointGaussianDis(Eigen::Vector2d& vertice) const;

    Parallelogram vertex_;
    // The average value is valid since an area includes many pixel points
    Gaussian2D gaussian_dis_;
};


// Occpuied area is represented with a uncertainty occupied area instead of a set of bool values in the cv::mat
class UncertaintyStGraph : public StGraph {
 public:

    enum BoundType {
        UPPER = 0,
        LOWER = 1,
        UNKNOWN = 2,
    };

    struct UncertaintyParam {
        double obstacle_related_variance_coeff = 0.3;
        double ego_vehicle_related_variance_coeff = 0.15;
        double required_confidence = 0.9;
        double front_vehicle_max_theta_diff = 0.15; 
    };

    using StGraph::StGraph;

    bool generateInitialCubePath(const std::vector<DecisionMaking::Obstacle>& obstacles, std::vector<std::vector<Cube2D<double>>>* cube_paths, std::vector<std::pair<double, double>>* s_range);

    void loadUncertaintyObstacle(const DecisionMaking::Obstacle& uncertainty_obs);

    void loadUncertaintyObstacles(const std::vector<DecisionMaking::Obstacle>& uncertainty_obstacles);

    bool enhanceSafety(const std::vector<std::vector<Cube2D<double>>>& initial_cube_paths, std::vector<std::vector<Cube2D<double>>>* enhanced_cube_paths);

    std::vector<UncertaintyCube2D<double>> transformCubesPathToUncertaintyCubesPath(const std::vector<Cube2D<double>>& cubes);

    bool checkCubesPathsContinuity(const std::vector<std::vector<Cube2D<double>>>& input_cubes_paths, std::vector<std::vector<Cube2D<double>>>* output_cubes_paths);

    bool checkSingleCubesPathContinuity(const std::vector<Cube2D<double>>& cubes_path);

    /**
     * @description: limit the cube's upper and lower bounds due to the confidence and uncertainty 
     * @return is successful
     */    
    void limitUncertaintyCube(UncertaintyCube2D<double>* uncertainty_cube);

    /**
     * @description: limit the single bounds for each cube
     * @return {*}
     */    
    void limitSingleBound(const Gaussian1D& line_gaussian_dis, const double& t_start, const double& t_end, const BoundType& bound_type, const CollisionType& collision_type, double* limited_bound);

    std::vector<UncertaintyOccupiedArea> uncertainty_occupied_areas_;

    UncertaintyParam uncertainty_param_;

}; 







} // End of namespace VelocityPlanning