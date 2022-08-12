/*
 * @Author: fujiawei0724
 * @Date: 2022-08-04 14:14:08
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-08-11 19:12:55
 * @Description: velocity optimization.
 */

#include <unordered_map>
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"
#include "Status.h"
#include "StGraph.hpp"

namespace VelocityPlanning {

// Optimization interface, based on OOQP
class OoqpOptimizationInterface {
 public:
    OoqpOptimizationInterface();
    ~OoqpOptimizationInterface();

    /**
     * @brief load data
     * @param ref_stamps time stamps of the point in in the intersection of two cubes
     * @param start_constraints start points' constraints
     * @param end_constraints end points' constraints
     * @param unequal_constraints position limit of each point
     * @param equal_constraints ensure the continuity of the connections between each two cubes
     */    
    void load(const std::vector<double>& ref_stamps, const std::array<double, 3>& start_constraints, const double& end_s_constraint, std::array<std::vector<double>, 2>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints, std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>>& polymonial_unequal_constraints);

    /**
     * @brief Run optimization
     * @param {*}
     * @return {*}
     */    
    bool runOnce(std::vector<double>* optimized_s, double* objective_value);

    /**
     * @brief Optimize in single dimension
     * @param {*}
     */
    void optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const double& end_s_constraint, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries);

    /**
     * @brief calculate objective function
     * @param {*}
     * @return {*}
     */    
    void calculateQcMatrix(Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, Eigen::VectorXd& c);

    void calculateCdfMatrix(Eigen::SparseMatrix<double, Eigen::RowMajor>& C, Eigen::VectorXd& d, Eigen::VectorXd& f);

    /**
     * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
     * @param {*}
     */
    void calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const double& end_s_constraint, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b);

    // DEBUG
    // Test the situation without end point constraints
    void calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b);
    // END DEBUG

    /**
     * @brief Calculate boundaries for intermediate points
     * @param {*}
     */    
    void calculateBoundariesForIntermediatePoints(const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX);

    /**
     * @brief solve quartic programming
     * @param {*}
     * @return {*}
     */
    bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                                          const Eigen::VectorXd& c,
                                          const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                                          const Eigen::VectorXd& b,
                                          const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                                          const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                                          Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, 
                                          Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, 
                                          Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX, 
                                          std::vector<double>* optimized_values, 
                                          double* objective_value);

    std::vector<double> ref_stamps_;
    std::array<double, 3> start_constraints_;
    double end_s_constraint_;
    std::array<std::vector<double>, 2> unequal_constraints_;
    std::vector<std::vector<double>> equal_constraints_;
    std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>> polymonial_unequal_constraints_;



    std::vector<double> optimized_data_;
    bool optimization_res_;
    double objective_value_;
};

class VelocityOptimizer {
 public:
    VelocityOptimizer();
    ~VelocityOptimizer();

    bool runOnce(const std::vector<std::vector<Cube2D<double>>>& cube_paths, const std::array<double, 3>& start_state, std::vector<std::pair<double, double>>& last_s_range, const double& max_velocity, const double& min_velocity, const double& max_acceleration, const double& min_acceleration, std::vector<double>* s, std::vector<double>* t);
    
    void runSingleCubesPath(const std::vector<Cube2D<double>>& cube_path, const std::array<double, 3>& start_state, const double& end_s, const double& max_velocity, const double& min_velocity, const double& max_acceleration, const double& min_acceleration, int index);
    
    std::array<std::vector<double>, 2> generateUnequalConstraints(const std::vector<Cube2D<double>>& cube_path);

    std::vector<std::vector<double>> generateEqualConstraints(const std::vector<Cube2D<double>>& cube_path);

    std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>> generatePolynimalUnequalConstraints(const std::vector<Cube2D<double>>& cube_path, const double& max_velocity, const double& min_velocity, const double& max_acceleration, const double& min_acceleration);

    

    OoqpOptimizationInterface* ooqp_itf_{nullptr};

    // Store the information for different cube paths
    std::vector<std::vector<double>> ss_;
    std::vector<std::vector<double>> tt_;
    std::vector<bool> ress_;
    std::vector<double> values_;
};

// Generate interpolated curves
class BezierPiecewiseCurve {
 public:
    BezierPiecewiseCurve(const std::vector<double>& s, std::vector<double>& ref_stamps);
    ~BezierPiecewiseCurve();

    /**
     * @brief Calculate curve
     * @param {*}
     * @return {*}
     */    
    std::pair<std::vector<double>, std::vector<double>> generateTraj(double sample_gap = 0.01);

    /**
     * @brief Calculate single point 
     * @param {*}
     * @return {*}
     */    
    Eigen::Vector3d generatePoint(int segment_index, double remain, double time_stamp);

    int segment_num_;
    std::vector<double> ref_stamps_;

    std::vector<std::vector<double>> s_coefficients_;
};

class VelocityPlanner {
 public:
    VelocityPlanner(DecisionMaking::StandardState* current_state);
    ~VelocityPlanner();

    bool runOnce(const std::vector<DecisionMaking::Obstacle>& obstacles);

    StGraph* st_graph_{nullptr};
    VelocityOptimizer* velocity_optimizer_{nullptr};
    BezierPiecewiseCurve* bezier_curve_traj_generator_{nullptr};
    DecisionMaking::StandardState* planning_state_{nullptr};

    std::array<double, 3> start_state_;

};

} // End of namespace VelocityPlanning
