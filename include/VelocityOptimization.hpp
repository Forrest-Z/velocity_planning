/*
 * @Author: fujiawei0724
 * @Date: 2022-08-04 14:14:08
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-08-07 09:05:37
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
    void load(const std::vector<double>& ref_stamps, const std::array<double, 3>& start_constraints, const std::array<double, 3>& end_constraints, std::array<std::vector<double>, 2>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints);

    /**
     * @brief Run optimization
     * @param {*}
     * @return {*}
     */    
    bool runOnce(std::vector<double>* optimized_s);

    /**
     * @brief Optimize in single dimension
     * @param {*}
     */
    void optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries);

    /**
     * @brief calculate objective function
     * @param {*}
     * @return {*}
     */    
    void calculateQcMatrix(Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, Eigen::VectorXd& c);

    /**
     * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
     * @param {*}
     */
    void calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b);

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
                    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, 
                    Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX, std::vector<double>* optimized_values);

    std::vector<double> ref_stamps_;
    std::array<double, 3> start_constraints_;
    std::array<double, 3> end_constraints_;
    std::array<std::vector<double>, 2> unequal_constraints_;
    std::vector<std::vector<double>> equal_constraints_;



    std::vector<double> optimized_data_;
    bool optimization_res_;
};

class VelocityOptimizer {
 public:
    VelocityOptimizer();
    ~VelocityOptimizer();

    bool runOnce(const std::vector<std::vector<Cube2D<double>>>& cube_paths, const std::array<double, 3>& start_state, std::vector<double>* s, std::vector<double>* t);

    void runSingleCubesPath(const std::vector<Cube2D<double>>& cube_path, const std::array<double, 3>& start_state, int index);

    std::array<std::vector<double>, 2> generateUnequalConstraints(const std::vector<Cube2D<double>>& cube_path);

    std::vector<std::vector<double>> generateEqualConstraints(const std::vector<Cube2D<double>>& cube_path);

    OoqpOptimizationInterface* ooqp_itf_{nullptr};

    // Store the information for different cube paths
    std::vector<std::vector<double>> ss_;
    std::vector<std::vector<double>> tt_;
    std::vector<bool> ress_;
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
    std::vector<Eigen::Vector2d> generateTraj(double sample_gap = 0.01);

    /**
     * @brief Calculate single point 
     * @param {*}
     * @return {*}
     */    
    Eigen::Vector2d generatePoint(int segment_index, double remain, double time_stamp);

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
    std::array<double, 3> start_state_;
    BezierPiecewiseCurve* bezier_curve_traj_generator_{nullptr};
    DecisionMaking::StandardState* planning_state_{nullptr};
};

} // End of namespace VelocityPlanning
