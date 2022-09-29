/*
 * @Author: fujiawei0724
 * @Date: 2022-08-04 14:14:24
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-29 20:14:38
 * @Description: velocity optimization.
 */

#include "Common.hpp"


namespace VelocityPlanning {

bool OsqpOptimizationInterface::runOnce(const std::vector<double>& ref_times, const std::array<double, 3>& start_state, const std::array<double, 3>& end_state, const std::array<std::vector<double>, 2>& unequal_constraints, const std::vector<std::vector<double>>& equal_constraints, const std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>>& polynomial_unequal_constraints, std::vector<double>* optimized_s, double* objective_value) {
    // ~Stage I: calculate objective function Q and c
    Eigen::SparseMatrix<double, Eigen::RowMajor> Q;
    Eigen::VectorXd c;
    OsqpOptimizationInterface::calculateQcMatrix(ref_times, Q, c);

    // ~Stage II: calculate constraints
    Eigen::SparseMatrix<double, Eigen::RowMajor> constraints;
    Eigen::VectorXd lower_bounds;
    Eigen::VectorXd upper_bounds;
    OsqpOptimizationInterface::calculateConstraintsMatrix(ref_times, start_state, end_state, unequal_constraints, equal_constraints, polynomial_unequal_constraints, constraints, lower_bounds, upper_bounds);

    // ~Stage III: solve 
    OsqpEigen::Solver solver;

    solver.settings()->setVerbosity(false);

    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(Q.rows());
    solver.data()->setNumberOfConstraints(constraints.rows());



    if (!solver.data()->setHessianMatrix(Q)) return false;
    if (!solver.data()->setGradient(c)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(constraints)) return false;
    if (!solver.data()->setLowerBound(lower_bounds)) return false;
    if (!solver.data()->setUpperBound(upper_bounds)) return false;

    // instantiate the solver
    if (!solver.initSolver()) return false;

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if (!solver.solve()) return false;

    QPSolution = solver.getSolution();
    // std::cout << "QPSolution: " << std::endl << QPSolution << std::endl;

    std::vector<double> s(&QPSolution[0], QPSolution.data()+QPSolution.cols()*QPSolution.rows());

    double objective_val = solver.workspace()->info->obj_val;

    // std::cout << "Objective value: " << objective_val << std::endl;



    *objective_value = objective_val;
    *optimized_s = s;

    return true;

}

void OsqpOptimizationInterface::calculateQcMatrix(const std::vector<double>& ref_stamps, Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, Eigen::VectorXd& c) {
    // Initialize Q matrix
    int variables_num = (static_cast<int>(ref_stamps.size()) - 1) * 6;
    Eigen::MatrixXd Q_matrix = Eigen::MatrixXd::Zero(variables_num, variables_num);

    // Calculate D matrix
    for (int i = 0; i < static_cast<int>(ref_stamps.size()) - 1; i++) {
        // Calculate time span
        double time_span = ref_stamps[i + 1] - ref_stamps[i];
        double time_coefficient = pow(time_span, -3);

        // Intergrate to objective function
        int influenced_variable_index = i * 6;
        Q_matrix.block(influenced_variable_index, influenced_variable_index, 6, 6) += BezierCurveHessianMatrix * time_coefficient;
    }

    // // DEBUG
    // std::cout << "Q_matrix: " << Q_matrix << std::endl;
    // // END DEBUG

    // // DEBUG
    // for (int i = 0; i < Q_matrix.rows(); i++) {
    //     for (int j = 0; j < Q_matrix.cols(); j++) {
    //         std::cout << Q_matrix(i, j) << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << "-----------------" << std::endl;
    // // END DEBUG

    Q = Q_matrix.sparseView();
    c.resize(variables_num);
    c.setZero();
}

void OsqpOptimizationInterface::calculateConstraintsMatrix(const std::vector<double>& ref_times, const std::array<double, 3>& start_state, const std::array<double, 3>& end_state, const std::array<std::vector<double>, 2>& unequal_constraints, const std::vector<std::vector<double>>& equal_constraints, const std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>>& polynomial_unequal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& constraints, Eigen::VectorXd& lower_bounds, Eigen::VectorXd& upper_bounds) {
    // ~Stage I: calculate constraints number
    int variables_num = (static_cast<int>(ref_times.size()) - 1) * 6;
    /*
    @brief: @3: start state constraints
            @1: end s constraints
            @unequal_constraints[0].size() - 1: holding the same description with OOQP, the last control point has 
            no constraint. But there is still a placeholder, which will be ignored here
            @equal_constraints: the constraints for the continuity between two adjacent cubes
            @polynomial_unequal_constraints: the constraints for the limitation of velocity and acceleration
    */
    int constraints_num = 3 + 1 + (unequal_constraints[0].size() - 1) + equal_constraints.size() + std::get<1>(polynomial_unequal_constraints).size();

    Eigen::MatrixXd constraints_matrix = Eigen::MatrixXd::Zero(constraints_num, variables_num);
    Eigen::VectorXd lower_bounds_vec = Eigen::MatrixXd::Zero(constraints_num, 1);
    Eigen::VectorXd upper_bounds_vec = Eigen::MatrixXd::Zero(constraints_num, 1);
    int iter = 0;

    // ~Stage II: supply start state and end state constraint
    // Supply start point and end point position constraints
    double start_cube_time_span = ref_times[1] - ref_times[0];
    double end_cube_time_span = ref_times[ref_times.size() - 1] - ref_times[ref_times.size() - 2];
    constraints_matrix(iter, 0) = 1.0;
    lower_bounds_vec(iter) = start_state[0];
    upper_bounds_vec(iter) = start_state[0];
    iter += 1;
    constraints_matrix(iter, variables_num - 1) = 1.0;
    lower_bounds_vec(iter) = end_state[0];
    upper_bounds_vec(iter) = end_state[0];
    iter += 1;
    // Supply start point velocity constraint
    constraints_matrix(iter, 0) = -5.0, constraints_matrix(iter, 1) = 5.0;
    lower_bounds_vec(iter) = start_state[1] * start_cube_time_span;
    upper_bounds_vec(iter) = start_state[1] * start_cube_time_span;
    iter += 1;
    // Supply start point acceleration constraint
    constraints_matrix(iter, 0) = 20.0, constraints_matrix(iter, 1) = -40.0, constraints_matrix(iter, 2) = 20.0;
    lower_bounds_vec(iter) = start_state[2] * start_cube_time_span;
    upper_bounds_vec(iter) = start_state[2] * start_cube_time_span;
    iter += 1;

    // ~Stage III: supply unequal constraints 
    for (int i = 1; i < unequal_constraints[0].size(); i++) {
        constraints_matrix(iter, i) = 1.0;
        lower_bounds_vec(iter) = unequal_constraints[0][i];
        upper_bounds_vec(iter) = unequal_constraints[1][i];
        iter += 1;
    }

    // ~Stage IV: supply equal constraints
    for (int i = 0; i < static_cast<int>(equal_constraints.size()); i++) {
        for (int j = 0; j < variables_num; j++) {

            // // DEBUG: check this logic
            // assert(static_cast<int>(equal_constraints[i].size()) == variables_num);
            // // END DEBUG

            constraints_matrix(iter, j) = equal_constraints[i][j];

        }
        lower_bounds_vec(iter) = 0.0;
        upper_bounds_vec(iter) = 0.0;
        iter += 1;
    }

    // ~Stage V: supply polynomial unequal constraints
    std::vector<std::vector<double>> polynomial_coefficients = std::get<0>(polynomial_unequal_constraints);
    std::vector<double> polynomial_lower_boundaries = std::get<1>(polynomial_unequal_constraints);
    std::vector<double> polynomial_upper_boundaries = std::get<2>(polynomial_unequal_constraints);
    for (int i = 0; i < std::get<1>(polynomial_unequal_constraints).size(); i++) {
        for (int j = 0; j < variables_num; j++) {
            constraints_matrix(iter, j) = polynomial_coefficients[i][j];
        }
        lower_bounds_vec(iter) = polynomial_lower_boundaries[i];
        upper_bounds_vec(iter) = polynomial_upper_boundaries[i];
        iter += 1;
    }

    constraints = constraints_matrix.sparseView();
    lower_bounds = lower_bounds_vec;
    upper_bounds = upper_bounds_vec;
}

OoqpOptimizationInterface::OoqpOptimizationInterface() = default;
OoqpOptimizationInterface::~OoqpOptimizationInterface() = default;

/**
 * @brief load data
 * @param ref_stamps time stamps of the point in in the intersection of two cubes
 * @param start_constraints start points' constraints
 * @param end_constraints end points' constraints
 * @param unequal_constraints position limit of each point
 * @param equal_constraints ensure the continuity of the connections between each two cubes
 */    
void OoqpOptimizationInterface::load(const std::vector<double>& ref_stamps, const std::array<double, 3>& start_constraints, const double& end_s_constraint, std::array<std::vector<double>, 2>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints, std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>>& polymonial_unequal_constraints) {

    // // DEBUG
    // std::cout << "start velocity: " << start_constraints[1] << std::endl;
    // // END DEBUG

    ref_stamps_ = ref_stamps;
    start_constraints_ = start_constraints;
    end_s_constraint_ = end_s_constraint;
    unequal_constraints_ = unequal_constraints;
    equal_constraints_ = equal_constraints;
    polymonial_unequal_constraints_ = polymonial_unequal_constraints;

    // // DEBUG
    // std::cout << "Load successful" << std::endl;
    // // END DEBUG
}

/**
 * @brief Run optimization
 * @param {*}
 * @return {*}
 */    
bool OoqpOptimizationInterface::runOnce(std::vector<double>* optimized_s, double* objective_value) {

    // // Multi thread calculation
    // // TODO: add logic to handle the situation where the optimization process is failed
    // clock_t single_dim_optimization_start_time = clock();
    // std::thread s_thread(&OoqpOptimizationInterface::optimizeSingleDim, this, s_start_constraints, s_end_constraints, s_unequal_constraints[0], s_unequal_constraints[1], "s");
    // std::thread d_thread(&OoqpOptimizationInterface::optimizeSingleDim, this, d_start_constraints, d_end_constraints, d_unequal_constraints[0], d_unequal_constraints[1], "d");
    // s_thread.join();
    // d_thread.join();
    // clock_t single_dim_optimization_end_time = clock();


    optimizeSingleDim(start_constraints_, end_s_constraint_, unequal_constraints_[0], unequal_constraints_[1]);
    
    *optimized_s = optimized_data_;
    *objective_value = objective_value_;
    return optimization_res_;
}

/**
 * @brief Optimize in single dimension
 * @param {*}
 */
void OoqpOptimizationInterface::optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const double& end_s_constraint, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries) {
    // ~Stage I: calculate objective function Q and c
    Eigen::SparseMatrix<double, Eigen::RowMajor> Q;
    Eigen::VectorXd c;
    calculateQcMatrix(Q, c);

    // // DEBUG 
    // std::cout << "Q: " << Q << std::endl;
    // std::cout << "c: " << c << std::endl; 
    // // END DEBUG

    // ~Stage II: calculate equal constraints, includes start point constraints, end point constraints and continuity constraints
    Eigen::SparseMatrix<double, Eigen::RowMajor> A;
    Eigen::VectorXd b;
    calculateAbMatrix(single_start_constraints, end_s_constraint, equal_constraints_, A, b);

    // // DEBUG
    // std::cout << "A: " << A << std::endl;
    // std::cout << "b: " << b << std::endl;
    // // END DEBUG

    // ~Stage III: calculate low and up boundaries for intermediate points
    Eigen::Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
    Eigen::Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
    Eigen::VectorXd lowerLimitForX;
    Eigen::VectorXd upperLimitForX;
    calculateBoundariesForIntermediatePoints(single_lower_boundaries, single_upper_boundaries, useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX);

    // // DEBUG
    // std::cout << "lowerLimitForX: " << lowerLimitForX << std::endl;
    // std::cout << "upperLimitForX: " << upperLimitForX << std::endl;
    // for (int i = 0; i < useLowerLimitForX.rows(); i++) {
    //     printf("%d\n", useLowerLimitForX(i, 0));
    //     printf("%d\n", useUpperLimitForX(i, 0));
    // }
    // std::cout << "------------------" << std::endl;
    // // END DEBUG

    // // DEBUG
    // std::cout << "prepare success" << std::endl;
    // // END DBEUG

    // ~Stage IV: calculate polymonial unequal constraints
    Eigen::SparseMatrix<double, Eigen::RowMajor> C;
    Eigen::VectorXd d;
    Eigen::VectorXd f;
    calculateCdfMatrix(C, d, f);



    // ~Stage V: optimization
    std::vector<double> optimized_values;
    double objective_value = 0.0;
    bool optimization_status = solve(Q, c, A, b, C, d, f, useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX, &optimized_values, &objective_value);

    // ~Stage VI: store information
    optimized_data_ = optimized_values;
    optimization_res_ = optimization_status;
    objective_value_ = objective_value;
}

/**
 * @brief calculate objective function
 * @param {*}
 * @return {*}
 */    
void OoqpOptimizationInterface::calculateQcMatrix(Eigen::SparseMatrix<double, Eigen::RowMajor>& Q, Eigen::VectorXd& c) {
    // Initialize Q matrix
    int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
    Eigen::MatrixXd Q_matrix = Eigen::MatrixXd::Zero(variables_num, variables_num);

    // Calculate D matrix
    for (int i = 0; i < static_cast<int>(ref_stamps_.size()) - 1; i++) {
        // Calculate time span
        double time_span = ref_stamps_[i + 1] - ref_stamps_[i];
        double time_coefficient = pow(time_span, -3);

        // Intergrate to objective function
        int influenced_variable_index = i * 6;
        Q_matrix.block(influenced_variable_index, influenced_variable_index, 6, 6) += BezierCurveHessianMatrix * time_coefficient;
    }

    // // DEBUG
    // std::cout << "Q_matrix: " << Q_matrix << std::endl;
    // // END DEBUG

    // // DEBUG
    // for (int i = 0; i < Q_matrix.rows(); i++) {
    //     for (int j = 0; j < Q_matrix.cols(); j++) {
    //         std::cout << Q_matrix(i, j) << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << "-----------------" << std::endl;
    // // END DEBUG

    Q = Q_matrix.sparseView();
    c.resize(variables_num);
    c.setZero();
}

void OoqpOptimizationInterface::calculateCdfMatrix(Eigen::SparseMatrix<double, Eigen::RowMajor>& C, Eigen::VectorXd& d, Eigen::VectorXd& f) {
    // Parse data
    std::vector<std::vector<double>> polynomial_coefficients = std::get<0>(polymonial_unequal_constraints_);
    std::vector<double> polynomial_lower_boundaries = std::get<1>(polymonial_unequal_constraints_);
    std::vector<double> polynomial_upper_boundaries = std::get<2>(polymonial_unequal_constraints_);
    int n = static_cast<int>(polynomial_coefficients.size());

    // Initialize
    int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
    Eigen::MatrixXd C_matrix = Eigen::MatrixXd::Zero(n, variables_num);
    Eigen::MatrixXd d_matrix = Eigen::MatrixXd::Zero(n, 1);
    Eigen::MatrixXd f_matrix = Eigen::MatrixXd::Zero(n, 1);

    // Transform
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < variables_num; j++) {
            C_matrix(i, j) = polynomial_coefficients[i][j];
        }
        d_matrix(i, 0) = polynomial_lower_boundaries[i];
        f_matrix(i, 0) = polynomial_upper_boundaries[i];
    }

    C = C_matrix.sparseView();
    d = d_matrix;
    f = f_matrix;
}


// DEBUG
// Test the optimization process without the end point constraint
void OoqpOptimizationInterface::calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b) {
            
    // Calculate dimensions and initialize
    int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
    int equal_constraints_num = 3 + (static_cast<int>(ref_stamps_.size()) - 2) * 3;
    double start_cube_time_span = ref_stamps_[1] - ref_stamps_[0];
    double end_cube_time_span = ref_stamps_[ref_stamps_.size() - 1] - ref_stamps_[ref_stamps_.size() - 2];
    Eigen::MatrixXd A_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, variables_num);
    Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, 1);

    // // supply start point and end point position constraints 
    // A_matrix(0, 0) = 1.0, A_matrix(1, variables_num - 1) = 1.0;
    // b_matrix(0, 0) = single_start_constraints[0], b_matrix(1, 0) = single_end_constraints[0];

    // // supply start point and end point velocity constraints
    // A_matrix(2, 0) = -5.0, A_matrix(2, 1) = 5.0;
    // b_matrix(2, 0) = single_start_constraints[1] * start_cube_time_span;
    // A_matrix(3, variables_num - 2) = -5.0, A_matrix(3, variables_num - 1) = 5.0;
    // b_matrix(3, 0) = single_end_constraints[1] * end_cube_time_span;

    // // supply start point and end point acceleration constraints
    // A_matrix(4, 0) = 20.0, A_matrix(4, 1) = -40.0, A_matrix(4, 2) = 20.0;
    // b_matrix(4, 0) = single_start_constraints[2] * start_cube_time_span;
    // A_matrix(5, variables_num - 3) = 20.0, A_matrix(5, variables_num - 2) = -40.0, A_matrix(5, variables_num - 1) = 20.0;
    // b_matrix(5, 0) = single_end_constraints[2] * end_cube_time_span;

    // Supply start point s constraint
    A_matrix(0, 0) = 1.0;
    b_matrix(0, 0) = single_start_constraints[0];

    // Supply start point v constraint
    A_matrix(1, 0) = -5.0, A_matrix(2, 1) = 5.0;
    b_matrix(1, 0) = single_start_constraints[1] * start_cube_time_span;

    // Supply start point a constraint
    A_matrix(2, 0) = 20.0, A_matrix(2, 1) = -40.0, A_matrix(2, 2) = 20.0;
    b_matrix(2, 0) = single_start_constraints[2] * start_cube_time_span;

    // supply continuity ensurance constraints
    for (int i = 0; i < static_cast<int>(equal_constraints.size()); i++) {
        int constraint_index = i + 3;
        for (int j = 0; j < variables_num; j++) {
            
            // DEBUG: check this logic
            assert(static_cast<int>(equal_constraints[i].size()) == variables_num);
            // END DEBUG

            A_matrix(constraint_index, j) = equal_constraints[i][j];
        }
    }
    A = A_matrix.sparseView();
    b = b_matrix;

    // // DEBUG
    // std::cout << "A_matrix: " << std::endl;
    // for (int i = 0; i < A_matrix.rows(); i++) {
    //     for (int j = 0; j < A_matrix.cols(); j++) {
    //         std::cout << A_matrix(i, j) << ", ";
    //     }
    // }
    // std::cout << "-------------------" << std::endl;
    // std::cout << b_matrix << std::endl;
    // std::cout << "-------------------" << std::endl;
    // // END DEBUG
}

/**
 * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
 * @param {*}
 */
// This version delete the v and a constraint of the final point
void OoqpOptimizationInterface::calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const double& end_s_constraint, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b) {
            
    // Calculate dimensions and initialize
    int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
    int equal_constraints_num = 4 + (static_cast<int>(ref_stamps_.size()) - 2) * 3;
    double start_cube_time_span = ref_stamps_[1] - ref_stamps_[0];
    double end_cube_time_span = ref_stamps_[ref_stamps_.size() - 1] - ref_stamps_[ref_stamps_.size() - 2];
    Eigen::MatrixXd A_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, variables_num);
    Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, 1);

    // supply start point and end point position constraints 
    A_matrix(0, 0) = 1.0, A_matrix(1, variables_num - 1) = 1.0;
    b_matrix(0, 0) = single_start_constraints[0], b_matrix(1, 0) = end_s_constraint;

    // supply start point and end point velocity constraints
    A_matrix(2, 0) = -5.0, A_matrix(2, 1) = 5.0;
    b_matrix(2, 0) = single_start_constraints[1] * start_cube_time_span;
    // A_matrix(3, variables_num - 2) = -5.0, A_matrix(3, variables_num - 1) = 5.0;
    // b_matrix(3, 0) = single_end_constraints[1] * end_cube_time_span;

    // supply start point and end point acceleration constraints
    A_matrix(3, 0) = 20.0, A_matrix(3, 1) = -40.0, A_matrix(3, 2) = 20.0;
    b_matrix(3, 0) = single_start_constraints[2] * start_cube_time_span;
    // A_matrix(5, variables_num - 3) = 20.0, A_matrix(5, variables_num - 2) = -40.0, A_matrix(5, variables_num - 1) = 20.0;
    // b_matrix(5, 0) = single_end_constraints[2] * end_cube_time_span;
  
    // supply continuity ensurance constraints
    for (int i = 0; i < static_cast<int>(equal_constraints.size()); i++) {
        int constraint_index = i + 4;
        for (int j = 0; j < variables_num; j++) {
            
            // // DEBUG: check this logic
            // assert(static_cast<int>(equal_constraints[i].size()) == variables_num);
            // // END DEBUG

            A_matrix(constraint_index, j) = equal_constraints[i][j];
        }
    }
    A = A_matrix.sparseView();
    b = b_matrix;

    // // DEBUG
    // std::cout << "A_matrix: " << std::endl;
    // for (int i = 0; i < A_matrix.rows(); i++) {
    //     for (int j = 0; j < A_matrix.cols(); j++) {
    //         std::cout << A_matrix(i, j) << ", ";
    //     }
    // }
    // std::cout << "-------------------" << std::endl;
    // std::cout << b_matrix << std::endl;
    // std::cout << "-------------------" << std::endl;
    // // END DEBUG
}

/**
 * @brief Calculate boundaries for intermediate points
 * @param {*}
 */    
void OoqpOptimizationInterface::calculateBoundariesForIntermediatePoints(const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries, Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX) {
    int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
    useLowerLimitForX.setConstant(variables_num, 1);
    useUpperLimitForX.setConstant(variables_num, 1);
    lowerLimitForX.resize(variables_num);
    upperLimitForX.resize(variables_num);

    for (int i = 0; i < variables_num; i++) {
        if (i == 0 || i == variables_num - 1) {
            // For the first point and last point, the unequal constraints are invalid
            useLowerLimitForX(i) = 0;
            useUpperLimitForX(i) = 0;
            lowerLimitForX(i) = 0.0;
            upperLimitForX(i) = 0.0;
        } else {
            useLowerLimitForX(i) = 1;
            useUpperLimitForX(i) = 1;
            lowerLimitForX(i) = single_lower_boundaries[i];
            upperLimitForX(i) = single_upper_boundaries[i];
        }
    }
}

/**
 * @brief solve quartic programming
 * @param {*}
 * @return {*}
 */
bool OoqpOptimizationInterface::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                const Eigen::VectorXd& c,
                const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                const Eigen::VectorXd& b,
                const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, 
                Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX, std::vector<double>* optimized_values, double* objective_value) {
    
    // // DEBUG
    // std::cout << "Q: " << Q << std::endl;
    // std::cout << "c: " << c << std::endl;
    // std::cout << "A: " << A << std::endl;
    // std::cout << "b: " << b << std::endl;
    // std::cout << "useLowerLimitForX: " << useLowerLimitForX << std::endl;
    // std::cout << "useUpperLimitForX: " << useUpperLimitForX << std::endl;
    // std::cout << "lowerLimitForX: " << lowerLimitForX << std::endl;
    // std::cout << "upperLimitForX: " << upperLimitForX << std::endl;
    // // END DEBUG

    // // DEBUG
    // std::cout << "solve once" << std::endl;
    // // END DEBUG

    Eigen::VectorXd x;
    int nx = Q.rows();
    x.setZero(nx);
    // Make copies of variables that are changed
    auto ccopy(c);
    auto Acopy(A);
    auto bcopy(b);
    auto Ccopy(C);
    auto dcopy(d);
    auto fcopy(f);

    Eigen::SparseMatrix<double, Eigen::RowMajor> Q_triangular = Q.triangularView<Eigen::Lower>();
    
    // Compress sparse Eigen matrices (refer to Eigen Sparse Matrix user manual)
    Q_triangular.makeCompressed();
    Acopy.makeCompressed();
    Ccopy.makeCompressed();

    assert(Ccopy.rows() == dcopy.size());
    assert(Ccopy.rows() == fcopy.size());

    // Calculate used flag for polynomial inequal constraints
    Eigen::Matrix<char, Eigen::Dynamic, 1> useLowerLimitForPolynomialInequalConstraints;
    Eigen::Matrix<char, Eigen::Dynamic, 1> useUpperLimitForPolynomialInequalConstraints;
    useLowerLimitForPolynomialInequalConstraints.setConstant(static_cast<int>(dcopy.size()), 1);
    useUpperLimitForPolynomialInequalConstraints.setConstant(static_cast<int>(dcopy.size()), 1);

    // Initialize new problem formulation.
    int my = bcopy.size();
    int mz = dcopy.size(); 
    int nnzQ = Q_triangular.nonZeros();
    int nnzA = Acopy.nonZeros();
    int nnzC = Ccopy.nonZeros(); // Unequal constraint , set with 0


    // // DEBUG
    // // Set the number of the inequal constraints to 0 to test the remain function
    // mz = 0;
    // nnzC = 0;
    // // END DEBUG

    QpGenSparseMa27* qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
    
    // Fill in problem data
    double* cp = &ccopy.coeffRef(0);
    std::vector<int> irowQ_vec, jcolQ_vec;
    Tools::calculateParam(Q_triangular, &irowQ_vec, &jcolQ_vec);
    int irowQ[irowQ_vec.size()], jcolQ[jcolQ_vec.size()];
    memcpy(irowQ, &irowQ_vec[0], irowQ_vec.size() * sizeof(irowQ_vec[0]));
    memcpy(jcolQ, &jcolQ_vec[0], jcolQ_vec.size() * sizeof(jcolQ_vec[0]));
    double* dQ = Q_triangular.valuePtr();

    double* xlow = &lowerLimitForX.coeffRef(0);
    char* ixlow = &useLowerLimitForX.coeffRef(0);
    double* xupp = &upperLimitForX.coeffRef(0);
    char* ixupp = &useUpperLimitForX.coeffRef(0);
    
    std::vector<int> irowA_vec, jcolA_vec;
    Tools::calculateParam(Acopy, &irowA_vec, &jcolA_vec);
    int irowA[irowA_vec.size()], jcolA[jcolA_vec.size()];
    memcpy(irowA, &irowA_vec[0], irowA_vec.size() * sizeof(irowA_vec[0]));
    memcpy(jcolA, &jcolA_vec[0], jcolA_vec.size() * sizeof(jcolA_vec[0]));
    double* dA = Acopy.valuePtr();
    double* bA = &bcopy.coeffRef(0);
    
    // // DEBUG
    // int irowC[] = {};
    // int jcolC[] = {};
    // double dC[] = {};
    // double clow[] = {};
    // char iclow[] = {};
    // double cupp[] = {};
    // char icupp[] = {};
    // // END DEBUG

    std::vector<int> irowC_vec, jcolC_vec;
    Tools::calculateParam(Ccopy, &irowC_vec, &jcolC_vec);
    int irowC[irowC_vec.size()], jcolC[jcolC_vec.size()];
    memcpy(irowC, &irowC_vec[0], irowC_vec.size() * sizeof(irowC_vec[0]));
    memcpy(jcolC, &jcolC_vec[0], jcolC_vec.size() * sizeof(jcolC_vec[0]));
    double* dC = Ccopy.valuePtr();
    double* clow = &dcopy.coeffRef(0);
    char* iclow = &useLowerLimitForPolynomialInequalConstraints.coeffRef(0);
    double* cupp = &fcopy.coeffRef(0);
    char* icupp = &useUpperLimitForPolynomialInequalConstraints.coeffRef(0);



    QpGenData *prob = (QpGenData *)qp->copyDataFromSparseTriple(cp, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, irowA, nnzA, jcolA, dA, bA, irowC, nnzC, jcolC, dC, clow, iclow, cupp, icupp);
    
    // Create object to store problem variables
    QpGenVars* vars = (QpGenVars*)qp->makeVariables(prob);

    // // DEBUG
    // prob->print();
    // // END DEBUG

    // Create object to store problem residual data
    QpGenResiduals* resid = (QpGenResiduals*)qp->makeResiduals(prob);

    // Create solver object
    GondzioSolver* s = new GondzioSolver(qp, prob);

    // // DEBUG
    // s->monitorSelf();
    // // END DEBUG

    // Solve
    int status = s->solve(prob, vars, resid);

    // // Test
    // double objective_value = prob->objectiveValue(vars);
    // std::cout << "Objective value: " << objective_value << std::endl;

    // DEBUG
    std::cout << "Optimization status: " << status << std::endl;
    // END DEBUG

    if (status == TerminationCode::SUCCESSFUL_TERMINATION) {
        vars->x->copyIntoArray(&x.coeffRef(0));
        *optimized_values = std::vector<double>(x.data(), x.data() + x.rows() * x.cols());
        *objective_value = prob->objectiveValue(vars);
    }

    delete s;
    delete resid;
    delete vars;
    delete prob;
    delete qp;

    return status == TerminationCode::SUCCESSFUL_TERMINATION;

}

VelocityOptimizer::VelocityOptimizer() {
    // ooqp_itf_ = new OoqpOptimizationInterface();

}

VelocityOptimizer::~VelocityOptimizer() = default;

bool VelocityOptimizer::runOnce(const std::vector<std::vector<Cube2D<double>>>& cube_paths, const std::array<double, 3>& start_state, std::vector<std::pair<double, double>>& last_s_range, const double& max_velocity, const double& min_velocity, const double& max_acceleration, const double& min_acceleration, std::vector<double>* s, std::vector<double>* t) {

    // ~Stage I: determine the s sampling number due to the number of the available paths
    int available_cube_paths_num = cube_paths.size();
    const int optimization_maximum_number = 10;
    int s_sampling_number = static_cast<int>(optimization_maximum_number / available_cube_paths_num);
    int practical_optimization_number = available_cube_paths_num * s_sampling_number;

    // ~Stage II: sampling s with a fixed interval
    double s_available_range = last_s_range.back().second - last_s_range.front().first;
    double s_start = last_s_range.front().first;
    double s_end = last_s_range.back().second;
    double s_interval = s_available_range / static_cast<double>(s_sampling_number - 1);
    std::vector<double> sampled_s;
    while (s_start <= s_end) {
        sampled_s.emplace_back(s_start);
        s_start += s_interval;
    }

    // ~Stage III: optimization
    int n = cube_paths.size() * sampled_s.size();


    ss_.resize(n);
    tt_.resize(n);
    ress_.resize(n);
    values_.resize(n);

    // std::vector<std::thread> thread_set(cube_paths.size() * sampled_s.size());

    for (int i = 0; i < cube_paths.size(); i++) {
        for (int j = 0; j < sampled_s.size(); j++) {
            int index = i * sampled_s.size() + j;
            runSingleCubesPath(cube_paths[i], start_state, sampled_s[j], max_velocity, min_velocity, max_acceleration, min_acceleration, index);
            // thread_set[index] = std::thread(&VelocityPlanning::VelocityOptimizer::runSingleCubesPath, this, cube_paths[i], start_state, sampled_s[j], max_velocity, min_velocity, max_acceleration, min_acceleration, index);
        }
    }

    // for (int i = 0; i < thread_set.size(); i++) {
    //     thread_set[i].join();
    // }


    // ~Stage III: select the res with the optimal jerk
    std::vector<std::pair<double, int>> s_info;


    for (int i = 0; i < n; i++) {
        if (!ress_[i]) continue;

        // bool fall_back = false;
        // for (int k = 6; k < ss_[i].size(); k += 6) {

        //     // DEBUG
        //     double cur = 0.0;
        //     double prev = 0.0;
        //     for (int j = k; j < k + 6; j++) {
        //         cur += ss_[i][j];
        //         prev += ss_[i][j - 6];
        //     }
        //     // std::cout << "Cur: " << std::accumulate(ss_[i].begin() + k, ss_[i].begin() + k + 6, 0) << std::endl;
        //     // std::cout << "Prev: " << std::accumulate(ss_[i].begin() + k - 6, ss_[i].begin() + k, 0) << std::endl;
        //     // std::cout << "Cur new: " << cur << std::endl;
        //     // std::cout << "Prev new: " << prev << std::endl;
        //     // END DEBUG

        //     if (cur <= prev) {
        //         fall_back = true;
        //         break;
        //     }
        // }
        // if (fall_back) {
        //     continue;
        // }

        double cur_s = ss_[i].back();
        double cur_jerk = values_[i];
        s_info.emplace_back(std::make_pair(cur_s, i));

        // // DEBUG
        // std::cout << "Number: " << i << std::endl;
        // std::cout << "End s: " << cur_s << std::endl;
        // std::cout << "Jerk: " << cur_jerk << std::endl;
        // std::cout << "s: " << std::endl;
        // for (int j = 0; j < ss_[i].size(); j++) {
        //     std::cout << ss_[i][j] << ", ";
        // }
        // std::cout << std::endl;
        // std::cout << "t: " << std::endl;
        // for (int j = 0; j < tt_[i].size(); j++) {
        //     std::cout << tt_[i][j] << ", ";
        // }
        // std::cout << std::endl;
        // // END DEBUG

    }

    // Select the index with the top three s
    double min_jerk = static_cast<double>(INT_MAX);
    int win_index = -1;
    std::sort(s_info.rbegin(), s_info.rend());
    for (int i = 0; i < std::min(1, static_cast<int>(s_info.size())); i++) {
        double cur_jerk = values_[s_info[i].second];
        
        // // DEBUG
        // std::cout << "s: " << s_info[i].first << ", jerk: " << cur_jerk << ", min jerk: " << min_jerk << std::endl;
        // // END DEBUG

        if (cur_jerk < min_jerk) {
            min_jerk = cur_jerk;
            win_index = s_info[i].second;
        }
    }

    if (win_index == -1) {
        return false;
    }
    
    *s = ss_[win_index];
    *t = tt_[win_index];
    return true;

}

void VelocityOptimizer::runSingleCubesPath(const std::vector<Cube2D<double>>& cube_path, const std::array<double, 3>& start_state, const double& end_s, const double& max_velocity, const double& min_velocity, const double& max_acceleration, const double& min_acceleration, int index) {

    Cube2D<double> last_cube = cube_path.back();
    if (end_s > last_cube.s_end_ || end_s < last_cube.s_start_) {

        ress_[index] = false;

        return;
    }

    // ~Stage I: calculate time stamps of split point (include start point and end point)
    std::vector<double> ref_stamps;
    for (int i = 0; i < cube_path.size(); i++) {
        if (i == 0) {
            ref_stamps.emplace_back(cube_path[i].t_start_);
        }
        ref_stamps.emplace_back(cube_path[i].t_end_);
    }

    // // DEBUG
    // std::cout << "ref t" << std::endl;
    // for (int i = 0; i < ref_stamps.size(); i++) {
    //     std::cout << ref_stamps[i] << std::endl;
    // }
    // // END DEBUG

    // ~Stage II: calculate unequal constraints for variables (except start point and end point)
    std::array<std::vector<double>, 2> unequal_constraints = generateUnequalConstraints(cube_path);

    // // DEBUG
    // std::cout << "Index: " << index << std::endl;
    // std::cout << "Unequal constraints: " << std::endl;
    // for (int i = 0; i < unequal_constraints[0].size(); i++) {
    //     std::cout << "Lower: " << unequal_constraints[0][i] << ", upper: " << unequal_constraints[1][i] << std::endl;
    // }
    // // END DEBUG

    // // DEBUG
    // std::cout << "unequal constraints calculation success" << std::endl;
    // // END DEBUG

    // ~Stage III: calculate equal constraints to ensure the continuity
    std::vector<std::vector<double>> equal_constraints = generateEqualConstraints(cube_path);

    // ~Stage IV: calculate polynomial inequality constraints
    std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>> polynomial_unequal_constraints = generatePolynimalUnequalConstraints(cube_path, start_state[1], max_velocity, min_velocity, max_acceleration, min_acceleration);



    // // DEBUG
    // std::cout << "equal constraints calculation success" << std::endl;
    // // END DEBUG

    // ~Stage V: optimization
    std::vector<double> optimized_s;
    double objective_value = 0.0;
    // ooqp_itf_->load(ref_stamps, start_state, end_s, unequal_constraints, equal_constraints, polynomial_unequal_constraints);
    // bool optimization_res = ooqp_itf_->runOnce(&optimized_s, &objective_value);

    std::array<double, 3> end_state = {end_s, 0.0, 0.0};
    bool optimization_res = OsqpOptimizationInterface::runOnce(ref_stamps, start_state, end_state, unequal_constraints, equal_constraints, polynomial_unequal_constraints, &optimized_s, &objective_value);

    // // DEBUG
    // std::cout << "end s: " << end_s << std::endl;
    // std::cout << "optimization res: " << optimization_res << std::endl;
    // // END DEBUG

    printf("[VelocityOptimizer] End s: %lf.\n", end_s);
    printf("[VelocityOptimizer] Optimization result: %d.\n", optimization_res);

    ss_[index] = optimized_s;
    tt_[index] = ref_stamps;
    ress_[index] = optimization_res;
    values_[index] = objective_value;
    
}

std::array<std::vector<double>, 2> VelocityOptimizer::generateUnequalConstraints(const std::vector<Cube2D<double>>& cube_path) {
    // Initialize 
    int variables_num = static_cast<int>(cube_path.size()) * 6;
    std::array<std::vector<double>, 2> unequal_constraints = {};
    for (int i = 0; i < 2; i++) {
        unequal_constraints[i].resize(static_cast<int>(variables_num));
    } 

    // Calculate unequal constraints
    for (int i = 0; i < variables_num; i++) {
        if (i == 0) {
            // Delete unequal constraints for start point and end point
            continue;
        }

        // if (i % 5 == 0) {
        //     // Points in intersection which have two constraint cubes 
        //     int next_corridor_index = i / 5;
        //     DrivingCubeWorldMetric current_cube = driving_corridor_.cubes[next_corridor_index - 1];
        //     DrivingCubeWorldMetric next_cube = driving_corridor_.cubes[next_corridor_index];
        //     tmp_unequal_constraints[0][i] = std::max(current_cube.cube.s_start_, next_cube.cube.s_start_); // s_low
        //     tmp_unequal_constraints[1][i] = std::min(current_cube.cube.s_end_, next_cube.cube.s_end_); // s_up
        //     tmp_unequal_constraints[2][i] = std::max(current_cube.cube.d_start_, next_cube.cube.d_start_); // d_start
        //     tmp_unequal_constraints[3][i] = std::min(current_cube.cube.d_end_, next_cube.cube.d_end_); // d_end
        // } else {
        //     // Normal points which only have one constraint cube
        //     int current_corridor_index = i / 5;
        //     DrivingCubeWorldMetric current_cube = driving_corridor_.cubes[current_corridor_index];
        //     tmp_unequal_constraints[0][i] = current_cube.cube.s_start_;
        //     tmp_unequal_constraints[1][i] = current_cube.cube.s_end_;
        //     tmp_unequal_constraints[2][i] = current_cube.cube.d_start_;
        //     tmp_unequal_constraints[3][i] = current_cube.cube.d_end_;
        // }

        int current_cube_index = i / 6;
        Cube2D<double> current_cube = cube_path[current_cube_index];
        unequal_constraints[0][i] = current_cube.s_start_;
        unequal_constraints[1][i] = current_cube.s_end_;

    }

    return unequal_constraints;


}

std::vector<std::vector<double>> VelocityOptimizer::generateEqualConstraints(const std::vector<Cube2D<double>>& cube_path) {
    // Initialize
    std::vector<std::vector<double>> equal_constraints;
    int variables_num = static_cast<int>(cube_path.size()) * 6;

    // Calculate equal constraints
    for (int i = 0; i < static_cast<int>(cube_path.size()) - 1; i++) {
        // Calculate two related cubes and their time span
        int current_cube_index = i;
        int next_cube_index = i + 1;
        Cube2D<double> current_cube = cube_path[i];
        Cube2D<double> next_cube = cube_path[i + 1];
        double current_cube_time_span = current_cube.t_end_ - current_cube.t_start_;
        double next_cube_time_span = next_cube.t_end_ - next_cube.t_start_;

        // Initialize equal constraints
        int current_cube_start_index = i * 6;
        int next_cube_start_index = (i + 1) * 6;
        std::vector<double> current_position_equal_constraints(variables_num, 0.0);
        std::vector<double> current_velocity_equal_constraints(variables_num, 0.0);
        std::vector<double> current_acceleration_equal_constraints(variables_num, 0.0);

        // Supply position constraints
        current_position_equal_constraints[current_cube_start_index + 5] = 1.0;
        current_position_equal_constraints[next_cube_start_index] = -1.0;

        // Supply velocity constraints 
        current_velocity_equal_constraints[current_cube_start_index + 4] = -5.0 / current_cube_time_span;
        current_velocity_equal_constraints[current_cube_start_index + 5] = 5.0 / current_cube_time_span;
        current_velocity_equal_constraints[next_cube_start_index] = -1.0 * -5.0 / next_cube_time_span;
        current_velocity_equal_constraints[next_cube_start_index + 1] = -1.0 * 5.0 / next_cube_time_span;

        // Supply acceleration constraints 
        current_acceleration_equal_constraints[current_cube_start_index + 3] = 20.0 / current_cube_time_span;
        current_acceleration_equal_constraints[current_cube_start_index + 4] = -40.0 / current_cube_time_span;
        current_acceleration_equal_constraints[current_cube_start_index + 5] = 20.0 / current_cube_time_span;
        current_acceleration_equal_constraints[next_cube_start_index] = -1.0 * 20.0 / next_cube_time_span;
        current_acceleration_equal_constraints[next_cube_start_index + 1] = -1.0 * -40.0 / next_cube_time_span;
        current_acceleration_equal_constraints[next_cube_start_index + 2] = -1.0 * 20.0 / next_cube_time_span;

        equal_constraints.emplace_back(current_position_equal_constraints);
        equal_constraints.emplace_back(current_velocity_equal_constraints);
        equal_constraints.emplace_back(current_acceleration_equal_constraints);
    }

    return equal_constraints;
}

std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>> VelocityOptimizer::generatePolynimalUnequalConstraints(const std::vector<Cube2D<double>>& cube_path, const double& start_velocity, const double& max_velocity, const double& min_velocity, const double& max_acceleration, const double& min_acceleration) {
    // Initialize
    int variables_num = static_cast<int>(cube_path.size()) * 6;
    int n = static_cast<int>(cube_path.size());
    std::vector<std::vector<double>> coefficients;
    std::vector<double> lower_boundaries;
    std::vector<double> upper_boundaries;
    
    // Supply velocity constraints
    if (start_velocity <= max_velocity) {
        // Start velocity is within the speed limitation, constraining all the velocity in the profile
        for (int i = 0; i < n; i++) {
            int current_cube_start_index = i * 6;
            double time_span = cube_path[i].t_end_ - cube_path[i].t_start_;
            for (int j = current_cube_start_index; j < current_cube_start_index + 5; j++) {
                std::vector<double> current_velocity_constraints_coefficients(variables_num, 0.0);
                current_velocity_constraints_coefficients[j + 1] = 5.0;
                current_velocity_constraints_coefficients[j] = -5.0;
                upper_boundaries.emplace_back(max_velocity * time_span);
                lower_boundaries.emplace_back(min_velocity * time_span);
                coefficients.emplace_back(current_velocity_constraints_coefficients);
            }
        }
    } else {
        // Start velocity excesses the speed limitation, constraining all the velocity using a linear profile
        double velocity_diff = start_velocity - max_velocity;
        double velocity_diff_interval = velocity_diff / (n - 1);
        std::vector<double> max_velocities(n, 0.0);
        for (int i = 0; i < n; i++) {
            max_velocities[i] = start_velocity + LARGE_EPS - (i * velocity_diff_interval);
        }

        for (int i = 0; i < n; i++) {
            int current_cube_start_index = i * 6;
            double time_span = cube_path[i].t_end_ - cube_path[i].t_start_;
            for (int j = current_cube_start_index; j < current_cube_start_index + 5; j++) {
                std::vector<double> current_velocity_constraints_coefficients(variables_num, 0.0);
                current_velocity_constraints_coefficients[j + 1] = 5.0;
                current_velocity_constraints_coefficients[j] = -5.0;
                upper_boundaries.emplace_back(max_velocities[i] * time_span);
                lower_boundaries.emplace_back(min_velocity * time_span);
                coefficients.emplace_back(current_velocity_constraints_coefficients);
            }
        }

    }


    // Supply acceleration constraints
    for (int i = 0; i < n; i++) {
        int current_cube_start_index = i * 6;
        double time_span = cube_path[i].t_end_ - cube_path[i].t_start_;
        for (int j = current_cube_start_index; j < current_cube_start_index + 4; j++) {
            std::vector<double> current_acceleratrion_constraints_coefficients(variables_num, 0.0);
            current_acceleratrion_constraints_coefficients[j] = 20.0;
            current_acceleratrion_constraints_coefficients[j + 1] = -40.0;
            current_acceleratrion_constraints_coefficients[j + 2] = 20.0;
            upper_boundaries.emplace_back(max_acceleration * time_span);
            lower_boundaries.emplace_back(min_acceleration * time_span);
            coefficients.emplace_back(current_acceleratrion_constraints_coefficients);
        }
    }

    std::tuple<std::vector<std::vector<double>>, std::vector<double>, std::vector<double>> polynomial_unequal_constraints = std::make_tuple(coefficients, lower_boundaries, upper_boundaries);

    return polynomial_unequal_constraints;


    
}



BezierPiecewiseCurve::BezierPiecewiseCurve(const std::vector<double>& s, std::vector<double>& ref_stamps) {
    // Check data
    assert((static_cast<int>(ref_stamps.size()) - 1) * 6 == static_cast<int>(s.size()));

    // Calculate segments number
    ref_stamps_ = ref_stamps;
    segment_num_ = static_cast<int>(ref_stamps.size()) - 1;

    // Calculate coefficient for both s dimension and d dimension
    s_coefficients_.resize(segment_num_);

    for (int i = 0; i < segment_num_; i++) {
        s_coefficients_[i].resize(6);
        
        // supply coefficients
        int start_influenced_index = i * 6;
        s_coefficients_[i][0] = s[start_influenced_index];
        s_coefficients_[i][1] = s[start_influenced_index + 1];
        s_coefficients_[i][2] = s[start_influenced_index + 2];
        s_coefficients_[i][3] = s[start_influenced_index + 3];
        s_coefficients_[i][4] = s[start_influenced_index + 4];
        s_coefficients_[i][5] = s[start_influenced_index + 5];

    }

}
BezierPiecewiseCurve::~BezierPiecewiseCurve() = default;

/**
 * @brief Calculate curve
 * @param {*}
 * @return {*}
 */    
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> BezierPiecewiseCurve::generateTraj(double sample_gap) {

    std::vector<double> s;
    std::vector<double> v;
    std::vector<double> a;
    std::vector<double> t;

    // Calculate point for each segment
    // Note that for each segment, the sample gap is the same, which means the sample points' number is different according to the segment's time span
    for (int segment_index = 0; segment_index < segment_num_; segment_index++) {
        double time_span = ref_stamps_[segment_index + 1] - ref_stamps_[segment_index];
        int sample_num = static_cast<int>(time_span / sample_gap);
        // Calculate seeds in this segment
        std::vector<double> segment_seeds;
        if (segment_index == segment_num_ - 1) {
            segment_seeds = Tools::linspace(0.0, 1.0, sample_num);
        } else {
            segment_seeds = Tools::linspace(0.0, 1.0, sample_num, false);
        }

        // For each seed, generate a point
        for (const auto& current_seed : segment_seeds) {
            double time_stamp = ref_stamps_[segment_index] + (time_span * current_seed);

            Eigen::Vector4d point = generatePoint(segment_index, current_seed, time_stamp);

            s.emplace_back(point(0));
            v.emplace_back(point(1));
            a.emplace_back(point(2));
            t.emplace_back(point(3));


        }
    }


    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> traj = std::make_tuple(s, v, a, t);

    return traj;
    
}

/**
 * @brief Calculate single point 
 * @param {*}
 * @return {*}
 */    
Eigen::Vector4d BezierPiecewiseCurve::generatePoint(int segment_index, double remain, double time_stamp) {
    // Calculate s and d value
    double s_value = s_coefficients_[segment_index][0] * pow(1.0 - remain, 5) + 5.0 * s_coefficients_[segment_index][1] * remain * pow(1.0 - remain, 4) + 10.0 * s_coefficients_[segment_index][2] * pow(remain, 2) * pow(1.0 - remain, 3) + 10.0 * s_coefficients_[segment_index][3] * pow(remain, 3) * pow(1.0 - remain, 2) + 5.0 * s_coefficients_[segment_index][4] * pow(remain, 4) * (1.0 - remain) + s_coefficients_[segment_index][5] * pow(remain, 5);

    double v_value = 5.0 * ((s_coefficients_[segment_index][1] - s_coefficients_[segment_index][0]) * pow(1.0 - remain, 4) + 4.0 * (s_coefficients_[segment_index][2] - s_coefficients_[segment_index][1]) * pow(1.0 - remain, 3) * remain + 6.0 * (s_coefficients_[segment_index][3] - s_coefficients_[segment_index][2]) * pow(1.0 - remain, 2) * pow(remain, 2) + 4.0 * (s_coefficients_[segment_index][4] - s_coefficients_[segment_index][3]) * (1.0 - remain) * pow(remain, 3) + (s_coefficients_[segment_index][5] - s_coefficients_[segment_index][4]) * pow(remain, 4)) / (ref_stamps_[segment_index + 1] - ref_stamps_[segment_index]);

    double a_value = 20.0 * ((s_coefficients_[segment_index][0] - 2.0 * s_coefficients_[segment_index][1] + s_coefficients_[segment_index][2]) * pow(1.0 - remain, 3) + 3.0 * (s_coefficients_[segment_index][1] - 2.0 * s_coefficients_[segment_index][2] + s_coefficients_[segment_index][3]) * pow(1.0 - remain, 2) * remain + 3.0 * (s_coefficients_[segment_index][2] - 2.0 * s_coefficients_[segment_index][3] + s_coefficients_[segment_index][4]) * (1.0 - remain) * pow(remain, 2) + (s_coefficients_[segment_index][3] - 2.0 * s_coefficients_[segment_index][4] + s_coefficients_[segment_index][5]) * pow(remain, 3)) / (ref_stamps_[segment_index + 1] - ref_stamps_[segment_index]);

    Eigen::Vector4d point{s_value, v_value, a_value, time_stamp};

    return point;
}





VelocityPlanner::VelocityPlanner(DecisionMaking::StandardState* current_state) {

    planning_state_ = current_state;
    if (!current_state->getCapability()) {
        return;
    }

    // find nearest point
    int current_position_index_in_trajectory = Tools::findNearestPositionIndexInCurve(current_state->getTotalTrajectory(), current_state->getVehicleStartState().position_, current_state->getVehicleCurrentPositionIndexInTrajectory());

    // Get vehicle current movement
    PathPlanningUtilities::VehicleMovementState vehicle_movement_state = current_state->getVehicleCurrentMovement();



    // Declare parameters
    StGraph::Param st_graph_param;
    st_graph_param.velocity_max = std::max(current_state->getVelocityLimitationMax(), vehicle_movement_state.velocity_);
    if (!planning_state_->last_planned_curve_.empty()) {
        // Get current position
        size_t nearest_index = Tools::findNearestPositionIndexInCurve(planning_state_->last_planned_curve_, current_state->getVehicleStartState().position_);

        double last_s = nearest_index * LANE_GAP_DISTANCE;
        int lower_index = std::lower_bound(planning_state_->s_.begin(), planning_state_->s_.end(), last_s) - planning_state_->s_.begin();

        // // DEBUG
        // std::cout << "DEBUGDEBUGDEBUG" << std::endl;
        // std::cout << "Lower index: " << lower_index << std::endl;
        // // END DEBUG

        if (lower_index < planning_state_->s_.size()) {
            st_graph_param.velocity_max = std::max(st_graph_param.velocity_max, planning_state_->v_[lower_index]);
        } 
    } 

    st_graph_param.s_max = std::max(st_graph_param.s_max, st_graph_param.t_max * st_graph_param.velocity_max);
    
    // Cut path segment
    PathPlanningUtilities::Curve total_curve = current_state->getTotalTrajectory();

    // // DEBUG
    // std::cout << "Curve length: " << total_curve.size() << std::endl;
    // // END DEBUG

    int total_length = total_curve.size();

    // // DEBUG
    // std::cout << "Total length: " << total_length << std::endl;
    // std::cout << "Current position index in trajectory: " << current_position_index_in_trajectory << std::endl;
    // // END DEBUG

    int cut_end_point = std::min(total_length - current_position_index_in_trajectory, static_cast<int>(st_graph_param.s_max / LANE_GAP_DISTANCE));

    // // DEBUG
    // std::cout << "Cut end point: " << cut_end_point << std::endl;
    // // END DEBUG

    // st_graph_param.s_max = (cut_end_point - current_position_index_in_trajectory) * LANE_GAP_DISTANCE;

    double real_planning_length = (std::min(total_curve.begin() + cut_end_point, total_curve.end()) - (total_curve.begin() + current_position_index_in_trajectory)) / LANE_GAP_DISTANCE;

    if (real_planning_length <= 10.0) {
        return;
    }

    PathPlanningUtilities::Curve velocity_planning_curve{total_curve.begin() + current_position_index_in_trajectory, std::min(total_curve.begin() + cut_end_point, total_curve.end())};


    // // DEBUG
    // std_msgs::ColorRGBA c;
    // c.a = 1.0;
    // c.r = 1.0;
    // c.g = 0.0;
    // c.b = 0.0;
    // visualization_msgs::Marker curve_marker = VisualizationMethods::visualizeCurvesToMarker(total_curve, c, 123456);
    // visualization_msgs::MarkerArray marker_array;
    // marker_array.markers.push_back(curve_marker);

    // pub.publish(marker_array);

    // // END DEBUG



    // Supply start state
    bool excess_limit = false;
    if (!planning_state_->last_planned_curve_.empty()) {
        // Attempt to replan from the the nearest point in the last planned path
        // Get current position
        size_t nearest_index = Tools::findNearestPositionIndexInCurve(planning_state_->last_planned_curve_, current_state->getVehicleStartState().position_);
        double last_s = nearest_index * LANE_GAP_DISTANCE;
        int lower_index = std::lower_bound(planning_state_->s_.begin(), planning_state_->s_.end(), last_s) - planning_state_->s_.begin();
        if (lower_index == planning_state_->s_.size()) {
            excess_limit = true;
        }

        // TODO: adjust and test the logic
        if (!excess_limit) {
            // Judge the difference between the current velocity and the corresponding velocity in the previous trajectory
            if ((planning_state_->v_[lower_index] - vehicle_movement_state.velocity_) * planning_state_->a_[lower_index] < 0.0) {
                excess_limit = true;
            }
            // if (fabs(planning_state_->v_[lower_index] - vehicle_movement_state.velocity_) > 1.5) {
            //     excess_limit = true;
            // }
        }

        // // DEBUG
        // excess_limit = true;
        // // END DEBUG

        if (!excess_limit) {

            // // DEBUG
            // std::cout << "Planning from the point in the previous trajectory" << std::endl;
            // std::cout << "v: " << planning_state_->v_[lower_index] << ", a: " << planning_state_->a_[lower_index] << std::endl;
            // // END DEBUG 

            printf("[VelocityPlanner] Planning from the point in the previous trajectory with v: %lf, a: %lf.\n", planning_state_->v_[lower_index], planning_state_->a_[lower_index]);

            start_state_ = {0.0, planning_state_->v_[lower_index], planning_state_->a_[lower_index]};
            // Construct s-t graph
            st_graph_ = new UncertaintyStGraph(velocity_planning_curve, st_graph_param, std::max(planning_state_->v_[lower_index], 2.0));
        }
    } 

    // // DEBUG
    // std::cout << "Excess limit: " << excess_limit << std::endl;
    // std::cout << "Last planned path length: " << planning_state_->last_planned_curve_.size() << std::endl;
    // std::cout << "s size: " << planning_state_->s_.size() << std::endl;
    // std::cout << "v size: " << planning_state_->v_.size() << std::endl;
    // std::cout << "a size: " << planning_state_->a_.size() << std::endl;

    // // END DEBUG

    if (planning_state_->last_planned_curve_.empty() || excess_limit) {

        // // DEBUG
        // std::cout << "Planning from current state" << std::endl;
        // std::cout << "v: " << vehicle_movement_state.velocity_ << ", a: " << vehicle_movement_state.acceleration_ << std::endl;
        // // END DEBUG 

        printf("[VelocityPlanner] Planning from current state with v: %lf, a: %lf.\n", vehicle_movement_state.velocity_, vehicle_movement_state.acceleration_);

        // Test limit the acceleration
        // Acceleration information from the IMU may include noise
        // vehicle_movement_state.acceleration_ = std::max(std::min(vehicle_movement_state.acceleration_, 1.6), -1.6);

        start_state_ = {0.0, vehicle_movement_state.velocity_, vehicle_movement_state.acceleration_};
        // Construct s-t graph
        st_graph_ = new UncertaintyStGraph(velocity_planning_curve, st_graph_param, std::max(vehicle_movement_state.velocity_, 2.0));
    }

    // // DEBUG
    // start_state_ = {0.0, vehicle_movement_state.velocity_, vehicle_movement_state.acceleration_};
    // // Construct s-t graph
    // st_graph_ = new StGraph(velocity_planning_curve, st_graph_param, vehicle_movement_state.velocity_);
    // // END DEBUG

    

}

VelocityPlanner::VelocityPlanner(DecisionMaking::StandardState* current_state, const ros::Publisher& st_graph_publisher) : VelocityPlanner(current_state) {
    python_visualization_ = true;
    st_graph_publisher_ = st_graph_publisher;
}

bool VelocityPlanner::runOnce(const std::vector<DecisionMaking::Obstacle>& obstacles) {
    
    if (!planning_state_->getCapability()) {
        return false;
    }
    if (st_graph_ == nullptr) {
        return false;
    }

    // ~Stage I: generate connected cubes paths
    std::vector<std::vector<Cube2D<double>>> cube_paths;
    std::vector<std::pair<double, double>> last_s_range;

    // // DEBUG
    // std::cout << "Obstacle number: " << obstacles.size() << std::endl; 
    // // END DEBUG

    // // Filter obstacles (for simulation only)
    // std::vector<DecisionMaking::Obstacle> valid_obstacles;
    // for (const auto& obs : obstacles) {
    //     if (sqrt(pow(planning_state_->getVehicleStartState().position_.x_ - obs.getObstaclePosition().x_, 2) + pow(planning_state_->getVehicleStartState().position_.y_ - obs.getObstaclePosition().y_, 2)) <= 50.0) {
    //         valid_obstacles.emplace_back(obs);
    //     }
    // }

    bool graph_success = st_graph_->generateInitialCubePath(obstacles, &cube_paths, &last_s_range);
    if (!graph_success) {
        planning_state_->setSafety(false);
        planning_state_->velocity_profile_generation_state_ = false;
        std::cout << "State name: " << planning_state_->getStateName() << " is not safe due to graph failure." << std::endl;
        return false;
    }

    // ~Stage II: safety enhancement
    std::vector<std::vector<Cube2D<double>>> enhanced_cube_paths;
    bool enhancement_success = st_graph_->enhanceSafety(cube_paths, &enhanced_cube_paths);
    if (!enhancement_success) {
        planning_state_->setSafety(false);
        planning_state_->velocity_profile_generation_state_ = false;
        std::cout << "State name: " << planning_state_->getStateName() << " is not safe due to safety enhancement failure." << std::endl;
        return false;
    }

    // // DEBUG
    // st_graph_->visualization();
    // // END DEBUG

    // ~Stage III: generate s-t parameters
    std::vector<double> s;
    std::vector<double> t;

    // std::cout << "State name: " << planning_state_->getStateName() << std::endl;
    // std::cout << "Max velocity: " << planning_state_->getVelocityLimitationMax() << ", min velocity: " << planning_state_->getVelocityLimitationMin() << std::endl;
    // std::cout << "Max acceleration: " << planning_state_->getAccelerationLimitationMax() << ", min acceleration: " << planning_state_->getAccelerationLimitationMin() << std::endl;

    printf("[VelocityPlanner] Planned state name: %s.\n", DIC_STATE_NAME[planning_state_->getStateName()].c_str());
    printf("[VelocityPlanner] Velocity range: %lf - %lf, acceleration range: %lf - %lf.\n", planning_state_->getVelocityLimitationMax(), planning_state_->getVelocityLimitationMin(), planning_state_->getAccelerationLimitationMax(), planning_state_->getAccelerationLimitationMin());

    velocity_optimizer_ = new VelocityPlanning::VelocityOptimizer();
    bool optimization_success = velocity_optimizer_->runOnce(enhanced_cube_paths, start_state_, last_s_range, planning_state_->getVelocityLimitationMax(), planning_state_->getVelocityLimitationMin(), planning_state_->getAccelerationLimitationMax(), planning_state_->getAccelerationLimitationMin(), &s, &t);

    if (optimization_success) {
        // std::cout << "Velocity profile generation success." << std::endl;
        // std::cout << "Final selected s: " << s.back() << std::endl;
        printf("[VelocityPlanner] %s velocity profile generation success.\n", DIC_STATE_NAME[planning_state_->getStateName()].c_str());
        printf("[VelocityPlanner] Final selected s: %lf, average velocity: %lf.\n", s.back(), s.back() / st_graph_->param_.t_max);
    } 

    if (!optimization_success) {
        planning_state_->setSafety(false);
        planning_state_->velocity_profile_generation_state_ = false;
        std::cout << "State name: " << planning_state_->getStateName() << " is not safe due to optimization failure." << std::endl;
        return false;
    }

    // ~Stage IV: generate s-t profile
    bezier_curve_traj_generator_ = new BezierPiecewiseCurve(s, t);
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> profile = bezier_curve_traj_generator_->generateTraj();

    // // DEBUG
    // std::vector<double> ss, vv, tt;
    // ss = std::get<0>(profile);
    // vv = std::get<1>(profile);
    // tt = std::get<3>(profile);
    // std::cout << "Planned trajectory" << std::endl;
    // std::cout << "s: " << std::endl;
    // for (int i = 0; i < ss.size(); i++) {
    //     std::cout << ss[i] << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << "v: " << std::endl;
    // for (int i = 0; i < vv.size(); i++) {
    //     std::cout << vv[i] << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << "t: " << std::endl;
    // for (int i = 0; i < tt.size(); i++) {
    //     std::cout << tt[i] << ", ";
    // }
    // std::cout << std::endl;
    // // END DEBUG

    // ~Stage V: supply s-t profile to the standard state
    planning_state_->loadStProfile(std::get<0>(profile), std::get<1>(profile), std::get<2>(profile));
    planning_state_->velocity_profile_generation_state_ = true;

    // ~Stage VI: publish information to python visualization interface
    if (python_visualization_) {
        // Initialize published msg
        data_visualization_msg::StGraph st_graph_msg;

        // Supply veliocity results
        data_visualization_msg::VelocityInfo velocity_res_info_msg;
        velocity_res_info_msg.s = std::get<0>(profile);
        velocity_res_info_msg.v = std::get<1>(profile);
        velocity_res_info_msg.a = std::get<2>(profile);
        velocity_res_info_msg.t = std::get<3>(profile);
        st_graph_msg.velocity_info = velocity_res_info_msg;

        // Supply initial cubes paths
        data_visualization_msg::CubesPaths initial_cubes_paths_msg;
        for (int i = 0; i < cube_paths.size(); i++) {
            data_visualization_msg::Cubes single_cubes_path_msg;
            for (int j = 0; j < cube_paths[i].size(); j++) {
                data_visualization_msg::Cube cube_msg;
                cube_msg.t_start = cube_paths[i][j].t_start_;
                cube_msg.t_end = cube_paths[i][j].t_end_;
                cube_msg.s_start = cube_paths[i][j].s_start_;
                cube_msg.s_end = cube_paths[i][j].s_end_;
                single_cubes_path_msg.cubes.emplace_back(cube_msg);
            }
            initial_cubes_paths_msg.cubes_paths.emplace_back(single_cubes_path_msg);
        }
        st_graph_msg.initial_corridors = initial_cubes_paths_msg;

        // Supply enhanced cubes paths
        data_visualization_msg::CubesPaths enhanced_cubes_paths_msg;
        for (int i = 0; i < enhanced_cube_paths.size(); i++) {
            data_visualization_msg::Cubes single_cubes_path_msg;
            for (int j = 0; j < enhanced_cube_paths[i].size(); j++) {
                data_visualization_msg::Cube cube_msg;
                cube_msg.t_start = enhanced_cube_paths[i][j].t_start_;
                cube_msg.t_end = enhanced_cube_paths[i][j].t_end_;
                cube_msg.s_start = enhanced_cube_paths[i][j].s_start_;
                cube_msg.s_end = enhanced_cube_paths[i][j].s_end_;
                single_cubes_path_msg.cubes.emplace_back(cube_msg);
            }
            enhanced_cubes_paths_msg.cubes_paths.emplace_back(single_cubes_path_msg);
        }
        st_graph_msg.enhanced_corridors = enhanced_cubes_paths_msg;

        // Supply occupied parallelograms
        for (const auto& uncer_occ_area : st_graph_->uncertainty_occupied_areas_) {
            data_visualization_msg::Parallelogram cur_parallelogram_msg;
            cur_parallelogram_msg.vertex.resize(4);
            for (int i = 0; i < 4; i++) {
                cur_parallelogram_msg.vertex[i].t = uncer_occ_area.vertex_[i](0);
                cur_parallelogram_msg.vertex[i].s = uncer_occ_area.vertex_[i](1);
            }
            st_graph_msg.occupied_areas.emplace_back(cur_parallelogram_msg);
        }

        // Supply state name 
        st_graph_msg.state_name = DIC_STATE_NAME[planning_state_->getStateName()];

        st_graph_publisher_.publish(st_graph_msg);

    }


    return true;


    

    
}




VelocityPlanner::~VelocityPlanner() = default;


} // End of namespace VelocityPlanning