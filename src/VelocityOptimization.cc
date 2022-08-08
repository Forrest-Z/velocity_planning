/*
 * @Author: fujiawei0724
 * @Date: 2022-08-04 14:14:24
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-08-08 17:06:31
 * @Description: velocity optimization.
 */

#include "Common.hpp"


namespace VelocityPlanning {

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
void OoqpOptimizationInterface::load(const std::vector<double>& ref_stamps, const std::array<double, 3>& start_constraints, const std::array<double, 3>& end_constraints, std::array<std::vector<double>, 2>& unequal_constraints, std::vector<std::vector<double>>& equal_constraints) {
    ref_stamps_ = ref_stamps;
    start_constraints_ = start_constraints;
    end_constraints_ = end_constraints;
    unequal_constraints_ = unequal_constraints;
    equal_constraints_ = equal_constraints;
}

/**
 * @brief Run optimization
 * @param {*}
 * @return {*}
 */    
bool OoqpOptimizationInterface::runOnce(std::vector<double>* optimized_s) {

    // // Multi thread calculation
    // // TODO: add logic to handle the situation where the optimization process is failed
    // clock_t single_dim_optimization_start_time = clock();
    // std::thread s_thread(&OoqpOptimizationInterface::optimizeSingleDim, this, s_start_constraints, s_end_constraints, s_unequal_constraints[0], s_unequal_constraints[1], "s");
    // std::thread d_thread(&OoqpOptimizationInterface::optimizeSingleDim, this, d_start_constraints, d_end_constraints, d_unequal_constraints[0], d_unequal_constraints[1], "d");
    // s_thread.join();
    // d_thread.join();
    // clock_t single_dim_optimization_end_time = clock();


    optimizeSingleDim(start_constraints_, end_constraints_, unequal_constraints_[0], unequal_constraints_[1]);
    
    *optimized_s = optimized_data_;
    return optimization_res_;
}

/**
 * @brief Optimize in single dimension
 * @param {*}
 */
void OoqpOptimizationInterface::optimizeSingleDim(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<double>& single_lower_boundaries, const std::vector<double>& single_upper_boundaries) {
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
    calculateAbMatrix(single_start_constraints, single_end_constraints, equal_constraints_, A, b);

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

    // ~Stage IV: optimization
    std::vector<double> optimized_values;
    bool optimization_status = solve(Q, c, A, b, useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX, &optimized_values);

    // ~Stage V: store information
    optimized_data_ = optimized_values;
    optimization_res_ = optimization_status;
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

/**
 * @brief Calculate equal constraints, note that position constraints in the connection don't need to be considered
 * @param {*}
 */
void OoqpOptimizationInterface::calculateAbMatrix(const std::array<double, 3>& single_start_constraints, const std::array<double, 3>& single_end_constraints, const std::vector<std::vector<double>>& equal_constraints, Eigen::SparseMatrix<double, Eigen::RowMajor>& A, Eigen::VectorXd& b) {
            
    // Calculate dimensions and initialize
    int variables_num = (static_cast<int>(ref_stamps_.size()) - 1) * 6;
    int equal_constraints_num = 6 + (static_cast<int>(ref_stamps_.size()) - 2) * 3;
    double start_cube_time_span = ref_stamps_[1] - ref_stamps_[0];
    double end_cube_time_span = ref_stamps_[ref_stamps_.size() - 1] - ref_stamps_[ref_stamps_.size() - 2];
    Eigen::MatrixXd A_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, variables_num);
    Eigen::MatrixXd b_matrix = Eigen::MatrixXd::Zero(equal_constraints_num, 1);

    // supply start point and end point position constraints 
    A_matrix(0, 0) = 1.0, A_matrix(1, variables_num - 1) = 1.0;
    b_matrix(0, 0) = single_start_constraints[0], b_matrix(1, 0) = single_end_constraints[0];

    // supply start point and end point velocity constraints
    A_matrix(2, 0) = -5.0, A_matrix(2, 1) = 5.0;
    b_matrix(2, 0) = single_start_constraints[1] * start_cube_time_span;
    A_matrix(3, variables_num - 2) = -5.0, A_matrix(3, variables_num - 1) = 5.0;
    b_matrix(3, 0) = single_end_constraints[1] * end_cube_time_span;

    // supply start point and end point acceleration constraints
    A_matrix(4, 0) = 20.0, A_matrix(4, 1) = -40.0, A_matrix(4, 2) = 20.0;
    b_matrix(4, 0) = single_start_constraints[2] * start_cube_time_span;
    A_matrix(5, variables_num - 3) = 20.0, A_matrix(5, variables_num - 2) = -40.0, A_matrix(5, variables_num - 1) = 20.0;
    b_matrix(5, 0) = single_end_constraints[2] * end_cube_time_span;

    // supply continuity ensurance constraints
    for (int i = 0; i < static_cast<int>(equal_constraints.size()); i++) {
        int constraint_index = i + 6;
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
                Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimitForX, Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimitForX, 
                Eigen::VectorXd& lowerLimitForX, Eigen::VectorXd& upperLimitForX, std::vector<double>* optimized_values) {
    
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

    Eigen::VectorXd x;
    int nx = Q.rows();
    x.setZero(nx);
    // Make copies of variables that are changed
    auto ccopy(c);
    auto Acopy(A);
    auto bcopy(b);

    Eigen::SparseMatrix<double, Eigen::RowMajor> Q_triangular = Q.triangularView<Eigen::Lower>();
    
    // Compress sparse Eigen matrices (refer to Eigen Sparse Matrix user manual)
    Q_triangular.makeCompressed();
    Acopy.makeCompressed();

    // Initialize new problem formulation.
    int my = bcopy.size();
    int mz = 0; // TODO: check this parameter, unequal constraint , set with 0
    int nnzQ = Q_triangular.nonZeros();
    // int nnzA = Acopy.nonZeros();
    int nnzA = Acopy.nonZeros();
    int nnzC = 0; // Unequal constraint , set with 0

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
    
    int krowC[] = {};
    int jcolC[] = {};
    double dC[] = {};
    double clow[] = {};
    char iclow[] = {};
    double cupp[] = {};
    char icupp[] = {};

    QpGenData *prob = (QpGenData *)qp->copyDataFromSparseTriple(cp, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, irowA, nnzA, jcolA, dA, bA, krowC, nnzC, jcolC, dC, clow, iclow, cupp, icupp);
    
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

    if (status == TerminationCode::SUCCESSFUL_TERMINATION) {
        vars->x->copyIntoArray(&x.coeffRef(0));
        *optimized_values = std::vector<double>(x.data(), x.data() + x.rows() * x.cols());
    }

    delete s;
    delete resid;
    delete vars;
    delete prob;
    delete qp;

    return status == TerminationCode::SUCCESSFUL_TERMINATION;

}

VelocityOptimizer::VelocityOptimizer() {
    ooqp_itf_ = new OoqpOptimizationInterface();
}

VelocityOptimizer::~VelocityOptimizer() = default;

bool VelocityOptimizer::runOnce(const std::vector<std::vector<Cube2D<double>>>& cube_paths, const std::array<double, 3>& start_state, std::vector<double>* s, std::vector<double>* t) {
    // Initialize container
    int n = cube_paths.size();
    ss_.resize(n);
    tt_.resize(n);
    ress_.resize(n);

    // // Construct threads
    // // TODO: check the reason why the result from single thread and multiple threads are different
    // std::vector<std::thread> threads(n);
    // for (int i = 0; i < n; i++) {
    //     threads[i] = std::thread(&VelocityOptimizer::runSingleCubesPath, this, cube_paths[i], start_state, i);
    // }
    // for (int i = 0; i < n; i++) {
    //     threads[i].join();
    // }

    // DEBUG
    for (int i = 0; i < n; i++) {
        runSingleCubesPath(cube_paths[i], start_state, i);
    }
    // END DEBUG

    // Evaluate and get result
    int cur_max_final_s = INT_MIN;
    bool final_optimization_res = false;
    std::vector<double> calculated_s;
    std::vector<double> calculated_t;
    for (int i = 0; i < n; i++) {
        if (ress_[i]) {
            final_optimization_res = true;
            if (ss_[i].back() > cur_max_final_s) {
                cur_max_final_s = ss_[i].back();
                calculated_s = ss_[i];
                calculated_t = tt_[i];
            }
            
            // DEBUG
            std::cout << "i: " << i << std::endl;
            std::cout << "t: " << std::endl;
            for (int j = 0; j < tt_[i].size(); j++) {
                std::cout << tt_[i][j] << ", ";
            }
            std::cout << std::endl;
            for (int j = 0; j < ss_[i].size(); j++) {
                std::cout << ss_[i][j] << ", ";
            }
            std::cout << std::endl;

            // END DEBUG
        }
    }

    *s = calculated_s;
    *t = calculated_t;

    return final_optimization_res;


}

void VelocityOptimizer::runSingleCubesPath(const std::vector<Cube2D<double>>& cube_path, const std::array<double, 3>& start_state, int index) {
    // Estimate the end state constraint
    std::array<double, 3> end_state = {(cube_path.back().s_start_ + cube_path.back().s_end_) / 2.0, 0.0, 0.0};

    // ~Stage I: calculate time stamps of split point (include start point and end point)
    std::vector<double> ref_stamps;
    for (int i = 0; i < cube_path.size(); i++) {
        if (i == 0) {
            ref_stamps.emplace_back(cube_path[i].t_start_);
        }
        ref_stamps.emplace_back(cube_path[i].t_end_);
    }

    // ~Stage II: calculate unequal constraints for variables (except start point and end point)
    std::array<std::vector<double>, 2> unequal_constraints = generateUnequalConstraints(cube_path);

    // DEBUG
    std::cout << "Index: " << index << std::endl;
    std::cout << "Unequal constraints: " << std::endl;
    for (int i = 0; i < unequal_constraints[0].size(); i++) {
        std::cout << "Lower: " << unequal_constraints[0][i] << ", upper: " << unequal_constraints[1][i] << std::endl;
    }
    // END DEBUG

    // // DEBUG
    // std::cout << "unequal constraints calculation success" << std::endl;
    // // END DEBUG

    // ~Stage III: calculate equal constraints to ensure the continuity
    std::vector<std::vector<double>> equal_constraints = generateEqualConstraints(cube_path);

    // // DEBUG
    // std::cout << "equal constraints calculation success" << std::endl;
    // // END DEBUG

    // ~Stage IV: optimization
    std::vector<double> optimized_s;
    ooqp_itf_->load(ref_stamps, start_state, end_state, unequal_constraints, equal_constraints);
    bool optimization_res = ooqp_itf_->runOnce(&optimized_s);

    // // DEBUG
    // std::cout << "optimization res: " << optimization_res << std::endl;
    // // END DEBUG

    ss_[index] = optimized_s;
    tt_[index] = ref_stamps;
    ress_[index] = optimization_res;
    
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
        if (i == 0 || i == variables_num - 1) {
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
std::vector<Eigen::Vector2d> BezierPiecewiseCurve::generateTraj(double sample_gap) {
    // Initialize
    std::vector<Eigen::Vector2d> traj;

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
            traj.push_back(generatePoint(segment_index, current_seed, time_stamp));
        }
    }

    return traj;
    
}

/**
 * @brief Calculate single point 
 * @param {*}
 * @return {*}
 */    
Eigen::Vector2d BezierPiecewiseCurve::generatePoint(int segment_index, double remain, double time_stamp) {
    // Calculate s and d value
    double s_value = s_coefficients_[segment_index][0] * pow(1.0 - remain, 5) + 5.0 * s_coefficients_[segment_index][1] * remain * pow(1.0 - remain, 4) + 10.0 * s_coefficients_[segment_index][2] * pow(remain, 2) * pow(1.0 - remain, 3) + 10.0 * s_coefficients_[segment_index][3] * pow(remain, 3) * pow(1.0 - remain, 2) + 5.0 * s_coefficients_[segment_index][4] * pow(remain, 4) * (1.0 - remain) + s_coefficients_[segment_index][5] * pow(remain, 5);

    Eigen::Vector2d point{s_value, time_stamp};

    return point;
}





VelocityPlanner::VelocityPlanner(DecisionMaking::StandardState* current_state) {
    planning_state_ = current_state;

    // find nearest point
    int current_position_index_in_trajectory = Tools::findNearestPositionIndexInCurve(current_state->getTotalTrajectory(), current_state->getVehicleStartState().position_, current_state->getVehicleCurrentPositionIndexInTrajectory());

    // Declare parameters
    StGraph::Param st_graph_param;
    st_graph_param.velocity_max = current_state->getVelocityLimitationMax();
    
    // Cut path segment
    PathPlanningUtilities::Curve total_curve = current_state->getTotalTrajectory();
    int total_length = current_state->getTrajectoryLength();
    int cut_end_point = std::min(total_length - current_position_index_in_trajectory, static_cast<int>(st_graph_param.s_max / LANE_GAP_DISTANCE));
    st_graph_param.s_max = (cut_end_point - current_position_index_in_trajectory) * LANE_GAP_DISTANCE;
    PathPlanningUtilities::Curve velocity_planning_curve{total_curve.begin() + current_position_index_in_trajectory, total_curve.begin() + cut_end_point};

    // Get vehicle current movement
    PathPlanningUtilities::VehicleMovementState vehicle_movement_state = current_state->getVehicleCurrentMovement();

    // Construct s-t graph
    st_graph_ = new StGraph(velocity_planning_curve, st_graph_param, vehicle_movement_state.velocity_);

    // Supply start state
    start_state_ = {0.0, vehicle_movement_state.velocity_, vehicle_movement_state.acceleration_};

}

bool VelocityPlanner::runOnce(const std::vector<DecisionMaking::Obstacle>& obstacles) {
    // ~Stage I: generate connected cubes paths
    std::vector<std::vector<Cube2D<double>>> cube_paths;
    st_graph_->runOnce(obstacles, &cube_paths);

    // ~Stage II: generate s-t parameters
    std::vector<double> s;
    std::vector<double> t;
    velocity_optimizer_->runOnce(cube_paths, start_state_, &s, &t);

    // ~Stage III: generate s-t profile
    bezier_curve_traj_generator_ = new BezierPiecewiseCurve(s, t);

    // ~Stage IV: supply s-t profile to the standard state
    

    
}




VelocityPlanner::~VelocityPlanner() = default;


} // End of namespace VelocityPlanning