/*
 * @Author: fujiawei0724
 * @Date: 2022-09-13 15:52:18
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-13 16:16:15
 * @Description: description of parallelogram
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "Const.hpp"

class Parallelogram {
 public:
    Parallelogram();
    Parallelogram(const std::vector<Eigen::Vector2d>& vertex);
    ~Parallelogram();

    Eigen::Vector2d& operator[] (int i);

    double maxS();

    double minS();

    inline double maxT() {
        return vertex_[0](0);
    }

    inline double minT() {
        return vertex_[2](0);
    }

    std::pair<double, double> calculateS(const double& t);

    std::vector<Eigen::Vector2d> vertex_;
};


class ShapeUtils {
 public:

    /**
     * @description: calculate the positions relation between a line (paralled with t axis) and a polynomial (parallelogram)
     * Note that this method is an approximate method, it is possible that the result is not strictly correct
     * @param {double&} line_s: s scale of the line
     * @param {double&} t_start: start time of the line
     * @param {double&} t_end: end time of the line
     * @param {double*} nearest_t_in_line: the nearest point's t in the line
     * @param {Vector2d&} nearest_vertice_in_polynomial: the nearest point of the polynomial
     * @return {*} is calculation successful, if there is a collision, the calculation will be failed
     */
    static bool judgeLineWithPolynomial(const double& line_s, const double& t_start, const double& t_end, const std::vector<Eigen::Vector2d>& polynomial_vertex, double* nearest_t_in_line, Eigen::Vector2d& nearest_vertice_in_polynomial);
};