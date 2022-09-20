/*
 * @Author: fujiawei0724
 * @Date: 2022-09-13 15:52:18
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-20 14:47:56
 * @Description: description of parallelogram
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "Const.hpp"

enum class SRelativePositionType {
    ABOVE = 0,
    BELOW = 1,
    IGNORED = 2,
    UNKNOWN = 3,
};

enum class TRelativePositionType {
    LEFT = 0,
    RIGHT = 1,
    OVERLAPPED = 2,
    UNKNOWN = 3,
};

enum class BoundType {
    UPPER = 0,
    LOWER = 1,
    UNKNOWN = 2,
};

enum class DimensionType {
    T = 0,
    S = 1,
    UNKNOWN = 2,
};

class Parallelogram {
 public:
    Parallelogram();
    Parallelogram(const std::vector<Eigen::Vector2d>& vertex);
    ~Parallelogram();

    const Eigen::Vector2d& operator[] (int i) const;

    double maxS() const;

    double minS() const;

    inline double maxT() const {
        return vertex_[2](0);
    }

    inline double minT() const {
        return vertex_[1](0);
    }

    std::pair<double, double> calculateS(const double& t) const;

    Parallelogram calculateTruncatedParallelogram(const double& t_start, const double& t_end) const;

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
    static bool judgeLineWithPolynomial(const double& line_s, const double& t_start, const double& t_end, const Parallelogram& polynomial_vertex, const double& tolerance, double* nearest_t_in_line, Eigen::Vector2d& nearest_vertice_in_polynomial, SRelativePositionType* relative_pos, TRelativePositionType* t_relative_pos);
};

class CoordinateUtils {
 public:
    static Eigen::Matrix2d getRotationMatrix(const double& theta);

    static Eigen::Matrix2d getScaleMatrix(const double& scale_1, const double& scale_2);
};