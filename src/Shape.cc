/*
 * @Author: fujiawei0724
 * @Date: 2022-09-13 15:55:25
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-13 16:17:55
 * @Description: description of shapes and its functions
 */

#include "Common.hpp"

Parallelogram::Parallelogram() = default;

Parallelogram::Parallelogram(const std::vector<Eigen::Vector2d>& vertex) {
    vertex_ = vertex;
}

Parallelogram::~Parallelogram() = default;

Eigen::Vector2d& Parallelogram::operator[] (int i) {
    return vertex_[i];
}

double Parallelogram::maxS() {
    return std::max({vertex_[0](1), vertex_[1](1), vertex_[2](1), vertex_[3](1)});
}

double Parallelogram::minS() {
    return std::min({vertex_[0](1), vertex_[1](1), vertex_[2](1), vertex_[3](1)});
}

std::pair<double, double> Parallelogram::calculateS(const double& t) {
    if (t < minT() || t > maxT()) {
        printf("[Parallelogram] illegal t input!!!\n");
        assert(false);
    }

    

}


bool ShapeUtils::judgeLineWithPolynomial(const double& line_s, const double& t_start, const double& t_end, const std::vector<Eigen::Vector2d>& polynomial_vertex, double* nearest_t_in_line, Eigen::Vector2d& nearest_vertice_in_polynomial) {
    // Judge whether relative positions in the t dimension
    if (polynomial_vertex[0](0) > t_end) {
        // Polynomial is on the right direction of line 
        // Record t
        *nearest_t_in_line = t_end;

        // Judge relative positions in the s dimension
        if (polynomial_vertex[0](1) > line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[0];
        } else if (polynomial_vertex[1](1) < line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[1];
        } else {
            nearest_vertice_in_polynomial = {polynomial_vertex[0](0), line_s};
        }

    } else if (polynomial_vertex[2](0) < t_start) {
        // Polynomial is on the left direction of line
        // Record t
        *nearest_t_in_line = t_start;

        // Judge relative positions in the s dimension
        if (polynomial_vertex[3](1) > line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[3];
        } else if (polynomial_vertex[2](1) < line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[2];
        } else {
            nearest_vertice_in_polynomial = {polynomial_vertex[2](0), line_s};
        }

    } else {
        // Polynomial and line has some overlapped ranges in the t dimension
        // Subdivide relative positions in t dimension
        if (polynomial_vertex[0](0) > t_start && polynomial_vertex[2](0) < t_end) {
            double max_s = std::max({polynomial_vertex[0](1), polynomial_vertex[1](1), polynomial_vertex[2](1), polynomial_vertex[3](1)});
            double min_s = std::min({polynomial_vertex[0](1), polynomial_vertex[1](1), polynomial_vertex[2](1), polynomial_vertex[3](1)});
            if (min_s >= line_s) {
                // if (fabs)
            } else if (max_s <= line_s) {

            } else {
                return false;
            }

        } else if (polynomial_vertex[0](0) < t_start && polynomial_vertex[2](0) > t_end) {

        } else if (polynomial_vertex[0](0) < t_end && polynomial_vertex[2](0) > t_end) {

        } else if (polynomial_vertex[0](0) < t_start && polynomial_vertex[2](0) > t_start) {

        } else {
            printf("[ShapeUtils] Unknwon relative positions situations!!!\n");
            assert(false);
        }

    }

    return true;
}

