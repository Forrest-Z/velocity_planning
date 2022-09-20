/*
 * @Author: fujiawei0724
 * @Date: 2022-09-13 15:55:25
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-20 10:02:35
 * @Description: description of shapes and its functions
 */

#include "Common.hpp"

Parallelogram::Parallelogram() = default;

Parallelogram::Parallelogram(const std::vector<Eigen::Vector2d>& vertex) {
    vertex_ = vertex;
}

Parallelogram::~Parallelogram() = default;

const Eigen::Vector2d& Parallelogram::operator[] (int i) const {
    return vertex_[i];
}

double Parallelogram::maxS() const {
    return std::max({vertex_[0](1), vertex_[1](1), vertex_[2](1), vertex_[3](1)});
}

double Parallelogram::minS() const {
    return std::min({vertex_[0](1), vertex_[1](1), vertex_[2](1), vertex_[3](1)});
}

std::pair<double, double> Parallelogram::calculateS(const double& t) const {
    if (t < minT() || t > maxT()) {
        printf("[Parallelogram] illegal t input!!!\n");
        assert(false);
    }

    // Calculate ratio in the t dimension
    double t_span = maxT() - minT();
    double cur_relative_t = t - minT();
    double ratio = cur_relative_t / t_span;

    // Calculate larger value
    double larger_value = vertex_[1](1) + ratio * (vertex_[2](1) - vertex_[1](1));
    // Calculate smaller value
    double smaller_value = vertex_[0](1) + ratio * (vertex_[3](1) - vertex_[0](1));

    return std::make_pair(larger_value, smaller_value);

}

Parallelogram Parallelogram::calculateTruncatedParallelogram(const double& t_start, const double& t_end) const {
    // Calculate the two points given t_start
    std::pair<double, double> t_start_ss = calculateS(t_start);

    // Calculate the two points given t_end
    std::pair<double, double> t_end_ss = calculateS(t_end);

    // Calculate the vertex of the new parallelogram via the specific order
    std::vector<Eigen::Vector2d> vertex = {{t_start, t_start_ss.second}, {t_start, t_start_ss.first}, {t_end, t_end_ss.first}, {t_end, t_end_ss.second}};

    return Parallelogram(vertex);
}



bool ShapeUtils::judgeLineWithPolynomial(const double& line_s, const double& t_start, const double& t_end, const Parallelogram& polynomial_vertex, const double& tolerance, double* nearest_t_in_line, Eigen::Vector2d& nearest_vertice_in_polynomial, RelativePositionType* relative_pos) {
    // Judge whether relative positions in the t dimension
    if (polynomial_vertex[0](0) > t_end) {
        // Polynomial is on the right direction of line 
        // Record t
        *nearest_t_in_line = t_end;

        // Judge relative positions in the s dimension
        if (polynomial_vertex[0](1) > line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[0];
            *relative_pos = RelativePositionType::ABOVE;
        } else if (polynomial_vertex[1](1) < line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[1];
            *relative_pos = RelativePositionType::BELOW;
        } else {
            nearest_vertice_in_polynomial = {polynomial_vertex[0](0), line_s};
            *relative_pos = RelativePositionType::IGNORED;
        }

    } else if (polynomial_vertex[2](0) < t_start) {
        // Polynomial is on the left direction of line
        // Record t
        *nearest_t_in_line = t_start;

        // Judge relative positions in the s dimension
        if (polynomial_vertex[3](1) > line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[3];
            *relative_pos = RelativePositionType::ABOVE;
        } else if (polynomial_vertex[2](1) < line_s) {
            nearest_vertice_in_polynomial = polynomial_vertex[2];
            *relative_pos = RelativePositionType::BELOW;
        } else {
            nearest_vertice_in_polynomial = {polynomial_vertex[2](0), line_s};
            *relative_pos = RelativePositionType::IGNORED;
        }

    } else {
        // Polynomial and line has some overlapped ranges in the t dimension
        // Subdivide relative positions in t dimension
        Parallelogram valid_parallelogram;
        if (polynomial_vertex[0](0) >= t_start && polynomial_vertex[2](0) <= t_end) {
            valid_parallelogram = polynomial_vertex;
        } else if (polynomial_vertex[0](0) < t_start && polynomial_vertex[2](0) > t_end) {
            valid_parallelogram = polynomial_vertex.calculateTruncatedParallelogram(t_start, t_end);
        } else if (polynomial_vertex[0](0) < t_end && polynomial_vertex[2](0) > t_end) {
            valid_parallelogram = polynomial_vertex.calculateTruncatedParallelogram(polynomial_vertex[0](0), t_end);
        } else if (polynomial_vertex[0](0) < t_start && polynomial_vertex[2](0) > t_start) {
            valid_parallelogram = polynomial_vertex.calculateTruncatedParallelogram(t_start, polynomial_vertex[2](0));
        } else {
            printf("[ShapeUtils] Unknown relative positions situations!!!\n");
            assert(false);
        }

        // Follow the original parallelogram
        double max_s = valid_parallelogram.maxS();
        double min_s = valid_parallelogram.minS();
        if (min_s + tolerance >= line_s) {
            if (fabs(valid_parallelogram[0](1) - line_s) <= fabs(valid_parallelogram[3](1) - line_s)) {
                *nearest_t_in_line = valid_parallelogram[0](0);
                nearest_vertice_in_polynomial = valid_parallelogram[0];
            } else {
                *nearest_t_in_line = valid_parallelogram[3](0);
                nearest_vertice_in_polynomial = valid_parallelogram[3];
            }
            *relative_pos = RelativePositionType::ABOVE;
            
        } else if (max_s - tolerance <= line_s) {
            if (fabs(line_s - valid_parallelogram[1](1)) <= fabs(line_s - valid_parallelogram[2](1))) {
                *nearest_t_in_line = valid_parallelogram[1](0);
                nearest_vertice_in_polynomial = valid_parallelogram[1];
            } else {
                *nearest_t_in_line = valid_parallelogram[2](0);
                nearest_vertice_in_polynomial = valid_parallelogram[2];
            }
            *relative_pos = RelativePositionType::BELOW;

        } else {

            printf("[ShapeUtils] Error collision!!!\n");
            assert(false);
            return false;
        }

    }

    return true;
}

Eigen::Matrix2d CoordinateUtils::getRotationMatrix(const double& theta) {
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), 
                       -sin(theta), cos(theta);
    return rotation_matrix;
}

Eigen::Matrix2d CoordinateUtils::getScaleMatrix(const double& scale_1, const double& scale_2) {
    Eigen::Matrix2d scale_matrix;
    scale_matrix << scale_1, 0.0, 
                    0.0, scale_2;
    return scale_matrix;
}


