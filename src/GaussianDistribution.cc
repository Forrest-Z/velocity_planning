/*
 * @Author: fujiawei0724
 * @Date: 2022-09-13 09:29:45
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-14 19:12:40
 * @Description: gaussian distribution
 */

#include "Common.hpp"

double GaussianUtils::calculateDistributionProbability(const Gaussian2D& gaussian_dis_2d, const DimensionType& dimension_type, const double& start_value, const double& end_value) {
    // Calculate edge distribution 
    Gaussian1D edge_dis = gaussian_dis_2d.edgeDistribution(static_cast<int>(dimension_type));
 
    // Transform the average value of the edge distribution to zero
    double gap = edge_dis.ave_values_(0, 0);
    double transformed_start_value = start_value - gap;
    double transformed_end_value = end_value - gap;

    // Get variance
    double variance = edge_dis.covariance_(0, 0);

    // Search distribution probability
    double end_prob = LookUpTable::GaussianIntegral::calculate(variance, end_value);
    double start_prob = LookUpTable::GaussianIntegral::calculate(variance, start_value);
    double res_prob = end_prob - start_prob;

    assert(res_prob > 0.0);

    return res_prob;
}

void transformGaussian2DTo1D(const Gaussian2D& input_gaussian_dis_2d, const DimensionType& target_dimension_type, const double& start_value, const double& end_value, Gaussian1D* candi_dis_start, Gaussian1D* candi_dis_end) {
    // Get variance
    double variance = input_gaussian_dis_2d.covariance_(static_cast<int>(target_dimension_type), static_cast<int>(target_dimension_type));

    DimensionType other_dimension_type = DimensionType::UNKNOWN;
    if (target_dimension_type == DimensionType::S) {
        other_dimension_type == DimensionType::T;
    } else {
        other_dimension_type == DimensionType::S;
    }

    // Get two average values for the start value and end point
    double correlation = input_gaussian_dis_2d.getCorrelation();
    double start_position_average_value = input_gaussian_dis_2d.ave_values_(static_cast<int>(target_dimension_type), 0) + correlation * ((start_value - input_gaussian_dis_2d.ave_values_(static_cast<int>(other_dimension_type), 0)) / sqrt(input_gaussian_dis_2d.covariance_(static_cast<int>(other_dimension_type), static_cast<int>(other_dimension_type)))) * sqrt(input_gaussian_dis_2d.covariance_(static_cast<int>(target_dimension_type), static_cast<int>(target_dimension_type)));
    double end_position_average_value = input_gaussian_dis_2d.ave_values_(static_cast<int>(target_dimension_type), 0) + correlation * ((end_value - input_gaussian_dis_2d.ave_values_(static_cast<int>(other_dimension_type), 0)) / sqrt(input_gaussian_dis_2d.covariance_(static_cast<int>(other_dimension_type), static_cast<int>(other_dimension_type)))) * sqrt(input_gaussian_dis_2d.covariance_(static_cast<int>(target_dimension_type), static_cast<int>(target_dimension_type)));

    // Supply results
    Eigen::Matrix<double, 1, 1> covariance_matrix{variance};
    Eigen::Matrix<double, 1, 1> start_position_average_value_matrix{start_position_average_value};
    Eigen::Matrix<double, 1, 1> end_position_average_value_matrix{end_position_average_value};

    candi_dis_start->ave_values_ = start_position_average_value_matrix;
    candi_dis_start->covariance_ = covariance_matrix;
    candi_dis_end->ave_values_ = end_position_average_value_matrix;
    candi_dis_end->covariance_ = covariance_matrix;
    
}