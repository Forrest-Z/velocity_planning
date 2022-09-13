/*
 * @Author: fujiawei0724
 * @Date: 2022-09-13 09:29:45
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-13 21:58:58
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

}