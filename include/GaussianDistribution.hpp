/*
 * @Author: fujiawei0724
 * @Date: 2022-09-12 16:14:10
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-14 08:12:27
 * @Description: gaussian distribution
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "Const.hpp"
#include "Shape.hpp"

template <typename T, int N_DIM>
class GaussianND {
 public:

    GaussianND() = default;

    GaussianND(Eigen::Matrix<T, N_DIM, 1>& ave_values, Eigen::Matrix<T, N_DIM, N_DIM>& covariance) {
        ave_values_ = ave_values;
        covariance_ = covariance;

    }

    // For the situation where the average value is not specific
    GaussianND(Eigen::Matrix<T, N_DIM, N_DIM>& covariance) {
        covariance_ = covariance;
    }

    ~GaussianND() = default;

    // The correctness of this function is only verified in the situation from 2D to 1D
    GaussianND<T, 1> edgeDistribution(int dimension_index) const {
        assert(dimension_index >= 0 && dimension_index < N_DIM);
        Eigen::Matrix<T, 1, 1> ave_values{ave_values_(dimension_index, 0)};
        Eigen::Matrix<T, 1, 1> covariance{covariance_(dimension_index, dimension_index)};
        
        return GaussianND<T, 1>(ave_values, covariance);

    }

    Eigen::Matrix<T, N_DIM, 1> ave_values_;
    Eigen::Matrix<T, N_DIM, N_DIM> covariance_;

};

using Gaussian1D = GaussianND<double, 1>;

using Gaussian2D = GaussianND<double, 2>;


class GaussianUtils {
 public:

    /**
     * @description: given a 2d gaussian distribution, calculate the possibility corresponding to the speific dimension given the upper and lower values
     * @param {Gaussian2D&} gaussian_dis_2d gaussian distribution need to be calculated
     * @param {DimensionType&} dimension_type dimension name
     * @param {double&} start_value start value
     * @param {double&} end_value end value
     * @return {*} the possibility
     */ 
    static double calculateDistributionProbability(const Gaussian2D& gaussian_dis_2d, const DimensionType& dimension_type, const double& start_value, const double& end_value);
    
};

