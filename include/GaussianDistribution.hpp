/*
 * @Author: fujiawei0724
 * @Date: 2022-09-12 16:14:10
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-14 17:29:09
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

    // The calculation process is only for 2D
    T getCorrelation() const {
        assert(N_DIM == 2);
        T variance_0 = covariance_(0, 0);
        T variance_1 = covariance_(1, 1);
        T mixed = covariance_(0, 1);
        return mixed / (sqrt(variance_0 * variance_1));
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

    /**
     * @description: transform a 2d gaussian distribution to 1d given a range, the difference between this with an edge *               defintion is that in this process, a range is defined instead of from negative infinity to positive *               infinity
     * @note this method is an approximate method, only when the relative coefficient is zero, the result is precise
     * @param {Gaussian2D&} input_gaussian_dis_2d
     * @param {DimensionType&} target_dimension_type
     * @param {double&} start_value
     * @param {double&} end_value
     * @return {*}
     */
    static void transformGaussian2DTo1D(const Gaussian2D& input_gaussian_dis_2d, const DimensionType& target_dimension_type, const double& start_value, const double& end_value, Gaussian1D* candi_dis_start, Gaussian1D* candi_dis_end);
    
};

