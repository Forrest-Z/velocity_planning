/*
 * @Author: fujiawei0724
 * @Date: 2022-09-12 16:14:10
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-13 15:55:09
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

    Eigen::Matrix<T, N_DIM, 1> ave_values_;
    Eigen::Matrix<T, N_DIM, N_DIM> covariance_;

};

using Gaussian1D = GaussianND<double, 1>;

using Gaussian2D = GaussianND<double, 2>;


class GaussianUtils {
 public:
    
};

