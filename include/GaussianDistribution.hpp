/*
 * @Author: fujiawei0724
 * @Date: 2022-09-12 16:14:10
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-12 16:28:33
 * @Description: gaussian distribution
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

template <typename T, int N_DIM>
class GaussianND {
 public:

    GaussianND() = default;

    GaussianND(Eigen::Matrix<T, N_DIM, 1>& ave_values, Eigen::Matrix<T, N_DIM, N_DIM>& covariance) {
        ave_values_ = ave_values;
        covariances_ = covariance;

    }

    ~GaussianND() = default;

    Eigen::Matrix<T, N_DIM, 1> ave_values_;
    Eigen::Matrix<T, N_DIM, N_DIM> covariances_;

};

using Gaussian2D = GaussianND<double, 2>;
