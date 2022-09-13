/*
 * @Author: fujiawei0724
 * @Date: 2022-09-04 10:43:39
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-13 22:27:48
 * @Description: 
 */
#include "Common.hpp"

namespace LookUpTable {

constexpr int LookUpTable::sampling_number;

std::vector<double> LookUpTable::variances(200, -1);

std::vector<double> LookUpTable::data(200, -1);

void LookUpTable::initialize(const double& confidence) {
    // Calculate delta
    double delta = 1.0 - confidence;

    assert(delta < 0.5);

    // Initialize average values and variances
    // TODO: adjust the upper bound and lower bound
    variances = Tools::linspace(0.0, 50.0, sampling_number);

    // Supply data using inverse erf approximation
    for (int j = 0; j < sampling_number; j++) {
        double cur_variance = variances[j];
        double tmp = 1.0 - 2.0 * delta;
        double c = sqrt(2.0 * cur_variance) * (sqrt(M_PI) / 2.0) * (tmp + (M_PI / 12.0) * pow(tmp, 3.0) + (7.0 * pow(M_PI, 2.0) / 480.0) * pow(tmp, 5.0) + (127.0 * pow(M_PI, 3.0) / 40320.0) * pow(tmp, 7.0) + (4369.0 * pow(M_PI, 4.0) / 5806080.0) * pow(tmp, 9.0) + (34807.0 * pow(M_PI, 5.0) / 182476800.0) * pow(tmp, 11.0));
        data[j] = c;
    }
    
}

double LookUpTable::find(const double& ave_value, const double& variance) {
    // Get corresponding index of variance
    int variance_index = std::lower_bound(variances.begin(), variances.end(), variance) - variances.begin();

    assert(variance_index < sampling_number && variance_index >= 0);

    return data[variance_index];

}



} // End of namespace LookUpTable
