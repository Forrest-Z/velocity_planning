/*
 * @Author: fujiawei0724
 * @Date: 2022-09-04 10:43:39
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-14 15:41:04
 * @Description: 
 */
#include "Common.hpp"

namespace LookUpTable {

double ErrorFunction::forwardCalculate(const double& val) {
    double tmp = val;
    double res = (2.0 / sqrt(M_PI)) * (tmp > 0.0 ? 1.0 : -1.0) * sqrt(1.0 - exp(-1.0 * pow(tmp, 2.0))) * ((sqrt(M_PI) / 2.0) + (31.0 / 200.0) * exp(-1.0 * pow(tmp, 2.0)) - (341.0 / 8000.0) * exp(-2.0 * pow(tmp, 2.0))); 
    return res;
}

double ErrorFunction::inverseCalculate(const double& val) {
    double tmp = val;
    double res = (sqrt(M_PI) / 2.0) * (tmp + (M_PI / 12.0) * pow(tmp, 3.0) + (7.0 * pow(M_PI, 2.0) / 480.0) * pow(tmp, 5.0) + (127.0 * pow(M_PI, 3.0) / 40320.0) * pow(tmp, 7.0) + (4369.0 * pow(M_PI, 4.0) / 5806080.0) * pow(tmp, 9.0) + (34807.0 * pow(M_PI, 5.0) / 182476800.0) * pow(tmp, 11.0));
    return res;
}

constexpr int GaussianAverageValue::variance_sampling_number;

constexpr int GaussianAverageValue::confidence_sampling_number;

constexpr double GaussianAverageValue::variance_start_value;

constexpr double GaussianAverageValue::variance_end_value;

constexpr double GaussianAverageValue::confidence_start_value;

constexpr double GaussianAverageValue::confidence_end_value;

std::vector<double> GaussianAverageValue::variances(variance_sampling_number, -1.0);

std::vector<double> GaussianAverageValue::confidences(confidence_sampling_number, -1.0);

std::vector<std::vector<double>> GaussianAverageValue::data(confidence_sampling_number, std::vector<double>(variance_sampling_number, -1.0));

void GaussianAverageValue::initialize() {
    // Generate sampling values
    variances = Tools::linspace(variance_start_value, variance_end_value, variance_sampling_number);
    confidences = Tools::linspace(confidence_start_value, confidence_end_value, confidence_sampling_number);

    // Supply data
    for (int i = 0; i < confidence_sampling_number; i++) {
        double cur_confidence = confidences[i];
        // Calculate delta
        double delta = 1.0 - cur_confidence;
        assert(delta <= 0.5);

        for (int j = 0; j < variance_sampling_number; j++) {
            double cur_variance = variances[j];
            double tmp = 1.0 - 2.0 * delta;
            double c = sqrt(2.0 * cur_variance) * ErrorFunction::inverseCalculate(tmp);
            data[i][j] = c;
        }
    }
    
}

double GaussianAverageValue::find(const double& variance, const double& confidence) {
    // Get corresponding index
    int variance_index = std::lower_bound(variances.begin(), variances.end(), variance) - variances.begin();
    int confidence_index = std::lower_bound(confidences.begin(), confidences.end(), confidence) - confidences.begin();

    assert(variance_index < variance_sampling_number && variance_index >= 0);
    assert(confidence_index < confidence_sampling_number && confidence_index >= 0);

    return data[confidence_index][variance_index];

}

double GaussianAverageValue::calculate(const double& variance, const double& confidence) {
    double delta = 1.0 - confidence;
    assert(delta <= 0.5);
    double tmp = 1.0 - 2.0 * delta;
    double res = sqrt(2.0 * variance) * ErrorFunction::inverseCalculate(tmp);
    return res;
}

constexpr int GaussianIntegral::variance_sampling_number;

constexpr int GaussianIntegral::value_sampling_number;

constexpr double GaussianIntegral::variance_start_value;

constexpr double GaussianIntegral::variance_end_value;

constexpr double GaussianIntegral::value_start_value;

constexpr double GaussianIntegral::value_end_value;

std::vector<double> GaussianIntegral::variances(variance_sampling_number, -1.0);

std::vector<double> GaussianIntegral::values(value_sampling_number, -1.0);

std::vector<std::vector<double>> GaussianIntegral::data(value_sampling_number, std::vector<double>(variance_sampling_number, -1.0));

void GaussianIntegral::initialize() {
    // Generate sampling values
    variances = Tools::linspace(variance_start_value, variance_end_value, variance_sampling_number);
    values = Tools::linspace(value_start_value, value_end_value, value_sampling_number);

    // Supply data 
    for (int i = 0; i < value_sampling_number; i++) {
        double cur_value = values[i];
        for (int j = 0; j < variance_sampling_number; j++) {
            double cur_variance = variances[j];
            double tmp = cur_value / sqrt(2.0 * cur_variance);
            double erf_tmp = ErrorFunction::forwardCalculate(tmp);
            double res = 0.5 * (1.0 + erf_tmp);

            data[i][j] = res;
        }
    }
}

double GaussianIntegral::find(const double& variance, const double& value) {
    // Get corresponding index
    int variance_index = std::lower_bound(variances.begin(), variances.end(), variance) - variances.begin();
    int value_index = std::lower_bound(values.begin(), values.end(), value) - values.begin();

    assert(variance_index < variance_sampling_number && variance_index >= 0);
    assert(value_index < value_sampling_number && value_index >= 0);

    return data[value][variance_index];

}

double GaussianIntegral::calculate(const double& variance, const double& value) {
    double tmp = value / sqrt(2.0 * variance);
    double erf_tmp = ErrorFunction::forwardCalculate(tmp);
    double res = 0.5 * (1.0 + erf_tmp);
    return res;
}

} // End of namespace LookUpTable
