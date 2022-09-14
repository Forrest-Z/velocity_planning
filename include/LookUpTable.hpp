/*
 * @Author: fujiawei0724
 * @Date: 2022-09-04 08:34:54
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-14 15:37:27
 * @Description: look up table for gaussian integral
 */
#include <vector>
#include "Const.hpp"

namespace LookUpTable {


class ErrorFunction {
 public:
    
    static double forwardCalculate(const double& val);

    static double inverseCalculate(const double& val);
};



class GaussianAverageValue {
 public:
 
    static constexpr int variance_sampling_number{200};

    static constexpr int confidence_sampling_number{20};

    static constexpr double variance_start_value{0.0};

    static constexpr double variance_end_value{50.0};

    static constexpr double confidence_start_value{0.5};

    static constexpr double confidence_end_value{1.0};

    static void initialize();

    static double find(const double& variance, const double& confidence);

    static double calculate(const double& variance, const double& confidence);

    static std::vector<double> variances;

    static std::vector<double> confidences;

    static std::vector<std::vector<double>> data;


};

class GaussianIntegral {
 public:

    static constexpr int variance_sampling_number{200};

    static constexpr int value_sampling_number{200};

    static constexpr double variance_start_value{0.0};

    static constexpr double variance_end_value{50.0};

    static constexpr double value_start_value{-50.0};

    static constexpr double value_end_value{50.0};
    
    static void initialize();

    static double find(const double& variance, const double& value);

    static double calculate(const double& variance, const double& value);

    static std::vector<double> variances;

    static std::vector<double> values;

    static std::vector<std::vector<double>> data;

};

} // End of namespace LookUpTable