/*
 * @Author: fujiawei0724
 * @Date: 2022-09-04 08:34:54
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-13 22:26:38
 * @Description: look up table for gaussian integral
 */
#include <vector>
#include "Const.hpp"

namespace LookUpTable {



class LookUpTable {
 public:

    static void initialize(const double& confidence);

    static double find(const double& ave_value, const double& variance);

    static constexpr int sampling_number{200};

    static std::vector<double> data;

    static std::vector<double> variances;

};

class GaussianIntegral {
 public:
    
    static void initialize();

    static double find(const double& variance, const double& value);

    static std::vector<double> variances;

    static std::vector<double> values;

    static std::vector<std::vector<double>> data;

};

} // End of namespace LookUpTable