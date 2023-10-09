#include "seabot2_temperature_profile/temperature_profile.h"

TemperatureProfile::TemperatureProfile(){

}

void TemperatureProfile::update_temperature(double temperature, double depth){
    temperature_depth_data_.emplace_back(temperature, depth);
    if(temperature_depth_data_.size()>max_number_data_)
        temperature_depth_data_.pop_front();
}

void TemperatureProfile::compute_profile(){
    if(temperature_depth_data_.size()>10){
        VectorXd data_temp(temperature_depth_data_.size());
        VectorXd data_depth(temperature_depth_data_.size());

        // Define the vector data_temp and data_depth for the least squares problem
        std::transform(temperature_depth_data_.begin(), temperature_depth_data_.end(), data_temp.data(),
                       [](const std::pair<double, double>& p) { return p.first; });
        std::transform(temperature_depth_data_.begin(), temperature_depth_data_.end(), data_depth.data(),
                       [](const std::pair<double, double>& p) { return p.second; });

        // Construct the matrix A by stacking x and a column of ones
        MatrixXd A(temperature_depth_data_.size(), 2);
        A << data_temp, VectorXd::Ones(temperature_depth_data_.size());

        // Solve the least squares problem
        VectorXd coefficients = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(data_depth);

        // Extract the slope (a) and intercept (b)
        profile_slope_ = coefficients[0];
        profile_intercept_ = coefficients[1];
    }
}