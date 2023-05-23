//
// Created by lemezoth on 11/04/23.
//

#include "seabot2_depth_control/alpha_solver.h"
#include <iostream>
#include "ibex/ibex_Function.h"
#include "ibex/ibex_SepFwdBwd.h"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

using namespace std;
using namespace ibex;

pair<bool, double> AlphaSolver::exist_in_memory(const double beta) {
    for (auto &i : computed_memory_) {
        if (abs(i[0]-beta)<0.001)
            return {true, i[1]};
    }
    return {false, 1.0};
}

void AlphaSolver::add_to_memory(const double beta, const double alpha) {
    array<double, 2> a = {beta, alpha};
    computed_memory_.push_back(a);
}

double AlphaSolver::compute_alpha(const double velocity_limit) {
    auto exist = exist_in_memory(velocity_limit);
    if(velocity_limit<=0)
        return 1.0;
    else if(exist.first)
        return exist.second;
    else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[alpha_solver] Start compute new alpha (velocity_limit = %f)", velocity_limit);
        auto beta = Interval(velocity_limit);
        IntervalVector x_init(2);
        x_init[0] = Interval(0., z_search_max_); // z
        x_init[1] = Interval(0., alpha_search_max_); // alpha

        Variable v(2);
        // x[0] = z
        // x[1] = alpha
        Function f(v, (
                      v[1] * pow(beta, 3) * tanh(v[1] * v[0]) * (
                              2 * B_ * Cf_ * tanh(v[1] * v[0]) * (pow(tanh(v[1] * v[0]), 2) - 1)
                              + v[1] * pow(tanh(v[1] * v[0]), 2) * (3 * pow(tanh(v[1] * v[0]), 2) - 4.0)
                              + v[1])
              ) / A_ + dVp_max_
        );
        SepFwdBwd s(f, GEQ);

        vector<IntervalVector> v_list, v_solution;
        IntervalVector v_out = IntervalVector::empty(2);
        v_list.push_back(x_init);

        while (!v_list.empty()) {
            IntervalVector x = IntervalVector::empty(2);
            if (!v_list.empty()) {
                x = v_list.back();
                v_list.pop_back();
            }

            if (!x.is_empty()) {
                IntervalVector x_in(x), x_out(x);
                s.separate(x_in, x_out);
                if (x_in.is_empty()) {
                    // do nothing
                } else if (x_out.is_empty()) {
                    v_out |= x_in;
                } else if (!x_out.is_empty()) {
                    if(x_out[0].diam() > epsilon_){
                        std::pair<IntervalVector, IntervalVector> p = x_out.bisect(0);
                        if(x_out[1].diam() > epsilon_){
                            std::pair<IntervalVector, IntervalVector> p1 = p.first.bisect(1);
                            std::pair<IntervalVector, IntervalVector> p2 = p.second.bisect(1);
                            v_list.push_back(p1.first);
                            v_list.push_back(p1.second);
                            v_list.push_back(p2.first);
                            v_list.push_back(p2.second);
                        }
                        else{
                            v_list.push_back(p.first);
                            v_list.push_back(p.second);
                        }
                    }
                    else if(x_out[1].diam() > epsilon_){
                        std::pair<IntervalVector, IntervalVector> p = x_out.bisect(1);
                        v_list.push_back(p.first);
                        v_list.push_back(p.second);
                    }
                } else {
                    v_out |= x_out;
                }
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[alpha_solver] Compute alpha = %f for velocity_limit = %f", v_out[1].lb(), velocity_limit);
        if(!::isnan(v_out[1].lb())) {
            add_to_memory(velocity_limit, v_out[1].lb());
            return v_out[1].lb();
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[alpha_solver] Set alpha search max (%f)", alpha_search_max_);
            add_to_memory(velocity_limit, alpha_search_max_);
            return alpha_search_max_;
        }
    }
}


void AlphaSolver::update_coeff(const double Cf, const double A, const double B,
                               const double dVp_max) {
    Cf_ = Interval(Cf);
    A_ = Interval(A);
    B_ = Interval(B);
    dVp_max_ = Interval(dVp_max);
}

//int main(int argc, char *argv[]) {
//
//    using std::chrono::high_resolution_clock;
//    using std::chrono::milliseconds;
//    using std::chrono::duration;
//
//    AlphaSolver as;
//    auto t1 = high_resolution_clock::now();
//    cout << "Compute [0.1] = " << as.compute_alpha(0.1) << endl;
//    auto t2 = high_resolution_clock::now();
//    duration<double, std::milli> ms_double = t2 - t1;
//    std::cout << ms_double.count() << "ms\n";
//
//    t1 = high_resolution_clock::now();
//    cout << "Compute [0.2] = " << as.compute_alpha(0.2) << endl;
//    t2 = high_resolution_clock::now();
//    ms_double = t2 - t1;
//    std::cout << ms_double.count() << "ms\n";
//
//    t1 = high_resolution_clock::now();
//    cout << "Compute [0.05] = " << as.compute_alpha(0.05) << endl;
//    t2 = high_resolution_clock::now();
//    ms_double = t2 - t1;
//    std::cout << ms_double.count() << "ms\n";
//
//    t1 = high_resolution_clock::now();
//    cout << "Compute [0.1] = " << as.compute_alpha(0.1) << endl;
//    t2 = high_resolution_clock::now();
//    ms_double = t2 - t1;
//    std::cout << ms_double.count() << "ms\n";
//
//    t1 = high_resolution_clock::now();
//    cout << "Compute [0.3] = " << as.compute_alpha(0.3) << endl;
//    t2 = high_resolution_clock::now();
//    ms_double = t2 - t1;
//    std::cout << ms_double.count() << "ms\n";
//
//    t1 = high_resolution_clock::now();
//    cout << "Compute [0.01] = " << as.compute_alpha(0.01) << endl;
//    t2 = high_resolution_clock::now();
//    ms_double = t2 - t1;
//    std::cout << ms_double.count() << "ms\n";
//
//    return 0;
//}
