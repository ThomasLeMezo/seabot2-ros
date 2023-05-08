//
// Created by lemezoth on 11/04/23.
//

#ifndef BUILD_ALPHA_SOLVER_H
#define BUILD_ALPHA_SOLVER_H

#include "ibex.h"
#include <array>
using namespace std;

class AlphaSolver {

public:
    AlphaSolver()= default; // Default constructor

    /*
     * Compute the alpha value
     */
    double compute_alpha(const double beta);

    /**
     * Update the coefficients
     * @param Cz
     * @param A
     * @param B
     * @param dVp_max
     */
    void update_coeff(const double Cf, const double A, const double B, const double dVp_max);

    /**
     *  Check if the value is already computed
     */
    pair<bool, double> exist_in_memory(const double beta);

    /**
     * Add the value to the memory
     * @param beta
     * @param alpha
     */
    void add_to_memory(const double beta, const double alpha);

private:
    // B = rho*S/(2*mv)
    // A = rho*g/mv
    // S= math.pi*(0.125/2.)**2
    ibex::Interval Cf_ = ibex::Interval(1.0);
    ibex::Interval A_ = ibex::Interval(418.96875);
    ibex::Interval B_ = ibex::Interval(0.262055051263797);
    ibex::Interval dVp_max_ = ibex::Interval(1.0072731445572273e-06);

    double alpha_search_max_ = 20.0;
    double z_search_max_ = 10.0;
    double epsilon_ = 0.01; // Drive the time of the algorithm

    vector<array<double, 2>> computed_memory_;

};


#endif //BUILD_ALPHA_SOLVER_H
