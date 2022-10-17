//
// Created by lemezoth on 25/08/22.
//

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <stdlib.h>

#include "pressure_ms5803_driver/pressure_ms5803.h"

using namespace std;

TEST(TestMS5803, ComputeResultTest) {
    Pressure_ms5803 p = Pressure_ms5803(nullptr);
    p.setCoefficient(46546,0);
    p.setCoefficient(42845,1);
    p.setCoefficient(29751,2);
    p.setCoefficient(29457,3);
    p.setCoefficient(32745,4);
    p.setCoefficient(29059,5);
    p.setD1(4311550);
    p.setD2(8387300);
    p.compute();
    cout << "Pressure = " << p.get_pression() << endl;
    cout << "Temperature = " << p.get_temperature() << endl;
    EXPECT_FLOAT_EQ(p.get_pression(), (float)1000.5);
    EXPECT_FLOAT_EQ(p.get_temperature(), (float)20.15);
    EXPECT_TRUE(true);
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}