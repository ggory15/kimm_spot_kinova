/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2021 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */
#pragma once
#ifndef KORTEX_EXAMPLES_UTILITIES_H
#define KORTEX_EXAMPLES_UTILITIES_H

#include "spot_kinova_framework/utilities/cxxopts.hpp"
#include <Eigen/Core>

using namespace Eigen;

struct ExampleArgs
{
    std::string ip_address;
    std::string   username;
    std::string   password;
};

ExampleArgs ParseExampleArguments(int argc, char *argv[]);

double wrapRadiansFromMinusPiToPi(double rad_not_wrapped, int& number_of_turns);

double wrapRadiansFromMinusPiToPi(double rad_not_wrapped);

#endif //KORTEX_EXAMPLES_UTILITIES_H