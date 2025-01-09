#pragma once
#include "dynobench/motions.hpp"
#include <string>

bool execute_payloadTransportOptimization(std::string &env_file,
                                    std::string &initial_guess_file,
                                    std::string &output_file,
                                    std::string &output_file_anytime,
                                    dynobench::Trajectory &sol,
                                    const std::string &dynobench_base,
                                    bool sum_robots_cost);
