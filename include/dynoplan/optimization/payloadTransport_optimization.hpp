#pragma once

#include <string>

bool execute_payloadTransportOptimization(std::string &env_file,
                                    std::string &initial_guess_file,
                                    std::string &output_file,
                                    const std::string &dynobench_base,
                                    bool sum_robots_cost);
