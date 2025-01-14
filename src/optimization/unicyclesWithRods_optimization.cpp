#include "dynobench/motions.hpp"
#include <dynoplan/optimization/payloadTransport_optimization.hpp>
#include <dynoplan/optimization/ocp.hpp>
#include <string>
#include <vector>

  using namespace dynoplan;
  using namespace dynobench;


bool execute_unicyclesWithRodsOptimization(std::string &env_file,
                                    std::string &initial_guess_file,
                                    std::string &output_file,
                                    std::string &output_file_anytime,
                                    Trajectory &sol,
                                    const std::string &dynobench_base,
                                    bool sum_robots_cost) {


  Result_opti result;

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.collision_weight = 500.;
  options_trajopt.weight_goal = 200.;
  options_trajopt.max_iter = 50;
  Problem problem(env_file);
  problem.models_base_path = dynobench_base + std::string("models/");
  Trajectory init_guess(initial_guess_file);
  trajectory_optimization(problem, init_guess, options_trajopt, sol, result);

  // sol.to_yaml_format(output_file);
  
  if (result.feasible) {
    std::cout << "croco success -- returning " << std::endl;
    std::cout << "optimization done! " << std::endl;
    std::cout << "trajectory written in: " << output_file << std::endl;
    // sol.to_yaml_format(output_file.c_str());  
    // sol.to_yaml_format(output_file_anytime.c_str());  
    return true;
  } else {
    std::cout << "croco failed -- returning " << std::endl;
    return false;
  }
}

