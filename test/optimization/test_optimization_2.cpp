#include "dynoplan/optimization/ocp.hpp"

// #define BOOST_TEST_MODULE test module name
// #define BOOST_TEST_DYN_LINK
#include <boost/test/tools/old/interface.hpp>
#include <boost/test/unit_test.hpp>

#include "Eigen/Core"
#include <boost/program_options.hpp>

// #include "collision_checker.hpp"

// save data without the cluster stuff

#include <filesystem>
#include <random>
#include <regex>
#include <type_traits>

#include <filesystem>
#include <regex>

#include "dynobench/motions.hpp"
#include "dynobench/planar_rotor.hpp"
#include "dynobench/planar_rotor_pole.hpp"
#include <Eigen/Dense>
#include <iostream>

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"

using namespace dynoplan;
using namespace dynobench;



BOOST_AUTO_TEST_CASE(t_moving_obstacle2) {


  std::string env_file = DYNOBENCH_BASE "envs/multirobot/example/moving_obs_twd_start.yaml";
  // std::string env_file = DYNOBENCH_BASE "envs/multirobot/example/moving_obs_same_path.yaml";


  Problem problem(env_file);


  problem.models_base_path = DYNOBENCH_BASE "models/";

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 0;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 80;
  options_trajopt.max_iter = 50;
  options_trajopt.soft_control_bounds = true; 
  
  // solve the optimization problem

  Trajectory init_guess;
  Trajectory sol;
  Result_opti opti_out;

  init_guess.num_time_steps  = 200; 
  // todo: what happens if I have one more or one less?


  // problem.obstacles.clear();

  trajectory_optimization(problem, init_guess, options_trajopt, sol,
                          opti_out);

  sol.to_yaml_format("/tmp/moving_obs_sol3.yaml");

}



