#include "dynoplan/optimization/ocp.hpp"

// #define BOOST_TEST_MODULE test module name
// #define BOOST_TEST_DYN_LINK
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
#include <Eigen/Dense>
#include <iostream>

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"
#define DYNOBENCH_BASE_DATA "../../dynobench/"

using namespace dynoplan;
using namespace dynobench;

BOOST_AUTO_TEST_CASE(second_order_park_traj_opt2) {

  Options_trajopt options_trajopt;
  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle2_v0/parallelpark_0.yaml"));

  Trajectory init_guess(
      DYNOBENCH_BASE_DATA +
      std::string("data/unicycle2_0_parallelark_guess_0.yaml"));

  options_trajopt.solver_id = static_cast<int>(SOLVER::traj_opt);
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  options_trajopt.soft_control_bounds = 1;
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");

  Result_opti result;
  Trajectory sol;
  trajectory_optimization(problem, init_guess, options_trajopt, sol, result);
  BOOST_TEST_CHECK(result.feasible);
  std::cout << "cost is " << result.cost << std::endl;
  BOOST_TEST_CHECK(result.cost <= 10.);
}

BOOST_AUTO_TEST_CASE(t_opti_integrator1_v2) {

  Options_trajopt options;
  options.soft_control_bounds = true;
  Problem problem(DYNOBENCH_BASE "envs/integrator1_2d_v0/empty.yaml");
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Trajectory init_guess, traj_out;
  init_guess.num_time_steps = 50;
  // init_guess.num_time_steps = 5; use this to see how it saturates control
  // limits
  Result_opti opti_out;
  trajectory_optimization(problem, init_guess, options, traj_out, opti_out);
  BOOST_TEST(opti_out.feasible);

  // write down the generated trajectory

  std::string filename = "/tmp/dynoplan/traj_t_opti_integrator1_2d.yaml";
  create_dir_if_necessary(filename.c_str());
  std::ofstream out(filename);
  traj_out.to_yaml_format(out);
}

