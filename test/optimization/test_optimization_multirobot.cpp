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
#include "dynobench/multirobot_trajectory.hpp"
#include "dynobench/planar_rotor.hpp"
#include "dynobench/planar_rotor_pole.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace dynoplan;
using namespace dynobench;

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_multi_robot_cli) {

  std::vector<std::string> run_cmd_new = {
      "../main_multirobot_optimization",
      "--env",
      DYNOBENCH_BASE "envs/multirobot/straight.yaml",
      "--init",
      DYNOBENCH_BASE "envs/multirobot/guess_indiv_straight.yaml",
      "--base",
      DYNOBENCH_BASE,
      "--out",
      "buu.yaml",
      "--s",
      "1",
      ">",
      "/tmp/db_log.txt"};

  std::string cmd = "";

  for (auto &s : run_cmd_new) {
    cmd += s + " ";
  }

  {
    std::cout << "running:\n" << cmd << std::endl;
    int out = std::system(cmd.c_str());
    BOOST_TEST(out == 0);
  }

  // TODO: @akmaral, Can you add the other test in test_standalone?
}

BOOST_AUTO_TEST_CASE(t_multi_robot) {

  // TODO: @akmaral, Can you add the other test in  test_standalone?

  // 0: optimize the Max time of arrival.
  // 1: optimize the sum of the time of arrival of all robots.
  bool sum_robots_cost = 1;
  std::string env_file = DYNOBENCH_BASE "envs/multirobot/straight.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/guess_indiv_straight.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check2.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_robot_swap2_trailer) {

  bool sum_robots_cost = 1;
  std::string env_file =
      DYNOBENCH_BASE "envs/multirobot/example/swap2_trailer.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/swap2_trailer_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check3.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi_swap2_trailer.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_robot_swap4_unicycle) {

  bool sum_robots_cost = 1;
  std::string env_file =
      DYNOBENCH_BASE "envs/multirobot/example/swap4_unicycle.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/swap4_unicycle_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();

  init_guess_joint.to_yaml_format("/tmp/check4.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi_swap4_unicycle.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_but_only_one) {

  bool sum_robots_cost = 1;
  std::string env_file = DYNOBENCH_BASE "envs/multirobot/swap1_trailer.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/swap1_trailer_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess_joint.to_yaml_format("/tmp/check4.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi_swap1_unicycle.yaml");
}

BOOST_AUTO_TEST_CASE(t_hetero_random_2) {

  bool sum_robots_cost = 1;
  std::string env_file =
      DYNOBENCH_BASE "envs/multirobot/example/gen_p10_n2_1_hetero.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/gen_p10_n2_1_hetero_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess_joint.to_yaml_format("/tmp/check5.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_gen_p10_n2_1_hetero_solution.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_robot_swap2_unicycle) {

  bool sum_robots_cost = 1;
  std::string env_file =
      DYNOBENCH_BASE "envs/multirobot/example/swap2_unicycle.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/swap2_unicycle_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check3.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi_swap2_unicycle.yaml");
}
// drones examples
BOOST_AUTO_TEST_CASE(t_multi_robot_drone2c) {

  bool sum_robots_cost = 1;
  std::string env_file = DYNOBENCH_BASE "envs/multirobot/example/drone2c.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/drone2c_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check12.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi_robot_drone2c_solution.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_robot_drone12c) {

  bool sum_robots_cost = 1;
  std::string env_file = DYNOBENCH_BASE "envs/multirobot/example/drone12c.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/drone12c_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check12.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("/tmp/test_multi_robot_drone12c_solution.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_robot_drone16c) {

  bool sum_robots_cost = 1;
  std::string env_file = DYNOBENCH_BASE "envs/multirobot/example/drone16c.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/drone16c_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 80;
  options_trajopt.max_iter = 50;
  options_trajopt.soft_control_bounds = true;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check16.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("envs/multirobot/results/drone16c_solution.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_robot_drone24c) {

  bool sum_robots_cost = 1;
  std::string env_file = DYNOBENCH_BASE "envs/multirobot/example/drone24c.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/drone24c_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  options_trajopt.soft_control_bounds = true;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check24.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  multi_out.to_yaml_format("envs/multirobot/results/drone24c_solution.yaml");
}

BOOST_AUTO_TEST_CASE(t_multi_robot_drone32c) {

  bool sum_robots_cost = 1;
  std::string env_file = DYNOBENCH_BASE "envs/multirobot/example/drone32c.yaml";
  std::string initial_guess_file =
      DYNOBENCH_BASE "envs/multirobot/results/drone32c_db.yaml";

  Problem problem(env_file);
  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  std::cout << "goal times are " << std::endl;
  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  Trajectory init_guess;
  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  Options_trajopt options_trajopt;
  options_trajopt.solver_id = 1;
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  options_trajopt.soft_control_bounds = true;
  problem.models_base_path = DYNOBENCH_BASE "models/";

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess.to_yaml_format("/tmp/check32.yaml");

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);

  BOOST_TEST(result.feasible == 1);

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);
  //
  multi_out.to_yaml_format("envs/multirobot/results/drone32c_solution.yaml");
}

// moving obstacles
BOOST_AUTO_TEST_CASE(t_moving_obstacles) {

  std::string env_file =
      DYNOBENCH_BASE "envs/multirobot/example/moving_obs_twd_start.yaml";
  // std::string env_file =
  // "/home/akmarak-laptop/IMRC/db-CBS/dynoplan/dynobench/"
  //  "envs/multirobot/example/moving_obs_swap4_drone.yaml";

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

  init_guess.num_time_steps = 200;
  trajectory_optimization(problem, init_guess, options_trajopt, sol, opti_out);

  sol.to_yaml_format("moving_obs_swap4_drone.yaml");
}

// residual force included
BOOST_AUTO_TEST_CASE(t_coupled_integrator2d) {

  Options_trajopt options_trajopt;
  std::string env_file =
      "/home/akmarak-laptop/IMRC/db-CBS/example/swap2_integrator2_coupled.yaml";
  std::string initial_guess_file =
      "/home/akmarak-laptop/IMRC/db-CBS/results/result_dbecbs_joint.yaml";

  Problem problem(env_file);
  Trajectory init_guess(initial_guess_file);

  options_trajopt.solver_id = 1; // static_cast<int>(SOLVER::traj_opt);
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path =
      "/home/akmarak-laptop/IMRC/db-CBS/dynoplan/dynobench/models/";

  Result_opti result;
  Trajectory sol;
  trajectory_optimization(problem, init_guess, options_trajopt, sol, result);
  BOOST_TEST_CHECK(result.feasible);
  std::cout << "cost is " << result.cost << std::endl;

  std::vector<int> index_time_goals{sol.states.size(), sol.states.size()};
  std::vector<int> nxs{4, 4};
  std::vector<int> nus{2, 2};
  MultiRobotTrajectory multi_out =
      from_joint_to_indiv_trajectory(sol, nxs, nus, index_time_goals);

  multi_out.to_yaml_format("../../results/integrator2d_coupled_opt.yaml");
}

// residual force included 3D
BOOST_AUTO_TEST_CASE(t_coupled_integrator3d) {

  Options_trajopt options_trajopt;
  std::string env_file = "/home/akmarak-laptop/IMRC/db-CBS/example/"
                         "swap2_integrator2_3d_coupled.yaml";
  std::string initial_guess_file =
      "/home/akmarak-laptop/IMRC/db-CBS/results/result_dbecbs_joint.yaml";

  Problem problem(env_file);
  Trajectory init_guess(initial_guess_file);

  options_trajopt.solver_id = 1; // static_cast<int>(SOLVER::traj_opt);
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 400;
  options_trajopt.max_iter = 50;
  problem.models_base_path =
      "/home/akmarak-laptop/IMRC/db-CBS/dynoplan/dynobench/models/";

  Result_opti result;
  Trajectory sol;
  trajectory_optimization(problem, init_guess, options_trajopt, sol, result);
  BOOST_TEST_CHECK(result.feasible);
  std::cout << "cost is " << result.cost << std::endl;

  std::vector<int> index_time_goals{sol.states.size(), sol.states.size()};
  std::vector<int> nxs{6, 6};
  std::vector<int> nus{3, 3};
  MultiRobotTrajectory multi_out =
      from_joint_to_indiv_trajectory(sol, nxs, nus, index_time_goals);

  multi_out.to_yaml_format("../../results/integrator3d_joint_opt.yaml");
}

// residual force included 3D, state augmented with f_res
BOOST_AUTO_TEST_CASE(t_joint_integrator3d) {

  Options_trajopt options_trajopt;
  // std::string env_file =
  // "/home/akmarak-laptop/IMRC/db-CBS/example/swap2_integrator2_3d_coupled.yaml";
  // std::string initial_guess_file =
  // "/home/akmarak-laptop/IMRC/db-CBS/results/result_joint.yaml";
  std::string env_file =
      "/home/akmarak-laptop/IMRC/db-CBS/example/debug_nn.yaml";
  std::string initial_guess_file =
      "/home/akmarak-laptop/IMRC/db-CBS/results/debug_nn_joint.yaml";
  Problem problem(env_file);
  Trajectory init_guess(initial_guess_file);

  options_trajopt.solver_id = 1; // static_cast<int>(SOLVER::traj_opt);
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 200;
  options_trajopt.max_iter = 50;
  options_trajopt.collision_weight = 200;
  problem.models_base_path =
      "/home/akmarak-laptop/IMRC/db-CBS/dynoplan/dynobench/models/";

  Result_opti result;
  Trajectory sol;
  trajectory_optimization(problem, init_guess, options_trajopt, sol, result);
  BOOST_TEST_CHECK(result.feasible);
  std::cout << "cost is " << result.cost << std::endl;
  // remove f_res from your state
  Trajectory sol2;
  for (auto &s : sol.states) {
    Eigen::VectorXd s_tmp(12);
    s_tmp << s.head<6>(), s.segment<6>(7);
    sol2.states.push_back(s_tmp);
  }
  sol2.actions = sol.actions;
  std::vector<int> index_time_goals{sol2.states.size(), sol2.states.size()};
  std::vector<int> nxs{6, 6};
  std::vector<int> nus{3, 3};
  MultiRobotTrajectory multi_out =
      from_joint_to_indiv_trajectory(sol2, nxs, nus, index_time_goals);
  std::string optimizationFile = "../../results/result_opt_debug.yaml";
  multi_out.to_yaml_format(optimizationFile.c_str());
  // save the residual force for one of robots, robot id = 1, which passes under
  bool debug = true;
  if (debug) {
    std::ofstream fout(optimizationFile, std::ios::app);
    fout << "fa:" << std::endl;
    for (auto &s : sol.states) {
      std::cout << s(6) * 101.97 << std::endl;      // in grams
      fout << "  - " << s(6) * 101.97 << std::endl; // in gramms
    }
  }
}
