#include "dynobench/multirobot_trajectory.hpp"
#include <algorithm>
#include <bits/stdc++.h>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <yaml-cpp/yaml.h>

// OMPL headers

#include <dynoplan/optimization/multirobot_optimization.hpp>
// #include <dynoplan/optimization/ocp.hpp>
#include "dynobench/motions.hpp"
#include <dynobench/multirobot_trajectory.hpp>
#include <dynoplan/optimization/ocp.hpp>
#include <string>
struct Obstacle {
  std::vector<double> center;
  std::vector<double> size;
  std::string type;
};

// Convert Obstacle struct to YAML node
YAML::Node to_yaml(const Obstacle &obs) {
  YAML::Node node;
  node["center"] = obs.center;
  node["size"] = obs.size;
  node["type"] = obs.type;
  return node;
}
void test_env(const std::string &env_file,
              const std::string &initial_guess_file,
              const std::string &out_file,
              const std::unordered_set<size_t> &cluster) {
  // custom params for the obstacle
  double size = 0.5;
  std::string type = "sphere";
  YAML::Node env = YAML::LoadFile(env_file);
  const auto &env_min = env["environment"]["min"];
  const auto &env_max = env["environment"]["max"];
  // data
  YAML::Node data;
  data["environment"]["max"] = env_max;
  data["environment"]["min"] = env_min;
  // read the result
  YAML::Node initial_guess = YAML::LoadFile(initial_guess_file);
  size_t num_robots = initial_guess["result"].size();

  for (size_t i = 0; i < num_robots; i++) {
    if (cluster.find(i) != cluster.end()) { //  robots that are within cluster
      YAML::Node robot_node;
      robot_node["start"] = env["robots"][i]["start"];
      robot_node["goal"] = env["robots"][i]["goal"];
      robot_node["type"] = env["robots"][i]["type"];
      data["robots"].push_back(robot_node);
    }
  }

  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());
  size_t max_t = 0;
  size_t robot_idx = 0;
  for (const auto &traj : init_guess_multi_robot.trajectories) {
    // if(cluster.find(robot_idx) == cluster.end()){
    max_t = std::max(max_t, traj.states.size() - 1);
    // }
    ++robot_idx;
  }
  YAML::Node moving_obstacles_node; // for all robots
  Eigen::VectorXd state;
  std::vector<Obstacle> moving_obs_per_time;
  std::vector<std::vector<Obstacle>> moving_obs;
  std::cout << "MAXT: " << max_t << std::endl;
  for (size_t t = 0; t <= max_t; ++t) {
    moving_obs_per_time.clear();
    for (size_t i = 0; i < num_robots; i++) {
      if (cluster.find(i) == cluster.end()) {
        if (t >= init_guess_multi_robot.trajectories.at(i).states.size()) {
          state = init_guess_multi_robot.trajectories.at(i).states.back();
        } else {
          state = init_guess_multi_robot.trajectories.at(i).states[t];
        }
        // into vector
        Obstacle obs;
        obs.center = {state(0), state(1), state(2)};
        obs.size = {size};
        obs.type = "sphere";
        moving_obs_per_time.push_back({obs});
      }
    }
    moving_obs.push_back(moving_obs_per_time);
  }
  for (const auto &obs_list : moving_obs) {
    YAML::Node yaml_obs_list;
    for (const auto &obs : obs_list) {
      yaml_obs_list.push_back(to_yaml(obs));
    }
    data["environment"]["moving_obstacles"].push_back(yaml_obs_list);
  }
  // Write YAML node to file
  std::ofstream fout(out_file);
  fout << data;
  fout.close();
}

int main(int argc, char *argv[]) {

  namespace po = boost::program_options;
  po::options_description desc("Allowed options");
  std::string envFile;
  std::string initFile;
  std::string outFile;
  std::string dynobench_base;
  bool sum_robots_cost = true;
  desc.add_options()("help", "produce help message")(
      "env,e", po::value<std::string>(&envFile)->required())(
      "init,i", po::value<std::string>(&initFile)->required())(
      "out,o", po::value<std::string>(&outFile)->required())(
      "base,b", po::value<std::string>(&dynobench_base)->required())(
      "sum,s", po::value<bool>(&sum_robots_cost)->required());

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  // std::unordered_set<size_t> cluster = {2, 3};
  MultiRobotTrajectory multi_out;
  multi_out.trajectories.resize(4);
  std::vector<std::unordered_set<size_t>> clusters{{0, 1}, {2, 3}};
  for (size_t i = 0; i < clusters.size(); i++) {
    std::cout << "cluster " << i << std::endl;
    std::string env_file_id =
        "/tmp/dynoplan/env_file_" + gen_random(4) + ".yaml";
    test_env(envFile, initFile, /*outputFile*/ env_file_id, clusters.at(i));
    execute_optimizationMetaRobot(env_file_id, initFile, multi_out,
                                  dynobench_base, clusters.at(i),
                                  sum_robots_cost);
  }
}
