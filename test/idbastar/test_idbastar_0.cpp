#include "idbastar/idbastar/idbastar.hpp"

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

#define DYNOBENCH_BASE "../../dynobench/"

BOOST_AUTO_TEST_CASE(test_bugtrap_heu) {

  Problem problem(DYNOBENCH_BASE +
                  std::string("envs/unicycle1_v0/bugtrap_0.yaml"));

  Options_idbAStar options_idbas;
  Options_dbastar options_dbastar;
  const Options_trajopt options_trajopt;
  Trajectory traj_out;
  Info_out_idbastar out_info_idbas;

  idbA(problem, options_idbas, options_dbastar, options_trajopt, traj_out,
       out_info_idbas);

  BOOST_TEST(out_info_idbas.solved);
  CSTR_(out_info_idbas.cost);
  BOOST_TEST(out_info_idbas.cost < 60.);
}