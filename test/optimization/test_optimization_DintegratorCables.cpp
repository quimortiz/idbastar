#include <boost/test/unit_test.hpp>
#include "dynoplan/optimization/ocp.hpp"


using namespace dynoplan;
using namespace dynobench;

#define relative_base "../"

BOOST_AUTO_TEST_CASE(t_DintegratorCables_optimization) {


  Options_trajopt options;
  options.solver_id = 1;
  options.max_iter = 200;
  // options.max_iter = 18;
  options.weight_goal = 100;
  options.collision_weight = 100;

  Problem problem(relative_base "example/cables_integrator2_2d_window.yaml");

  problem.models_base_path =  relative_base "dynoplan/dynobench/models/";
  Trajectory init_guess(relative_base "init_guess_cables.yaml");

  Trajectory sol;
  Result_opti result;

  trajectory_optimization(problem, init_guess, options, sol, result);

  sol.to_yaml_format(relative_base "cables_integrator2_2d_window_opt.yaml");
  // BOOST_TEST(result.feasible);

}