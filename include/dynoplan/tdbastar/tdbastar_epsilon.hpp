#pragma once
#include <algorithm>
// // #include <boost/graph/graphviz.hpp>
#include "Eigen/Core"
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
//
// #include <flann/flann.hpp>
// #include <msgpack.hpp>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
//
// // #include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
// OMPL
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
// #include "fclHelper.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/ScopedState.h"
#include <fcl/fcl.h>

#include "dynoplan/dbastar/heuristics.hpp"
#include "dynoplan/tdbastar/options.hpp"
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"

namespace dynoplan {

namespace ob = ompl::base;

using Sample = std::vector<double>;
using Sample_ = ob::State;

// focalSet
struct compareFocalHeuristic {
  bool operator()(const open_t::handle_type &h1,
                  const open_t::handle_type &h2) const;
};

typedef typename boost::heap::d_ary_heap<
    open_t::handle_type, boost::heap::arity<2>,
    boost::heap::compare<compareFocalHeuristic>, boost::heap::mutable_<true>>
    focal_t;

void tdbastar_epsilon(
    dynobench::Problem &problem, Options_tdbastar options_dbastar,
    dynobench::Trajectory &traj_out, const std::vector<Constraint> &constraints,
    Out_info_tdb &out_info_tdb, size_t &robot_id, bool reverse_search,
    std::vector<dynobench::Trajectory> &expanded_trajs,
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    std::map<std::string, std::vector<Motion>> &robot_motions,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs,
    ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *heuristic_nn = nullptr,
    ompl::NearestNeighbors<std::shared_ptr<AStarNode>> **heuristic_result =
        nullptr,
    bool heterogeneous = false, float w = 0.0,
    bool run_focal_heuristic = false);

// R1 with (R2,R3,R4), R2 with (R3,R4) and R3 with R4, state-by-state
// not for car with trailer
int highLevelfocalHeuristicStatePrecise(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs);

// less accurate, but faster
int highLevelfocalHeuristicState(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs);

// computationally less efficient version of the low-level focal heuristic
int lowLevelfocalHeuristicSequential(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    Time_benchmark &time_bench,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    dynobench::TrajWrapper &current_tmp_traj, size_t &current_robot_idx,
    const float current_gScore,
    std::vector<fcl::CollisionObjectd *> &robot_objs, bool reachesGoal = false);

// less accurate focal heuristic, but faster
int lowLevelfocalHeuristicState(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    Time_benchmark &time_bench,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    dynobench::TrajWrapper &current_tmp_traj, size_t &current_robot_idx,
    const float current_gScore,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs, bool reachesGoal = false,
    bool run_focal_heuristic = false);

int lowLevelfocalHeuristicSingleState(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    Time_benchmark &time_bench,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    Eigen::VectorXd state1, size_t &current_robot_idx,
    const float current_gScore,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots,
    std::vector<fcl::CollisionObjectd *> &robot_objs, bool reachesGoal = false,
    bool run_focal_heuristic = false);

// not clear for the reverse search, for now valid for forward search for now
bool check_lazy_trajectory_heterogeneous(
    std::vector<LowLevelPlan<dynobench::Trajectory>> &solution,
    const std::vector<std::shared_ptr<dynobench::Model_robot>> &all_robots,
    std::vector<std::string> &robot_types, LazyTraj &lazy_traj,
    dynobench::Model_robot &robot, size_t &current_robot_idx,
    const Eigen::Ref<const Eigen::VectorXd> &goal, Time_benchmark &time_bench,
    dynobench::TrajWrapper &tmp_traj,
    const std::vector<Constraint> &constraints, const float best_node_gscore,
    float delta, Eigen::Ref<Eigen::VectorXd> aux_last_state,
    std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *check_state = nullptr,
    int *num_valid_states = nullptr, bool forward = true);

} // namespace dynoplan
