#ifndef SOLVER_OPTS_H
#define SOLVER_OPTS_H

#include <bits/types/clock_t.h>
#include <bits/types/time_t.h>
#include <ctime>
#include <limits>
#include <vector>

#include "robot_dynamics/functionbase.h"

enum class SolverName {
  ALTRO,
  AugmentedLagrangian,
  iLQR,
  ProjectedNewton,
};
enum class TerminationStatus {
  UNSOLVED,
  LINESEARCH_FAIL,
  SOLVE_SUCCEEDED,
  MAX_ITERATIONS,
  MAX_ITERATIONS_OUTER,
  MAXIMUM_COST,
  STATE_LIMIT,
  CONTROL_LIMIT,
  NO_PROGRESS,
  COST_INCREASE,
};
enum class BpRegType {
  state,
  control,
};

template <typename T> struct AbstractSolverOptions {};

template <typename T = double> struct SolverOptions : AbstractSolverOptions<T> {
  T constraint_tolerance = 1e-6;
  T cost_tolerance = 0.001;
  T cost_tolerance_intermediate = 1e-4;
  T gradient_tolerance = 10.0;
  T gradient_tolerance_intermediate = 1.0;

  double expected_decrease_tolerance = 1e-10;
  int iterations_inner = 300;
  int dJ_counter_limit = 10;
  bool square_root = false;
  T line_search_lower_bound = 1e-8;
  T line_search_upper_bound = 10.0;
  T line_search_decrease_factor = 0.5;
  int iterations_linesearch = 20;
  T max_cost_value = 1.0e8;
  T max_state_value = 1.0e8;
  T max_control_value = 1.0e8;
  bool static_bp = true;
  bool save_S = false;
  bool closed_loop_initial_rollout = false;

  bool bp_reg = false;
  T bp_reg_initial = 0.0;
  T bp_reg_increase_factor = 1.6;
  T bp_reg_max = 1.0e8;
  T bp_reg_min = 1.0e-8;
  BpRegType bp_reg_type = BpRegType::control;
  T bp_reg_fp = 10.0;

  bool use_conic_cost = false;
  T penalty_initial = 1.0;
  T penalty_scaling = 10.0;
  T penalty_max = 1e8;
  T dual_max = 1e8;

  T active_set_tolerance_al = 1e-3;
  int iterations_outer = 30;
  bool kickout_max_penalty = false;
  bool reset_duals = true;
  bool reset_penalties = true;

  bool force_pn = false;
  bool verbose_pn = false;
  int n_steps = 2;
  // solve_type::Symbol = :feasible;
  T projected_newton_tolerance = 1e-3;
  T active_set_tolerance_pn = 1e-3;
  bool multiplier_projection = true;
  T rho_chol = 1e-2;
  T rho_primal = 1.0e-8;
  T rho_dual = 1.0e-8;
  T r_threshold = 1.1;

  StaticReturn dynamics_funsig;
  UserDefined dynamics_diffmethod; // Default is ForwardDiff;
  bool projected_newton = true;
  bool reuse_jacobians = false;
  bool trim_stats = true;
  int iterations = 40;
  bool show_summary = true;
  int verbose = 0;
};
using SolverOptionsD = SolverOptions<double>;

template <typename T = double> struct SolverStats {
  int iterations = 0;
  int iterations_outer = 0;
  int iterations_pn = 0;

  std::vector<int> iteration;
  std::vector<int> iteration_outer;
  std::vector<bool> iteration_pn;
  std::vector<T> cost;
  std::vector<T> dJ;
  std::vector<T> c_max;
  std::vector<T> gradient;
  std::vector<T> penalty_max;

  int dJ_zero_counter = 0;
  bool ls_failed = false;

  clock_t tstart = clock();
  time_t tsolve = clock();
  // TimerOutput to;
  TerminationStatus status = TerminationStatus::UNSOLVED;
  bool is_reset = false;

  // Which solver is the top-level solver and responsible for resetting and
  // trimming.
  SolverName parent;
};
using SolverStatsD = SolverStats<double>;

template <typename T>
void reset(SolverStats<T> &stats, int N, SolverName parent) {
  if (parent == stats.parent) {
    stats.is_reset = false;
    reset(stats, N);
  }
}

template <typename T> void reset(SolverStats<T> &stats, int N = 0) {
  if (stats.is_reset) {
    return;
  }
  stats.iterations = 0;
  stats.iterations_outer = 0;
  stats.iterations_pn = 0;
  auto reset_func = [](auto &v, int N) { v.resize(N, 0); };
  reset_func(stats.iteration, N);
  reset_func(stats.iteration_outer, N);
  reset_func(stats.iteration_pn, N);
  reset_func(stats.cost, N);
  reset_func(stats.dJ, N);
  reset_func(stats.c_max, N);
  reset_func(stats.gradient, N);
  reset_func(stats.penalty_max, N);
  stats.tstart = clock();
  stats.tsolve = clock();
  stats.dJ_zero_counter = 0;
  stats.is_reset = true;
  stats.status = TerminationStatus::UNSOLVED;
}

struct RecordParam {
  double cost = std::numeric_limits<double>::quiet_NaN();
  double dJ = std::numeric_limits<double>::quiet_NaN();
  double c_max = std::numeric_limits<double>::quiet_NaN();
  double gradient = std::numeric_limits<double>::quiet_NaN();
  double penalty_max = std::numeric_limits<double>::quiet_NaN();
};

template <typename T>
int record_iteration(SolverStats<T> &stats, RecordParam param,
                     bool is_pn = false, bool is_outer = false) {
  if (is_outer) {
    stats.iterations_outer += 1;
  } else {
    stats.iterations += 1;
  }
  const int i = stats.iterations;
  auto record = [&i](std::vector<T> &vec, T val) {
    if (std::isnan(val)) {
      if (vec[i] != 0) {
        val = vec[i];
      } else if (i > 0) {
        val = vec[i - 1];
      } else {
        val = vec[i];
      }
    }
    vec[i] = val;
  };

  stats.iteration[i] = i;
  stats.iteration_outer[i] = stats.iterations_outer;
  stats.iteration_pn[i] = is_pn;
  record(stats.cost, param.cost);
  record(stats.dJ, param.dJ);
  record(stats.c_max, param.c_max);
  record(stats.gradient, param.gradient);
  record(stats.penalty_max, param.penalty_max);
  if (is_pn) {
    stats.iterations_pn += 1;
  }
  return stats.iterations;
}

template <typename T> void trim(SolverStats<T> &stats, SolverName parent) {
  if (parent == stats.parent) {
    trim(stats);
  }
}
template <typename T> int trim(SolverStats<T> &stats) {
  const auto N = stats.iterations;
  stats.iteration.resize(N);
  stats.iteration_outer.resize(N);
  stats.iteration_pn.resize(N);
  stats.cost.resize(N);
  stats.dJ.resize(N);
  stats.c_max.resize(N);
  stats.gradient.resize(N);
  stats.penalty_max.resize(N);
  stats.is_reset = false;
  return N;
}

#endif
