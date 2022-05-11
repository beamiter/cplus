#ifndef SOLVER_OPTS_H
#define SOLVER_OPTS_H

#include <vector>

#include "robot_dynamics/functionbase.h"
#include "solver.h"

template <typename T> struct AbstractSolverOptions {};

template <typename T = double> struct SolverOptions : AbstractSolverOptions<T> {
  T constraint_tolerance = 1e-6;
  T cost_tolerance = 1e-4;
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
  // bp_reg_type::Symbol = :control;
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

  FunctionSignature dynamics_funsig = StaticReturn();
  DiffMethod dynamics_diffmethod = DiffMethod::ForwardAD;
  bool projected_newton = true;
  bool reuse_jacobians = false;
  bool trim_stats = true;
  int iterations = 1000;
  bool show_summary = true;
  int verbose = 0;
};
using SolverOptionsD = SolverOptions<double>;

template <typename T=double> struct SolverStats {
  int iterations = 0;
  int iterations_outer = 0;
  int iterations_pn = 0;

  std::vector<int> iteration;
  std::vector<int> iteration_outer;
  std::vector<bool> iteration_pn;
  std::vector<T> cost;

  SolverName parent;
};
using SolverStatsD = SolverStats<double>;

#endif
