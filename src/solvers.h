#ifndef SOLVER_H
#define SOLVER_H

#include "base/base.h"
#include "robot_dynamics/trajectories.h"
#include "solver_opts.h"
#include "trajectory_optimization/objective.h"

#define AbstractSolverTemplate template <typename KP, typename C>
#define AbstractSolverDeclare AbstractSolver<KP, C>

AbstractSolverTemplate class AbstractSolver {

  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;
  using T = typename KP::base_type;
  using base_type = typename KP::base_type;
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;

public:
  // Pure virtual functions
  virtual const SolverName solvername() const = 0;
  virtual const SolverStats<T> &stats() const = 0;
  virtual SolverStats<T> &stats() = 0;
  virtual const SolverOptions<T> &options() const = 0;
  virtual SolverOptions<T> &options() = 0;
  virtual const SampledTrajectory<KP> &get_trajectory() const = 0;
  virtual SampledTrajectory<KP> &get_trajectory() = 0;
  virtual const Objective<C> *get_objective() const = 0;
  virtual state_type get_initial_state() const = 0;

  // Virtual functions.
  virtual bool is_constrained() { return true; }
  auto get_constraints() {}

  // Functions.
  bool is_parentsolver() const { return stats().parent == solvername(); }
  const int iterations() const { return stats().iterations; }
  const TerminationStatus &status() const { return stats().status; }
  void set_options(const SolverOptions<T> &opt) { *options() = opt; }
  void resetstats() { reset(stats(), iterations(), solvername()); }
  void reset_solver() {
    const auto &opts = options();
    reset(stats(), opts.iterations, solvername());
    if (is_constrained()) {
      reset(get_constraints(), opts);
    }
  }
  double cost(const SampledTrajectory<KP> &Z) const {
    return get_objective()->cost(Z);
  }
  std::vector<state_type> states() const {
    std::vector<state_type> rtn;
    for (const auto &z : get_trajectory()) {
      rtn.push_back(z.state());
    }
  }
  std::tuple<std::vector<int>, std::vector<int>, int> dims() const {
    return dims(get_trajectory());
  }
  std::vector<control_type> controls() const {
    std::vector<control_type> rtn;
    int N = std::get<2>(dims());
    auto Z = get_trajectory();
    for (int i = 0; i < N - 1; ++i) {
      rtn.push_back(Z[i].control());
    }
  }
  void set_initial_state(const state_type &x0) { *get_initial_state() = x0; }
  std::vector<base_type> gettimes() const { return gettimes(get_trajectory()); }
  int num_constraints() const { return num_constraints(get_constraints()); }
  template <typename P> void initial_states(const P &X0) {
    setstates(get_trajectory(), X0);
  }
  template <typename Q> void initial_controls(const Q &U0) {
    setcontrols(get_trajectory(), U0);
  }
  void initial_trajectory(const SampledTrajectory<KP> &Z0) {
    auto Z = get_trajectory();
    for (int k = 0; k < Z->size(); ++k) {
      setdata(*Z[k], Z0[k].z);
    }
  }
};

AbstractSolverTemplate void terminate(AbstractSolverDeclare *solver) {
  auto &stat = solver->stats();
  stat.tsolve = (clock() - stat.tstart) / CLOCKS_PER_SEC;
  const auto &opts = solver->options();
  if (opts.trim_stats) {
    trim(stat, solver->solvername());
  }
  if (solver->options().show_summary && solver->is_parentsolver()) {
    // TODO.
    // print_summary(solver);
  }
}

AbstractSolverTemplate struct UnconstrainedSolver : AbstractSolverDeclare {
  bool is_constrained() final { return false; }
};
AbstractSolverTemplate struct ConstrainedSolver : AbstractSolverDeclare {
  bool is_constrained() final { return true; }
};

#endif
