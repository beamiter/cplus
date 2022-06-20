#ifndef SOLVER_H
#define SOLVER_H

#include "base/base.h"
#include "solver_opts.h"

template <typename T> class AbstractSolver {
public:
  virtual SolverName solvername() = 0;
};

template <typename T> auto stats(AbstractSolver<T> solver) {
  return solver.stats;
}
template <typename T> auto iterations(AbstractSolver<T> solver) {
  return stats(solver).iterations;
}
template <typename T> auto options(AbstractSolver<T> solver) {
  return solver.opts;
}
template <typename T>
void set_options(AbstractSolver<T> solver, SolverOptions<T>) {
  CHECK(0);
}
template <typename T> SolverName solvername(AbstractSolver<T>) { CHECK(0); };
template <typename T> bool is_parentsolver(AbstractSolver<T> solver) {
  stats(solver).parent == solvername(solver);
};
template <typename T> void resetstats(AbstractSolver<T> solver) {
  reset(stats(solver), iterations(solver), solvername(solver));
}

template <typename T> void reset_solver(AbstractSolver<T> solver) {
  auto opts = options(solver);
  reset(stats(solver), opts.iterations, solvername(solver));
  if (is_constrained(solver)) {
    reset(get_constraints(solver), opts);
  }
}
template <typename T> auto status(AbstractSolver<T> solver) {
  stats(solver).status;
}

template <typename T> struct UnconstrainedSolver : AbstractSolver<T> {};

template <typename T> struct ConstrainedSolver : AbstractSolver<T> {};

template <typename T> bool is_constrained(const AbstractSolver<T> &) {
  return true;
}
template <typename T> bool is_constrained(const UnconstrainedSolver<T> &) {
  return false;
}
template <typename T> bool is_constrained(const ConstrainedSolver<T> &) {
  return true;
}

template <typename T, typename Traj>
void cost(AbstractSolver<T> solver, Traj Z) {
  auto obj = get_objective(solver);
  cost(obj, Z);
}

template <typename T> void rollout(AbstractSolver<T> solver) {
  auto ilqr = get_ilqr(solver);
  rollout(ilqr);
}
template <typename T> auto states(AbstractSolver<T> solver) {
  // TODO: save state in vector
  for (const auto &z : get_trajectory(solver)) {
    z.state();
  }
}

template <typename T> auto controls(const AbstractSolver<T> &solver) {
  int N = std::get<2>(dims(solver));
  auto Z = get_trajectory(solver);
  // TODO: save control in vector
  for (int i = 0; i < N - 1; ++i) {
    Z[i].control();
  }
}

template <typename T, typename P>
void set_initial_state(AbstractSolver<T> *solver, P x0) {
  get_initial_state(solver) = x0;
}

template <typename T, typename P>
void initial_states(const AbstractSolver<T> &solver, P X0) {
  setstates(get_trajectory(solver), X0);
}

template <typename T, typename P>
void initial_controls(const AbstractSolver<T> &solver, P X0) {
  setcontrols(get_trajectory(solver), X0);
}

template <typename T, typename Traj>
void initial_trajectory(const AbstractSolver<T> &solver, Traj Z0) {
  auto Z = get_trajectory(solver);
  for (int k = 0; k < Z.size(); ++k) {
    setdata(Z[k], Z0[k].z);
  }
}

template <typename T> auto gettimes(const AbstractSolver<T> &solver) {
  return gettimes(get_trajectory(solver));
}

template <typename T> auto num_constraints(const AbstractSolver<T> &solver) {
  return num_constraints(get_constraints(solver));
}

#endif
