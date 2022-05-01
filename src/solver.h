#ifndef SOLVER_H
#define SOLVER_H

enum class SolverName {
  iLQR,
};

template <typename T> struct AbstractSolver {};

#define ABSTRACTSOLVER AbstractSolver<T>

template <typename T> auto statas(ABSTRACTSOLVER solver) {
  return solver.stats;
}

template <typename T> auto iterations(ABSTRACTSOLVER solver) {
  return stats(solver).iterations;
}

template <typename T>
auto options(ABSTRACTSOLVER solver) {
  return solver.opts;
}

template <typename T, typename ... Args>
auto set_options(ABSTRACTSOLVER solver, Args ... args) {
  set_options(options(solver), args...);
}

template <typename T>
auto solvername(ABSTRACTSOLVER solver) {
  return solvername(solver);
} 

template <typename T>
auto is_parentsolver(ABSTRACTSOLVER solver) {
  return stats(solver).parent == solvername(solver); 
}

template <typename T>
auto resetstats(ABSTRACTSOLVER solver) {
  reset(stats(solver), iterations(solver), solvername(solver));
}

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

template <typename T>
auto status(ABSTRACTSOLVER solver) {
  return stats(solver).status;
}

template <typename T>
struct UnconstrainedSolver : ABSTRACTSOLVER {};
#define UNCONSTRAINEDSOLVER UnconstrainedSolver<T>

template <typename T>
struct ConstrainedSolver : ABSTRACTSOLVER {};
#define CONSTRAINEDSOLVER ConstrainedSolver<T>

template <typename T>
auto is_constrained(ABSTRACTSOLVER) {
  return true;
}
template <typename T>
auto is_constrained(UNCONSTRAINEDSOLVER) {
  return false;
}
template <typename T>
auto is_constrained(CONSTRAINEDSOLVER) {
  return true;
}

#endif
