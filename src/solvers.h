#ifndef SOLVER_H
#define SOLVER_H

#include "base/base.h"
#include "solver_opts.h"

class AbstractSolver {
public:
  virtual SolverName solvername() = 0;
  // virtual bool usestatic() { return true; }
  // virtual FunctionSignature dynamics_signature() {
  //   return usestatic() ? FunctionSignature::StaticReturn
  //                      : FunctionSignature::Inplace;
  // }
  // virtual FunctionSignature function_signature() {
  //   return usestatic() ? FunctionSignature::StaticReturn
  //                      : FunctionSignature::Inplace;
  // }
};

// template <typename T> auto iterations(AbstractSolverDeclare solver) {
//   return stats(solver).iterations;
// }

// template <typename T> auto options(AbstractSolverDeclare solver) {
//   return solver.opts;
// }

// template <typename T, typename... Args>
// auto set_options(AbstractSolverDeclare solver, Args... args) {
//   set_options(options(solver), args...);
// }

// template <typename T> auto is_parentsolver(AbstractSolverDeclare solver) {
//   return stats(solver).parent == solvername(solver);
// }

// template <typename T> auto resetstats(AbstractSolverDeclare solver) {
//   reset(stats(solver), iterations(solver), solvername(solver));
// }

// template <typename T> auto status(AbstractSolverDeclare solver) {
//   return stats(solver).status;
// }

struct UnconstrainedSolver : AbstractSolver {};

struct ConstrainedSolver : AbstractSolver {};

inline bool is_constrained(const AbstractSolver &) { return true; }
inline bool is_constrained(const UnconstrainedSolver &) { return false; }
inline bool is_constrained(const ConstrainedSolver &) { return true; }

#endif
