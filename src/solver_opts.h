#ifndef SOLVER_OPTS_H
#define SOLVER_OPTS_H

#include <vector>

template <typename T> struct AbstractSolverOptions {};

template<typename T>
struct SolverOptions : AbstractSolverOptions<T> {
  T constraint_tolerance = 1e-6;
  T cost_tolerance = 1e-4;
  T cost_tolerance_intermediate = 1e-4;
  T gradient_tolerance = 10.0;
  T gradient_tolerance_intermediate = 1.0;
};

template<typename T>
struct SolverStats {
  int iterations = 0;
  int iterations_outer = 0;
  int iterations_pn = 0;

  std::vector<int> iteration;
  std::vector<int> iteration_outer;
  std::vector<bool> iteration_pn;
  std::vector<T> cost;

};

#endif
