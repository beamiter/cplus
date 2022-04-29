#ifndef ILQR_SOLVER_H
#define ILQR_SOLVER_H

#include "solver.h"
#include "solver_opts.h"

#include <Eigen/Dense>
using Eigen::VectorX;

template <typename T> struct DynamicRegularization {
  T rou;
  T d_rou;
};

template <typename M, typename O, int Nx, int Ne, int Nu, typename T, typename V>
struct iLQRSolver : UNCONSTRAINEDSOLVER {
  M model;
  O obj;
  VectorX<T> x0;
  T tf;
  int N;
  SolverOptions<T> opts;
  SolverStats<T> stats;

};

#endif
