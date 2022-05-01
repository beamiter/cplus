#ifndef PROBLEM_H
#define PROBLEM_H

#include <Eigen/Dense>

#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/trajectories.h"

#include "trajectory_optimization/objective.h"

using Eigen::MatrixX;

template <typename T> struct Problem {
  DiscreteDynamics model;
  AbstractObjective obj;
  MatrixX<T> x0;
  MatrixX<T> xf;
  // SampledTrajectory Z;
  int N;
  T t0;
  T tf;
};

#endif
