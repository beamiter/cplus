#ifndef PROBLEM_H
#define PROBLEM_H

#include <Eigen/Dense>

#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/trajectories.h"
#include "dynamics.h"

#include "trajectory_optimization/objective.h"

using Eigen::MatrixX;

#define PROBLEM_PARAM Nx, Nu, T
#define PROBLEM_TYPENAME int Nx, int Nu, typename T
#define PROBLEM_TEMPLATE template <PROBLEM_TYPENAME>

#define PROBLEM Problem<PROBLEM_PARAM>
PROBLEM_TEMPLATE struct Problem {
  std::vector<DiscreteDynamics> model;
  AbstractObjective obj;
  MatrixX<T> x0;
  MatrixX<T> xf;
  SampledTrajectory<Nx, Nu, T, KnotPoint<Nx, Nu, VectorX<T>, T>> Z;
  int N;
};

PROBLEM_TEMPLATE auto dims(PROBLEM prob) { return dims(prob.model); }

PROBLEM_TEMPLATE auto dims(PROBLEM prob, int i) {
  int n = 0, m = 0;
  std::tie(n, m) = dims(prob.model[i]);
  return std::make_tuple(n, m, prob.N);
}

PROBLEM_TEMPLATE auto state_dim(PROBLEM prob, int k) {
  return state_dim(prob.model[k]);
}
PROBLEM_TEMPLATE auto control_dim(PROBLEM prob, int k) {
  return control_dim(prob.model[k]);
}

PROBLEM_TEMPLATE auto horizonlength(PROBLEM prob) { return prob.N; }

template<PROBLEM_TYPENAME, typename ... Args>
auto controls(PROBLEM prob, Args ... args) {
  return controls(get_trajectory(prob), args...);
}
template <PROBLEM_TYPENAME , typename ... Args>
auto states(PROBLEM prob, Args ... args) {
  return states(get_trajectory(prob), args...);
}
PROBLEM_TEMPLATE
auto gettimes(PROBLEM prob) {
  return gettimes(get_trajectory(prob));
}
PROBLEM_TEMPLATE
auto get_model(PROBLEM prob) {
  return prob.model;
}
PROBLEM_TEMPLATE
auto get_model(PROBLEM prob, int k) {
  return prob.model[k];
}
PROBLEM_TEMPLATE
auto get_objective(PROBLEM prob) {
  return prob.obj;
}
PROBLEM_TEMPLATE
auto get_trajectory(PROBLEM prob) {
  return prob.Z;
}
PROBLEM_TEMPLATE
auto get_initial_state(PROBLEM prob) {
  return prob.x0;
}
PROBLEM_TEMPLATE
auto get_final_state(PROBLEM prob) {
  return prob.xf;
}

#endif
