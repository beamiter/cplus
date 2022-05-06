#ifndef PROBLEM_H
#define PROBLEM_H

#include <Eigen/Dense>

#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/integration.h"
#include "robot_dynamics/trajectories.h"

#include "constraint_list.h"
#include "dynamics.h"
#include "objective.h"

#define PROBLEM_PARAM Nx, Nu, T
#define PROBLEM_TYPENAME int Nx, int Nu, typename T
#define PROBLEM_TEMPLATE template <int Nx, int Nu, typename T>
#define PROBLEM Problem<Nx, Nu, T>

PROBLEM_TEMPLATE struct Problem {
  Problem(std::vector<DiscreteDynamics> model_in, AbstractObjective obj_in,
          ConstraintList constraints_in, std::vector<T> x0_in,
          std::vector<T> xf_in,
          SampledTrajectory<Nx, Nu, VectorX<T>, T, KnotPoint> Z_in, int N_in,
          T t0, T tf)
      : model(std::move(model_in)), obj(std::move(obj_in)),
        constraints(std::move(constraints_in)), Z(std::move(Z_in)),
        x0(std::move(x0_in)), xf(std::move(xf_in)) {}

  std::vector<DiscreteDynamics> model;
  AbstractObjective obj;
  ConstraintList constraints;
  std::vector<T> x0;
  std::vector<T> xf;
  SampledTrajectory<Nx, Nu, VectorX<T>, T, KnotPoint> Z;
  int N;
};

struct ProblemHelper {

  template <int Nx, int Nu, typename T>
  static auto init(std::vector<DiscreteDynamics> models, AbstractObjective obj,
                   std::vector<T> x0, double tf) {
    std::vector<T> xf = std::vector<T>(state_dim(models.back()), 0);
    auto constraints = ConstraintList(models);
    auto t0 = zero(tf);
    // X0, U0;
    std::vector<int> nx, nu;
    std::tie(nx, nu) = dims(models);
    const bool same_state_dimension = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx[0]; });
    const bool same_control_dimension = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu[0]; });
    assert(same_state_dimension && same_control_dimension);
    const auto N = length(obj);
    SampledTrajectory<Nx, Nu, VectorX<T>, T, KnotPoint> Z;
    return Problem<Nx, Nu, T>(models, obj, constraints, x0, xf, Z, N, t0, tf);
  }

  template <int Nx, int Nu, typename T, typename C>
  static auto init(DiscreteDynamics model, Objective<C> obj, std::vector<T> x0,
                   T tf) {
    const auto N = length(obj);
    std::vector<DiscreteDynamics> models;
    for (auto k = 0; k < N - 1; ++k) {
      models.push_back(model);
    }
    return init<Nx, Nu, T>(models, obj, x0, tf);
  }

  template <int Nx, int Nu, typename T, typename C>
  static auto init(ContinuousDynamics model, Objective<C> obj,
                   std::vector<T> x0, T tf) {
    auto discrete_model = DiscretizedDynamics(model, RK4());
    return init<Nx, Nu, T, C>(discrete_model, obj, x0, tf);
  }
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

template <PROBLEM_TYPENAME, typename... Args>
auto controls(PROBLEM prob, Args... args) {
  return controls(get_trajectory(prob), args...);
}
template <PROBLEM_TYPENAME, typename... Args>
auto states(PROBLEM prob, Args... args) {
  return states(get_trajectory(prob), args...);
}
PROBLEM_TEMPLATE
auto gettimes(PROBLEM prob) { return gettimes(get_trajectory(prob)); }
PROBLEM_TEMPLATE
auto get_model(PROBLEM prob) { return prob.model; }
PROBLEM_TEMPLATE
auto get_model(PROBLEM prob, int k) { return prob.model[k]; }
PROBLEM_TEMPLATE
auto get_objective(PROBLEM prob) { return prob.obj; }
PROBLEM_TEMPLATE
auto get_trajectory(PROBLEM prob) { return prob.Z; }
PROBLEM_TEMPLATE
auto get_initial_state(PROBLEM prob) { return prob.x0; }
PROBLEM_TEMPLATE
auto get_final_state(PROBLEM prob) { return prob.xf; }

#endif
