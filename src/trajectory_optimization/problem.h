#ifndef ProblemDeclare_H
#define ProblemDeclare_H

#include <Eigen/Dense>

#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/integration.h"
#include "robot_dynamics/trajectories.h"

#include "constraint_list.h"
#include "objective.h"

#define ProblemTypeName int Nx, int Nu, typename T, AbstractModelTypeName
#define ProblemTemplate                                                        \
  template <int Nx, int Nu, typename T, AbstractModelTypeName>
#define ProblemDeclare Problem<Nx, Nu, T, F, S>

ProblemTemplate struct Problem {
  Problem(std::vector<const DiscreteDynamicsDeclare *> model_in,
          AbstractObjective obj_in, ConstraintList constraints_in,
          std::vector<T> x0_in, std::vector<T> xf_in,
          SampledTrajectoryX<Nx, Nu, T> Z_in, int N_in, T t0, T tf)
      : model(model_in), obj(std::move(obj_in)),
        constraints(std::move(constraints_in)), Z(std::move(Z_in)),
        x0(std::move(x0_in)), xf(std::move(xf_in)) {}

  std::vector<const DiscreteDynamicsDeclare *> model;
  AbstractObjective obj;
  ConstraintList constraints;
  std::vector<T> x0;
  std::vector<T> xf;
  SampledTrajectoryX<Nx, Nu, T> Z;
  int N;
};

struct ProblemHelper {
  template <int Nx, int Nu, typename T, AbstractModelTypeName>
  static ProblemDeclare
  init(std::vector<const DiscreteDynamicsDeclare *> models,
       AbstractObjective obj, std::vector<T> x0, double tf) {
    std::vector<T> xf = std::vector<T>(models.back()->state_dim(), 0);
    auto constraints = ConstraintList(models);
    auto t0 = zero(tf);

    std::vector<int> nx, nu;
    std::tie(nx, nu) = dims(models);
    const bool same_state_dimension = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx[0]; });
    const bool same_control_dimension = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu[0]; });
    assert(same_state_dimension && same_control_dimension);

    std::vector<VectorX<T>> X0, U0;
    for (auto d : nx) {
      X0.push_back(VectorX<T>::Zero(d));
    }
    for (const auto &m : models) {
      U0.push_back(VectorX<T>::Zero(m->control_dim()));
    }

    const auto N = length(obj);
    TrivialParam param;
    param.tf = 5.0;
    param.dt = 0.1;
    param.N = N;
    auto Z = SampledTrajectoryHelper::init<Nx, Nu>(X0, U0, param);
    return ProblemDeclare(models, obj, constraints, x0, xf, Z, N, t0, tf);
  }

  template <int Nx, int Nu, typename T, typename C, AbstractModelTypeName>
  static ProblemDeclare init(const DiscreteDynamicsDeclare *model,
                             Objective<C> obj, std::vector<T> x0, T tf) {
    const auto N = length(obj);
    std::vector<const DiscreteDynamicsDeclare *> models;
    for (auto k = 0; k < N - 1; ++k) {
      models.push_back(model);
    }
    return init<Nx, Nu, T>(models, obj, x0, tf);
  }

  template <int Nx, int Nu, typename T, typename C, AbstractModelTypeName>
  static ProblemDeclare init(const ContinuousDynamicsDeclare &model,
                             Objective<C> obj, std::vector<T> x0, T tf) {
    auto discretized_model = DiscretizedDynamicsDeclare(model, RK4());
    return init<Nx, Nu, T, C>(&discretized_model, obj, x0, tf);
  }
};

ProblemTemplate std::tuple<std::vector<int>, std::vector<int>>
dims(ProblemDeclare prob) {
  return dims(prob.model);
}

ProblemTemplate auto dims(ProblemDeclare prob, int i) {
  int n = 0, m = 0;
  std::tie(n, m) = dims(prob.model[i]);
  return std::make_tuple(n, m, prob.N);
}

ProblemTemplate int state_dim(ProblemDeclare prob, int k) {
  return state_dim(prob.model[k]);
}
ProblemTemplate int control_dim(ProblemDeclare prob, int k) {
  return control_dim(prob.model[k]);
}

ProblemTemplate int horizonlength(ProblemDeclare prob) { return prob.N; }

template <ProblemTypeName, typename... Args>
auto controls(ProblemDeclare prob, Args... args) {
  return controls(get_trajectory(prob), args...);
}
template <ProblemTypeName, typename... Args>
auto states(ProblemDeclare prob, Args... args) {
  return states(get_trajectory(prob), args...);
}
ProblemTemplate auto gettimes(ProblemDeclare prob) {
  return gettimes(get_trajectory(prob));
}

ProblemTemplate auto get_initial_time(ProblemDeclare prob) {
  return get_trajectory(prob).front().time();
}
ProblemTemplate T get_final_time(ProblemDeclare prob) {
  return get_trajectory(prob).back().time();
}
ProblemTemplate auto get_constraints(ProblemDeclare prob) {
  return prob.constraints;
}
ProblemTemplate auto num_constraints(ProblemDeclare prob) {
  return get_constraints(prob).p;
}

ProblemTemplate auto get_model(ProblemDeclare prob) { return prob.model; }
ProblemTemplate auto get_model(ProblemDeclare prob, int k) {
  return prob.model[k];
}
ProblemTemplate auto get_objective(ProblemDeclare prob) { return prob.obj; }
ProblemTemplate auto get_trajectory(ProblemDeclare prob) { return prob.Z; }
ProblemTemplate auto get_initial_state(ProblemDeclare prob) { return prob.x0; }
ProblemTemplate auto get_final_state(ProblemDeclare prob) { return prob.xf; }

#endif
