#ifndef ProblemDeclare_H
#define ProblemDeclare_H

#include <Eigen/Dense>
#include <glog/logging.h>
#include <memory>

#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/integration.h"
#include "robot_dynamics/trajectories.h"

#include "constraint_list.h"
#include "objective.h"

#define ProblemTypeName int Nx, int Nu, typename T
#define ProblemTemplate template <int Nx, int Nu, typename T>
#define ProblemDeclare Problem<Nx, Nu, T>

ProblemTemplate class Problem {
public:
  // Constructors
  Problem() = default;
  Problem(const std::vector<std::shared_ptr<DiscreteDynamics>> &model_in,
          const AbstractObjective *obj_in, ConstraintList constraints_in,
          VectorX<T> x0_in, VectorX<T> xf_in,
          SampledTrajectoryX<Nx, Nu, T> Z_in, int N_in)
      : model(model_in), obj(std::move(obj_in)),
        constraints(std::move(constraints_in)), x0(std::move(x0_in)),
        xf(std::move(xf_in)), Z(std::move(Z_in)), N(N_in) {
  }

  explicit Problem(const ProblemDeclare &prob)
      : obj(prob.obj), constraints(prob.constraints), x0(prob.x0), xf(prob.xf),
        Z(prob.Z), N(prob.N) {
    std::copy(prob.model.begin(), prob.model.end(),
              std::back_inserter(this->model));
  }

  // Initializer
  void init(std::vector<std::shared_ptr<DiscreteDynamics>> models,
            const AbstractObjective *obj_in, VectorX<T> x0_in, double tf_in) {
    this->N = obj_in->length();
    this->x0 = x0_in;
    this->xf = VectorX<T>::Zero(models.back()->state_dim());
    this->model = models;
    this->obj = obj_in;
    this->constraints = ConstraintList(models);

    std::vector<int> nx, nu;
    std::tie(nx, nu) = dims(models);
    const bool same_state_dimension = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx[0]; });
    const bool same_control_dimension = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu[0]; });
    CHECK(same_state_dimension && same_control_dimension);

    std::vector<VectorX<T>> X0, U0;
    for (auto d : nx) {
      X0.push_back(VectorX<T>::Zero(d));
    }
    for (const auto &m : models) {
      U0.push_back(VectorX<T>::Zero(m->control_dim()));
    }

    TrivialParam param;
    param.tf = tf_in;
    param.dt = 0.1;
    param.N = N;
    this->Z = SampledTrajectoryHelper::init<Nx, Nu>(X0, U0, param);
  }

  void init(const std::shared_ptr<DiscreteDynamics> &model,
            const AbstractObjective *obj, VectorX<T> x0, T tf) {
    const auto N = obj->length();
    std::vector<std::shared_ptr<DiscreteDynamics>> models;
    for (auto k = 0; k < N - 1; ++k) {
      models.push_back(model);
    }
    init(models, obj, x0, tf);
  }

  void init(const ContinuousDynamics *model, const AbstractObjective *obj,
            VectorX<T> x0, T tf) {
    std::shared_ptr<DiscreteDynamics> discretized_car(
        new DiscretizedDynamics(model, RK4()));
    init(discretized_car, obj, x0, tf);
  }

  // Members
  int N = 0;
  VectorX<T> x0;
  VectorX<T> xf;
  SampledTrajectoryX<Nx, Nu, T> Z;
  std::vector<std::shared_ptr<DiscreteDynamics>> model;
  const AbstractObjective *obj = nullptr;
  ConstraintList constraints;
};

ProblemTemplate std::tuple<std::vector<int>, std::vector<int>>
dims(const ProblemDeclare &prob) {
  return dims(prob.model);
}

ProblemTemplate auto dims(const ProblemDeclare &prob, int i) {
  int n = 0, m = 0;
  std::tie(n, m) = dims(prob.model[i]);
  return std::make_tuple(n, m, prob.N);
}

ProblemTemplate int state_dim(const ProblemDeclare &prob, int k) {
  return state_dim(prob.model[k]);
}
ProblemTemplate int control_dim(const ProblemDeclare &prob, int k) {
  return control_dim(prob.model[k]);
}

ProblemTemplate int horizonlength(const ProblemDeclare &prob) { return prob.N; }

template <ProblemTypeName, typename... Args>
auto controls(const ProblemDeclare &prob, Args... args) {
  return controls(get_trajectory(prob), args...);
}
template <ProblemTypeName, typename... Args>
auto states(const ProblemDeclare &prob, Args... args) {
  return states(get_trajectory(prob), args...);
}
ProblemTemplate auto gettimes(const ProblemDeclare &prob) {
  return gettimes(get_trajectory(prob));
}

ProblemTemplate auto get_initial_time(const ProblemDeclare &prob) {
  return get_trajectory(prob).front().time();
}
ProblemTemplate T get_final_time(const ProblemDeclare &prob) {
  return get_trajectory(prob).back().time();
}
ProblemTemplate auto get_constraints(const ProblemDeclare &prob) {
  return prob.constraints;
}
ProblemTemplate auto num_constraints(const ProblemDeclare &prob) {
  return get_constraints(prob).p;
}

ProblemTemplate auto get_model(const ProblemDeclare &prob) {
  return prob.model;
}
ProblemTemplate auto get_model(const ProblemDeclare &prob, int k) {
  return prob.model[k];
}
ProblemTemplate auto get_objective(const ProblemDeclare &prob) {
  return prob.obj;
}
ProblemTemplate auto get_trajectory(const ProblemDeclare &prob) {
  return prob.Z;
}
ProblemTemplate auto get_initial_state(const ProblemDeclare &prob) {
  return prob.x0;
}
ProblemTemplate auto get_final_state(const ProblemDeclare &prob) {
  return prob.xf;
}

#endif
