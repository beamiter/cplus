#ifndef PROBLEM_H
#define PROBLEM_H

#include <Eigen/Dense>
#include <glog/logging.h>
#include <memory>

#include "robot_dynamics/discretized_dynamics.h"
#include "robot_dynamics/integration.h"
#include "robot_dynamics/trajectories.h"

#include "constraint_list.h"
#include "objective.h"

template <typename KP> class Problem {
public:
  // Constructors
  Problem() = default;
  Problem(const std::vector<std::shared_ptr<DiscreteDynamics>> &model_in,
          const AbstractObjective *obj_in, ConstraintList constraints_in,
          VectorX<typename KP::base_type> x0_in,
          VectorX<typename KP::base_type> xf_in,
          SampledTrajectoryS<KP::N, KP::M, typename KP::base_type> Z_in,
          int N_in)
      : model(model_in), obj(std::move(obj_in)),
        constraints(std::move(constraints_in)), x0(std::move(x0_in)),
        xf(std::move(xf_in)), Z(std::move(Z_in)), N(N_in) {}

  explicit Problem(const Problem<KP> &prob)
      : obj(prob.obj), constraints(prob.constraints), x0(prob.x0), xf(prob.xf),
        Z(prob.Z), N(prob.N) {
    std::copy(prob.model.begin(), prob.model.end(),
              std::back_inserter(this->model));
  }

  // Initializer
  void init(std::vector<std::shared_ptr<DiscreteDynamics>> models,
            const AbstractObjective *obj_in,
            VectorX<typename KP::base_type> x0_in, double tf_in) {
    this->N = obj_in->length();
    this->x0 = x0_in;
    this->xf =
        VectorX<typename KP::base_type>::Zero(models.back()->state_dim());
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

    std::vector<VectorX<typename KP::base_type>> X0, U0;
    for (auto d : nx) {
      X0.push_back(VectorX<typename KP::base_type>::Zero(d));
    }
    for (const auto &m : models) {
      U0.push_back(VectorX<typename KP::base_type>::Zero(m->control_dim()));
    }

    TrivialParam param;
    param.tf = tf_in;
    param.dt = 0.1;
    param.N = N;
    this->Z = SampledTrajectoryHelper::init<KP::N, KP::M>(X0, U0, param);
  }

  void init(const std::shared_ptr<DiscreteDynamics> &model,
            const AbstractObjective *obj, VectorX<typename KP::base_type> x0,
            typename KP::base_type tf) {
    const auto N = obj->length();
    std::vector<std::shared_ptr<DiscreteDynamics>> models;
    for (auto k = 0; k < N - 1; ++k) {
      models.push_back(model);
    }
    init(models, obj, x0, tf);
  }

  void init(const ContinuousDynamics<KP> *model, const AbstractObjective *obj,
            VectorX<typename KP::base_type> x0, typename KP::base_type tf) {
    std::shared_ptr<DiscreteDynamics> discretized_car(
        new DiscretizedDynamics<RK4, KP>(model));
    init(discretized_car, obj, x0, tf);
  }

  // Members
  // Object length.
  int N = 0;
  Vector<typename KP::base_type, KP::N> x0;
  Vector<typename KP::base_type, KP::N> xf;
  SampledTrajectory<KP> Z;
  std::vector<std::shared_ptr<DiscreteDynamics>> model;
  const AbstractObjective *obj = nullptr;
  ConstraintList constraints;
};

template <typename KP>
std::tuple<std::vector<int>, std::vector<int>> dims(const Problem<KP> &prob) {
  return dims(prob.model);
}

template <typename KP> auto dims(const Problem<KP> &prob, int i) {
  int n = 0, m = 0;
  std::tie(n, m) = dims(prob.model[i]);
  return std::make_tuple(n, m, prob.N);
}

template <typename KP> int state_dim(const Problem<KP> &prob, int k) {
  return state_dim(prob.model[k]);
}
template <typename KP> int control_dim(const Problem<KP> &prob, int k) {
  return control_dim(prob.model[k]);
}

template <typename KP> int horizonlength(const Problem<KP> &prob) {
  return prob.N;
}

template <typename KP, typename... Args>
auto controls(const Problem<KP> &prob, Args... args) {
  return controls(get_trajectory(prob), args...);
}
template <typename KP, typename... Args>
auto states(const Problem<KP> &prob, Args... args) {
  return states(get_trajectory(prob), args...);
}
template <typename KP> auto gettimes(const Problem<KP> &prob) {
  return gettimes(get_trajectory(prob));
}

template <typename KP> auto get_initial_time(const Problem<KP> &prob) {
  return get_trajectory(prob).front().time();
}
template <typename KP>
typename KP::base_type get_final_time(const Problem<KP> &prob) {
  return get_trajectory(prob).back().time();
}
template <typename KP> auto get_constraints(const Problem<KP> &prob) {
  return prob.constraints;
}
template <typename KP> auto num_constraints(const Problem<KP> &prob) {
  return get_constraints(prob).p;
}

template <typename KP> auto get_model(const Problem<KP> &prob) {
  return prob.model;
}
template <typename KP> auto get_model(const Problem<KP> &prob, int k) {
  return prob.model[k];
}
template <typename KP> auto get_objective(const Problem<KP> &prob) {
  return prob.obj;
}
template <typename KP> auto get_trajectory(const Problem<KP> &prob) {
  return prob.Z;
}
template <typename KP> auto get_initial_state(const Problem<KP> &prob) {
  return prob.x0;
}
template <typename KP> auto get_final_state(const Problem<KP> &prob) {
  return prob.xf;
}

#endif
