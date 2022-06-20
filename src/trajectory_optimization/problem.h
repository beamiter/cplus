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

namespace {} // namespace

template <typename KP> class Problem {
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;
  using base_type = typename KP::base_type;
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;

public:
  // Constructors
  Problem() = default;
  Problem(const std::vector<std::shared_ptr<DiscreteDynamics>> &model_in,
          const AbstractObjective *obj_in, ConstraintList constraints_in,
          state_type x0_in, state_type xf_in, SampledTrajectory<KP> Z_in,
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
            const AbstractObjective *obj_in, state_type x0_in, double tf_in) {
    this->N = obj_in->length();
    this->x0 = x0_in;
    this->xf = VectorX<base_type>::Zero(models.back()->state_dim());
    this->model = models;
    this->obj = obj_in;
    this->constraints = ConstraintList(models);

    std::vector<int> nx, nu;
    std::tie(nx, nu) = ::dims(models);
    const bool same_state_dimension = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx[0]; });
    const bool same_control_dimension = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu[0]; });
    CHECK(same_state_dimension && same_control_dimension);

    std::vector<state_type> X0;
    std::vector<control_type> U0;
    for (auto d : nx) {
      X0.push_back(VectorX<base_type>::Zero(d));
    }
    for (const auto &m : models) {
      U0.push_back(VectorX<base_type>::Zero(m->control_dim()));
    }

    TrivialParam param;
    param.tf = tf_in;
    param.dt = 0.1;
    param.N = N;
    this->Z = SampledTrajectoryHelper::init<Nx, Nu>(X0, U0, param);
    ::setinitialtime<KP>(Z, 0.0);
  }

  void init(const std::shared_ptr<DiscreteDynamics> &model,
            const AbstractObjective *obj, state_type x0, base_type tf) {
    const auto N = obj->length();
    std::vector<std::shared_ptr<DiscreteDynamics>> models;
    for (auto k = 0; k < N - 1; ++k) {
      models.push_back(model);
    }
    init(models, obj, x0, tf);
  }

  void init(const ContinuousDynamics<KP> *model, const AbstractObjective *obj,
            state_type, base_type tf) {
    std::shared_ptr<DiscreteDynamics> discretized_car(
        new DiscretizedDynamics<RK4, KP>(model));
    init(discretized_car, obj, x0, tf);
  }

  // Members
  // Object length.
  int N = 0;
  state_type x0;
  state_type xf;
  SampledTrajectory<KP> Z;
  std::vector<std::shared_ptr<DiscreteDynamics>> model;
  const AbstractObjective *obj = nullptr;
  ConstraintList constraints;

  std::tuple<std::vector<int>, std::vector<int>> dims() const {
    return ::dims(model);
  }
  std::tuple<int, int, int> dims(int i) const {
    int n = 0, m = 0;
    std::tie(n, m) = dims(model[i]);
    return std::make_tuple(n, m, N);
  }
  int state_dim(int k) const { return state_dim(model[k]); }
  int control_dim(int k) const { return control_dim(model[k]); }
  int horizonlength() const { return N; }
  auto controls() { return controls(get_trajectory()); }
  auto states() { return states(get_trajectory()); }
  auto gettimes() { return gettimes(get_trajectory()); }
  auto get_initial_time() { return get_trajectory().front().time(); }
  auto get_final_time() { return get_trajectory().back().time(); }
  auto get_constraints() { return this->constraints; }
  auto num_constraints() { return get_constraints().p; }
  auto get_model() { return this->model; }
  auto get_model(int k) { return this->model[k]; }
  auto get_objective() { return this->obj; }
  auto get_trajectory() { return this->Z; }
  bool is_constrained() { return get_constraints().empty(); }
  auto get_initial_state() { return this->x0; }
  auto get_final_state() { return this->xf; }
  void initial_trajectory(const SampledTrajectory<KP> &Z0) { this->Z = Z0; }
  template <typename P> void initial_states(const P &X0) {
    setstates(get_trajectory(), X0);
  }
  template <typename Q> void initial_controls(const Q &U0) {
    setcontrols(get_trajectory(), U0);
  }
  void set_initial_state(const state_type &x0) { this->x0 = x0; }
  void setinitialtime(double t0) {
    auto Z = get_trajectory();
    setinitialtime(Z, t0);
  }
  void set_goal_state(const state_type &xf, bool objective = true,
                      bool constraint = true) {
    if (objective) {
      auto obj = get_objective();
      for (int k = 0; k < obj.cost.size(); ++k) {
        set_LQR_goal(obj[k], xf);
      }
    }
    if (constraint) {
      for (auto &con : get_constraints()) {
        // if (con isa GoalConstraint) {
        // set_goal_state(con, xf);
        //}
      }
    }
    this->xf = xf;
  }

  auto cost() { return cost(this->obj, this->Z); }
};

template <typename KP> auto rollout(const Problem<KP> &prob) {
  return rollout(FunctionSignature::StaticReturn, prob);
}
template <typename KP>
auto rollout(FunctionSignature sig, const Problem<KP> &prob) {
  return rollout(sig, prob.get_model(), prob.get_trajectory(),
                 prob.get_initial_state());
}
template <typename KP>
auto rollout(FunctionSignature sig, std::vector<DiscreteDynamics> &models,
             SampledTrajectory<KP> Z, typename KP::state_type x0) {
  Z[0].setstate(x0);
  for (int k = 1; k < length(Z); ++k) {
    propagate_dynamics(sig, models[k - 1], Z[k], Z[k - 1]);
  }
}

#endif
