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

// Knot point type, objective type
template <typename KP, typename C> class Problem {
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;
  using base_type = typename KP::base_type;
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;

public:
  // Constructors
  Problem() = default;
  Problem(const std::vector<std::shared_ptr<DiscreteDynamics<KP>>> &model_in,
          const Objective<C> *obj_in, ConstraintList constraints_in,
          state_type x0_in, state_type xf_in, SampledTrajectory<KP> Z_in,
          int N_in)
      : model(model_in), obj(std::move(obj_in)),
        constraints(std::move(constraints_in)), x0(std::move(x0_in)),
        xf(std::move(xf_in)), Z(std::move(Z_in)), N(N_in) {}

  explicit Problem(const Problem<KP, C> &prob)
      : obj(prob.obj), constraints(prob.constraints), x0(prob.x0), xf(prob.xf),
        Z(prob.Z), N(prob.N) {
    std::copy(prob.model.begin(), prob.model.end(),
              std::back_inserter(this->model));
  }

  // Initializer
  void init(std::vector<std::shared_ptr<DiscreteDynamics<KP>>> models_in,
            const Objective<C> *obj_in, state_type x0_in, double tf_in) {
    this->N = obj_in->length();
    this->x0 = x0_in;
    this->xf = VectorX<base_type>::Zero(models_in.back()->state_dim());
    this->model = models_in;
    this->obj = obj_in;
    this->constraints = ConstraintList(models_in);

    std::vector<int> nx, nu;
    std::tie(nx, nu) = ::dims(models_in);
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
    for (const auto &m : models_in) {
      U0.push_back(VectorX<base_type>::Zero(m->control_dim()));
    }

    TrivialParam param;
    param.tf = tf_in;
    param.dt = 0.1;
    param.N = N;
    this->Z = SampledTrajectory<KP>(X0, U0, param);
    this->Z.setinitialtime(0.0);
  }

  void init(const std::shared_ptr<DiscreteDynamics<KP>> &model_in,
            const Objective<C> *obj_in, state_type x0_in, base_type tf_in) {
    const auto N = obj_in->length();
    std::vector<std::shared_ptr<DiscreteDynamics<KP>>> models;
    for (auto k = 0; k < N - 1; ++k) {
      models.push_back(model_in);
    }
    init(models, obj_in, x0_in, tf_in);
  }

  // Members
  // Object length.
  int N = 0;
  state_type x0;
  state_type xf;
  SampledTrajectory<KP> Z;
  std::vector<std::shared_ptr<DiscreteDynamics<KP>>> model;
  const Objective<C> *obj = nullptr;
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
  std::vector<typename KP::control_type> controls() const {
    return get_trajectory()->controls();
  }
  std::vector<typename KP::state_type> states() const {
    return get_trajectory()->states();
  }
  std::vector<typename KP::base_type> gettimes() const {
    return get_trajectory()->gettimes();
  }
  double get_initial_time() const { return get_trajectory()->front().time(); }
  double get_final_time() const { return get_trajectory()->back().time(); }
  auto get_constraints() { return this->constraints; }
  auto num_constraints() { return get_constraints().p; }
  const std::vector<std::shared_ptr<DiscreteDynamics<KP>>> &get_model() const {
    return this->model;
  }
  const std::shared_ptr<DiscreteDynamics<KP>> &get_model(int k) const {
    return this->model[k];
  }
  const Objective<C> *get_objective() { return this->obj; }
  const SampledTrajectory<KP> *get_trajectory() const { return &this->Z; }
  SampledTrajectory<KP> *get_trajectory() { return &this->Z; }
  bool is_constrained() { return get_constraints().empty(); }
  state_type get_initial_state() const { return this->x0; }
  state_type get_final_state() const { return this->xf; }

  void initial_trajectory(const SampledTrajectory<KP> &Z0) { this->Z = Z0; }
  template <typename P> void initial_states(const P &X0) {
    get_trajectory()->setstates(X0);
  }
  template <typename Q> void initial_controls(const Q &U0) {
    get_trajectory()->setcontrols(U0);
  }

  void set_initial_state(const state_type &x0) { this->x0 = x0; }
  void setinitialtime(double t0) { get_trajectory()->setinitialtime(Z, t0); }
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

  auto cost() { return this->obj->cost(this->Z); }
};

template <typename KP, typename C>
void rollout(FunctionSignature sig,
             const std::vector<std::shared_ptr<DiscreteDynamics<KP>>> &models,
             SampledTrajectory<KP> *Z, const typename KP::state_type &x0) {
  Z->at(0).setstate(x0);
  for (int k = 1; k < Z->length(); ++k) {
    propagate_dynamics(sig, models[k - 1].get(), &Z->at(k), Z->at(k - 1));
  }
}
template <typename KP, typename C>
void rollout(FunctionSignature sig, const Problem<KP, C> *prob) {
  rollout<KP, C>(sig, prob->get_model(),
                 const_cast<Problem<KP, C> *>(prob)->get_trajectory(),
                 prob->get_initial_state());
}
template <typename KP, typename C> void rollout(const Problem<KP, C> *prob) {
  rollout<KP, C>(FunctionSignature::StaticReturn, prob);
}

#endif
