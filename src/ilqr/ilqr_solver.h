#ifndef ILQR_SOLVER_H
#define ILQR_SOLVER_H

#include <Eigen/Dense>
#include <iomanip>
#include <type_traits>

#include "base/base.h"
#include "cost_expansion.h"
#include "dynamics_expansion.h"
#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "solver_opts.h"
#include "solvers.h"
#include "trajectory_optimization/objective.h"
#include "trajectory_optimization/problem.h"

using Eigen::Map;
using Eigen::MatrixX;
using Eigen::Ref;
using Eigen::VectorX;
using Eigen::VectorXd;

template <typename T> class DynamicRegularization {
public:
  DynamicRegularization() = default;
  DynamicRegularization(T rou_in, T d_rou_in) : rou(rou_in), d_rou(d_rou_in) {}
  DynamicRegularization(const DynamicRegularization &in)
      : rou(in.rou), d_rou(in.d_rou) {}
  T rou;
  T d_rou;
};

// First way.
// template <typename O>
// typename std::enable_if<UseStatic<typename O::vectype>::value,
//                         StaticReturn>::type
// dynamics_signature(const O &obj) {
//   return StaticReturn();
// }
// template <typename O>
// typename std::enable_if<!UseStatic<typename O::vectype>::value,
// Inplace>::type dynamics_signature(const O &obj) {
//   return Inplace();
// }
// Second way.
template <typename O>
auto dynamics_signature(const O &obj) ->
    typename std::conditional<UseStatic<typename O::vectype>::value,
                              StaticReturn, Inplace>::type {
  if constexpr (UseStatic<typename O::vectype>::value) {
    return StaticReturn();
  } else {
    return Inplace();
  }
  // Third way.
  // typedef typename std::conditional<UseStatic<typename O::vectype>::value,
  //                                   StaticReturn, Inplace>::type type;
  // return type();
}

template <typename KP>
bool usestaticdefault(const AbstractFunction<KP> *model) {
  return default_signature(model) == StaticReturn();
}

#define iLQRSolverTypeName typename KP, typename C, bool USE
#define iLQRSolverTemplate template <typename KP, typename C, bool USE = true>
#define iLQRSolverDeclare iLQRSolver<KP, C, USE>

iLQRSolverTemplate class iLQRSolver : public UnconstrainedSolver<KP, C> {
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;
  using T = typename KP::base_type;
  using state_type = typename KP::state_type;
  using matrix_type = MatrixX<typename KP::base_type>;
  using vector_type = VectorX<typename KP::base_type>;

public:
  using vectype = typename KP::value_type;
  virtual ~iLQRSolver() = default;

  iLQRSolver(Problem<KP, C> *prob, SolverOptions<T> opts_in,
             SolverStats<T> stats_in, DiffMethod diff, Valbool<USE>) {
    shared_models_ = prob->shared_models_;
    obj = prob->obj;
    x0 = prob->x0;
    tf = prob->get_final_time();
    N = prob->horizonlength();
    opts = opts_in;
    stats_ = stats_in;
    Z = prob->Z;
    Z_dot = Z;

    std::vector<int> nx, nu, ne;
    std::tie(nx, nu) = prob->dims();
    for (const auto &m : prob->shared_models_) {
      ne.push_back(::errstate_dim(m.get()));
    }
    ne.push_back(ne.back());

    const bool samestatedim = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx.front(); });
    const bool samecontroldim = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu.front(); });
    if constexpr (USE) {
      assert(samecontroldim && samestatedim);
      assert(Nx == nx.front());
      assert(Nu == nu.front());
      this->Ne = ne.front();
    } else {
      assert(Nx == (samestatedim ? nx.front() : Nx));
      assert(Nu == (samecontroldim ? nu.front() : Nu));
      this->Ne = samestatedim ? ne.front() : 0;
    }

    if (std::any_of(Z[0].state().begin(), Z[0].state().end(),
                    [](const auto &s) { return std::isnan(s); })) {
      rollout(dynamics_signature(Z), prob->shared_models_[0].get(), &Z,
              prob->x0);
    }
    VectorX<T> v = Map<Eigen::VectorX<T>>(prob->x0.data(), prob->x0.size());
    Z[0].setstate(v);

    loop(0, N,
         [&ne, this](const int k) { dx.push_back(VectorX<T>::Zero(ne[k])); });

    loop(0, N - 1,
         [&nu, this](const int k) { du.push_back(VectorX<T>::Zero(nu[k])); });

    loop(0, N - 1, [&ne, &nu, this](const int k) {
      gains.push_back(MatrixX<T>::Zero(nu[k], ne[k] + 1));
    });

    loop(0, gains.size(), [this](const int k) {
      K_vec.push_back(gains[k](all, seq(0, last - 1)));
      d_vec.push_back(gains[k](all, last));
    });

    loop(0, N - 1, [&nx, &ne, &nu, this](const int k) {
      D_vec.push_back(
          std::make_unique<DynamicsExpansion<T>>(nx[k], ne[k], nu[k]));
    });

    loop(0, N, [&nx, &ne, this](const int k) {
      MatrixX<T> a(nx[k], ne[k]);
      a.diagonal().setOnes();
      G_vec.push_back(a);
    });

    Eerr_ = std::make_shared<CostExpansion<T>>(ne, nu);
    Efull_ = FullStateExpansion(Eerr_, prob->shared_models_.front().get());

    loop(0, N, [&ne, &nu, this](const int k) {
      Q_vec.push_back(
          std::make_unique<StateControlExpansion<T, true>>(ne[k], nu[k]));
    });

    loop(0, N, [&ne, this](const int k) {
      S_vec.push_back(std::make_unique<StateControlExpansion<T, false>>(ne[k]));
    });

    DV = std::vector<T>(2, 0);

    Qtmp = std::make_unique<StateControlExpansion<T, true>>(
        StateControlExpansion<T, true>(ne[0], nu[0]));
    Quu_reg = MatrixX<T>::Zero(nu[0], nu[0]);
    Qux_reg = MatrixX<T>::Zero(nu[0], ne[0]);
    reg = DynamicRegularization<T>(opts.bp_reg_initial, 0);
    grad = std::vector<T>(N - 1, 0);
    xdot = std::vector<T>(nx[0], 0);

    reset(*this);
  }

  // Overrides.
  const SolverName solvername() const final { return SolverName::iLQR; }
  const SampledTrajectory<KP> &get_trajectory() const final { return Z; }
  SampledTrajectory<KP> &get_trajectory() final { return Z; }
  const Objective<C> *get_objective() const final { return obj; }
  state_type get_initial_state() const final { return x0; }
  const SolverStats<T> &stats() const final { return stats_; }
  SolverStats<T> &stats() final { return stats_; }
  const SolverOptions<T> &options() const final { return opts; }
  SolverOptions<T> &options() final { return opts; }

  // Functions.
  int state_dim() { return Nx; }
  int errstate_dim() { return Ne; }
  int control_dim() { return Nu; }
  int state_dim(int k) { return state_dim(shared_models_[k]); }
  int errstate_dim(int k) { return errstate_dim(shared_models_[k]); }
  int control_dim(int k) { return shared_models_[k]->control_dim(); }
  const std::vector<std::shared_ptr<DiscreteDynamics<KP>>> &get_model() {
    return shared_models_;
  }
  const std::vector<Ref<matrix_type>> &get_feedbackgains() { return K_vec; }

  // Members.
  std::vector<std::shared_ptr<DiscreteDynamics<KP>>> shared_models_;
  const Objective<C> *obj;

  state_type x0;
  T tf;
  // Number of horizonlength.
  int N = 0;
  int Ne = 0;

  SolverOptions<T> opts;
  SolverStats<T> stats_;

  // TODO: Use shared_ptr for this.
  SampledTrajectory<KP> Z;
  SampledTrajectory<KP> Z_dot;
  std::vector<vector_type> dx;
  std::vector<vector_type> du;

  std::vector<matrix_type> gains;      // N-1 * (Nu, Ne+1)
  std::vector<Ref<matrix_type>> K_vec; // N-1 * (Nu, Ne)
  std::vector<Ref<vector_type>> d_vec; // N-1 * (Nu,)

  std::vector<std::unique_ptr<DynamicsExpansion<T>>> D_vec;
  std::vector<matrix_type> G_vec; // N * (Nx,Ne)

  // TODO: Link this to error cost expansion.
  std::shared_ptr<CostExpansion<T>> Efull_; // Cost expansion (full state)
  std::shared_ptr<CostExpansion<T>> Eerr_;  // Cost expansion (error state)

  std::vector<std::unique_ptr<StateControlExpansion<T, true>>>
      Q_vec; // Action-value expansion
  std::vector<std::unique_ptr<StateControlExpansion<T, false>>>
      S_vec; // Action-value expansion

  std::vector<T> DV;

  std::unique_ptr<StateControlExpansion<T, true>> Qtmp = nullptr;
  matrix_type Quu_reg;
  matrix_type Qux_reg;
  DynamicRegularization<T> reg;

  std::vector<T> grad;
  std::vector<T> xdot;
};
template <int Nx, int Nu, typename T, template <int, int, typename> class C>
using iLQRSolverX = iLQRSolver<KnotPointX<Nx, Nu, T>, C<Nx, Nu, T>>;
template <int Nx, int Nu, template <int, int, typename> class C>
using iLQRSolverXd = iLQRSolver<KnotPointXd<Nx, Nu>, C<Nx, Nu, double>>;
template <int Nx, int Nu, typename T, template <int, int, typename> class C>
using iLQRSolverS = iLQRSolver<KnotPointS<Nx, Nu, T>, C<Nx, Nu, T>>;
template <int Nx, int Nu, template <int, int, typename> class C>
using iLQRSolverSd = iLQRSolver<KnotPointSd<Nx, Nu>, C<Nx, Nu, double>>;

iLQRSolverTemplate void reset(iLQRSolverDeclare &solver) {
  reset(solver.stats_, solver.opts.iterations, solver.solvername());
  solver.reg.rou = solver.opts.bp_reg_initial;
  solver.reg.d_rou = 0.0;
}

iLQRSolverTemplate void dynamics_expansion(iLQRSolverDeclare *solver,
                                           const SampledTrajectory<KP> &Z) {
  const auto &diff = solver->opts.dynamics_diffmethod;
  // const auto &shared_models = solver->shared_models_;
  for (int k = 0; k < solver->D_vec.size(); ++k) {
    jacobian(dynamics_signature(*solver), diff, solver->shared_models_[k].get(),
             solver->D_vec[k].get(), Z[k]);
  }
  error_expansion(solver->shared_models_, solver->D_vec, solver->G_vec);
}

iLQRSolverTemplate auto increaseregularization(iLQRSolverDeclare *solver)
    -> decltype(solver->reg) {
  auto &reg = solver->reg;
  const auto rou_dot = solver->opts.bp_reg_increase_factor;
  const auto rou_min = solver->opts.bp_reg_min;
  reg.d_rou = std::max(reg.d_rou * rou_dot, rou_dot);
  reg.rou = std::max(reg.rou * reg.d_rou, rou_min);
  return reg;
}

iLQRSolverTemplate auto decreaseregularization(iLQRSolverDeclare *solver)
    -> decltype(solver->reg) {
  auto &reg = solver->reg;
  const auto rou_dot = solver->opts.bp_reg_increase_factor;
  const auto rou_min = solver->opts.bp_reg_min;
  reg.d_rou = std::min(reg.d_rou / rou_dot, 1 / rou_dot);
  reg.rou = std::max(rou_min, reg.rou * reg.d_rou);
  return reg;
}

#endif
