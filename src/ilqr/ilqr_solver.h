#ifndef ILQR_SOLVER_H
#define ILQR_SOLVER_H

#include <Eigen/Dense>

#include "base/base.h"
#include "cost_expansion.h"
#include "dynamics_expansion.h"
#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "solver.h"
#include "solver_opts.h"
#include "trajectory_optimization/problem.h"

using Eigen::Map;
using Eigen::MatrixX;
using Eigen::Ref;
using Eigen::VectorX;
using Eigen::VectorXd;

template <bool B, typename T> struct StaticArray {
  typedef std::vector<T> type;
};
template <typename T> struct StaticArray<true, T> { typedef VectorX<T> type; };

template <typename T> class DynamicRegularization {
public:
  DynamicRegularization() = default;
  DynamicRegularization(T rou_in, T d_rou_in) : rou(rou_in), d_rou(d_rou_in) {}

  void operator=(const DynamicRegularization &rhs) { *this = rhs; }
  T rou;
  T d_rou;
};

template <bool> auto dynamics_signature() { return Inplace(); }
template <> inline auto dynamics_signature<true>() { return StaticReturn(); }
template <bool> auto function_signature() { return Inplace(); }
template <> inline auto function_signature<true>() { return StaticReturn(); }

// template <typename T>  auto dynamics_signature(T obj) {
//   return  usestatic(obj) ? StaticReturn() : Inplace();
// }
//
// template <typename T> auto function_signature(T obj) {
//   return usestatic(obj) ? StaticReturn() : Inplace();
// }
//

#define ILQR_SOLVER_TYPENAME                                                   \
  int Nx, int Ne, int Nu, typename T, typename V, AbstractModelTypeName
#define ILQR_SOLVER_TEMPLATE                                                   \
  template <int Nx, int Ne, int Nu, typename T, typename V,                    \
            AbstractModelTypeName, bool USE_STATIC = true>
#define ILQR_SOLVER iLQRSolver<Nx, Ne, Nu, T, V, F, S>

ILQR_SOLVER_TEMPLATE class iLQRSolver : UnconstrainedSolver<T> {
public:
  using vectype = V;
  using m_data_type = MatrixX<T>;
  using v_data_type = VectorX<T>;

  iLQRSolver(std::vector<const DiscreteDynamicsDeclare *> model_in,
             AbstractObjective obj_in, std::vector<T> x0_in, T tf_in, int N_in,
             SolverOptions<T> opts_in, SolverStats<T> stats_in,
             SampledTrajectory<Nx, Nu, V, T, KnotPoint<Nx, Nu, V, T>> Z_in,
             SampledTrajectory<Nx, Nu, V, T, KnotPoint<Nx, Nu, V, T>> Z_dot_in,
             std::vector<v_data_type> dx_in, std::vector<v_data_type> du_in,
             std::vector<m_data_type> gains_in,
             std::vector<Ref<m_data_type>> K_in,
             std::vector<Ref<v_data_type>> d_in,
             std::vector<DynamicsExpansion<T>> D_in,
             std::vector<m_data_type> G_in, CostExpansion<T> Efull_in,
             CostExpansion<T> Eerr_in,
             std::vector<StateControlExpansion<T, true>> Q_in,
             std::vector<StateControlExpansion<T, false>> S_in,
             std::vector<T> DV_in, StateControlExpansion<T> Qtmp_in,
             m_data_type Quu_reg_in, m_data_type Qux_reg_in,
             DynamicRegularization<T> reg_in, std::vector<T> grad_in,
             std::vector<T> xdot_in)
      : model(model_in), obj(obj_in), x0(x0_in), tf(tf_in), N(N_in),
        opts(opts_in), stats(stats_in), Z(Z_in), Z_dot(Z_dot_in), dx(dx_in),
        du(du_in), gains(gains_in), K(K_in), d(d_in), D(D_in), G(G_in),
        Efull(Efull_in), Eerr(Eerr_in), Q_vec(Q_in), S_vec(S_in), DV(DV_in),
        Qtmp(Qtmp_in), Quu_reg(Quu_reg_in), Qux_reg(Qux_reg_in), reg(reg_in),
        grad(grad_in), xdot(xdot_in) {}

  iLQRSolver(ProblemDeclare prob, SolverOptions<T> opts, SolverStats<T> stats,
             DiffMethod dynamics_diffmethod, ValBool<USE_STATIC>, ValInt<Ne>) {
    model = prob.model;
    obj = prob.obj;
    x0 = prob.x0;
    tf = get_final_time(prob);
    N = horizonlength(prob);
    opts = opts;
    stats = stats;
    Z = prob.Z;
    Z_dot = Z;

    std::vector<int> nx, nu, ne;
    std::tie(nx, nu) = dims(prob);
    for (const auto &m : prob.model) {
      ne.push_back(m->errstate_dim());
    }
    ne.push_back(ne.back());

    const bool samestatedim = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx[0]; });
    const bool samecontroldim = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu[0]; });
    if (USE_STATIC) {
      assert(samecontroldim && samestatedim);
      assert(Nx == nx[0]);
      assert(Nu == nu[0]);
      assert(Ne == ne[0]);
    } else {
      assert(Nx == samestatedim ? nx[0] : Nx);
      assert(Nu == samestatedim ? nu[0] : Nu);
      assert(Ne == samecontroldim ? ne[0] : 0);
    }

    if (std::any_of(Z[0].state().begin(), Z[0].state().end(),
                    [](const auto &s) { return std::isnan(s); })) {
      // Change std vector to Eigen Vector;
      VectorX<T> v = Map<Eigen::VectorX<T>>(prob.x0.data(), prob.x0.size());
      // rollout(dynamics_signature<UseStatic<
      // typename SampledTrajectoryX<Nx, Nu, T>::value_type>::val>(),
      // prob.model[0], Z, v);
    }
    VectorX<T> v = Map<Eigen::VectorX<T>>(prob.x0.data(), prob.x0.size());
    Z[0].setstate(v);

    loop(0, N,
         [&ne, this](const int k) { dx.push_back(VectorX<T>::Zero(ne[k])); });

    loop(0, N - 1,
         [&nu, this](const int k) { du.push_back(VectorX<T>::Zero(nu[k])); });

    loop(0, N - 1, [&ne, &nu, this](const int k) {
      gains.push_back(MatrixX<T>::Zero(nu[k], ne[k] + 1));
    });

    loop(0, gains.size(), [&ne, this](const int k) {
      K.push_back(gains[k](all, seq(0, last - 1)));
      d.push_back(gains[k](all, last));
    });

    loop(0, N - 1, [&nx, &ne, &nu, this](const int k) {
      D.push_back(DynamicsExpansion<T>::init(nx[k], ne[k], nu[k]));
    });

    loop(0, N, [&nx, &ne, this](const int k) {
      MatrixX<T> a(nx[k], ne[k]);
      a.diagonal().setOnes();
      G.push_back(a);
    });

    Eerr(ne, nu);
    Efull = FullStateExpansion(Eerr, prob.model[0]);

    loop(0, N, [&ne, &nu, this](const int k) {
      Q_vec.push_back(StateControlExpansionHelper<T>()(ne[k], nu[k]));
    });

    loop(0, N, [&ne, this](const int k) {
      S_vec.push_back(StateControlExpansionHelper<T>()(ne[k]));
    });

    DV = std::vector<T>(2, 0);

    *Qtmp = StateControlExpansionHelper<T>()(ne[0], nu[0]);
    Quu_reg = MatrixX<T>::Zero(nu[0], nu[0]);
    Qux_reg = MatrixX<T>::Zero(nu[0], ne[0]);
    reg = DynamicRegularization<T>(opts.bp_reg_initial, 0);
    grad = std::vector<T>(N - 1, 0);
    xdot = std::vector<T>(nx[0], 0);
  }

  std::vector<const DiscreteDynamicsDeclare *> model;
  AbstractObjective obj;

  std::vector<T> x0;
  T tf;
  int N;

  SolverOptions<T> opts;
  SolverStats<T> stats;

  SampledTrajectory<Nx, Nu, V, T, KnotPoint<Nx, Nu, V, T>> Z;
  SampledTrajectory<Nx, Nu, V, T, KnotPoint<Nx, Nu, V, T>> Z_dot;
  std::vector<v_data_type> dx;
  std::vector<v_data_type> du;

  std::vector<m_data_type> gains;
  std::vector<Ref<m_data_type>> K;
  std::vector<Ref<v_data_type>> d;

  std::vector<DynamicsExpansion<T>> D;
  std::vector<m_data_type> G;

  CostExpansion<T> Efull;
  CostExpansion<T> Eerr;

  std::vector<StateControlExpansion<T, true>> Q_vec;
  std::vector<StateControlExpansion<T, false>> S_vec;

  std::vector<T> DV;

  StateControlExpansion<T, true> *Qtmp = nullptr;
  m_data_type Quu_reg;
  m_data_type Qux_reg;
  DynamicRegularization<T> reg;

  std::vector<T> grad;
  std::vector<T> xdot;
};

ILQR_SOLVER_TEMPLATE
struct iLQRSolverHelper {};

ILQR_SOLVER_TEMPLATE
auto dims(ILQR_SOLVER solver) { return dims(solver.Z); }

ILQR_SOLVER_TEMPLATE
auto state_dim(ILQR_SOLVER) { return Nx; }

ILQR_SOLVER_TEMPLATE
auto errstate_dim(ILQR_SOLVER) { return Ne; }

ILQR_SOLVER_TEMPLATE
auto control_dim(ILQR_SOLVER) { return Nu; }

ILQR_SOLVER_TEMPLATE
auto state_dim(ILQR_SOLVER solver, int k) { return state_dim(solver.model[k]); }

ILQR_SOLVER_TEMPLATE
auto errstate_dim(ILQR_SOLVER solver, int k) {
  return errstate_dim(solver.model[k]);
}

ILQR_SOLVER_TEMPLATE
auto control_dim(ILQR_SOLVER solver, int k) {
  return control_dim(solver.model[k]);
}

ILQR_SOLVER_TEMPLATE
auto get_trajectory(ILQR_SOLVER solver) { return solver.Z; }

ILQR_SOLVER_TEMPLATE
auto get_objective(ILQR_SOLVER solver) { return solver.obj; }

ILQR_SOLVER_TEMPLATE
auto get_model(ILQR_SOLVER solver) { return solver.model; }

ILQR_SOLVER_TEMPLATE
auto get_initial_state(ILQR_SOLVER solver) { return solver.x0; }

ILQR_SOLVER_TEMPLATE
auto solvername(ILQR_SOLVER) { return SolverName::iLQR; }

ILQR_SOLVER_TEMPLATE
auto get_feedbackgains(ILQR_SOLVER solver) { return solver.K; }

AbstractFunctionTemplate inline auto
usestaticdefault(const AbstractFunctionDeclare &model) {
  return model.default_signature() == StaticReturn();
}

#endif
