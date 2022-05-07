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

template <typename T> struct DynamicRegularization {
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
  typename L, typename O, int Nx, int Ne, int Nu, typename T, typename V
#define ILQR_SOLVER_TEMPLATE                                                   \
  template <typename L, typename O, int Nx, int Ne, int Nu, typename T,        \
            typename V>
#define ILQR_SOLVER iLQRSolver<L, O, Nx, Ne, Nu, T, V>

struct iLQRSolverHelper {
  template <int Nx, int Nu, bool USE_STATIC, typename T>
  static auto init(Problem<Nx, Nu, T> prob, SolverOptions<T> opts,
                   SolverStats<T> stats, ValBool<USE_STATIC>,
                   DiffMethod dynamics_diffmethod, ...) {
    std::vector<int> nx, nu, ne;
    std::tie(nx, nu) = dims(prob);
    auto N = horizonlength(prob);
    for (const auto &m : prob.model) {
      ne.push_back(errstate_dim(m));
    }
    ne.push_back(ne.back());

    auto x0 = prob.x0;
    const bool samestatedim = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx[0]; });
    const bool samecontroldim = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu[0]; });
    auto Nx_tmp = Nx;
    auto Nu_tmp = Nu;
    auto Ne_tmp = 0;
    if (USE_STATIC) {
      assert(samecontroldim && samestatedim);
      Nx_tmp = nx[0];
      Nu_tmp = nu[0];
      Ne_tmp = ne[0];
    } else {
      Nx_tmp = samestatedim ? nx[0] : Nx;
      Nu_tmp = samestatedim ? nx[0] : Nu;
      Ne_tmp = samecontroldim ? nu[0] : 0;
    }
    typedef typename StaticArray<USE_STATIC, T>::type V;
    auto Z = prob.Z;
    auto d_Z = Z;

    if (std::any_of(state(Z[0]).begin(), state(Z[0]).end(),
                    [](const auto &s) { return std::isnan(s); })) {
      // Change std vector to Eigen Vector;
      VectorX<T> v = Map<Eigen::VectorX<T>>(prob.x0.data(), prob.x0.size());
      rollout(dynamics_signature<UseStatic<
                  typename SampledTrajectoryX<Nx, Nu, T>::value_type>::val>(),
              prob.model[0], Z, v);
    }
    VectorX<T> v = Map<Eigen::VectorX<T>>(prob.x0.data(), prob.x0.size());
    setstate(Z[0], v);

    std::vector<VectorX<T>> dx, du;
    loop(0, N,
         [&ne, &dx](const int k) { dx.push_back(VectorX<T>::Zero(ne[k])); });
    loop(0, N - 1,
         [&nu, &du](const int k) { du.push_back(VectorX<T>::Zero(nu[k])); });

    std::vector<MatrixX<T>> gains;
    loop(0, N - 1, [&ne, &nu, &gains](const int k) {
      gains.push_back(MatrixX<T>::Zero(nu[k], ne[k] + 1));
    });
    std::vector<Ref<MatrixX<T>>> K;
    std::vector<Ref<VectorX<T>>> d;
    loop(0, gains.size(), [&ne, &gains, &K, &d](const int k) {
      K.push_back(gains[k](all, seq(0, last - 1)));
      d.push_back(gains[k](all, last));
    });
    std::vector<DynamicsExpansion<T>> D;
    loop(0, N - 1, [&nx, &ne, &nu, &D](const int k) {
      D.push_back(DynamicsExpansion<T>::init(nx[k], ne[k], nu[k]));
    });
    std::vector<MatrixX<T>> G;

    return 1;
  }
};

ILQR_SOLVER_TEMPLATE struct iLQRSolver : UNCONSTRAINEDSOLVER {
  using vectype = V;
  using m_data_type = MatrixX<T>;
  using v_data_type = VectorX<T>;

  L model;
  O obj;

  std::vector<T> x0;
  T tf;
  int N;

  SolverOptions<T> opts;
  SolverStats<T> stats;

  SampledTrajectory<Nx, Nu, V, T, KnotPoint<Nx, Nu, V, T>> Z;
  SampledTrajectory<Nx, Nu, V, T, KnotPoint<Nx, Nu, V, T>> z_dot;
  std::vector<v_data_type> dx;
  std::vector<v_data_type> du;

  std::vector<m_data_type> gains;
  std::vector<Ref<m_data_type>> K;
  std::vector<Ref<v_data_type>> d;

  std::vector<DynamicsExpansion<T>> D;
  std::vector<m_data_type> G;

  CostExpansion<T> Efull;
  CostExpansion<T> Eerr;

  std::vector<StateControlExpansion<T>> Q;
  std::vector<StateControlExpansion<T>> S;

  std::vector<T> DV;

  StateControlExpansion<T> Qtmp;
  m_data_type Quu_reg;
  m_data_type Qux_reg;
  DynamicsExpansion<T> reg;

  std::vector<T> grad;
  std::vector<T> xdot;
};

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

inline auto usestaticdefault(AbstractFunction model) {
  return default_signature(model) == StaticReturn();
}

#endif
