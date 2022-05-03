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

using Eigen::MatrixX;
using Eigen::Ref;
using Eigen::VectorX;
using Eigen::VectorXd;

template <typename T> struct DynamicRegularization {
  T rou;
  T d_rou;
};

#define ILQR_SOLVER_TYPENAME                                                   \
  typename L, typename O, int Nx, int Ne, int Nu, typename T, typename V
#define ILQR_SOLVER_TEMPLATE template <ILQR_SOLVER_TYPENAME>
#define ILQR_SOLVER iLQRSolver<L, O, Nx, Ne, Nu, T, V>

struct iLQRSolverHelper {
template <int Nx, int Nu, typename T> 
 static auto init(Problem<Nx, Nu, T> prob, SolverOptions<T> opts, SolverStats<T> stats,
            Val<bool> use_static, DiffMethod dynamics_diffmethod, ...) {
    // set_options(opts);

    std::vector<int> nx, nu, ne;
    std::tie(nx, nu) = dims(prob);
    auto N = horizonlength(prob);
    for (const auto& m : prob.model) {
      ne.push_back(errstate_dim(m));
    }
    ne.push_back(ne.back());

    auto x0 = prob.x0;
    const bool samestatedim = true;
    const bool samecontroldim = true;
    if (use_static == Val<bool>(true)) {
    } else {
    }
    return 1;
  }
};

ILQR_SOLVER_TEMPLATE struct iLQRSolver : UNCONSTRAINEDSOLVER {
  using vectype = V;
  using m_data_type = MatrixX<T>;
  using v_data_type = VectorX<T>;

  L model;
  O obj;

  VectorX<T> x0;
  T tf;
  int N;

  SolverOptions<T> opts;
  SolverStats<T> stats;

  SampledTrajectory<Nx, Nu, T, KNOT_POINT> Z;
  SampledTrajectory<Nx, Nu, T, KNOT_POINT> z_dot;
  VectorX<VectorX<T>> dx;
  VectorX<VectorX<T>> du;

  VectorX<MatrixX<T>> gains;
  Ref<m_data_type> K;
  Ref<v_data_type> d;

  VectorX<DynamicsExpansion<T>> D;
  VectorX<MatrixX<T>> G;

  CostExpansion<T> Efull;
  CostExpansion<T> Eerr;

  VectorX<StateControlExpansion<T>> Q;
  VectorX<StateControlExpansion<T>> S;

  VectorX<T> d_V;

  StateControlExpansion<T> Qtmp;
  MatrixX<T> Quu_reg;
  MatrixX<T> Qux_reg;
  DynamicsExpansion<T> reg;

  VectorX<T> grad;
  VectorX<T> xdot;
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

template <typename T> auto usestatic(T obj) {
  return is_same_type<typename ValueType<T>::value_type,
                      typename VectorXd::value_type>::value;
}

template <typename T> auto dynamics_signature(T obj) {
  return usestatic(obj) ? StaticReturn() : Inplace();
}

template <typename T> auto function_signature(T obj) {
  return usestatic(obj) ? StaticReturn() : Inplace();
}

inline auto usestaticdefault(AbstractFunction model) {
  return default_signature(model) == StaticReturn();
}

#endif
