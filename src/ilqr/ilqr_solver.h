#ifndef ILQR_SOLVER_H
#define ILQR_SOLVER_H

#include <Eigen/Dense>
#include <type_traits>

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

// template <bool B, typename T> struct StaticArray {
//   typedef std::vector<T> type;
// };
// template <typename T> struct StaticArray<true, T> { typedef VectorX<T> type;
// };

template <typename T> class DynamicRegularization {
public:
  DynamicRegularization() = default;
  DynamicRegularization(T rou_in, T d_rou_in) : rou(rou_in), d_rou(d_rou_in) {}
  DynamicRegularization(const DynamicRegularization &in)
      : rou(in.rou), d_rou(in.d_rou) {}
  T rou;
  T d_rou;
};

// template <bool> auto dynamics_signature() { return
// FunctionSignature::Inplace; } template <> inline auto
// dynamics_signature<true>() {
//   return FunctionSignature::StaticReturn;
// }
// template <bool> auto function_signature() { return
// FunctionSignature::Inplace; } template <> inline auto
// function_signature<true>() {
//   return FunctionSignature::StaticReturn;
// }

template <typename O, typename T> bool usestatic(O) {
  return std::is_base_of<typename O::veltype, MatrixX<T>>::value;
}
template <typename T> auto dynamics_signature(T obj) {
  return usestatic(obj) ? FunctionSignature::StaticReturn
                        : FunctionSignature::Inplace;
}
template <typename T> auto function_signature(T obj) {
  return usestatic(obj) ? FunctionSignature::StaticReturn
                        : FunctionSignature::Inplace;
}

#define iLQRSolverTypeName int Nx, int Nu, typename T, typename V, bool B
#define iLQRSolverTemplate                                                     \
  template <int Nx, int Nu, typename T, typename V, bool B = true>
#define iLQRSolverDeclare iLQRSolver<Nx, Nu, T, V, B>

iLQRSolverTemplate class iLQRSolver : UnconstrainedSolverDeclare {
public:
  using vectype = V;
  using m_data_type = MatrixX<T>;
  using v_data_type = VectorX<T>;

  iLQRSolver(
      const std::vector<std::shared_ptr<DiscreteDynamics>> &model_in,
      const AbstractObjective *obj_in, VectorX<T> x0_in, T tf_in, int N_in,
      SolverOptions<T> opts_in, SolverStats<T> stats_in,
      SampledTrajectoryS<Nx, Nu, T> Z_in,
      SampledTrajectoryS<Nx, Nu, T> Z_dot_in, std::vector<v_data_type> dx_in,
      std::vector<v_data_type> du_in, std::vector<m_data_type> gains_in,
      std::vector<Ref<m_data_type>> K_in, std::vector<Ref<v_data_type>> d_in,
      std::vector<DynamicsExpansion<T>> D_in, std::vector<m_data_type> G_in,
      CostExpansion<T> Efull_in, CostExpansion<T> Eerr_in,
      std::vector<StateControlExpansion<T, true>> Q_in,
      std::vector<StateControlExpansion<T, false>> S_in, std::vector<T> DV_in,
      StateControlExpansion<T, true> Qtmp_in, m_data_type Quu_reg_in,
      m_data_type Qux_reg_in, DynamicRegularization<T> reg_in,
      std::vector<T> grad_in, std::vector<T> xdot_in)
      : model(model_in), obj(obj_in), x0(x0_in), tf(tf_in), N(N_in),
        opts(opts_in), stats(stats_in), Z(Z_in), Z_dot(Z_dot_in), dx(dx_in),
        du(du_in), gains(gains_in), K_vec(K_in), d_vec(d_in), D_vec(D_in),
        G_vec(G_in), Efull(Efull_in), Eerr(Eerr_in), Q_vec(Q_in), S_vec(S_in),
        DV(DV_in), Qtmp(Qtmp_in), Quu_reg(Quu_reg_in), Qux_reg(Qux_reg_in),
        reg(reg_in), grad(grad_in), xdot(xdot_in) {}

  iLQRSolver(ProblemDeclare *prob, SolverOptions<T> opts_in,
             SolverStats<T> stats_in, DiffMethod dynamics_diffmethod,
             Valbool<B>) {
    model = prob->model;
    // TODO: Many empty item.
    obj = prob->obj;
    x0 = prob->x0;
    tf = get_final_time(*prob);
    N = horizonlength(*prob);
    opts = opts_in;
    stats = stats_in;
    Z = prob->Z;
    Z_dot = Z;

    std::vector<int> nx, nu, ne;
    std::tie(nx, nu) = dims(*prob);
    for (const auto &m : prob->model) {
      ne.push_back(m->errstate_dim());
    }
    ne.push_back(ne.back());

    const bool samestatedim = std::all_of(
        nx.begin(), nx.end(), [&nx](const auto x) { return x == nx.front(); });
    const bool samecontroldim = std::all_of(
        nu.begin(), nu.end(), [&nu](const auto u) { return u == nu.front(); });
    if constexpr (B) {
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
      // Change std vector to Eigen Vector;
      VectorX<T> v = Map<Eigen::VectorX<T>>(prob->x0.data(), prob->x0.size());
      // rollout(dynamics_signature<UseStatic<
      // typename SampledTrajectoryX<Nx, Nu, T>::value_type>::val>(),
      // prob.model[0], Z, v);
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

    loop(0, gains.size(), [&ne, this](const int k) {
      K_vec.push_back(gains[k](all, seq(0, last - 1)));
      d_vec.push_back(gains[k](all, last));
    });

    loop(0, N - 1, [&nx, &ne, &nu, this](const int k) {
      D_vec.push_back(DynamicsExpansion<T>::init(nx[k], ne[k], nu[k]));
    });

    loop(0, N, [&nx, &ne, this](const int k) {
      MatrixX<T> a(nx[k], ne[k]);
      a.diagonal().setOnes();
      G_vec.push_back(a);
    });

    Eerr = std::make_unique<CostExpansion<T>>(CostExpansion<T>(ne, nu));
    Efull = std::make_unique<CostExpansion<T>>(
        FullStateExpansion(*Eerr.get(), prob->model.front().get()));

    loop(0, N, [&ne, &nu, this](const int k) {
      Q_vec.push_back(StateControlExpansion<T, true>(ne[k], nu[k]));
    });

    loop(0, N, [&ne, this](const int k) {
      S_vec.push_back(StateControlExpansion<T, false>(ne[k]));
    });

    DV = std::vector<T>(2, 0);

    Qtmp = std::make_unique<StateControlExpansion<T, true>>(
        StateControlExpansion<T, true>(ne[0], nu[0]));
    Quu_reg = MatrixX<T>::Zero(nu[0], nu[0]);
    Qux_reg = MatrixX<T>::Zero(nu[0], ne[0]);
    reg = DynamicRegularization<T>(opts.bp_reg_initial, 0);
    grad = std::vector<T>(N - 1, 0);
    xdot = std::vector<T>(nx[0], 0);
  }

  std::vector<std::shared_ptr<DiscreteDynamics>> model;
  const AbstractObjective *obj;

  Vector<T, Nx> x0;
  T tf;
  int N;
  int Ne;

  SolverOptions<T> opts;
  SolverStats<T> stats;

  SampledTrajectoryS<Nx, Nu, T> Z;
  SampledTrajectoryS<Nx, Nu, T> Z_dot;
  std::vector<v_data_type> dx;
  std::vector<v_data_type> du;

  std::vector<m_data_type> gains;
  std::vector<Ref<m_data_type>> K_vec;
  std::vector<Ref<v_data_type>> d_vec;

  std::vector<DynamicsExpansion<T>> D_vec;
  std::vector<m_data_type> G_vec;

  std::unique_ptr<CostExpansion<T>> Efull;
  std::unique_ptr<CostExpansion<T>> Eerr;

  std::vector<StateControlExpansion<T, true>> Q_vec;
  std::vector<StateControlExpansion<T, false>> S_vec;

  std::vector<T> DV;

  std::unique_ptr<StateControlExpansion<T, true>> Qtmp = nullptr;
  m_data_type Quu_reg;
  m_data_type Qux_reg;
  DynamicRegularization<T> reg;

  std::vector<T> grad;
  std::vector<T> xdot;
};
template <int Nx, int Nu, typename V>
using iLQRSolverD = iLQRSolver<Nx, Nu, double, V>;
template <int Nx, int Nu>
using iLQRSolverXd = iLQRSolver<Nx, Nu, double, VectorXd>;
template <int Nx, int Nu, typename T>
using iLQRSolverX = iLQRSolver<Nx, Nu, T, VectorX<T>>;

iLQRSolverTemplate struct iLQRSolverHelper {};

iLQRSolverTemplate auto dims(iLQRSolverDeclare solver) {
  return dims(solver.Z);
}

iLQRSolverTemplate auto state_dim(iLQRSolverDeclare) { return Nx; }

iLQRSolverTemplate auto errstate_dim(const iLQRSolverDeclare &ilqr) {
  return ilqr.Ne;
}

iLQRSolverTemplate auto control_dim(iLQRSolverDeclare) { return Nu; }

iLQRSolverTemplate auto state_dim(iLQRSolverDeclare solver, int k) {
  return state_dim(solver.model[k]);
}

iLQRSolverTemplate auto errstate_dim(iLQRSolverDeclare solver, int k) {
  return errstate_dim(solver.model[k]);
}

iLQRSolverTemplate auto control_dim(iLQRSolverDeclare solver, int k) {
  return control_dim(solver.model[k]);
}

iLQRSolverTemplate auto get_trajectory(iLQRSolverDeclare solver) {
  return solver.Z;
}

iLQRSolverTemplate auto get_objective(iLQRSolverDeclare solver) {
  return solver.obj;
}

iLQRSolverTemplate auto get_model(iLQRSolverDeclare solver) {
  return solver.model;
}

iLQRSolverTemplate auto get_initial_state(iLQRSolverDeclare solver) {
  return solver.x0;
}

iLQRSolverTemplate auto solvername(iLQRSolverDeclare) {
  return SolverName::iLQR;
}

iLQRSolverTemplate auto get_feedbackgains(iLQRSolverDeclare solver) {
  return solver.K;
}

inline bool usestaticdefault(const AbstractFunction *model) {
  return model->default_signature() == FunctionSignature::StaticReturn;
}

#endif
