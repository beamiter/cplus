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
#include "solver_opts.h"
#include "solvers.h"
#include "trajectory_optimization/objective.h"
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

template <typename O> bool usestatic(const O &) {
  return !std::is_base_of<std::vector<double>, typename O::vectype>::value;
}
template <typename T> FunctionSignature dynamics_signature(const T &obj) {
  return usestatic(obj) ? FunctionSignature::StaticReturn
                        : FunctionSignature::Inplace;
}
template <typename T> FunctionSignature function_signature(const T &obj) {
  return usestatic(obj) ? FunctionSignature::StaticReturn
                        : FunctionSignature::Inplace;
}
template <typename KP>
bool usestaticdefault(const AbstractFunction<KP> *model) {
  return model->default_signature() == FunctionSignature::StaticReturn;
}

#define iLQRSolverTypeName typename KP, typename C, bool B
#define iLQRSolverTemplate template <typename KP, typename C, bool B = true>
#define iLQRSolverDeclare iLQRSolver<KP, C, B>

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

  iLQRSolver(
      const std::vector<std::shared_ptr<DiscreteDynamics<KP>>> &model_in,
      const AbstractObjective *obj_in, VectorX<T> x0_in, T tf_in, int N_in,
      SolverOptions<T> opts_in, SolverStats<T> stats_in,
      SampledTrajectoryS<Nx, Nu, T> Z_in,
      SampledTrajectoryS<Nx, Nu, T> Z_dot_in, std::vector<vector_type> dx_in,
      std::vector<vector_type> du_in, std::vector<matrix_type> gains_in,
      std::vector<Ref<matrix_type>> K_in, std::vector<Ref<vector_type>> d_in,
      std::vector<DynamicsExpansion<T>> D_in, std::vector<matrix_type> G_in,
      CostExpansion<T> Efull_in, CostExpansion<T> Eerr_in,
      std::vector<StateControlExpansion<T, true>> Q_in,
      std::vector<StateControlExpansion<T, false>> S_in, std::vector<T> DV_in,
      StateControlExpansion<T, true> Qtmp_in, matrix_type Quu_reg_in,
      matrix_type Qux_reg_in, DynamicRegularization<T> reg_in,
      std::vector<T> grad_in, std::vector<T> xdot_in)
      : model(model_in), obj(obj_in), x0(x0_in), tf(tf_in), N(N_in),
        opts(opts_in), stats_(stats_in), Z(Z_in), Z_dot(Z_dot_in), dx(dx_in),
        du(du_in), gains(gains_in), K_vec(K_in), d_vec(d_in), D_vec(D_in),
        G_vec(G_in), Efull(Efull_in), Eerr(Eerr_in), Q_vec(Q_in), S_vec(S_in),
        DV(DV_in), Qtmp(Qtmp_in), Quu_reg(Quu_reg_in), Qux_reg(Qux_reg_in),
        reg(reg_in), grad(grad_in), xdot(xdot_in) {}

  iLQRSolver(Problem<KP, C> *prob, SolverOptions<T> opts_in,
             SolverStats<T> stats_in, DiffMethod dynamics_diffmethod,
             Valbool<B>) {
    model = prob->model;
    // TODO: Many empty item.
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
      rollout(dynamics_signature(Z), prob->model[0].get(), &Z, prob->x0);
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
      D_vec.push_back(DynamicsExpansion<T>(nx[k], ne[k], nu[k]));
    });

    loop(0, N, [&nx, &ne, this](const int k) {
      MatrixX<T> a(nx[k], ne[k]);
      a.diagonal().setOnes();
      G_vec.push_back(a);
    });

    Eerr = CostExpansion<T>(ne, nu);
    Efull = FullStateExpansion(Eerr, prob->model.front().get());

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

    reset(*this);
  }

  // Overrides.
  SolverName solvername() const final { return SolverName::iLQR; }
  SampledTrajectory<KP> get_trajectory() const final { return Z; }
  const Objective<C> *get_objective() const final { return obj; }
  state_type get_initial_state() const final { return x0; }
  state_type *get_initial_state() final { return &x0; }
  SolverStats<T> stats() const override { return stats_; }
  const SolverOptions<T> &options() const final { return opts; }
  SolverOptions<T> *options() final { return &opts; }

  // Functions.
  int state_dim() { return Nx; }
  int errstate_dim() { return Ne; }
  int control_dim() { return Nu; }
  auto state_dim(int k) { return state_dim(model[k]); }
  auto errstate_dim(int k) { return errstate_dim(model[k]); }
  auto control_dim(int k) { return control_dim(model[k]); }
  auto get_model() { return model; }
  auto get_feedbackgains() { return K_vec; }

  // Members.
  std::vector<std::shared_ptr<DiscreteDynamics<KP>>> model;
  const Objective<C> *obj;

  state_type x0;
  // Vector<T, Nx> x0;
  T tf;
  // Number of horizonlength.
  int N = 0;
  int Ne = 0;

  SolverOptions<T> opts;
  SolverStats<T> stats_;

  SampledTrajectory<KP> Z;
  SampledTrajectory<KP> Z_dot;
  // SampledTrajectoryS<Nx, Nu, T> Z;
  // SampledTrajectoryS<Nx, Nu, T> Z_dot;
  std::vector<vector_type> dx;
  std::vector<vector_type> du;

  std::vector<matrix_type> gains;
  std::vector<Ref<matrix_type>> K_vec;
  std::vector<Ref<vector_type>> d_vec;

  std::vector<DynamicsExpansion<T>> D_vec;
  std::vector<matrix_type> G_vec;

  CostExpansion<T> Efull;
  CostExpansion<T> Eerr;

  std::vector<StateControlExpansion<T, true>> Q_vec;
  std::vector<StateControlExpansion<T, false>> S_vec;

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

iLQRSolverTemplate void dynamics_expansion(iLQRSolverDeclare &solver,
                                           const SampledTrajectory<KP> &Z) {
  const auto &diff = solver.opts.dynamics_diffmethod;
  const auto &model = solver.model;
  for (int k = 0; k < solver.D_vec.size(); ++k) {
    jacobian(dynamics_signature(solver), diff, model[k].get(), &solver.D_vec[k],
             Z[k]);
  }
  error_expansion(solver.model, solver.D_vec, solver.G_vec);
}

#endif
