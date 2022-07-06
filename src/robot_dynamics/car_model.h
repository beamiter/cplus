#ifndef CAR_MODEL_
#define CAR_MODEL_

#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/util/Constants.h>

#include "base/base.h"
#include "dynamics.h"
#include "robot_dynamics/discrete_dynamics.h"
#include "trajectory_optimization/problem.h"

using Eigen::DiagonalMatrix;
using Eigen::Map;

enum class RefPos {
  rear,
  cg,
  fg,
};

template <typename KP> class CarModel : public ContinuousDynamics<KP> {
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;

public:
  virtual ~CarModel() = default;
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;
  CarModel(RefPos ref_in = RefPos::rear, double L_in = 2.7,
           double lr_in = 1.5) {
    this->ref = ref_in;
    this->L = L_in;
    this->lr = lr_in;
  }
  int state_dim() const final { return Nx; }
  int control_dim() const final { return Nu; }
  state_type dynamics(const state_type &x, const control_type &u) const final {
    const auto &da = u[0];
    const auto &phi = u[1];

    const auto &theta = x[2];
    const auto &delta = x[3];
    const auto &v = x[4];
    const auto &a = x[5];
    double beta, s, c, omega;
    if (RefPos::cg == ref) {
      beta = std::atan2(lr * delta, L);
      s = sin(theta + beta);
      c = cos(theta + beta);
      omega = v * cos(beta) * tan(delta) / L;
    } else if (RefPos::rear == ref) {
      s = sin(theta);
      c = cos(theta);
      omega = v * tan(delta) / L;
    } else if (RefPos::fg == ref) {
      s = sin(theta + delta);
      c = cos(theta + delta);
      omega = v * sin(delta) / L;
    } else {
      CHECK(0);
    }
    const auto xd = v * c;
    const auto yd = v * s;
    return state_type(xd, yd, omega, phi, a, da);
  }
  void dynamics(state_type *xdot, const state_type &x,
                const control_type &u) const final {
    const auto &da = u[0];
    const auto &phi = u[1];

    const auto &theta = x[2];
    const auto &delta = x[3];
    const auto &v = x[4];
    const auto &a = x[5];
    double beta, s, c, omega;
    if (RefPos::cg == ref) {
      beta = std::atan2(lr * delta, L);
      s = sin(theta + beta);
      c = cos(theta + beta);
      omega = v * cos(beta) * tan(delta) / L;
    } else if (RefPos::rear == ref) {
      s = sin(theta);
      c = cos(theta);
      omega = v * tan(delta) / L;
    } else if (RefPos::fg == ref) {
      s = sin(theta + delta);
      c = cos(theta + delta);
      omega = v * sin(delta) / L;
    } else {
      CHECK(0);
    }
    const auto xd = v * c;
    const auto yd = v * s;
    *xdot << xd, yd, omega, phi, a, da;
  }
  void jacobian(typename KP::jacobian_type &jaco,
                const typename KP::state_type &y,
                const typename KP::state_type &x,
                const typename KP::control_type &u) const final {
    CHECK(0);
  }

  RefPos ref;
  double L = 0.0;
  double lr = 0.0;
};

template <typename KP, typename C> class CarProblem : public Problem<KP, C> {
  using base_type = typename KP::base_type;
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;

public:
  CarProblem(std::vector<base_type> x0_in, std::vector<base_type> xf_in,
             std::vector<base_type> uf_in, int N, double tf) {
    VectorXd x0 = Map<VectorXd>(x0_in.data(), x0_in.size());
    VectorXd xf = Map<VectorXd>(xf_in.data(), xf_in.size());
    VectorXd uf = Map<VectorXd>(uf_in.data(), uf_in.size());
    init(x0, xf, uf, N, tf);
  }
  void init(const state_type &x0, const state_type &xf, const control_type &uf,
            int N, double tf) {
    DiagonalMatrix<double, Nx> Q(10, 10, 50, 1, 1, 1);
    auto R = DiagonalMatrix<double, Nu>(1, 1);
    const double rho = 1.0;
    R = rho * R;
    DiagonalMatrix<double, Nx> Qf(10, 10, 60, 1, 1, 1);

    obj_ = std::make_shared<Objective<C>>(
        LQRObjective<Nx, Nu, base_type>(Q, R, Qf, xf, uf, N));
    // Model initialize.
    car_ = std::make_shared<CarModel<KP>>(CarModel<KP>());
    Problem<KP, C>::init(car_.get(), obj_.get(), x0, tf);

    // initial_controls, initial_states, rollout
  }

  // Members.
  std::shared_ptr<CarModel<KP>> car_;
  std::shared_ptr<Objective<C>> obj_;
};
template <int Nx, int Nu, typename T, template <int, int, typename> class C>
using CarProblemX = CarProblem<KnotPointX<Nx, Nu, T>, C<Nx, Nu, T>>;
template <int Nx, int Nu, template <int, int, typename> class C>
using CarProblemXd = CarProblem<KnotPointXd<Nx, Nu>, C<Nx, Nu, double>>;
template <int Nx, int Nu, typename T, template <int, int, typename> class C>
using CarProblemS = CarProblem<KnotPointS<Nx, Nu, T>, C<Nx, Nu, T>>;
template <int Nx, int Nu, template <int, int, typename> class C>
using CarProblemSd = CarProblem<KnotPointSd<Nx, Nu>, C<Nx, Nu, double>>;

#endif
