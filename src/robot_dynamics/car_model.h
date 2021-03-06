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

template <typename KP, RefPos RP>
class CarModel : public ContinuousDynamics<KP> {
  // https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357
public:
  virtual ~CarModel() = default;
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;
  CarModel(double L_in, double lr_in) {
    this->L = L_in;
    this->lr = lr_in;
  }
  int state_dim() const final { return Nx; }
  int control_dim() const final { return Nu; }
  typename KP::state_type
  dynamics(const typename KP::state_type &x,
           const typename KP::control_type &u) const final {
    const auto &da = u[0];
    const auto &phi = u[1];

    const auto &theta = x[2];
    const auto &delta = x[3];
    const auto &v = x[4];
    const auto &a = x[5];
    double beta, s, c, omega;
    if constexpr (RefPos::cg == RP) {
      const double tan3 = tan(delta);
      beta = std::atan2(lr * tan3, L);
      s = sin(theta + beta);
      c = cos(theta + beta);
      omega = v * cos(beta) * tan3 / L;
    } else if constexpr (RefPos::rear == RP) {
      s = sin(theta);
      c = cos(theta);
      omega = v * tan(delta) / L;
    } else if constexpr (RefPos::fg == RP) {
      s = sin(theta + delta);
      c = cos(theta + delta);
      omega = v * sin(delta) / L;
    } else {
      CHECK(0);
    }
    const auto xd = v * c;
    const auto yd = v * s;
    return typename KP::state_type(xd, yd, omega, phi, a, da);
  }
  void dynamics(typename KP::ref_vector_type xdot,
                const typename KP::state_type &x,
                const typename KP::control_type &u) const final {
    const auto &da = u[0];
    const auto &phi = u[1];

    const auto &theta = x[2];
    const auto &delta = x[3];
    const auto &v = x[4];
    const auto &a = x[5];
    double beta, s, c, omega;
    if constexpr (RefPos::cg == RP) {
      const double tan3 = tan(delta);
      beta = std::atan2(lr * tan3, L);
      s = sin(theta + beta);
      c = cos(theta + beta);
      omega = v * cos(beta) * tan3 / L;
    } else if constexpr (RefPos::rear == RP) {
      s = sin(theta);
      c = cos(theta);
      omega = v * tan(delta) / L;
    } else if constexpr (RefPos::fg == RP) {
      s = sin(theta + delta);
      c = cos(theta + delta);
      omega = v * sin(delta) / L;
    } else {
      CHECK(0);
    }
    const auto xd = v * c;
    const auto yd = v * s;
    xdot << xd, yd, omega, phi, a, da;
  }
  void jacobian(typename KP::ref_matrix_type jaco,
                typename KP::ref_vector_type y,
                const typename KP::state_type &x,
                const typename KP::control_type &u) const final {
    if constexpr (RefPos::rear == RP) {
      // Works great.
      const double s2 = sin(x(2));
      const double c2 = cos(x(2));
      const double c3 = cos(x(3));
      const double s3 = sin(x(3));
      const double tan3 = s3 / c3;
      jaco(0, 2) = -s2 * x(4);
      jaco(0, 4) = c2;
      jaco(1, 2) = c2 * x(4);
      jaco(1, 4) = s2;
      jaco(2, 4) = tan3 / L;
      jaco(2, 3) = x(4) / (std::pow(c3, 2) * L);
      jaco(3, 7) = 1;
      jaco(4, 5) = 1;
      jaco(5, 6) = 1;
    } else if constexpr (RefPos::cg == RP) {
      // TODO: Jacobian with this is not stable, need to fix on it later.
      const double ratio = lr / L;
      // Wrong gradient for beta!
      const double c3 = cos(x(3));
      const double s3 = sin(x(3));
      // const double tan3 = tan(x(3));
      const double tan3 = s3 / c3;
      const double beta_param = ratio * tan3;
      const double beta = std::atan(beta_param);
      const double cbeta = cos(beta);
      const double sbeta = sin(beta);
      const double beta_dot_3 =
          1.0 / (1. + std::pow(beta_param, 2)) * ratio / std::pow(c3, 2);
      const double s2_beta = sin(x(2) + beta);
      const double c2_beta = cos(x(2) + beta);
      jaco(0, 2) = -x(4) * s2_beta;
      // jaco(0, 3) = -x(4) * s2_beta * beta_dot_3;
      jaco(0, 3) = -jaco(0, 2) * beta_dot_3;
      jaco(0, 4) = c2_beta;
      // jaco(1, 2) = x(4) * c2_beta;
      jaco(1, 2) = x(4) * jaco(0, 4);
      // jaco(1, 3) = x(4) * c2_beta * beta_dot_3;
      jaco(1, 3) = jaco(1, 2) * beta_dot_3;
      jaco(1, 4) = s2_beta;
      jaco(2, 3) =
          (x(4) / L) * (cbeta / std::pow(c3, 2) - tan3 * sbeta * beta_dot_3);
      jaco(2, 4) = cbeta * tan3 / L;
      jaco(3, 7) = 1;
      jaco(4, 5) = 1;
      jaco(5, 6) = 1;
    } else if constexpr (RefPos::fg == RP) {
      const double s23 = sin(x(2) + x(3));
      const double c23 = cos(x(2) + x(3));
      const double s3 = sin(x(3));
      const double c3 = cos(x(3));
      jaco(0, 2) = -x(4) * s23;
      // jaco(0, 3) = -x(4) * s23;
      jaco(0, 3) = jaco(0, 2);
      jaco(0, 4) = c23;
      jaco(1, 2) = x(4) * c23;
      // jaco(1, 3) = x(4) * c23;
      jaco(1, 3) = jaco(1, 2);
      jaco(1, 4) = s23;
      jaco(2, 3) = x(4) * c3 / L;
      jaco(2, 4) = s3 / L;
      jaco(3, 7) = 1;
      jaco(4, 5) = 1;
      jaco(5, 6) = 1;
    } else {
      CHECK(0);
    }
  }

  double L = 0.0;
  double lr = 0.0;

private:
  CarModel() = delete;
};

template <typename KP, typename C> class CarProblem : public Problem<KP, C> {
  using base_type = typename KP::base_type;
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;
  static constexpr int Nx = KP::N;
  static constexpr int Nu = KP::M;

  using discretized_type = DiscretizedDynamics<KP, RK4>;
  using model_type = CarModel<KP, RefPos::fg>;

public:
  CarProblem(std::vector<base_type> x0_in, std::vector<base_type> xf_in,
             std::vector<base_type> uf_in, int N, double tf) {
    CHECK(x0_in.size() == Nx && xf_in.size() == Nx && uf_in.size() == Nu);
    VectorXd x0 = Map<VectorXd>(x0_in.data(), x0_in.size());
    VectorXd xf = Map<VectorXd>(xf_in.data(), xf_in.size());
    VectorXd uf = Map<VectorXd>(uf_in.data(), uf_in.size());
    initialize(x0, xf, uf, N, tf);
  }
  void initialize(const state_type &x0, const state_type &xf,
                  const control_type &uf, int N, double tf) {
    DiagonalMatrix<double, Nx> Q(10, 10, 50, 1, 1, 1);
    auto R = DiagonalMatrix<double, Nu>(1, 1);
    const double rho = 1.0;
    R = rho * R;
    DiagonalMatrix<double, Nx> Qf(10, 10, 60, 1, 1, 1);

    car_ = std::make_shared<model_type>(2.7, 1.5);
    obj_ = LQRObjective<Nx, Nu, base_type>(Q, R, Qf, xf, uf, N, UserDefined());
    // Must be discretized.
    discretized_car_ = std::make_shared<discretized_type>(car_.get());

    this->init(discretized_car_, &obj_, x0, tf);

    // Initial_controls.
    this->initial_controls(uf);

    // Initial_states.

    // Rollout.
    rollout(this);
    // LOG(INFO) << *this->get_trajectory();
  }

  // Members.
  std::shared_ptr<model_type> car_;
  Objective<C> obj_;
  std::shared_ptr<discretized_type> discretized_car_;
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
