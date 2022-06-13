#ifndef INTEGRATION_H
#define INTEGRATION_H

#include <tuple>

#include "knotpoint.h"

// TODO: Fix cross-referencing

struct QuadratureRule {};
struct Explicit : QuadratureRule {};
struct Implicit : QuadratureRule {};

// Speedup policy
template <typename T> struct ADVecotor {};

class AbstractModel;
class ContinuousDynamics;
class DiscreteDynamics;
class DiscretizedDynamics;

/*
 * Explicit Methods
 */

struct Euler : Explicit {};
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
integrate(Euler, const AbstractModel *model,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
  auto xdot = dynamics(model, x, u, t);
  return x + h * xdot;
}
AbstractKnotPointTemplate void
integrate(Euler, const AbstractModel *model,
          typename AbstractKnotPointDeclare::state_type *xn,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
  dynamics(model, xn, x, u, t);
  xn *= h;
  xn += x;
}
template <typename P, AbstractKnotPointTypeName>
void jacobian(Euler, FunctionSignature, const ContinuousDynamics *model, P *J,
              const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h);

struct RK3 : Explicit {
  VectorXd k1;
  VectorXd k2;
  VectorXd k3;
  std::array<MatrixXd, 4> A;
  std::array<MatrixXd, 4> B;
  std::array<MatrixXd, 4> dA;
  std::array<MatrixXd, 4> dB;
  RK3() = default;
  RK3(int n, int m) {
    k1 = VectorXd::Zero(n);
    k2 = VectorXd::Zero(n);
    k3 = VectorXd::Zero(n);
    loop(0, 3, [this, n, m](const int k) {
      this->A.at(k) = MatrixXd::Zero(n, n);
      this->B.at(k) = MatrixXd::Zero(n, m);
      this->dA.at(k) = MatrixXd::Zero(n, n);
      this->dB.at(k) = MatrixXd::Zero(n, m);
    });
  }
};
inline auto getks(RK3 inte)
    -> decltype(std::make_tuple(inte.k1, inte.k2, inte.k3)) {
  return std::make_tuple(inte.k1, inte.k2, inte.k3);
}
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
integrate(RK3, const AbstractModel *model,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
  auto k1 = dynamics(model, x, u, t) * h;
  auto k2 = dynamics(model, x + k1 / 2, u, t + h / 2) * h;
  auto k3 = dynamics(model, x - k1 + 2 * k2, u, t + h) * h;
  return x + (k1 + 4 * k2 + k3) / 6;
}
AbstractKnotPointTemplate void
integrate(RK3 *inte, const AbstractModel *model,
          typename AbstractKnotPointDeclare::state_type *xn,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
  auto &k1 = inte->k1;
  auto &k2 = inte->k2;
  auto &k3 = inte->k3;
  dynamics(model, k1, x, u, t);
  xn = x + k1 * h / 2;
  dynamics(model, k2, xn, u, t + h / 2);
  xn = x - k1 * h + 2 * k2 * h;
  dynamics(model, k3, xn, u, t + h);
  xn = x + h * (k1 + 4 * k2 + k3) / 6;
}
template <typename P, AbstractKnotPointTypeName>
void jacobian(RK3 *, FunctionSignature, const ContinuousDynamics *model, P *J,
              const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h);

struct RK4 : Explicit {
  VectorXd k1;
  VectorXd k2;
  VectorXd k3;
  VectorXd k4;
  std::array<MatrixXd, 4> A;
  std::array<MatrixXd, 4> B;
  std::array<MatrixXd, 4> dA;
  std::array<MatrixXd, 4> dB;
  RK4() = default;
  RK4(int n, int m) {
    k1 = VectorXd::Zero(n);
    k2 = VectorXd::Zero(n);
    k3 = VectorXd::Zero(n);
    k4 = VectorXd::Zero(n);
    loop(0, 4, [this, n, m](const int k) {
      this->A.at(k) = MatrixXd::Zero(n, n);
      this->B.at(k) = MatrixXd::Zero(n, m);
      this->dA.at(k) = MatrixXd::Zero(n, n);
      this->dB.at(k) = MatrixXd::Zero(n, m);
    });
  }
};
inline auto getks(RK4 inte)
    -> decltype(std::make_tuple(inte.k1, inte.k2, inte.k3, inte.k4)) {
  return std::make_tuple(inte.k1, inte.k2, inte.k3, inte.k4);
}
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
integrate(RK4, const AbstractModel *model,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
  const auto k1 = dynamics(model, x, u, t) * h;
  const auto k2 = dynamics(model, x + k1 / 2, u, t + h / 2) * h;
  const auto k3 = dynamics(model, x + k2 / 2, u, t + h / 2) * h;
  const auto k4 = dynamics(model, x + k3, u, t + h) * h;
  return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}
AbstractKnotPointTemplate void
integrate(RK4 *inte, const AbstractModel *model,
          typename AbstractKnotPointDeclare::state_type *xn,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
  auto &k1 = inte->k1;
  auto &k2 = inte->k2;
  auto &k3 = inte->k3;
  auto &k4 = inte->k4;
  dynamics(model, k1, x, u, t);
  xn = x + k1 * h / 2;
  dynamics(model, k2, xn, u, t + h / 2);
  xn = x + k2 * h / 2;
  dynamics(model, k3, xn, u, t + h / 2);
  xn = x + k3 * h;
  dynamics(model, k4, xn, u, t + h);
  xn = x + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}
template <typename P, AbstractKnotPointTypeName>
void jacobian(RK4 *, FunctionSignature, const ContinuousDynamics *model, P *J,
              const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h);

/*
 * Implicit Methods
 */

#endif
