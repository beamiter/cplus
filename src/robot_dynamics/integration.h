#ifndef INTEGRATION_H
#define INTEGRATION_H

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
              T h) {
  jacobian(model, J, xn, x, u, t);
  J *= h;
  // for (auto i = 0; i < state_dim(model)) {
  // J(i,  i) += 1.0;
  //}
}

struct RK3 : Explicit {
  VectorXd k1;
  VectorXd k2;
  VectorXd k3;
  std::vector<MatrixXd> A;
  std::vector<MatrixXd> B;
  std::vector<MatrixXd> dA;
  std::vector<MatrixXd> dB;
  RK3(int n, int m) {
    k1 = VectorXd::Zero(n);
    k2 = VectorXd::Zero(n);
    k3 = VectorXd::Zero(m);
    loop(0, 3, [this, n, m](const int k) {
      this->A.push_back(MatrixXd::Zero(n, n));
      this->B.push_back(MatrixXd::Zero(n, m));
      this->dA.push_back(MatrixXd::Zero(n, n));
      this->dB.push_back(MatrixXd::Zero(n, m));
    });
  }
};
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
integrate(RK3, const AbstractModel *model,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
}
AbstractKnotPointTemplate void
integrate(RK3, const AbstractModel *model,
          typename AbstractKnotPointDeclare::state_type *xn,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
}
template <typename P, AbstractKnotPointTypeName>
void jacobian(RK3, FunctionSignature, const ContinuousDynamics *model, P *J,
              const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h) {
}

struct RK4 : Explicit {};
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
integrate(RK4, const AbstractModel *model,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
}
AbstractKnotPointTemplate void
integrate(RK4, const AbstractModel *model,
          typename AbstractKnotPointDeclare::state_type *xn,
          const typename AbstractKnotPointDeclare::state_type &x,
          const typename AbstractKnotPointDeclare::control_type &u, T t, T h) {
}
template <typename P, AbstractKnotPointTypeName>
void jacobian(RK4, FunctionSignature, const ContinuousDynamics *model, P *J,
              const typename AbstractKnotPointDeclare::state_type &xn,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t,
              T h) {
}

#endif
