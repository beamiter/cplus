#ifndef ROBOT_DYNAMICS_FUNCTION_BASE_H
#define ROBOT_DYNAMICS_FUNCTION_BASE_H

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

template <typename Q>
auto state_diff(const AbstractFunction *fun, Q dx, Q x, Q x0) {
  state_diff(fun->statevectortype(), fun, dx, x, x0);
}
template <typename Q> auto state_diff(const AbstractFunction *fun, Q x, Q x0) {
  return state_diff(fun->statevectortype(), fun, x, x0);
}

template <typename G, AbstractKnotPointTypeName>
auto errstate_jacobian(const AbstractFunction *fun, G J,
                       const AbstractKnotPointDeclare &z) {
  errstate_jacobian(fun->statevectortype(), fun, J, state(z));
}

template <typename G, typename P>
auto errstate_jacobian(const AbstractFunction *fun, G J, P x) {
  errstate_jacobian(fun->statevectortype(), fun, J, x);
}

template <typename G, typename P, AbstractKnotPointTypeName>
auto d_errstate_jacobian(const AbstractFunction *fun, G J,
                         const AbstractKnotPointDeclare &z, P dx) {
  return d_errstate_jacobian(fun->statevectortype(), fun, J, state(z), dx);
}

template <typename G, typename P>
auto d_errstate_jacobian(const AbstractFunction *fun, G J, P x, P dx) {
  return d_errstate_jacobian(fun->statevectortype(), fun, J, x, dx);
}

template <typename T>
auto state_diff(FunctionSignature sig, const AbstractFunction *fun, T dx, T x,
                T x0) {
  dx = x - x0;
}
template <typename T>
auto state_diff(FunctionSignature sig, const AbstractFunction *fun, T x, T x0) {
  return x - x0;
}

template <typename T>
auto d_errstate_jacobian(FunctionSignature sig, const AbstractFunction *fun,
                         T d_G, T x, T dx) {
  d_G = 0;
}

template <typename T>
auto errstate_jacobian(FunctionSignature, const AbstractFunction *fun, T J,
                       T x) {
  J = 0;
  for (auto i = 0; i < J.size()[1]; ++i) {
    J(i, i) = 1;
  }
}

#endif
