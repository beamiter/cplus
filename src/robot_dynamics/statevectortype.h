#ifndef ROBOT_DYNAMICS_FUNCTION_BASE_H
#define ROBOT_DYNAMICS_FUNCTION_BASE_H

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

struct StateVectorType {};

struct EuclideanState : StateVectorType {};

struct RotationState : StateVectorType {};

inline auto statevectortype(AbstractFunction fun) {
  return EuclideanState();
}

template <typename T> auto errstate_dim(T t, AbstractFunction fun) {}
template <> inline auto errstate_dim(StateVectorType t, AbstractFunction fun) {}
template <> inline auto errstate_dim(EuclideanState t, AbstractFunction fun) {
  return state_dim(fun);
}
template <> inline auto errstate_dim(RotationState t, AbstractFunction fun) {}

inline auto errstate_dim(AbstractFunction fun) {
  return errstate_dim(statevectortype(fun), fun);
}

template <typename T, typename P, typename Q>
auto state_diff(T fun, P dx, Q x, Q x0) {
  state_diff(statevectortype(fun), fun, dx, x, x0);
}
template <typename T, typename Q> auto state_diff(T fun, Q x, Q x0) {
  return state_diff(statevectortype(fun), fun, x, x0);
}

template <typename G, AbstractKnotPointTypeName>
auto errstate_jacobian(AbstractFunction fun, G J,
                       const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  errstate_jacobian(statevectortype(fun), fun, J, state(z));
}

template <typename G, typename P>
auto errstate_jacobian(AbstractFunction fun, G J, P x) {
  errstate_jacobian(statevectortype(fun), fun, J, x);
}

template <typename G, typename P, AbstractKnotPointTypeName>
auto d_errstate_jacobian(AbstractFunction fun, G J,
                         const AbstractKnotPoint<Nx, Nu, V, T> &z, P dx) {
  return d_errstate_jacobian(statevectortype(fun), fun, J, state(z), dx);
}

template <typename G, typename P>
auto d_errstate_jacobian(AbstractFunction fun, G J, P x, P dx) {
  return d_errstate_jacobian(statevectortype(fun), fun, J, x, dx);
}

template <typename T>
auto state_diff(EuclideanState, AbstractFunction fun, T dx, T x, T x0) {
  dx = x - x0;
}
template <typename T>
auto state_diff(EuclideanState, AbstractFunction fun, T x, T x0) {
  return x - x0;
}

template <typename T>
auto d_errstate_jacobian(EuclideanState, AbstractFunction fun, T d_G, T x,
                         T dx) {
  d_G = 0;
}

template <typename T>
auto errstate_jacobian(EuclideanState, AbstractFunction fun, T J, T x) {
  J = 0;
  for (auto i = 0; i < J.size()[1]; ++i) {
    J(i, i) = 1;
  }
}

#endif
