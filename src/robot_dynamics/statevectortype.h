#ifndef ROBOT_DYNAMICS_FUNCTION_BASE_H
#define ROBOT_DYNAMICS_FUNCTION_BASE_H

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

template <typename KP>
auto state_diff(const AbstractFunction<KP> *func,
                typename KP::ref_vector_type dx,
                const typename KP::state_type &x, typename KP::state_type x0) {
  state_diff(statevectortype(func), func, dx, x, x0);
}
template <typename KP>
auto state_diff(const AbstractFunction<KP> *func,
                const typename KP::state_type &x,
                const typename KP::state_type &x0) {
  return state_diff(statevectortype(func), func, x, x0);
}

template <typename KP>
auto errstate_jacobian(const AbstractFunction<KP> *func,
                       typename KP::ref_matrix_type J, const KP &z) {
  errstate_jacobian(statevectortype(func), func, J, state(z));
}

template <typename KP>
auto errstate_jacobian(const AbstractFunction<KP> *func,
                       typename KP::ref_matrix_type J,
                       const typename KP::state_type &x) {
  errstate_jacobian(statevectortype(func), func, J, x);
}

template <typename KP>
auto d_errstate_jacobian(const AbstractFunction<KP> *func,
                         typename KP::ref_matrix_type J, const KP &z,
                         const typename KP::state_type &dx) {
  return d_errstate_jacobian(func->statevectortype(), func, J, state(z), dx);
}

template <typename KP>
auto d_errstate_jacobian(const AbstractFunction<KP> *func,
                         typename KP::ref_matrix_type J,
                         const typename KP::state_type &x,
                         const typename KP::state_type &dx) {
  return d_errstate_jacobian(func->statevectortype(), func, J, x, dx);
}

template <typename KP, typename FS>
auto state_diff(FS, const AbstractFunction<KP> *func,
                typename KP::ref_vector_type dx,
                const typename KP::state_type &x,
                const typename KP::state_type &x0) {
  dx = x - x0;
}
template <typename KP, typename FS>
auto state_diff(FS sig, const AbstractFunction<KP> *func,
                const typename KP::state_type &x,
                const typename KP::state_type &x0) {
  return x - x0;
}

template <typename KP, typename FS>
auto errstate_jacobian(FS, const AbstractFunction<KP> *func,
                       typename KP::ref_matrix_type J,
                       const typename KP::state_type &x) {
  J.setZero();
  for (int i = 0; i < J.rows(); ++i) {
    J(i, i) = 1;
  }
}

template <typename KP, typename FS>
auto d_errstate_jacobian(FS sig, const AbstractFunction<KP> *func,
                         typename KP::ref_matrix_type d_G,
                         const typename KP::state_type &x,
                         const typename KP::state_type &dx) {
  d_G = 0;
}

#endif
