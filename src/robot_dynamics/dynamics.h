#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

struct AbstractModel : AbstractFunction {};

inline auto output_dim(AbstractModel model) { return state_dim(model); }

struct ContinuousDynamics : AbstractModel {};

KNOT_TEMP_DECL
auto dynamics(ContinuousDynamics model, CONST_ABSTRACT_KNOT_Z) {
  return dynamics(model, state(z), control(z), time(z));
}

template <typename T>
auto dynamics(ContinuousDynamics model, T x, T u, double t) {
  return dynamics(model, x, u);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto dynamics(ContinuousDynamics model, Q xdot, CONST_ABSTRACT_KNOT_Z) {
  dynamics(model, xdot, state(z), control(z), time(z));
}

template <typename T>
auto dynamics(ContinuousDynamics model, T xdot, T x, T u, double t) {
  dynamics(model, xdot, x, u);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto dynamics(Inplace, ContinuousDynamics model, Q xdot, CONST_ABSTRACT_KNOT_Z) {
 dynamics(model, xdot, z);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto dynamics(StaticReturn, ContinuousDynamics model, Q xdot, CONST_ABSTRACT_KNOT_Z) {
  xdot = dynamics(model, z);
}

template <typename T, typename P>
auto evaluate(ContinuousDynamics model, T x, T u, P p) {
  return dynamics(model, x, u, p.t);
}

template <typename T, typename P>
auto evaluate(ContinuousDynamics model, T xdot,T x, T u, P p) {
  return dynamics(model,xdot, x, u, p.t);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto jacobian(FunctionSignature, UserDefined, ContinuousDynamics model, Q J, Q xdot, CONST_ABSTRACT_KNOT_Z) {
  jacobian(model, J, xdot, state(z), constrol(z), time(z));
}

#endif
