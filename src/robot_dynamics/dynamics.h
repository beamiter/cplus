#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

struct AbstractModel : AbstractFunction {};

inline auto output_dim(AbstractModel model) { return state_dim(model); }

struct ContinuousDynamics : AbstractModel {};

template <int Nx, int Nu, typename V, typename T>
auto dynamics(ContinuousDynamics model, const AbstractKnotPoint<Nx, Nu, V, T> & z) {
  return dynamics(model, state(z), control(z), time(z));
}

template <typename T>
auto dynamics(ContinuousDynamics model, T x, T u, double t) {
  return dynamics(model, x, u);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto dynamics(ContinuousDynamics model, Q xdot,
              const AbstractKnotPoint<Nx, Nu, V, T> & z) {
  dynamics(model, xdot, state(z), control(z), time(z));
}

template <typename T>
auto dynamics(ContinuousDynamics model, T xdot, T x, T u, double t) {
  dynamics(model, xdot, x, u);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto dynamics(Inplace, ContinuousDynamics model, Q xdot,
              const AbstractKnotPoint<Nx, Nu, V, T> & z) {
  dynamics(model, xdot, z);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto dynamics(StaticReturn, ContinuousDynamics model, Q xdot,
              const AbstractKnotPoint<Nx, Nu, V, T> & z) {
  xdot = dynamics(model, z);
}

template <typename T, typename P>
auto evaluate(ContinuousDynamics model, T x, T u, P p) {
  return dynamics(model, x, u, p.t);
}

template <typename T, typename P>
auto evaluate(ContinuousDynamics model, T xdot, T x, T u, P p) {
  return dynamics(model, xdot, x, u, p.t);
}

template <typename Q, int Nx, int Nu, typename V, typename T>
auto jacobian(FunctionSignature, UserDefined, ContinuousDynamics model, Q J,
              Q xdot, const AbstractKnotPoint<Nx, Nu, V, T> & z) {
  jacobian(model, J, xdot, state(z), constrol(z), time(z));
}

#endif
