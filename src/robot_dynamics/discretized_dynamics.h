#ifndef DISCRETIZED_DYNAMICS_H
#define DISCRETIZED_DYNAMICS_H

#include "discrete_dynamics.h"

struct QuadratureRule {};
struct Explicit : QuadratureRule {};
struct Implicit : QuadratureRule {};

class DiscretizedDynamics : public DiscreteDynamics {
public:
  DiscretizedDynamics() = default;
  virtual ~DiscretizedDynamics() = default;
  // Constructors
  DiscretizedDynamics(const ContinuousDynamics *dynamics_in,
                      QuadratureRule rule)
      : continuous_dynamics(dynamics_in), integrator(rule) {}

  // Overriding
  int state_dim() const override { return continuous_dynamics->state_dim(); }
  int control_dim() const override {
    return continuous_dynamics->control_dim();
  }
  int output_dim() const override { return continuous_dynamics->output_dim(); }

  // Members
  const ContinuousDynamics *continuous_dynamics = nullptr;

  QuadratureRule integrator;
};

// This method is called when using the 'StaticReturn'.
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
discrete_dynamics(const DiscretizedDynamics *model,
                  const AbstractKnotPointDeclare &z) {
  return discrete_dynamics<Nx, Nu, V, T>(model, z.state(), z.control(),
                                         z.time(), z.timestep());
}
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
discrete_dynamics(const DiscretizedDynamics *model,
                  const typename AbstractKnotPointDeclare::state_type &x,
                  const typename AbstractKnotPointDeclare::control_type &u, T t,
                  T dt) {
  CHECK(0);
}

// This method is called when using the 'InPlace'.
AbstractKnotPointTemplate void
discrete_dynamics(const DiscretizedDynamics *model,
                  typename AbstractKnotPointDeclare::state_type *xn,
                  const AbstractKnotPointDeclare &z) {
  discrete_dynamics<Nx, Nu, V, T>(model, xn, z.state(), z.control(), z.time(),
                                  z.timestep());
}
AbstractKnotPointTemplate void
discrete_dynamics(const DiscretizedDynamics *model,
                  typename AbstractKnotPointDeclare::state_type *xn,
                  const typename AbstractKnotPointDeclare::state_type &x,
                  const typename AbstractKnotPointDeclare::control_type &u, T t,
                  T dt) {
  CHECK(0);
}

// Function not support partial specialization yet.
AbstractKnotPointTemplate void
discrete_dynamics(FunctionSignature sig, const DiscretizedDynamics *model,
                  typename AbstractKnotPointDeclare::state_type *xn,
                  const AbstractKnotPointDeclare &z) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, xn, z);
  } else {
    xn = discrete_dynamics(model, z);
  }
}

// Propagate dynamics.
AbstractKnotPointTemplate void
propagate_dynamics(FunctionSignature sig, const DiscretizedDynamics *model,
                   AbstractKnotPointDeclare *z2,
                   const AbstractKnotPointDeclare &z1) {
  if (sig == FunctionSignature::Inplace) {
    discrete_dynamics(model, z2->state(), z1);
  } else {
    z2->setstate(discrete_dynamics(model, z1));
  }
}

#endif
