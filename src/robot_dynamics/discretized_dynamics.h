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

AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
discrete_dynamics(const DiscretizedDynamics *model,
                  const typename AbstractKnotPointDeclare::state_type &x,
                  const typename AbstractKnotPointDeclare::control_type &u, T t,
                  T dt) {
  CHECK(0);
}
AbstractKnotPointTemplate void
discrete_dynamics(const DiscretizedDynamics *model,
                  typename AbstractKnotPointDeclare::state_type *xn,
                  const typename AbstractKnotPointDeclare::state_type &x,
                  const typename AbstractKnotPointDeclare::control_type &u, T t,
                  T dt) {
  assert(0);
}

#endif
