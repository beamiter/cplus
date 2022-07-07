#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <functional>

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

template <typename KP> class AbstractModel : public AbstractFunction<KP> {
public:
  int output_dim() const override { return this->state_dim(); }
};

template <typename KP> class ContinuousDynamics : public AbstractModel<KP> {

public:
  int output_dim() const override { return this->state_dim(); }
  virtual typename KP::state_type
  dynamics(const typename KP::state_type &x,
           const typename KP::control_type &u) const {
    CHECK(0);
    return x;
  }
  virtual void dynamics(typename KP::ref_vector_type xdot,
                        const typename KP::state_type &x,
                        const typename KP::control_type &u) const {
    CHECK(0);
  }
  virtual void jacobian(typename KP::ref_vector_type jaco,
                        const typename KP::state_type &y,
                        const typename KP::state_type &x,
                        const typename KP::control_type &u) const {
    CHECK(0);
  }
  virtual void
  jacobian(typename KP::ref_matrix_type jaco, typename KP::ref_vector_type y,
           const typename KP::state_type &x, const typename KP::control_type &u,
           typename KP::base_type t, typename KP::base_type dt) const {
    jacobian(jaco, y, x, u);
    jaco *= dt;
  }
};

// StaticReturn.
template <typename KP>
typename KP::state_type dynamics(const ContinuousDynamics<KP> *model,
                                 const KP *z) {
  return dynamics(model, z->state(), z->control(), z->time());
}
template <typename KP>
typename KP::state_type
dynamics(const ContinuousDynamics<KP> *model, const typename KP::state_type &x,
         const typename KP::control_type &u, typename KP::base_type t) {
  return model->dynamics(x, u);
}

// Inplace.
template <typename KP>
void dynamics(const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot, const KP *z) {
  dynamics(model, xdot, z->state(), z->control(), z->time());
}
template <typename KP>
void dynamics(const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t) {
  model->dynamics(xdot, x, u);
}

// Depends on the FunctionSignature.
template <typename KP>
void dynamics(FunctionSignature sig, const ContinuousDynamics<KP> *model,
              typename KP::ref_vector_type xdot, const KP &z) {
  if (FunctionSignature::Inplace == sig) {
    dynamics(model, xdot, z);
  } else if (FunctionSignature::StaticReturn == sig) {
    xdot = dynamics(model, z);
  }
}

template <typename KP>
auto evaluate(const ContinuousDynamics<KP> *model,
              const typename KP::state_type &x,
              const typename KP::control_type &u,
              const typename KP::param_type &p) {
  return dynamics(model, x, u, p.first);
}
template <typename KP, typename P>
auto evaluate(const ContinuousDynamics<KP> *model,
              const typename KP::state_type &xdot,
              const typename KP::state_type &x,
              const typename KP::control_type &u,
              const typename KP::param_type &p) {
  return dynamics(model, xdot, x, u, p.first);
}
template <typename KP, typename Q>
auto jacobian(FunctionSignature, DiffMethod,
              const ContinuousDynamics<KP> *model, Q &J,
              typename KP::state_type &xdot, const KP &z) {
  jacobian(model, J, xdot, z.state(), z.constrol(), z.time());
}

#endif
