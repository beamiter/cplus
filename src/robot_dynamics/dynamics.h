#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <functional>

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

class AbstractModel : public AbstractFunction {
public:
  int output_dim() const override { return this->state_dim(); }
};

template <typename KP> class ContinuousDynamics : public AbstractModel {
  using state_type = typename KP::state_type;
  using control_type = typename KP::control_type;
  using value_type = typename KP::value_type;
  using base_type = typename KP::base_type;

public:
  int output_dim() const override { return this->state_dim(); }
  virtual state_type dynamics(const state_type &x,
                              const control_type &u) const {
    CHECK(0);
  }
  virtual void dynamics(state_type *xdot, const state_type &x,
                        const control_type &u) const {
    CHECK(0);
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
  // return dynamics(model, x, u);
  model->dynamics(x, u);
}

// Inplace.
template <typename P, typename KP>
void dynamics(const ContinuousDynamics<KP> *model, P *xdot, const KP *z) {
  dynamics(model, xdot, z->state(), z->control(), z->time());
}
template <typename P, typename KP>
void dynamics(const ContinuousDynamics<KP> *model, P *xdot,
              const typename KP::state_type &x,
              const typename KP::control_type &u, typename KP::base_type t) {
  // dynamics(model, xdot, x, u);
  model->dynamics(xdot, x, u);
}

// // Depends on the FunctionSignature.
// template <typename P, typename KP>
// void dynamics(FunctionSignature sig, const ContinuousDynamics *model, P
// *xdot,
//               const KP &z) {
//   if (FunctionSignature::Inplace == sig) {
//     dynamics(model, xdot, z);
//   } else if (FunctionSignature::StaticReturn == sig) {
//     xdot = dynamics(model, z);
//   }
// }
//
// template <typename P, typename KP>
// auto evaluate(const ContinuousDynamics *model,
//               const typename KP::state_type &x,
//               const typename KP::control_type &u, P p)
//               {
//   return dynamics(model, x, u, p.t);
// }
//
// template <typename P, typename KP>
// auto evaluate(const ContinuousDynamics *model,
//               const typename KP::state_type &xdot,
//               const typename KP::state_type &x,
//               const typename KP::control_type &u, P p)
//               {
//   return dynamics(model, xdot, x, u, p.t);
// }
//
// template <typename P, typename Q, typename KP>
// auto jacobian(FunctionSignature, DiffMethod, const ContinuousDynamics *model,
//               Q J, P xdot, const KP &z) {
//   jacobian(model, J, xdot, z.state(), z.constrol(), z.time());
// }

#endif
