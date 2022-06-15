#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <functional>

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

class AbstractModel : public AbstractFunction {
public:
  int output_dim() const override { return this->state_dim(); }
};

class ContinuousDynamics : public AbstractModel {
public:
  int output_dim() const override { return this->state_dim(); }
};

// StaticReturn.
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
dynamics(const ContinuousDynamics *model, const AbstractKnotPointDeclare *z) {
  return dynamics(model, z->state(), z->control(), z->time());
}
AbstractKnotPointTemplate typename AbstractKnotPointDeclare::state_type
dynamics(const ContinuousDynamics *model,
         const typename AbstractKnotPointDeclare::state_type &x,
         const typename AbstractKnotPointDeclare::control_type &u, T t) {
  // return dynamics(model, x, u);
}

// Inplace.
template <typename P, AbstractKnotPointTypeName>
void dynamics(const ContinuousDynamics *model, P *xdot,
              const AbstractKnotPointDeclare *z) {
  dynamics(model, xdot, z->state(), z->control(), z->time());
}
template <typename P, AbstractKnotPointTypeName>
void dynamics(const ContinuousDynamics *model, P *xdot,
              const typename AbstractKnotPointDeclare::state_type &x,
              const typename AbstractKnotPointDeclare::control_type &u, T t) {
  // dynamics(model, xdot, x, u);
}

// // Depends on the FunctionSignature.
// template <typename P, AbstractKnotPointTypeName>
// void dynamics(FunctionSignature sig, const ContinuousDynamics *model, P
// *xdot,
//               const AbstractKnotPointDeclare &z) {
//   if (FunctionSignature::Inplace == sig) {
//     dynamics(model, xdot, z);
//   } else if (FunctionSignature::StaticReturn == sig) {
//     xdot = dynamics(model, z);
//   }
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(const ContinuousDynamics *model,
//               const typename AbstractKnotPointDeclare::state_type &x,
//               const typename AbstractKnotPointDeclare::control_type &u, P p)
//               {
//   return dynamics(model, x, u, p.t);
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(const ContinuousDynamics *model,
//               const typename AbstractKnotPointDeclare::state_type &xdot,
//               const typename AbstractKnotPointDeclare::state_type &x,
//               const typename AbstractKnotPointDeclare::control_type &u, P p)
//               {
//   return dynamics(model, xdot, x, u, p.t);
// }
//
// template <typename P, typename Q, AbstractKnotPointTypeName>
// auto jacobian(FunctionSignature, DiffMethod, const ContinuousDynamics *model,
//               Q J, P xdot, const AbstractKnotPointDeclare &z) {
//   jacobian(model, J, xdot, z.state(), z.constrol(), z.time());
// }

#endif
