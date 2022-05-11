#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"

#define AbstractModelTypeName typename F, typename S
#define AbstractModelTemplate template <typename F, typename S>
#define AbstractModelDeclare AbstractFunction<F, S>
AbstractFunctionTemplate class AbstractModel : public AbstractFunctionDeclare {
public:
  int output_dim() const override { return this->state_dim(); }
};

#define ContinuousDynamicsDeclare ContinuousDynamics<F, S>
AbstractModelTemplate class ContinuousDynamics : public AbstractModelDeclare {
public:
  int output_dim() const override { return this->state_dim(); }
};

// AbstractKnotPointTemplate auto dynamics(const ContinuousDynamics *model,
// const AbstractKnotPointDeclare *z) {
// return dynamics(model, z->state(), z->control(), z->time());
//}

// template <typename T>
// auto dynamics(const ContinuousDynamics *model, T x, T u, double t) {
// return dynamics(model, x, u);
//}

// template <typename Q, int Nx, int Nu, typename V, typename T>
// auto dynamics(const ContinuousDynamics *model, Q xdot,
// const AbstractKnotPointDeclare *z) {
// dynamics(model, xdot, z->state(), z->control(), z->time(z));
//}

// template <typename T>
// auto dynamics(const ContinuousDynamics *model, T xdot, T x, T u, double t) {
// dynamics(model, xdot, x, u);
//}

//
// template <typename Q, int Nx, int Nu, typename V, typename T>
// auto dynamics(Inplace, ContinuousDynamics model, Q xdot,
//               const AbstractKnotPointDeclare &z) {
//   dynamics(model, xdot, z);
// }
//
// template <typename Q, int Nx, int Nu, typename V, typename T>
// auto dynamics(StaticReturn, ContinuousDynamics model, Q xdot,
//               const AbstractKnotPointDeclare &z) {
//   xdot = dynamics(model, z);
// }
//
// template <typename T, typename P>
// auto evaluate(const ContinuousDynamics *model, T x, T u, P p) {
// return dynamics(model, x, u, p.t);
//}

// template <typename T, typename P>
// auto evaluate(const ContinuousDynamics *model, T xdot, T x, T u, P p) {
// return dynamics(model, xdot, x, u, p.t);
//}

// template <typename Q, int Nx, int Nu, typename V, typename T>
// auto jacobian(FunctionSignature, DiffMethod, const ContinuousDynamics *model,
// Q J, Q xdot, const AbstractKnotPointDeclare &z) {
// jacobian(model, J, xdot, state(z), constrol(z), time(z));
//}

#endif
