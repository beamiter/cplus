#ifndef SCALAR_FUNTION_H
#define SCALAR_FUNTION_H

#include "functionbase.h"
#include "robot_dynamics/knotpoint.h"

template <typename KP> class ScalarFunction : public AbstractFunction<KP> {
public:
  int output_dim() const override { return 1; }
};

// Inplace reverts to scalar return
template <typename O, typename X, typename U, typename P, typename KP>
auto evaluate(const ScalarFunction<KP> &fun, O y, X x, U u, P p) {
  return evaluate(fun, x, u, p);
}
template <typename O, typename P, typename KP>
void jacobian(FunctionSignature sig, DiffMethod diff,
              const ScalarFunction<KP> &fun, O J, P y, const KP &z) {
  CHECK(length(y) == 1);
  gradient(sig, diff, fun, J.adjoint(), z);
}
template <typename KP, typename G>
void gradient(FunctionSignature, DiffMethod diff, const ScalarFunction<KP> &fun,
              G &grad, const KP &z) {
  gradient(diff, fun, grad, z);
}
template <typename KP, typename G>
void gradient(DiffMethod diff, const ScalarFunction<KP> &fun, G &grad,
              const KP &z) {
  if (DiffMethod::UserDefined == diff) {
    gradient(fun, grad, z);
  }
}
template <typename KP, typename G>
void gradient(const AbstractFunction<KP> &fun, G &grad, const KP &z) {
  gradient(fun, grad, z.state(), z.control(), z.getparams());
}
template <typename KP, typename G, typename X, typename U, typename P>
void gradient(const AbstractFunction<KP> &fun, G &grad, X x, U u, P p) {
  gradient(fun, grad, x, u);
}
template <typename KP, typename G, typename X, typename U>
void gradient(const AbstractFunction<KP> &fun, G &grad, X x, U u) {
  fun.gradient(grad, x, u);
}

#endif
