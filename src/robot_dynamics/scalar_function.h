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
  CHECK(DiffMethod::UserDefined == diff);
  gradient(fun, grad, z);
}
template <typename KP, typename G>
void gradient(const AbstractFunction<KP> &fun, G &grad, const KP &z) {
  gradient(fun, grad, z.state(), z.control(), z.getparams(), z.is_terminal());
}
template <typename KP, typename G, typename X, typename U, typename P>
void gradient(const AbstractFunction<KP> &fun, G &grad, const X &x, const U &u,
              const P &p, bool is_terminal) {
  gradient(fun, grad, x, u, is_terminal);
}
template <typename KP, typename G, typename X, typename U>
void gradient(const AbstractFunction<KP> &fun, G &grad, const X &x, const U &u,
              bool is_terminal) {
  fun.gradient(grad, x, u, is_terminal);
}

template <typename KP, typename HESS, typename P, typename Q>
void d_jacobian(FunctionSignature sig, DiffMethod diff,
                const ScalarFunction<KP> &fun, HESS &H, const P &b, const Q &y,
                const KP &z) {
  CHECK(b[0] == 1);
  CHECK(length(y) == 1);
  hessian(sig, diff, fun, H, z);
}
template <typename KP, typename H>
void hessian(DiffMethod diff, const ScalarFunction<KP> &fun, H &hess,
             const KP z) {
  CHECK(DiffMethod::UserDefined == diff);
  hessian(fun, hess, z);
}
template <typename KP, typename H>
void hessian(const ScalarFunction<KP> &fun, H &hess, const KP z) {
  hessian(fun, hess, z.state(), z.control(), z.getparams(), z.is_terminal());
}
template <typename KP, typename H, typename X, typename U, typename P>
void hessian(const ScalarFunction<KP> &fun, H &hess, const X &x, const U &u,
             const P &p, bool is_terminal) {
  hessian(fun, hess, x, u, is_terminal);
}
template <typename KP, typename H, typename X, typename U>
void hessian(const ScalarFunction<KP> &fun, H &hess, const X &x, const U &u,
             bool is_terminal) {
  fun.hessian(hess, x, u, is_terminal);
}

#endif
