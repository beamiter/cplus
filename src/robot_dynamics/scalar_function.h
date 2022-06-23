#ifndef SCALAR_FUNTION_H
#define SCALAR_FUNTION_H

#include "functionbase.h"
#include "robot_dynamics/knotpoint.h"

template <typename KP> class ScalarFunction : public AbstractFunction<KP> {
public:
  int output_dim() const override { return 1; }
};

// Inplace reverts to scalar return
template <typename V, typename P, typename KP>
auto evaluate(const ScalarFunction<KP> *fun, V y, V x, V u, P p) {
  return evaluate(fun, x, u, p);
}

#endif
