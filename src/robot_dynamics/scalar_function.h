#ifndef SCALAR_FUNTION_H
#define SCALAR_FUNTION_H

#include "functionbase.h"
#include "robot_dynamics/knotpoint.h"

class ScalarFunction : public AbstractFunction {
public:
  int output_dim() const override { return 1; }
};

// Inplace reverts to scalar return
template <typename V, typename P>
auto evaluate(const ScalarFunction *fun, V y, V x, V u, P p) {
  return evaluate(fun, x, u, p);
}

#endif
