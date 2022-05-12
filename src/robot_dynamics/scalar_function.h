#ifndef SCALAR_FUNTION_H
#define SCALAR_FUNTION_H

#include "functionbase.h"
#include "robot_dynamics/knotpoint.h"

AbstractFunctionTemplate class ScalarFunction : public AbstractFunctionDeclare {
public:
  int output_dim() const override { return 1; }
};
#define ScalarFuntionTypeName typename F, typename S
#define ScalarFunctionTemplate template <typename F, typename S>
#define ScalarFunctionDeclare ScalarFunction<F, S>

/* AbstractFunctionTemplate */
/* evaluate(ScalarFunctionDeclare fun, y, x, u, p) = evaluate(fun, x, u, p); */

#endif
