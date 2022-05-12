#ifndef COST_FUNTION_H
#define COST_FUNTION_H

#include "robot_dynamics/scalar_function.h"

ScalarFunctionTemplate class CostFunction : public ScalarFunctionDeclare {
public:
};
#define CostFunctionTypeName typename F, typename S
#define CostFunctionDeclare CostFunction<F, S>

#define QuadraticCostFunctionTemplate template <int n, int m, typename T, CostFunctionTypeName>
QuadraticCostFunctionTemplate
class QuadraticCostFunction : public CostFunctionDeclare {
public:
  bool is_blockdiag() const { return false; }
  int state_dim() const override { return n; }
  int control_dim() const override { return m; }

  virtual bool is_diag() const { return is_blockdiag(); }
};
#define QuadraticCostFunctionDeclare Quadracic

#endif
