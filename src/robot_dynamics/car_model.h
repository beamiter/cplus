#ifndef CAR_MODEL_
#define CAR_MODEL_

#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/util/Constants.h>

#include "base/base.h"
#include "dynamics.h"
#include "robot_dynamics/discrete_dynamics.h"
#include "trajectory_optimization/problem.h"

using Eigen::DiagonalMatrix;
using Eigen::Dynamic;

enum class RefPos {
  rear,
  cg,
  fg,
};

AbstractModelTemplate class CarModel : public ContinuousDynamicsDeclare {
public:
  CarModel(RefPos ref_in = RefPos::rear, double L_in = 2.7,
           double lr_in = 1.5) {
    this->ref = ref_in;
    this->L = L_in;
    this->lr = lr_in;
  }
  int state_dim() const final { return 6; }
  int control_dim() const final { return 2; }

  RefPos ref;
  double L = 0.0;
  double lr = 0.0;
};

inline auto BicycleCar(const std::vector<double>& x0, const std::vector<double>& xf,
		int N, double tf) {
  auto model = CarModel<Inplace, EuclideanState>();
  DiagonalMatrix<double, 6> Q(10, 10, 50, 1, 1, 1);
  double rho = 1.0;
  auto R = rho * DiagonalMatrix<double, 2>(1, 1);
  DiagonalMatrix<double, 6>Qf(10, 10, 60, 1, 1, 1);

  auto car = CarModel<Inplace, EuclideanState>();
  Objective<double>* obj;
  /* auto prob = ProblemHelper::init<6, 2>(car, obj, x0, tf); */

  // initial_controls, initial_states, rollout

  // return prob;
  return -1;
}

#endif
