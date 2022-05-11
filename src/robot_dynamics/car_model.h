#ifndef CAR_MODEL_
#define CAR_MODEL_

#include "dynamics.h"
#include "robot_dynamics/discrete_dynamics.h"

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

#endif
