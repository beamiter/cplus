#ifndef CAR_MODEL_
#define CAR_MODEL_

#include "dynamics.h"
#include "robot_dynamics/discrete_dynamics.h"

enum class RefPos {
  rear,
  cg,
  fg,
};

class CarModel : public ContinuousDynamics {
public:
  CarModel(RefPos ref_in = RefPos::rear, double L_in = 2.7,
           double lr_in = 1.5) {
    this->ref = ref_in;
    this->L = L_in;
    this->lr = lr_in;
  }
  virtual int state_dim() const final { return 6; }
  virtual int control_dim() const final { return 2; }
  virtual int errstate_dim() const final { return 7; }

  RefPos ref;
  double L = 0.0;
  double lr = 0.0;
};

#endif
