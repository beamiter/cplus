#ifndef ABSTRACT_CONSTRAINT_H
#define ABSTRACT_CONSTRAINT_H

#include <stdexcept>

#include "base/base.h"
#include "robot_dynamics/functionbase.h"

// AbstractFunctionTemplate class AbstractConstraint : AbstractFunctionDeclare
// {};
class AbstractConstraint {};

class StageConstraint : public AbstractConstraint {
  StateControl functioninputs(StageConstraint) { return StateControl(); }
};

class StateConstraint : public StageConstraint {
  StateOnly functioninputs(StateConstraint) { return StateOnly(); }
};

class ControlConstraint : public StageConstraint {
  ControlOnly functioninputs(ControlConstraint) { return ControlOnly(); }
};

inline auto sense(AbstractConstraint) {
  CHECK(0);
}

#endif
