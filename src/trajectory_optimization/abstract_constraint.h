#ifndef ABSTRACT_CONSTRAINT_H
#define ABSTRACT_CONSTRAINT_H

#include <stdexcept>

#include "robot_dynamics/functionbase.h"

class AbstractConstraint : AbstractFunction {};

class StageConstraint : AbstractConstraint {};
inline auto functioninputs(StageConstraint) {
  return StateControl();
}

class StateConstraint : StageConstraint {};
inline auto functioninputs(StateConstraint) {
  return StateOnly();
}

class ControlConstraint : StageConstraint {};
inline auto functioninputs(ControlConstraint) {
  return ControlOnly();
}

inline auto sense(AbstractConstraint) {
  throw std::runtime_error("Not implemented");
}

#endif
