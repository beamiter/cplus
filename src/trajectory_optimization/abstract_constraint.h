#ifndef ABSTRACT_CONSTRAINT_H
#define ABSTRACT_CONSTRAINT_H

#include <stdexcept>

#include "robot_dynamics/functionbase.h"

struct AbstractConstraint : AbstractFunction {};

struct StageConstraint : AbstractConstraint {};
inline auto functioninputs(StageConstraint) {
  return StateControl();
}

struct StateConstraint : StageConstraint {};
inline auto functioninputs(StateConstraint) {
  return StateOnly();
}

struct ControlConstraint : StageConstraint {};
inline auto functioninputs(ControlConstraint) {
  return ControlOnly();
}

inline auto sense(AbstractConstraint) {
  throw std::runtime_error("Not implemented");
}

#endif
