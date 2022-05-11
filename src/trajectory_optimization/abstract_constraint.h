#ifndef ABSTRACT_CONSTRAINT_H
#define ABSTRACT_CONSTRAINT_H

#include <stdexcept>

#include "robot_dynamics/functionbase.h"

// AbstractFunctionTemplate class AbstractConstraint : AbstractFunctionDeclare
// {};
class AbstractConstraint {};

class StageConstraint : AbstractConstraint {};
inline auto functioninputs(StageConstraint) {
  return FunctionInputs::StateControl;
}

class StateConstraint : StageConstraint {};
inline auto functioninputs(StateConstraint) {
  return FunctionInputs::StateOnly;
}

class ControlConstraint : StageConstraint {};
inline auto functioninputs(ControlConstraint) {
  return FunctionInputs::ControlOnly;
}

inline auto sense(AbstractConstraint) {
  throw std::runtime_error("Not implemented");
}

#endif
