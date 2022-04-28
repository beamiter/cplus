#ifndef FUNCTION_BASE
#define FUNCTION_BASE

#include <stdexcept>
#include <tuple>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"

struct AbstractFunction {};

struct FunctionSignature {};
struct Inplace : FunctionSignature {};
struct StaticReturn : FunctionSignature {};

struct DiffMethod {};
struct ForwardAD : DiffMethod {};
struct FiniteDifference : DiffMethod {};
struct UserDefined : DiffMethod {};

inline auto default_diffmethod(AbstractFunction) { return UserDefined(); }
inline auto default_signature(AbstractFunction) { return StaticReturn(); }

template <typename T> auto state_dim(T t) {
  throw std::runtime_error("Need to be implemented.");
}
template <typename T> auto control_dim(T t) {
  throw std::runtime_error("Need to be implemented.");
}
template <typename T> auto output_dim(T t) {
  throw std::runtime_error("Need to be implemented.");
}

template <> inline auto state_dim(AbstractFunction fun) {
  throw std::runtime_error("Need to be implemented.");
  return 0;
}
template <> inline auto control_dim(AbstractFunction fun) {
  throw std::runtime_error("Need to be implemented.");
  return 0;
}
template <> inline auto output_dim(AbstractFunction fun) {
  throw std::runtime_error("Need to be implemented.");
  return 0;
}

inline auto dims(AbstractFunction fun) {
  return std::make_tuple(state_dim(fun), control_dim(fun), output_dim(fun));
}

template <typename T> auto errstate_dim(T fun) { return -1; }

ABSTRACT_KNOT_POINT_TEMPLATE
auto errstate_dim(ABSTRACT_KNOT_POINT fun) { return state_dim(fun); }

inline auto jacobian_width(AbstractFunction fun) {
  return errstate_dim(fun) + control_dim(fun);
}

struct FunctionInputs {};
struct StateOnly : FunctionInputs {};
struct ControlOnly : FunctionInputs {};
struct StateControl : FunctionInputs {};

inline auto functioninputs(AbstractFunction) { return StateControl(); }

inline auto input_dim(::StateOnly, AbstractFunction fun) {
  return state_dim(fun);
}
inline auto input_dim(::ControlOnly, AbstractFunction fun) {
  return control_dim(fun);
}
inline auto input_dim(::StateControl, AbstractFunction fun) {
  return control_dim(fun) + state_dim(fun);
}
inline auto input_dim(AbstractFunction fun) {
  return input_dim(functioninputs(fun), fun);
}

ABSTRACT_KNOT_POINT_TEMPLATE
auto getinput(::StateControl, CONST_ABSTRACT_KNOT_POINT_REF z) {
  return getdata(z);
}
ABSTRACT_KNOT_POINT_TEMPLATE
auto getinput(::StateOnly, CONST_ABSTRACT_KNOT_POINT_REF z) { return state(z); }
ABSTRACT_KNOT_POINT_TEMPLATE
auto getinput(::ControlOnly, CONST_ABSTRACT_KNOT_POINT_REF z) {
  return control(z);
}
ABSTRACT_KNOT_POINT_TEMPLATE
auto getargs(::StateControl, CONST_ABSTRACT_KNOT_POINT_REF z) {
  return std::make_tuple(state(z), control(z), getparams(z));
}
ABSTRACT_KNOT_POINT_TEMPLATE
auto getargs(::StateOnly, CONST_ABSTRACT_KNOT_POINT_REF z) {
  return std::make_tuple(state(z));
}
ABSTRACT_KNOT_POINT_TEMPLATE
auto getargs(::ControlOnly, CONST_ABSTRACT_KNOT_POINT_REF z) {
  return std::make_tuple(control(z));
}

template <> struct DataType<AbstractFunction> { typedef double datatype; };

#endif
