#ifndef FUNCTION_BASE
#define FUNCTION_BASE

#include <stdexcept>
#include <tuple>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"

class AbstractFunction {
  ~AbstractFunction() = default;
  /*Pure virtual function*/
  virtual int state_dim() = 0;
  virtual int control_dim() = 0;
  virtual int output_dim() = 0;
  virtual int errstate_dim() = 0;

  /*Virtual function*/
  virtual FunctionInputs functioninputs() {
    return FunctionInputs::StateControl;
  }

  /*Function*/
  std::tuple<int, int, int> dims() {
    return std::make_tuple(state_dim(), control_dim(), output_dim());
  }
  DiffMethod default_diffmethod() { return DiffMethod::UserDefined; }
  FunctionSignature default_signature() {
    return FunctionSignature::StaticReturn;
  }
  int jacobian_width() { return errstate_dim() + control_dim(); }

  int input_dim() {
    if (functioninputs() == FunctionInputs::StateOnly) {
      return state_dim();
    } else if (functioninputs() == FunctionInputs::ControlOnly) {
      return control_dim();
    } else {
      return state_dim() + control_dim();
    }
  }

private:
};

//
// // TODO: need support vadiatic parameters
// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(FunctionInputs inputtype, AbstractFunction fun, P y,
//               const AbstractKnotPointDeclare & z) {
//   evaluate(fun, y, getargs(inputtype, z));
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(FunctionInputs inputtype, AbstractFunction fun,
//               const AbstractKnotPointDeclare & z) {
//   evaluate(fun, getargs(inputtype, z));
// }
//
// template <typename T, typename P>
// auto evaluate(AbstractFunction fun, T y, T x, T u, P p) {
//   evaluate(fun, y, x, u);
// }
// template <typename T, typename P>
// auto evaluate(AbstractFunction fun, T x, T u, P p) {
//   evaluate(fun, x, u);
// }
//
// template <typename T, typename... Args>
// auto evaluate(StaticReturn, AbstractFunction fun, T y, Args... args) {
//   evaluate(fun, args...);
// }
// template <typename T, typename... Args>
// auto evaluate(Inplace, AbstractFunction fun, T y, Args... args) {
//   evaluate(fun, y, args...);
// }
//
// template <typename T> auto evaluate(AbstractFunction fun, T y, T x, T u) {
//   throw std::runtime_error("Not implemented");
// }
// template <typename T> auto evaluate(AbstractFunction fun, T x, T u) {
//   throw std::runtime_error("Not implemented");
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(StaticReturn, AbstractFunction fun, P y,
//               const AbstractKnotPointDeclare & z) {
//   y = evaluate(fun, z);
// }
// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(Inplace, AbstractFunction fun, P y,
//               const AbstractKnotPointDeclare & z) {
//   evaluate(fun, y, z);
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto jacobian(FunctionSignature, UserDefined, AbstractFunction fun, P J, P y,
//               const AbstractKnotPointDeclare & z) {
//   jacobian(fun, J, y, z);
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto jacobian(AbstractFunction fun, P J, P y, const AbstractKnotPointDeclare
// & z) {
//   jacobian(functioninputs(fun), fun, J, y, z);
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto jacobian(FunctionInputs inputtype, AbstractFunction fun, P J, P y,
//               const AbstractKnotPointDeclare & z) {
//   jacobian(fun, J, y, getargs(inputtype, z));
// }
//
// template <typename T, typename P>
// auto jacobian(AbstractFunction fun, T J, T y, T x, T u, P p) {
//   jacobian(fun, J, y, x, u);
// }
// template <typename T> auto jacobian(AbstractFunction fun, T J, T y, T x, T u)
// {
//   throw std::runtime_error("User-defined jacobian not implemented");
// }
//
// template <typename P, AbstractKnotPointTypeName>
// auto d_jacobian(FunctionSignature, UserDefined, AbstractFunction fun, P H, P
// b,
//                 P y, const AbstractKnotPointDeclare & z) {
//   d_jacobian(fun, H, b, y, state(z), control(z), getparams(z));
// }
//
// template <typename T, typename Q>
// auto d_jacobian(AbstractFunction fun, T H, T b, T y, T x, T u, Q p) {
//   d_jacobian(fun, H, b, y, x, u);
// }

// template <typename P, AbstractKnotPointTypeName>
// auto evaluate(AbstractFunction fun, P y, const AbstractKnotPointDeclare & z)
// {
//   evaluate(functioninputs(fun), fun, y, z);
// }
//
// AbstractKnotPointTemplate
// auto evaluate(AbstractFunction fun, const AbstractKnotPointDeclare & z) {
//   evaluate(functioninputs(fun), fun, z);
// }
//
#endif
