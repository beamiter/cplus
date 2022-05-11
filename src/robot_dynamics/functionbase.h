#ifndef FUNCTION_BASE
#define FUNCTION_BASE

#include <stdexcept>
#include <tuple>
#include <type_traits>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"

#define AbstractFunctionTypeName typename F, typename S
#define AbstractFunctionTemplate template <typename F, typename S>
#define AbstractFunctionDeclare AbstractFunction<F, S>

AbstractFunctionTemplate class AbstractFunction {
  static_assert(std::is_base_of<FunctionSignature, F>::value,
                "T is not derived of FunctionSignature");
  static_assert(std::is_base_of<StateVectorType, S>::value,
                "P is not derived of StateVectorType");

public:
  // typedef F default_signature;
  // typedef S statevectortype;
  typedef typename std::enable_if<std::is_base_of<FunctionSignature, F>::value,
                                  F>::type default_signature;
  typedef typename std::enable_if<std::is_base_of<StateVectorType, S>::value,
                                  S>::type statevectortype;
  ~AbstractFunction() = default;
  /*Pure virtual function*/
  virtual int state_dim() const = 0;
  virtual int control_dim() const = 0;
  virtual int output_dim() const = 0;

  /*Virtual function*/
  virtual FunctionInputs functioninputs() const {
    return FunctionInputs::StateControl;
  }

  /*Function*/
  int errstate_dim(EuclideanState) const { return state_dim(); }
  int errstate_dim(RotationState) const {
    throw std::runtime_error("not implemented.");
    return -1;
  }
  int errstate_dim(StateVectorType) const {
    throw std::runtime_error("not implemented.");
    return -1;
  }

  int errstate_dim() const { return errstate_dim(statevectortype()); }
  std::tuple<int, int, int> dims() {
    return std::make_tuple(state_dim(), control_dim(), output_dim());
  }
  DiffMethod default_diffmethod() { return DiffMethod::UserDefined; }

  // FunctionSignature default_signature() {
  // return StaticReturn();
  //}
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

// TODO: need support vadiatic parameters
template <typename P, AbstractKnotPointTypeName, AbstractFunctionTypeName>
auto evaluate(FunctionInputs inputtype, const AbstractFunctionDeclare &fun, P y,
              const AbstractKnotPointDeclare &z) {
  evaluate(fun, y, getargs(inputtype, z));
}

template <typename P, AbstractKnotPointTypeName, AbstractFunctionTypeName>
auto evaluate(FunctionInputs inputtype, const AbstractFunctionDeclare &fun,
              const AbstractKnotPointDeclare &z) {
  evaluate(fun, getargs(inputtype, z));
}

template <typename T, typename P, AbstractFunctionTypeName>
auto evaluate(const AbstractFunctionDeclare &fun, T y, T x, T u, P p) {
  evaluate(fun, y, x, u);
}

template <typename T, typename P, AbstractFunctionTypeName>
auto evaluate(const AbstractFunctionDeclare &fun, T x, T u, P p) {
  evaluate(fun, x, u);
}

// template <typename T, typename... Args>
// auto evaluate(StaticReturn, AbstractFunction fun, T y, Args... args) {
//   evaluate(fun, args...);
// }
// template <typename T, typename... Args>
// auto evaluate(Inplace, AbstractFunction fun, T y, Args... args) {
//   evaluate(fun, y, args...);
// }
//
template <typename T, AbstractFunctionTypeName>
auto evaluate(const AbstractFunctionDeclare &fun, T y, T x, T u) {
  throw std::runtime_error("Not implemented");
}
template <typename T, AbstractFunctionTypeName>
auto evaluate(const AbstractFunctionDeclare &fun, T x, T u) {
  throw std::runtime_error("Not implemented");
}
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
template <typename P, AbstractKnotPointTypeName, AbstractFunctionTypeName>
auto jacobian(FunctionSignature, DiffMethod, const AbstractFunctionDeclare &fun,
              P J, P y, const AbstractKnotPointDeclare &z) {
  jacobian(fun, J, y, z);
}

template <typename P, AbstractKnotPointTypeName, AbstractFunctionTypeName>
auto jacobian(const AbstractFunctionDeclare &fun, P J, P y,
              const AbstractKnotPointDeclare &z) {
  jacobian(fun.functioninputs(), fun, J, y, z);
}

template <typename P, AbstractKnotPointTypeName, AbstractFunctionTypeName>
auto jacobian(FunctionInputs inputtype, const AbstractFunctionDeclare &fun, P J,
              P y, const AbstractKnotPointDeclare &z) {
  jacobian(fun, J, y, getargs(inputtype, z));
}

template <typename T, typename P, AbstractFunctionTypeName>
auto jacobian(const AbstractFunctionDeclare &fun, T J, T y, T x, T u, P p) {
  jacobian(fun, J, y, x, u);
}
template <typename T, AbstractFunctionTypeName>
auto jacobian(const AbstractFunctionDeclare &fun, T J, T y, T x, T u) {
  throw std::runtime_error("User-defined jacobian not implemented");
}
//
// template <typename P, AbstractKnotPointTypeName>
// auto d_jacobian(FunctionSignature, UserDefined, AbstractFunction fun, P H, P
// b,
//                 P y, const AbstractKnotPointDeclare & z) {
//   d_jacobian(fun, H, b, y, state(z), control(z), getparams(z));
// }
//
template <typename T, typename Q, AbstractFunctionTypeName>
auto d_jacobian(const AbstractFunctionDeclare &fun, T H, T b, T y, T x, T u,
                Q p) {
  d_jacobian(fun, H, b, y, x, u);
}

template <typename P, AbstractKnotPointTypeName, AbstractFunctionTypeName>
auto evaluate(const AbstractFunctionDeclare &fun, P y,
              const AbstractKnotPointDeclare &z) {
  evaluate(fun.functioninputs(), fun, y, z);
}

template <AbstractKnotPointTypeName, AbstractFunctionTypeName>
auto evaluate(const AbstractFunctionDeclare &fun,
              const AbstractKnotPointDeclare &z) {
  evaluate(fun.functioninputs(), fun, z);
}

#endif
