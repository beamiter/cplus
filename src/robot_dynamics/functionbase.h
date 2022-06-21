#ifndef FUNCTION_BASE
#define FUNCTION_BASE

#include <stdexcept>
#include <tuple>
#include <type_traits>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"

class AbstractFunction {
  // For template type check.
  // static_assert(std::is_base_of<FunctionSignature, F>::value,
  //               "T is not derived of FunctionSignature");
  // static_assert(std::is_base_of<StateVectorType, S>::value,
  //               "P is not derived of StateVectorType");

public:
  // typedef F default_signature;
  // typedef S statevectortype;
  // For template type check.
  // typedef typename std::enable_if<std::is_base_of<FunctionSignature,
  // F>::value,
  //                                 F>::type default_signature;
  // typedef typename std::enable_if<std::is_base_of<StateVectorType, S>::value,
  //                                 S>::type statevectortype;
  ~AbstractFunction() = default;
  /*Pure virtual function*/
  virtual int state_dim() const = 0;
  virtual int control_dim() const = 0;
  virtual int output_dim() const = 0;

  /*Virtual function*/
  virtual FunctionInputs functioninputs() const {
    return FunctionInputs::StateControl;
  }
  virtual DiffMethod default_diffmethod() const {
    return DiffMethod::UserDefined;
  }
  virtual FunctionSignature default_signature() const {
    return FunctionSignature::StaticReturn;
  }
  virtual StateVectorType statevectortype() const {
    return StateVectorType::EuclideanState;
  }
  // Default for EuclideanState
  virtual int errstate_dim(StateVectorType type) const {
    assert(type == StateVectorType::EuclideanState);
    return state_dim();
  }

  /*Function.*/
  int errstate_dim() const { return errstate_dim(statevectortype()); }
  std::tuple<int, int, int> dims() const {
    return std::make_tuple(state_dim(), control_dim(), output_dim());
  }
  int jacobian_width() const { return errstate_dim() + control_dim(); }
  int input_dim() const {
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
// Evaluate.
template <typename P, typename KP>
auto evaluate(const AbstractFunction *fun, P y, const KP &z) {
  evaluate(fun->functioninputs(), fun, y, z);
}
template <typename KP> auto evaluate(const AbstractFunction *fun, const KP &z) {
  evaluate(fun->functioninputs(), fun, z);
}
//////////////////////////////////
template <typename P, typename KP>
auto evaluate(FunctionInputs inputtype, const AbstractFunction *fun, P y,
              const KP &z) {
  evaluate(fun, y, getargs(inputtype, z));
}
template <typename P, typename KP>
auto evaluate(FunctionInputs inputtype, const AbstractFunction &fun,
              const KP &z) {
  evaluate(fun, getargs(inputtype, z));
}
/////////////////////////////////
template <typename T, typename P>
auto evaluate(const AbstractFunction &fun, T y, T x, T u, P p) {
  evaluate(fun, y, x, u);
}
template <typename T, typename P>
auto evaluate(const AbstractFunction &fun, T x, T u, P p) {
  evaluate(fun, x, u);
}
///////////////////////////////////////
template <typename T, typename... Args>
auto evaluate(FunctionSignature sig, const AbstractFunction *fun, T y,
              Args... args) {
  if (sig == FunctionSignature::StaticReturn) {
    return evaluate(fun, args...);
  }
  evaluate(fun, y, args...);
}
/////////////////////
template <typename T>
auto evaluate(const AbstractFunction *fun, T y, T x, T u) {
  CHECK(0);
}
template <typename T> auto evaluate(const AbstractFunction *fun, T x, T u) {
  CHECK(0);
}
//////////////////////////////////
template <typename P, typename KP>
void evaluate(FunctionSignature sig, const AbstractFunction *fun, P y,
              const KP &z) {
  if (sig == FunctionSignature::StaticReturn) {
    y = evaluate(fun, z);
  } else {
    evaluate(fun, y, z);
  }
}

// Jacobian.
template <typename P, typename KP>
auto jacobian(FunctionSignature, DiffMethod, const AbstractFunction *fun, P J,
              P y, const KP &z) {
  jacobian(fun, J, y, z);
}

template <typename P, typename KP>
auto jacobian(const AbstractFunction *fun, P J, P y, const KP &z) {
  jacobian(fun->functioninputs(), fun, J, y, z);
}

template <typename P, typename KP>
auto jacobian(FunctionInputs inputtype, const AbstractFunction *fun, P J, P y,
              const KP &z) {
  jacobian(fun, J, y, getargs(inputtype, z));
}

template <typename T, typename P>
auto jacobian(const AbstractFunction *fun, T J, T y, T x, T u, P p) {
  jacobian(fun, J, y, x, u);
}
template <typename T>
auto jacobian(const AbstractFunction *fun, T J, T y, T x, T u) {
  CHECK(0);
}
template <typename P, typename KP>
auto d_jacobian(FunctionSignature, DiffMethod, const AbstractFunction *fun, P H,
                P b, P y, const KP &z) {
  d_jacobian(fun, H, b, y, state(z), control(z), getparams(z));
}

template <typename T, typename Q>
auto d_jacobian(const AbstractFunction *fun, T H, T b, T y, T x, T u, Q p) {
  d_jacobian(fun, H, b, y, x, u);
}

#endif
