#ifndef FUNCTION_BASE
#define FUNCTION_BASE

#include <stdexcept>
#include <tuple>
#include <type_traits>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"

template <typename KP> class AbstractFunction {
  // For template type check.
  // static_assert(std::is_base_of<FunctionSignature, F>::value,
  //               "T is not derived of FunctionSignature");
  // static_assert(std::is_base_of<StateVectorType, S>::value,
  //               "P is not derived of StateVectorType");
  // typedef typename std::enable_if<std::is_base_of<FunctionSignature,
  // F>::value,
  //                                 F>::type default_signature;
  // typedef typename std::enable_if<std::is_base_of<StateVectorType, S>::value,
  //                                 S>::type statevectortype;
public:
  ~AbstractFunction() = default;
  /*Pure virtual function*/
  virtual int state_dim() const = 0;
  virtual int control_dim() const = 0;
  virtual int output_dim() const = 0;
  /////////////////////
  // Minimal call that must be implemented.
  // TODO: What's the type of P?
  // virtual void evaluate(P y, const typename KP::state_type &x,
  // const typename KP::control_type &u) {
  // CHECK(0);
  // return;
  //}
  virtual double evaluate(const typename KP::state_type &x,
                          const typename KP::control_type &u) const {
    CHECK(0);
    return 0.0;
  }
  virtual void gradient(typename KP::gradient_type &grad,
                        const typename KP::state_type &x,
                        const typename KP::control_type &u,
                        bool is_terminal = false) const {}
  virtual void hessian(typename KP::hessian_type &hess,
                       const typename KP::state_type &x,
                       const typename KP::control_type &u,
                       bool is_terminal = false) const {}

  /*Virtual function*/
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
  void gradient(typename KP::gradient_type &grad, const KP &z) {
    gradient(grad, z.state(), z.control(), z.is_terminal());
  }

  constexpr StateControl functioninputs() const { return StateControl(); }
  int errstate_dim() const { return errstate_dim(statevectortype()); }
  std::tuple<int, int, int> dims() const {
    return std::make_tuple(state_dim(), control_dim(), output_dim());
  }
  int jacobian_width() const { return errstate_dim() + control_dim(); }
  int input_dim() { return input_dim(functioninputs()); }
  int input_dim(StateOnly) { return state_dim(); }
  int input_dim(ControlOnly) { return control_dim(); }
  int input_dim(StateControl) { return state_dim() + control_dim(); }

private:
};

// TODO: need support vadiatic parameters
// Evaluate.
// Top-level command that can be overridden.
// Should only be overridden if using hand-written Jacobian methods.
template <typename P, typename KP>
void evaluate(const AbstractFunction<KP> *fun, P y, const KP &z) {
  evaluate(fun->functioninputs(), fun, y, z);
  return;
}
template <typename KP>
auto evaluate(const AbstractFunction<KP> *fun, const KP &z) {
  return evaluate(fun->functioninputs(), fun, z);
}
//////////////////////////////////
template <typename P, typename KP, typename TP = FunctionInputs>
void evaluate(TP tp, const AbstractFunction<KP> *fun, P y, const KP &z) {
  typename KP::state_type state;
  typename KP::control_type control;
  std::tuple<typename KP::base_type, typename KP::base_type> param(0, 0);
  if constexpr (is_same_type<StateControl, decltype(tp)>::value) {
    std::tie(state, control, param) = z.getargs(tp);
  } else if constexpr (is_same_type<StateOnly, decltype(tp)>::value) {
    std::tie(state) = z.getargs(tp);
  } else if constexpr (is_same_type<ControlOnly, decltype(tp)>::value) {
    std::tie(control) = z.getargs(tp);
  } else {
    CHECK(0);
  }
  evaluate<P, KP, decltype(param)>(fun, y, state, control, param);
  return;
}
template <typename KP, typename TP = FunctionInputs>
auto evaluate(TP tp, const AbstractFunction<KP> *fun, const KP &z) {
  typename KP::state_type state;
  typename KP::control_type control;
  std::tuple<typename KP::base_type, typename KP::base_type> param(0, 0);
  if constexpr (is_same_type<StateControl, TP>::value) {
    std::tie(state, control, param) = z.getargs(tp);
  } else if constexpr (is_same_type<StateOnly, TP>::value) {
    std::tie(state) = z.getargs(tp);
  } else if constexpr (is_same_type<ControlOnly, TP>::value) {
    std::tie(control) = z.getargs(tp);
  } else {
    CHECK(0);
  }
  return evaluate<KP, decltype(param)>(fun, state, control, param);
}
/////////////////////////////////
// Strip the parameter.
template <typename P, typename KP, typename Q>
void evaluate(const AbstractFunction<KP> *fun, P y,
              const typename KP::state_type &x,
              const typename KP::control_type &u, const Q &p) {
  fun->evaluate(y, x, u);
  return;
}
template <typename KP, typename Q>
auto evaluate(const AbstractFunction<KP> *fun, const typename KP::state_type &x,
              const typename KP::control_type &u, Q p) {
  return fun->evaluate(x, u);
}
///////////////////////////////////////
// Dispatch on function signature.
template <typename P, typename KP>
void evaluate(FunctionSignature sig, const AbstractFunction<KP> *fun, P y,
              const KP &z) {
  if (sig == FunctionSignature::StaticReturn) {
    y = evaluate(fun, z);
  } else {
    evaluate(fun, y, z);
  }
}
// template <typename T, typename... Args>
// auto evaluate(FunctionSignature sig, const AbstractFunction *fun, T y,
// Args... args) {
// if (sig == FunctionSignature::StaticReturn) {
// return evaluate(fun, args...);
//}
// evaluate(fun, y, args...);
//}

// Jacobian.
// template <typename P, typename KP>
// auto jacobian(FunctionSignature, DiffMethod, const AbstractFunction<KP> *fun,
//               P J, P y, const KP &z) {
//   jacobian(fun, J, y, z);
// }

// template <typename P, typename KP>
// auto jacobian(const AbstractFunction<KP> *fun, P J, P y, const KP &z) {
//   jacobian(fun->functioninputs(), fun, J, y, z);
// }

// template <typename P, typename KP>
// auto jacobian(FunctionInputs inputtype, const AbstractFunction<KP> *fun, P J,
//               P y, const KP &z) {
//   jacobian(fun, J, y, getargs(inputtype, z));
// }

// template <typename T, typename P, typename KP>
// auto jacobian(const AbstractFunction<KP> *fun, T J, T y, T x, T u, P p) {
//   jacobian(fun, J, y, x, u);
// }
// template <typename T, typename KP>
// auto jacobian(const AbstractFunction<KP> *fun, T J, T y, T x, T u) {
//   CHECK(0);
// }
// template <typename P, typename KP>
// auto d_jacobian(FunctionSignature, DiffMethod, const AbstractFunction<KP>
// *fun,
//                 P H, P b, P y, const KP &z) {
//   d_jacobian(fun, H, b, y, state(z), control(z), getparams(z));
// }

// template <typename T, typename Q, typename KP>
// auto d_jacobian(const AbstractFunction<KP> *fun, T H, T b, T y, T x, T u, Q
// p) {
//   d_jacobian(fun, H, b, y, x, u);
// }

#endif
