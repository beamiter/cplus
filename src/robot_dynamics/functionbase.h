#ifndef FUNCTION_BASE
#define FUNCTION_BASE

#include <stdexcept>
#include <tuple>
#include <type_traits>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"

template <typename KP> class AbstractFunction {
public:
  ~AbstractFunction() = default;
  /*Pure virtual function*/
  virtual int state_dim() const = 0;
  virtual int control_dim() const = 0;
  virtual int output_dim() const = 0;
  /////////////////////
  // Minimal call that must be implemented.
  virtual void evaluate(typename KP::ref_vector_type y,
                        const typename KP::state_type &x,
                        const typename KP::control_type &u) {
    CHECK(0);
  }
  virtual double evaluate(const typename KP::state_type &x,
                          const typename KP::control_type &u) const {
    CHECK(0);
    return 0.0;
  }
  virtual void gradient(typename KP::ref_vector_type grad,
                        const typename KP::state_type &x,
                        const typename KP::control_type &u,
                        bool is_terminal = false) const {
    CHECK(0);
  }
  virtual void hessian(typename KP::ref_matrix_type hess,
                       const typename KP::state_type &x,
                       const typename KP::control_type &u,
                       bool is_terminal = false) const {
    CHECK(0);
  }
  virtual void jacobian(typename KP::ref_matrix_type jaco,
                        typename KP::ref_vector_type y,
                        const typename KP::state_type &x,
                        const typename KP::control_type &u) const {
    CHECK(0);
  }
  virtual void
  jacobian(typename KP::ref_matrix_type jaco, typename KP::ref_vector_type y,
           const typename KP::state_type &x, const typename KP::control_type &u,
           typename KP::base_type t, typename KP::base_type dt) const {
    jacobian(jaco, y, x, u);
    jaco *= dt;
  }

  /*Function.*/
  void gradient(typename KP::ref_vector_type grad, const KP &z) {
    gradient(grad, z.state(), z.control(), z.is_terminal());
  }

  std::tuple<int, int, int> dims() const {
    return std::make_tuple(state_dim(), control_dim(), output_dim());
  }
  int jacobian_width() const { return errstate_dim(this) + control_dim(); }
  int input_dim() { return input_dim(functioninputs(this)); }
  int input_dim(StateOnly) { return state_dim(); }
  int input_dim(ControlOnly) { return control_dim(); }
  int input_dim(StateControl) { return state_dim() + control_dim(); }

private:
};

template <typename KP>
UserDefined default_diffmethod(const AbstractFunction<KP> *) {
  return UserDefined();
}
template <typename KP>
StaticReturn default_signature(const AbstractFunction<KP> *) {
  return StaticReturn();
}
template <typename KP>
StateControl functioninputs(const AbstractFunction<KP> *) {
  return StateControl();
}
template <typename KP>
EuclideanState statevectortype(const AbstractFunction<KP> *) {
  return EuclideanState();
}
template <typename KP> int errstate_dim(const AbstractFunction<KP> *fun) {
  return errstate_dim(statevectortype(fun), fun);
}
template <typename KP, typename SV = StateVectorType>
int errstate_dim(SV, const AbstractFunction<KP> *fun) {
  static_assert(std::is_base_of<StateVectorType, SV>::value,
                "SV is not derived of FunctionInputs");
}
template <typename KP>
int errstate_dim(EuclideanState, const AbstractFunction<KP> *fun) {
  return fun->state_dim();
}

// TODO: need support vadiatic parameters
// Evaluate.
// Top-level command that can be overridden.
// Should only be overridden if using hand-written Jacobian methods.
template <typename KP>
void evaluate(const AbstractFunction<KP> *fun, typename KP::ref_vector_type y,
              const KP &z) {
  evaluate(fun->functioninputs(), fun, y, z);
}
template <typename KP>
auto evaluate(const AbstractFunction<KP> *fun, const KP &z) {
  return evaluate(functioninputs(fun), fun, z);
}
//////////////////////////////////

// SFINAE: Substitution failure is not an error.
template <typename KP, typename FI = FunctionInputs>
void evaluate(typename std::enable_if<
                  std::is_base_of<FunctionInputs, FI>::value, FI>::type tp,
              const AbstractFunction<KP> *fun, typename KP::ref_vector_type y,
              const KP &z) {
  static_assert(std::is_base_of<FunctionInputs, FI>::value,
                "FI is not derived of FunctionInputs");
}
template <typename KP>
void evaluate(StateControl tp, const AbstractFunction<KP> *fun,
              typename KP::ref_vector_type y, const KP &z) {
  typename KP::state_type state;
  typename KP::control_type control;
  std::tuple<typename KP::base_type, typename KP::base_type> param(0, 0);
  std::tie(state, control, param) = z.getargs(tp);
  evaluate<KP, decltype(param)>(fun, y, state, control, param);
}
template <typename KP>
void evaluate(StateOnly tp, const AbstractFunction<KP> *fun,
              typename KP::ref_vector_type y, const KP &z) {
  typename KP::state_type state;
  std::tie(state) = z.getargs(tp);
  evaluate<KP>(fun, y, state);
}
template <typename KP>
void evaluate(ControlOnly tp, const AbstractFunction<KP> *fun,
              typename KP::ref_vector_type y, const KP &z) {
  typename KP::control_type control;
  std::tie(control) = z.getargs(tp);
  evaluate<KP>(fun, y, control);
}

template <typename KP, typename FI = FunctionInputs>
auto evaluate(FI tp, const AbstractFunction<KP> *fun, const KP &z) {
  static_assert(std::is_base_of<FunctionInputs, FI>::value,
                "FI is not derived of FunctionInputs");
}
template <typename KP>
auto evaluate(StateControl tp, const AbstractFunction<KP> *fun, const KP &z) {
  typename KP::state_type state;
  typename KP::control_type control;
  std::tuple<typename KP::base_type, typename KP::base_type> param(0, 0);
  std::tie(state, control, param) = z.getargs(tp);
  return evaluate<KP, decltype(param)>(fun, state, control, param);
}
template <typename KP>
auto evaluate(StateOnly tp, const AbstractFunction<KP> *fun, const KP &z) {
  typename KP::state_type state;
  std::tie(state) = z.getargs(tp);
  return evaluate<KP>(fun, state);
}
template <typename KP>
auto evaluate(ControlOnly tp, const AbstractFunction<KP> *fun, const KP &z) {
  typename KP::control_type control;
  std::tie(control) = z.getargs(tp);
  return evaluate<KP>(fun, control);
}
/////////////////////////////////
// Strip the parameter.
template <typename KP, typename Q>
void evaluate(const AbstractFunction<KP> *fun, typename KP::ref_vector_type y,
              const typename KP::state_type &x,
              const typename KP::control_type &u, const Q &p) {
  fun->evaluate(y, x, u);
}
template <typename KP, typename Q>
auto evaluate(const AbstractFunction<KP> *fun, const typename KP::state_type &x,
              const typename KP::control_type &u, Q p) {
  return fun->evaluate(x, u);
}
///////////////////////////////////////
// Dispatch on function signature.
template <typename KP, typename FS = FunctionSignature>
void evaluate(
    typename std::enable_if<std::is_base_of<FunctionSignature, FS>::value, FS>
        sig,
    const AbstractFunction<KP> *fun, typename KP::ref_vector_type y,
    const KP &z) {
  static_assert(std::is_base_of<FunctionSignature, FS>::value,
                "FS is not derived of FunctionSignature");
}
template <typename KP>
void evaluate(StaticReturn, const AbstractFunction<KP> *fun,
              typename KP::ref_vector_type y, const KP &z) {
  return evaluate(fun, z);
}
template <typename KP>
void evaluate(Inplace, const AbstractFunction<KP> *fun,
              typename KP::ref_vector_type y, const KP &z) {
  return evaluate(fun, y, z);
}
template <typename KP, typename... Args>
auto evaluate(StaticReturn, const AbstractFunction<KP> *fun,
              typename KP::ref_vector_type y, Args... args) {
  return evaluate(fun, args...);
}
template <typename KP, typename... Args>
auto evaluate(Inplace, const AbstractFunction<KP> *fun,
              typename KP::ref_vector_type y, Args... args) {
  evaluate(fun, y, args...);
}

// Jacobian.
template <typename KP, typename FS = FunctionSignature,
          typename DM = DiffMethod>
void jacobian(FS, DM, const AbstractFunction<KP> *fun,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type y,
              const KP &z) {
  jacobian(fun, J, y, z);
}
template <typename KP>
void jacobian(const AbstractFunction<KP> *fun, typename KP::ref_matrix_type J,
              typename KP::ref_vector_type y, const KP &z) {
  jacobian(functioninputs(fun), fun, J, y, z);
}
template <typename KP, typename FI = FunctionInputs>
auto jacobian(FI, const AbstractFunction<KP> *fun,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type y,
              const KP &z) {
  static_assert(std::is_base_of<FunctionInputs, FI>::value,
                "FI is not derived of FunctionInputs");
}
template <typename KP>
auto jacobian(StateControl, const AbstractFunction<KP> *fun,
              typename KP::ref_matrix_type J, typename KP::ref_vector_type y,
              const KP &z) {
  jacobian(fun, J, y, z.state(), z.control(), z.params());
}
template <typename KP>
auto jacobian(const AbstractFunction<KP> *fun, typename KP::ref_matrix_type J,
              typename KP::ref_vector_type y, const typename KP::state_type &x,
              const typename KP::control_type &u,
              const typename KP::param_type &p) {
  jacobian(fun, J, y, x, u);
}
template <typename KP>
auto jacobian(const AbstractFunction<KP> *fun, typename KP::ref_matrix_type J,
              typename KP::ref_vector_type y, const typename KP::state_type &x,
              const typename KP::control_type &u) {
  fun->jacobian(J, y, x, u);
}

template <typename KP, typename P, typename Q, typename HESS>
auto d_jacobian(FunctionSignature, UserDefined, const AbstractFunction<KP> *fun,
                HESS H, P b, Q y, const KP &z) {
  d_jacobian(fun, H, b, y, z);
}
template <typename KP, typename P, typename Q, typename HESS>
auto d_jacobian(FunctionSignature, const AbstractFunction<KP> *fun, HESS H, P b,
                Q y, const KP &z) {
  d_jacobian(fun, H, b, y, z.state(), z.control(), z.params());
}
template <typename KP, typename P, typename Q, typename HESS, typename X,
          typename U, typename O>
auto d_jacobian(const AbstractFunction<KP> *fun, HESS H, P b, Q y, X x, U u,
                O p) {
  d_jacobian(fun, H, b, y, x, u);
}
template <typename KP, typename P, typename Q, typename HESS, typename X,
          typename U>
auto d_jacobian(const AbstractFunction<KP> *fun, HESS H, P b, Q y, X x, U u) {
  CHECK(0);
}

#endif
