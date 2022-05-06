#ifndef FUNCTION_BASE
#define FUNCTION_BASE

#include <stdexcept>
#include <tuple>

#include "base/base.h"
#include "robot_dynamics/knotpoint.h"

struct AbstractFunction {};

template <> struct ValueType<AbstractFunction> { typedef double value_type; };

struct FunctionSignature {
  bool operator==(const FunctionSignature &rhs) const { return true; }
};
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
  return 0;
}
template <typename T> auto control_dim(T t) {
  throw std::runtime_error("Need to be implemented.");
  return 0;
}
template <typename T> auto output_dim(T t) {
  throw std::runtime_error("Need to be implemented.");
  return 0;
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

AbstractKnotPointTemplate
auto errstate_dim(AbstractKnotPointDeclare fun) { return state_dim(fun); }

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

AbstractKnotPointTemplate
auto getinput(::StateControl, const AbstractKnotPointDeclare & z) {
  return getdata(z);
}
AbstractKnotPointTemplate
auto getinput(::StateOnly, const AbstractKnotPointDeclare & z) { return state(z); }
AbstractKnotPointTemplate
auto getinput(::ControlOnly, const AbstractKnotPointDeclare & z) {
  return control(z);
}
AbstractKnotPointTemplate
auto getargs(::StateControl, const AbstractKnotPointDeclare & z) {
  return std::make_tuple(state(z), control(z), getparams(z));
}
AbstractKnotPointTemplate
auto getargs(::StateOnly, const AbstractKnotPointDeclare & z) {
  return std::make_tuple(state(z));
}
AbstractKnotPointTemplate
auto getargs(::ControlOnly, const AbstractKnotPointDeclare & z) {
  return std::make_tuple(control(z));
}

template <typename P, int Nx, int Nu,typename V, typename T>
auto evaluate(AbstractFunction fun, P y, const AbstractKnotPointDeclare & z) {
  evaluate(functioninputs(fun), fun, y, z);
}

AbstractKnotPointTemplate
auto evaluate(AbstractFunction fun, const AbstractKnotPointDeclare & z) {
  evaluate(functioninputs(fun), fun, z);
}

// TODO: need support vadiatic parameters
template <typename P, int Nx, int Nu,typename V, typename T>
auto evaluate(FunctionInputs inputtype, AbstractFunction fun, P y,
              const AbstractKnotPointDeclare & z) {
  evaluate(fun, y, getargs(inputtype, z));
}

template <typename P, int Nx, int Nu,typename V, typename T>
auto evaluate(FunctionInputs inputtype, AbstractFunction fun,
              const AbstractKnotPointDeclare & z) {
  evaluate(fun, getargs(inputtype, z));
}

template <typename T, typename P>
auto evaluate(AbstractFunction fun, T y, T x, T u, P p) {
  evaluate(fun, y, x, u);
}
template <typename T, typename P>
auto evaluate(AbstractFunction fun, T x, T u, P p) {
  evaluate(fun, x, u);
}

template <typename T, typename... Args>
auto evaluate(StaticReturn, AbstractFunction fun, T y, Args... args) {
  evaluate(fun, args...);
}
template <typename T, typename... Args>
auto evaluate(Inplace, AbstractFunction fun, T y, Args... args) {
  evaluate(fun, y, args...);
}

template <typename T> auto evaluate(AbstractFunction fun, T y, T x, T u) {
  throw std::runtime_error("Not implemented");
}
template <typename T> auto evaluate(AbstractFunction fun, T x, T u) {
  throw std::runtime_error("Not implemented");
}

template <typename P, int Nx, int Nu,typename V, typename T>
auto evaluate(StaticReturn, AbstractFunction fun, P y,
              const AbstractKnotPointDeclare & z) {
  y = evaluate(fun, z);
}
template <typename P, int Nx, int Nu,typename V, typename T>
auto evaluate(Inplace, AbstractFunction fun, P y,
              const AbstractKnotPointDeclare & z) {
  evaluate(fun, y, z);
}

template <typename P, int Nx, int Nu,typename V, typename T>
auto jacobian(FunctionSignature, UserDefined, AbstractFunction fun, P J, P y,
              const AbstractKnotPointDeclare & z) {
  jacobian(fun, J, y, z);
}

template <typename P, int Nx, int Nu,typename V, typename T>
auto jacobian(AbstractFunction fun, P J, P y, const AbstractKnotPointDeclare & z) {
  jacobian(functioninputs(fun), fun, J, y, z);
}

template <typename P, int Nx, int Nu,typename V, typename T>
auto jacobian(FunctionInputs inputtype, AbstractFunction fun, P J, P y,
              const AbstractKnotPointDeclare & z) {
  jacobian(fun, J, y, getargs(inputtype, z));
}

template <typename T, typename P>
auto jacobian(AbstractFunction fun, T J, T y, T x, T u, P p) {
  jacobian(fun, J, y, x, u);
}
template <typename T> auto jacobian(AbstractFunction fun, T J, T y, T x, T u) {
  throw std::runtime_error("User-defined jacobian not implemented");
}

template <typename P, int Nx, int Nu,typename V, typename T>
auto d_jacobian(FunctionSignature, UserDefined, AbstractFunction fun, P H, P b,
                P y, const AbstractKnotPointDeclare & z) {
  d_jacobian(fun, H, b, y, state(z), control(z), getparams(z));
}

template <typename T, typename Q>
auto d_jacobian(AbstractFunction fun, T H, T b, T y, T x, T u, Q p) {
  d_jacobian(fun, H, b, y, x, u);
}

#endif
