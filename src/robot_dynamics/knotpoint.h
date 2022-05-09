#ifndef KNOT_POINT_H
#define KNOT_POINT_H

#include <Eigen/Dense>
#include <tuple>
#include <vector>

#include <base/base.h>

using Eigen::seqN;
using Eigen::VectorX;

#define AbstractKnotPointTypeName int Nx, int Nu, typename V, typename T
#define AbstractKnotPointTemplate                                              \
  template <int Nx, int Nu, typename V, typename T>
#define AbstractKnotPointDeclare AbstractKnotPoint<Nx, Nu, V, T>

AbstractKnotPointTemplate class AbstractKnotPoint {
public:
  ~AbstractKnotPoint() = default;
  typedef V vectype;
  typedef T datatype;


  /*Pure virtual function*/
  virtual int state_dim() = 0;
  virtual int control_dim() = 0;
  virtual V &getdata() = 0;
  virtual void settime(double t) = 0;
  virtual void settimestep(double dt) = 0;

  int errstate_dim() { return state_dim(); }

  /*Virtual function*/
  virtual std::tuple<double, double> getparams() = 0;

  /*Function*/
  auto getinput(FunctionInputs input) {
    if (input == FunctionInputs::StateOnly) {
      return state();
    } else if (input == FunctionInputs::ControlOnly) {
      return control();
    } else {
      return getdata();
    }
  }

  auto getargs(FunctionInputs) {
    return std::make_tuple(state(), control(), getparams());
  }

  auto size() const { return std::make_tuple(state_dim() + control_dim()); }

  const auto &operator[](int i) const { return getdata()[i]; }
  auto &operator[](int i) { return getdata()[i]; }

  const auto &operator*(T c) const {
    getdata() *= c;
    return *this;
  }
  auto &operator*(T c) {
    getdata() *= c;
    return *this;
  }

  std::tuple<int, int> dims() {
    return std::make_tuple(state_dim(), control_dim());
  }
  V getstate(const VectorX<T> &v) { return v(seqN(0, state_dim())); }
  auto state() { return getstate(getdata()); }
  V getcontrol(const VectorX<T> &v) {
    if (is_terminal()) {
      return VectorX<T>::Zero(std::get<1>(dims()));
    } else {
      int n = 0, m = 0;
      std::tie(n, m) = dims();
      return v(seqN(n, m));
    }
    // int n = 0, m = 0;
    // std::tie(n, m) = dims(z);
    // return !is_terminal(z) * v(seqN(n, m));
  }
  auto control() { return getcontrol(getdata()); }

  auto setdata(const VectorX<T> &v) { getdata() = v; }
  auto setstate(const VectorX<T> &x) {
    // state(z) = x;
    setdata(x);
  }
  auto setcontrol(const VectorX<T> &u) {
    // control(z) = u;
    setdata(u);
  }

  auto time() { return std::get<0>(getparams()); }
  auto timestep() { return std::get<1>(getparams()); }
  auto is_terminal(const AbstractKnotPointDeclare &z) { timestep() == 0.0; }
};

template <int Nx, int Nu, typename T>
using AbstractKnotPointX = AbstractKnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using AbstractKnotPointXd = AbstractKnotPoint<Nx, Nu, VectorXd, double>;

#define KnotPointTemplate template <int Nx, int Nu, typename V, typename T>
#define KnotPointDeclare KnotPoint<Nx, Nu, V, T>

KnotPointTemplate class StaticKnotPoint : public AbstractKnotPointDeclare {};
KnotPointTemplate class KnotPoint : public AbstractKnotPointDeclare {};

template <int Nx, int Nu, typename T>
using KnotPointX = KnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using KnotPointXd = KnotPoint<Nx, Nu, VectorXd, double>;

/*Specialization*/
template <int Nx, int Nu, typename T>
struct KnotPoint<Nx, Nu, VectorX<T>, T>
    : AbstractKnotPoint<Nx, Nu, VectorX<T>, T> {
  typedef VectorX<T> V;

public:
  int state_dim() final { return n_; }
  int control_dim() final { return m_; }
  V &getdata() final { return z_; }
  void settime(double t) final { t_ = t; }
  void settimestep(double dt) final { dt_ = dt; }
  std::tuple<double, double> getparams() final {
    return std::make_tuple(t_, dt_);
  }

  // Constructor
  KnotPoint() = default;
  KnotPoint(V z, T t, T dt) {
    z_ = z;
    t_ = t;
    dt_ = dt;
    n_ = Nx;
    m_ = Nu;
  }
  KnotPoint(V x, V u, T t, T dt) {
    assert(Nx == length(x));
    assert(Nu == length(u));
    z_.resize(x.size() + u.size());
    z_ << x, u;
    t_ = t;
    dt_ = dt;
    n_ = Nx;
    m_ = Nu;
  }
  KnotPoint(int n, int m, V z, double t, double dt) {
    assert(Nx == n);
    assert(Nu == m);
    assert(n + m == length(z));
    z_ = z;
    t_ = t;
    dt_ = dt;
    n_ = Nx;
    m_ = Nu;
  }
  KnotPoint(const KnotPoint &in) {
    z_ = in.z;
    t_ = in.t;
    dt_ = in.dt;
    n_ = in.n;
    m_ = in.m;
  }

private:
  V z_;
  T t_;
  T dt_;
  int n_;
  int m_;
};


#endif
