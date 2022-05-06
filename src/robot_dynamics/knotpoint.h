#ifndef KNOT_POINT_H
#define KNOT_POINT_H

#include <Eigen/Dense>
#include <tuple>
#include <vector>

#include <base/base.h>

using Eigen::seqN;
using Eigen::VectorX;

template <typename T> struct AbstractVector {};

#define AbstractKnotPointTypeName int Nx, int Nu, typename V, typename T
#define AbstractKnotPointTemplate                                              \
  template <int Nx, int Nu, typename V, typename T>
#define AbstractKnotPointDeclare AbstractKnotPoint<Nx, Nu, V, T>

AbstractKnotPointTemplate struct AbstractKnotPoint : AbstractVector<T> {
  typedef V vectype;
  typedef T datatype;
  auto size() const {
    return std::make_tuple(state_dim(*this) + control_dim(*this));
  }
  const auto &operator[](int i) const { return getdata(*this)[i]; }
  auto &operator[](int i) { return getdata(*this)[i]; }
  const auto &operator*(T c) const {
    getdata(*this) * c;
    return *this;
  }
  auto &operator*(T c) {
    getdata(*this) * c;
    return *this;
  }
  V z;
  T t;
  T dt;
  int n;
  int m;
};

template <int Nx, int Nu, typename T>
using AbstractKnotPointX = AbstractKnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using AbstractKnotPointXd = AbstractKnotPoint<Nx, Nu, VectorXd, double>;

AbstractKnotPointTemplate auto dims(const AbstractKnotPointDeclare &z) {
  return std::make_tuple(state_dim(z), control_dim(z));
}

// May support matrix in need.
AbstractKnotPointTemplate auto getstate(const AbstractKnotPointDeclare &z,
                                        const VectorX<T> &v) {
  return v(seqN(0, state_dim(z)));
}

// May support matrix in need.
AbstractKnotPointTemplate auto getcontrol(const AbstractKnotPointDeclare &z,
                                          const VectorX<T> &v) {
  if (is_terminal(z)) {
    return VectorX<T>::Zero(std::get<1>(dims(z)));
  } else {
    int n = 0, m = 0;
    std::tie(n, m) = dims(z);
    return v(seqN(n, m));
  }
  // int n = 0, m = 0;
  // std::tie(n, m) = dims(z);
  // return !is_terminal(z) * v(seqN(n, m));
}
AbstractKnotPointTemplate auto state_dim(const AbstractKnotPointDeclare &z) {
  return z.n;
}

AbstractKnotPointTemplate auto control_dim(const AbstractKnotPointDeclare &z) {
  return z.m;
}

AbstractKnotPointTemplate auto getdata(const AbstractKnotPointDeclare &z) {
  return z.z;
}

AbstractKnotPointTemplate auto state(const AbstractKnotPointDeclare &z) {
  return getstate(z, getdata(z));
}

AbstractKnotPointTemplate auto control(const AbstractKnotPointDeclare &z) {
  return getcontrol(z, getdata(z));
}

AbstractKnotPointTemplate auto setdata(AbstractKnotPointDeclare &z,
                                       const VectorX<T> &v) {
  z.z = v;
}

AbstractKnotPointTemplate auto setstate(AbstractKnotPointDeclare &z,
                                        const VectorX<T> &x) {
  // state(z) = x;
  setdata(z, x);
}

AbstractKnotPointTemplate auto setcontrol(AbstractKnotPointDeclare &z,
                                          const VectorX<T> &u) {
  // control(z) = u;
  setdata(z, u);
}

AbstractKnotPointTemplate auto settime(AbstractKnotPointDeclare &z, double t) {
  z.t = t;
}

AbstractKnotPointTemplate auto settimestep(AbstractKnotPointDeclare &z,
                                           double dt) {
  z.dt = dt;
}

AbstractKnotPointTemplate auto getparams(const AbstractKnotPointDeclare &z) {
  return std::make_tuple(z.t, z.dt);
}

AbstractKnotPointTemplate auto time(const AbstractKnotPointDeclare &z) {
  return std::get<0>(getparams(z));
}

AbstractKnotPointTemplate auto timestep(const AbstractKnotPointDeclare &z) {
  return std::get<1>(getparams(z));
}

AbstractKnotPointTemplate auto is_terminal(const AbstractKnotPointDeclare &z) {
  timestep(z) == 0.0;
}

#define KnotPointTemplate template <int Nx, int Nu, typename V, typename T>
#define KnotPointDeclare KnotPoint<Nx, Nu, V, T>

KnotPointTemplate struct KnotPoint : AbstractKnotPointDeclare {};

KnotPointTemplate struct StaticKnotPoint : AbstractKnotPointDeclare {};

template <int Nx, int Nu, typename T>
using KnotPointX = KnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using KnotPointXd = KnotPoint<Nx, Nu, VectorXd, double>;

/*Specialization*/
template <int Nx, int Nu, typename T>
struct KnotPoint<Nx, Nu, VectorX<T>, T>
    : AbstractKnotPoint<Nx, Nu, VectorX<T>, T> {
  // typedef typename AbstractKnotPoint<Nx, Nu, VectorX<T>, T>::vectype V;
  typedef VectorX<T> V;
  KnotPoint() {}
  KnotPoint(V z, T t, T dt) {
    this->z = z;
    this->t = t;
    this->dt = dt;
    this->n = Nx;
    this->m = Nu;
  }
  KnotPoint(V x, V u, T t, T dt) {
    assert(Nx == length(x));
    assert(Nu == length(u));
    // VectorX<double> vec_joined(x.size() + u.size());
    // vec_joined << x, u;
    this->z.resize(x.size() + u.size());
    this->z << x, u;
    this->t = t;
    this->dt = dt;
    this->n = Nx;
    this->m = Nu;
  }
  KnotPoint(int n, int m, V z, double t, double dt) {
    assert(Nx == n);
    assert(Nu == m);
    assert(n + m == length(z));
    this->z = z;
    this->t = t;
    this->dt = dt;
    this->n = Nx;
    this->m = Nu;
  }
  KnotPoint(const KnotPoint &in) {
    this->z = in.z;
    this->t = in.t;
    this->dt = in.dt;
    this->n = in.n;
    this->m = in.m;
  }
  V z;
  T t;
  T dt;
  int n;
  int m;
};

#endif
