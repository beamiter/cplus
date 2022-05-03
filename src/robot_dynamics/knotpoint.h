#ifndef KNOT_POINT_H
#define KNOT_POINT_H

#include <Eigen/Dense>
#include <tuple>
#include <vector>

#include <base/base.h>

using Eigen::seqN;
using Eigen::VectorX;

template <typename T> struct AbstractVector {};
#define ABSTRACT_KNOT_POINT_TYPENAME int Nx, int Nu, typename V, typename T
#define ABSTRACT_KNOT_POINT_TEMPLATE                                           \
  template <int Nx, int Nu, typename V, typename T>

#define CONST_ABSTRACT_KNOT_POINT_REF const AbstractKnotPoint<Nx, Nu, V, T> &
#define CONST_ABSTRACT_KNOT_POINT const AbstractKnotPoint<Nx, Nu, V, T>
#define ABSTRACT_KNOT_POINT_REF AbstractKnotPoint<Nx, Nu, V, T> &
#define ABSTRACT_KNOT_POINT AbstractKnotPoint<Nx, Nu, V, T>

ABSTRACT_KNOT_POINT_TEMPLATE
struct AbstractKnotPoint : AbstractVector<T> {
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
};

ABSTRACT_KNOT_POINT_TEMPLATE
auto dims(CONST_ABSTRACT_KNOT_POINT_REF z) {
  return std::make_tuple(state_dim(z), control_dim(z));
}

// May support matrix in need.
ABSTRACT_KNOT_POINT_TEMPLATE
auto getstate(CONST_ABSTRACT_KNOT_POINT_REF z, const VectorX<T> &v) {
  return v(seqN(0, state_dim(z)));
}

// May support matrix in need.
ABSTRACT_KNOT_POINT_TEMPLATE
auto getcontrol(CONST_ABSTRACT_KNOT_POINT_REF z, const VectorX<T> &v) {
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

ABSTRACT_KNOT_POINT_TEMPLATE
auto state(CONST_ABSTRACT_KNOT_POINT_REF z) { return getstate(z, getdata(z)); }

ABSTRACT_KNOT_POINT_TEMPLATE
auto control(CONST_ABSTRACT_KNOT_POINT_REF z) {
  return getcontrol(z, getdata(z));
}

ABSTRACT_KNOT_POINT_TEMPLATE
auto setdata(ABSTRACT_KNOT_POINT_REF z, const VectorX<T> &v) { z.z = v; }

ABSTRACT_KNOT_POINT_TEMPLATE
auto setstate(ABSTRACT_KNOT_POINT_REF z, const VectorX<T> &x) {
  // state(z) = x;
  setdata(z, x);
}

ABSTRACT_KNOT_POINT_TEMPLATE
auto setcontrol(ABSTRACT_KNOT_POINT_REF z, const VectorX<T> &u) {
  // control(z) = u;
  setdata(z, u);
}

ABSTRACT_KNOT_POINT_TEMPLATE
auto settime(ABSTRACT_KNOT_POINT_REF z, double t) { z.t = t; }

ABSTRACT_KNOT_POINT_TEMPLATE
auto settimestep(ABSTRACT_KNOT_POINT_REF z, double dt) { z.dt = dt; }

ABSTRACT_KNOT_POINT_TEMPLATE
auto time(CONST_ABSTRACT_KNOT_POINT_REF z) { return z.t; }

ABSTRACT_KNOT_POINT_TEMPLATE
auto timestep(CONST_ABSTRACT_KNOT_POINT_REF z) { return z.dt; }

ABSTRACT_KNOT_POINT_TEMPLATE
auto is_terminla(CONST_ABSTRACT_KNOT_POINT_REF z) { timestep(z) == 0.0; }

ABSTRACT_KNOT_POINT_TEMPLATE
struct KnotPoint : ABSTRACT_KNOT_POINT {};
ABSTRACT_KNOT_POINT_TEMPLATE
struct StaticKnotPoint : ABSTRACT_KNOT_POINT {};

#define CONST_KNOT_POINT_REF const KnotPoint<Nx, Nu, V, T> &
#define CONST_KNOT_POINT const KnotPoint<Nx, Nu, V, T>
#define KNOT_POINT_REF KnotPoint<Nx, Nu, V, T> &
#define KNOT_POINT KnotPoint<Nx, Nu, V, T>

#define KNOT_POINT_TEMPLATE template <int Nx, int Nu, typename T>
#define KNOT_POINT_PARAM Nx, Nu, VectorX<T>, T
KNOT_POINT_TEMPLATE
struct KnotPoint<KNOT_POINT_PARAM> : AbstractKnotPoint<KNOT_POINT_PARAM> {
  typedef typename AbstractKnotPoint<KNOT_POINT_PARAM>::vectype V;
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

ABSTRACT_KNOT_POINT_TEMPLATE
auto state_dim(CONST_ABSTRACT_KNOT_POINT_REF z) { return z.n; }
ABSTRACT_KNOT_POINT_TEMPLATE
auto control_dim(CONST_ABSTRACT_KNOT_POINT_REF z) { return z.m; }

ABSTRACT_KNOT_POINT_TEMPLATE
auto getparams(CONST_ABSTRACT_KNOT_POINT_REF z) {
  return std::make_tuple(z.t, z.dt);
}

ABSTRACT_KNOT_POINT_TEMPLATE
auto getdata(CONST_ABSTRACT_KNOT_POINT_REF z) { return z.z; }

#endif
