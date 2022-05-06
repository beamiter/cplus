#ifndef KNOT_POINT_H
#define KNOT_POINT_H

#include <Eigen/Dense>
#include <tuple>
#include <vector>

#include <base/base.h>

using Eigen::seqN;
using Eigen::VectorX;

template <typename T> struct AbstractVector {};

template <int Nx, int Nu, typename V, typename T>
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

template <int Nx, int Nu, typename T>
using AbstractKnotPointX = AbstractKnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using AbstractKnotPointXd = AbstractKnotPoint<Nx, Nu, VectorX<double>, double>;

template <int Nx, int Nu, typename V, typename T>
auto dims(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return std::make_tuple(state_dim(z), control_dim(z));
}

// May support matrix in need.
template <int Nx, int Nu, typename V, typename T>
auto getstate(const AbstractKnotPoint<Nx, Nu, V, T> &z, const VectorX<T> &v) {
  return v(seqN(0, state_dim(z)));
}

// May support matrix in need.
template <int Nx, int Nu, typename V, typename T>
auto getcontrol(const AbstractKnotPoint<Nx, Nu, V, T> &z, const VectorX<T> &v) {
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

template <int Nx, int Nu, typename V, typename T>
auto state(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return getstate(z, getdata(z));
}

template <int Nx, int Nu, typename V, typename T>
auto control(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return getcontrol(z, getdata(z));
}

template <int Nx, int Nu, typename V, typename T>
auto setdata(const AbstractKnotPoint<Nx, Nu, V, T> &z, const VectorX<T> &v) {
  z.z = v;
}

template <int Nx, int Nu, typename V, typename T>
auto setstate(const AbstractKnotPoint<Nx, Nu, V, T> &z, const VectorX<T> &x) {
  // state(z) = x;
  setdata(z, x);
}

template <int Nx, int Nu, typename V, typename T>
auto setcontrol(const AbstractKnotPoint<Nx, Nu, V, T> &z, const VectorX<T> &u) {
  // control(z) = u;
  setdata(z, u);
}

template <int Nx, int Nu, typename V, typename T>
auto settime(const AbstractKnotPoint<Nx, Nu, V, T> &z, double t) {
  z.t = t;
}

template <int Nx, int Nu, typename V, typename T>
auto settimestep(const AbstractKnotPoint<Nx, Nu, V, T> &z, double dt) {
  z.dt = dt;
}

template <int Nx, int Nu, typename V, typename T>
auto time(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return z.t;
}

template <int Nx, int Nu, typename V, typename T>
auto timestep(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return z.dt;
}

template <int Nx, int Nu, typename V, typename T>
auto is_terminla(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  timestep(z) == 0.0;
}

template <int Nx, int Nu, typename V, typename T>
struct KnotPoint : AbstractKnotPoint<Nx, Nu, V, T> {};

template <int Nx, int Nu, typename V, typename T>
struct StaticKnotPoint : AbstractKnotPoint<Nx, Nu, V, T> {};

template <int Nx, int Nu, typename T>
using KnotPointX = KnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using KnotPointXd = KnotPoint<Nx, Nu, VectorX<double>, double>;

template <int Nx, int Nu, typename T>
struct KnotPoint<Nx, Nu, VectorX<T>, T>
    : AbstractKnotPoint<Nx, Nu, VectorX<T>, T> {
  typedef typename AbstractKnotPoint<Nx, Nu, VectorX<T>, T>::vectype V;
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

template <int Nx, int Nu, typename V, typename T>
auto state_dim(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return z.n;
}
template <int Nx, int Nu, typename V, typename T>
auto control_dim(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return z.m;
}

template <int Nx, int Nu, typename V, typename T>
auto getparams(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return std::make_tuple(z.t, z.dt);
}

template <int Nx, int Nu, typename V, typename T>
auto getdata(const AbstractKnotPoint<Nx, Nu, V, T> &z) {
  return z.z;
}

#endif
