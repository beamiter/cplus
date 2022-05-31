#ifndef KNOT_POINT_H
#define KNOT_POINT_H

#include <Eigen/Dense>
#include <tuple>
#include <type_traits>
#include <vector>

#include <base/base.h>

using Eigen::Ref;
using Eigen::seqN;
using Eigen::Vector;
using Eigen::VectorX;

#define AbstractKnotPointTypeName int Nx, int Nu, typename V, typename T
#define AbstractKnotPointTemplate                                              \
  template <int Nx, int Nu, typename V, typename T>
#define AbstractKnotPointDeclare AbstractKnotPoint<Nx, Nu, V, T>

AbstractKnotPointTemplate class AbstractKnotPoint {
  static constexpr bool static_vector =
      std::is_base_of<Vector<T, Nx + Nu>, V>::value;
  static_assert(static_vector, "not static vector!!!");

public:
  ~AbstractKnotPoint() = default;
  typedef typename std::conditional<static_vector, Vector<T, Nx>, V>::type
      state_type;
  typedef typename std::conditional<static_vector, Vector<T, Nu>, V>::type
      control_type;
  typedef typename std::conditional<static_vector, Vector<T, Nx + Nu>, V>::type
      value_type;
  typedef T base_type;

  /*Pure virtual function*/
  virtual int state_dim() = 0;
  virtual int control_dim() = 0;
  virtual value_type &getdata() = 0;
  virtual void settime(double t) = 0;
  virtual void settimestep(double dt) = 0;

  int errstate_dim() { return state_dim(); }

  /*Virtual function*/
  virtual std::tuple<T, T> getparams() = 0;

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
  Ref<state_type> getstate(value_type &v) { return v(seqN(0, state_dim())); }
  Ref<state_type> state() { return getstate(getdata()); }
  Ref<control_type> getcontrol(value_type &v) {
    // if (is_terminal()) {
    //   return VectorX<T>::Zero(std::get<1>(dims()));
    // } else {
    //   int n = 0, m = 0;
    //   std::tie(n, m) = dims();
    //   return v(seqN(n, m));
    // }
    // TODO: Check is_terminal() when using this function!
    int n = 0, m = 0;
    std::tie(n, m) = dims();
    return v(seqN(n, m));
  }
  Ref<control_type> control() { return getcontrol(getdata()); }

  void setdata(const VectorX<T> &v) { getdata() = v; }
  void setstate(const VectorX<T> &x) { state() = x; }
  void setcontrol(const VectorX<T> &u) { control() = u; }

  base_type time() { return std::get<0>(getparams()); }
  base_type timestep() { return std::get<1>(getparams()); }
  bool is_terminal() { return timestep() == 0.0; }
};

template <int Nx, int Nu, typename T>
using AbstractKnotPointX = AbstractKnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using AbstractKnotPointXd = AbstractKnotPoint<Nx, Nu, VectorXd, double>;

#define KnotPointTemplate template <int Nx, int Nu, typename V, typename T>
#define KnotPointDeclare KnotPoint<Nx, Nu, V, T>

KnotPointTemplate class StaticKnotPoint : public AbstractKnotPointDeclare {};
KnotPointTemplate class KnotPoint : public AbstractKnotPointDeclare {
  static constexpr bool static_vector =
      std::is_base_of<Vector<T, Nx + Nu>, V>::value;
  static_assert(static_vector, "not static vector!!!");

public:
  typedef typename std::conditional<static_vector, Vector<T, Nx>, V>::type
      state_type;
  typedef typename std::conditional<static_vector, Vector<T, Nu>, V>::type
      control_type;
  typedef typename std::conditional<static_vector, Vector<T, Nx + Nu>, V>::type
      value_type;
  typedef T base_type;

  static constexpr int nx = Nx;
  static constexpr int nu = Nu;
};

template <int Nx, int Nu, typename T>
using KnotPointS = KnotPoint<Nx, Nu, Vector<T, Nx + Nu>, T>;
template <int Nx, int Nu>
using KnotPointSd = KnotPoint<Nx, Nu, Vector<double, Nx + Nu>, double>;

/*Specialization*/
template <int Nx, int Nu, typename T>
struct KnotPoint<Nx, Nu, Vector<T, Nx + Nu>, T>
    : AbstractKnotPoint<Nx, Nu, Vector<T, Nx + Nu>, T> {
  static constexpr bool static_vector = true;
  static_assert(static_vector, "not static vector!!!");

public:
  typedef
      typename std::conditional<static_vector, Vector<T, Nx>, VectorX<T>>::type
          state_type;
  typedef
      typename std::conditional<static_vector, Vector<T, Nu>, VectorX<T>>::type
          control_type;
  typedef typename std::conditional<static_vector, Vector<T, Nx + Nu>,
                                    VectorX<T>>::type value_type;
  typedef T base_type;

  static constexpr int nx = Nx;
  static constexpr int nu = Nu;

  int state_dim() final { return n_; }
  int control_dim() final { return m_; }
  value_type &getdata() final { return z_; }
  void settime(double t) final { t_ = t; }
  void settimestep(double dt) final { dt_ = dt; }
  std::tuple<double, double> getparams() final {
    return std::make_tuple(t_, dt_);
  }

  // Constructor
  KnotPoint() = default;
  KnotPoint(value_type z, T t, T dt) {
    assert(Nx + Nu == length(z));
    z_ = z;
    t_ = t;
    dt_ = dt;
  }
  KnotPoint(state_type x, control_type u, T t, T dt) {
    assert(Nx == length(x));
    assert(Nu == length(u));
    // z_.resize(x.size() + u.size());
    z_ << x, u;
    t_ = t;
    dt_ = dt;
  }
  KnotPoint(const KnotPointS<Nx, Nu, T> &in) {
    CHECK(length(z_) == length(in.z_));
    z_ = in.z_;
    t_ = in.t_;
    dt_ = in.dt_;
    n_ = in.n_;
    m_ = in.m_;
  }

private:
  // Members
  value_type z_;
  T t_;
  T dt_;
  int n_ = Nx;
  int m_ = Nu;
};

#endif
