#ifndef KNOT_POINT_H
#define KNOT_POINT_H

#include <Eigen/Dense>
#include <tuple>
#include <type_traits>
#include <vector>

#include <base/base.h>

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
  static_assert(static_vector, "Not static vector!!!");

public:
  ~AbstractKnotPoint() = default;
  typedef typename std::conditional<static_vector, Vector<T, Nx>, V>::type
      state_type;
  typedef typename std::conditional<static_vector, Vector<T, Nu>, V>::type
      control_type;
  typedef typename std::conditional<static_vector, Vector<T, Nx + Nu>, V>::type
      value_type;
  typedef
      typename std::conditional<static_vector, Eigen::Ref<MatrixX<T>>,
                                std::vector<std::vector<T>>>::type hessian_type;
  typedef
      typename std::conditional<static_vector, Eigen::Ref<VectorX<T>>, V>::type
          gradient_type;
  typedef T base_type;
  typedef value_type vectype;
  static constexpr int N = Nx;
  static constexpr int M = Nu;

  // Pure virtual functions.
  // virtual value_type &getdata() = 0;
  // virtual value_type &getstate() = 0;
  // virtual value_type &getcontrol() = 0;
  virtual void settime(T t) = 0;
  virtual void settimestep(T dt) = 0;

  virtual std::tuple<T, T> params() const = 0;

  virtual state_type *state() = 0;
  virtual const state_type &state() const = 0;
  virtual void setstate(const state_type &x) = 0;

  // TODO: check is_terminal()
  virtual control_type *control() = 0;
  virtual const control_type &control() const = 0;
  virtual void setcontrol(const control_type &u) = 0;

  virtual value_type *data() = 0;
  virtual const value_type &data() const = 0;
  virtual void setdata(const value_type &v) = 0;

  // Operators.
  const auto &operator[](int i) const { return data()[i]; }
  auto &operator[](int i) { return data()[i]; }
  auto &operator*(T c) {
    data() *= c;
    return *this;
  }

  // Functions.
  value_type getinput(StateControl) const { return data(); }
  state_type getinput(StateOnly) const { return state(); }
  control_type getinput(ControlOnly) const { return control(); }
  std::tuple<state_type, control_type, std::tuple<int, int>>
  getargs(StateControl) const {
    return std::make_tuple(state(), control(), params());
  }
  std::tuple<state_type> getargs(StateOnly) const {
    return std::make_tuple(state());
  }
  std::tuple<control_type> getargs(ControlOnly) const {
    return std::make_tuple(control());
  }

  int state_dim() const { return Nx; };
  int control_dim() const { return Nu; };
  int errstate_dim() const { return state_dim(); }
  int size() const { return state_dim() + control_dim(); }
  std::tuple<int, int> dims() {
    return std::make_tuple(state_dim(), control_dim());
  }

  base_type time() const { return std::get<0>(params()); }
  base_type timestep() const { return std::get<1>(params()); }
  bool is_terminal() const { return timestep() == 0.0; }
};
template <int Nx, int Nu, typename T>
using AbstractKnotPointX = AbstractKnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using AbstractKnotPointXd = AbstractKnotPoint<Nx, Nu, VectorXd, double>;
template <int Nx, int Nu, typename T>
using AbstractKnotPointS = AbstractKnotPoint<Nx, Nu, Vector<T, Nx + Nu>, T>;
template <int Nx, int Nu>
using AbstractKnotPointSd =
    AbstractKnotPoint<Nx, Nu, Vector<double, Nx + Nu>, double>;

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
  typedef
      typename std::conditional<static_vector, Eigen::Ref<MatrixX<T>>,
                                std::vector<std::vector<T>>>::type hessian_type;
  typedef
      typename std::conditional<static_vector, Eigen::Ref<VectorX<T>>, V>::type
          gradient_type;
  typedef T base_type;
  typedef value_type vectype;
  static constexpr int N = Nx;
  static constexpr int M = Nu;
};

template <int Nx, int Nu, typename T>
using KnotPointS = KnotPoint<Nx, Nu, Vector<T, Nx + Nu>, T>;
template <int Nx, int Nu>
using KnotPointSd = KnotPoint<Nx, Nu, Vector<double, Nx + Nu>, double>;
template <int Nx, int Nu, typename T>
using KnotPointX = KnotPoint<Nx, Nu, VectorX<T>, T>;
template <int Nx, int Nu>
using KnotPointXd = KnotPoint<Nx, Nu, VectorX<double>, double>;

/*Specialization*/
template <int Nx, int Nu, typename T>
class KnotPoint<Nx, Nu, Vector<T, Nx + Nu>, T>
    : public AbstractKnotPoint<Nx, Nu, Vector<T, Nx + Nu>, T> {
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
  typedef
      typename std::conditional<static_vector, Eigen::Ref<MatrixX<T>>,
                                std::vector<std::vector<T>>>::type hessian_type;
  typedef typename std::conditional<static_vector, Eigen::Ref<VectorX<T>>,
                                    std::vector<T>>::type gradient_type;
  typedef T base_type;
  typedef value_type vectype;
  static constexpr int N = Nx;
  static constexpr int M = Nu;

  // Overrides.
  state_type *state() override { return &zx_; }
  const state_type &state() const override { return zx_; }
  void setstate(const state_type &zx) override {
    zx_ = zx;
    z_ << zx_, zu_;
  }

  // TODO: check is_terminal()
  control_type *control() override { return &zu_; }
  const control_type &control() const override { return zu_; }
  void setcontrol(const control_type &zu) override {
    zu_ = zu;
    z_ << zx_, zu_;
  }

  value_type *data() override { return &z_; }
  const value_type &data() const override { return z_; }
  void setdata(const value_type &z) override {
    z_ = z;
    zx_ = z_(seqN(0, Nx));
    zu_ = z_(seqN(Nx, Nu));
  }

  void settime(T t) override { t_ = t; }
  void settimestep(T dt) override { dt_ = dt; }
  std::tuple<T, T> params() const override { return std::make_tuple(t_, dt_); }

  // Constructor
  KnotPoint() = default;
  KnotPoint(value_type z, T t, T dt) {
    CHECK(Nx + Nu == length(z));
    z_ = z;
    zx_ = z_(seqN(0, Nx));
    zu_ = z_(seqN(Nx, Nu));
    t_ = t;
    dt_ = dt;
  }
  KnotPoint(state_type x, control_type u, T t, T dt) {
    CHECK(Nx == length(x));
    CHECK(Nu == length(u));
    zx_ = x;
    zu_ = u;
    z_ << zx_, zu_;
    t_ = t;
    dt_ = dt;
  }
  KnotPoint(const KnotPointS<Nx, Nu, T> &in) {
    CHECK(length(z_) == length(in.z_));
    CHECK(Nx == length(in.zx_));
    CHECK(Nu == length(in.zu_));
    z_ = in.z_;
    zx_ = in.zx_;
    zu_ = in.zu_;
    t_ = in.t_;
    dt_ = in.dt_;
  }

private:
  // Members
  // Need to sync all data after updating.
  value_type z_;
  state_type zx_;
  control_type zu_;
  T t_;
  T dt_;
};

#endif
