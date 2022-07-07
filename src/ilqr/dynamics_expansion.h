#ifndef DYNAMICS_EXPANSION_H
#define DYNAMICS_EXPANSION_H

#include "base/base.h"
#include "robot_dynamics/discretized_dynamics.h"
#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/statevectortype.h"
#include <Eigen/Dense>

using Eigen::all;
using Eigen::MatrixX;
using Eigen::Ref;
using Eigen::seq;
using Eigen::seqN;
using Eigen::VectorX;

template <typename T> struct DynamicsExpansion {
  DynamicsExpansion(VectorX<T> f_in, MatrixX<T> Df_in, MatrixX<T> De_in,
                    MatrixX<T> tmp_in, int n, int e, int m)
      : f(f_in), Df(Df_in), A(Df(all, seq(0, n - 1))), B(Df(all, seqN(n, m))),
        De(De_in), fx(De(all, seq(0, e - 1))), fu(De(all, seqN(e, m))),
        tmp(tmp_in) {}
  VectorX<T> f;  // (Nx,)
  MatrixX<T> Df; // (Nx, Nx+Nu)
  Ref<MatrixX<T>> A;
  Ref<MatrixX<T>> B;
  MatrixX<T> De; // (Ne, Ne+Nu) or (Nx, Nx+Nu)
  Ref<MatrixX<T>> fx;
  Ref<MatrixX<T>> fu;
  MatrixX<T> tmp; // (Nx, Ne)
  DynamicsExpansion(int n, int e, int m)
      : f(VectorX<T>::Zero(n)), Df(MatrixX<T>::Zero(n, n + m)),
        A(Df(all, seq(0, n - 1))), B(Df(all, seqN(n, m))),
        De(n != e ? MatrixX<T>::Zero(e, e + m) : Df),
        fx(De(all, seq(0, e - 1))), fu(De(all, seqN(e, m))),
        tmp(MatrixX<T>::Zero(n, e)) {}
};

template <typename KP>
void jacobian(FunctionSignature sig, DiffMethod diff,
              const DiscreteDynamics<KP> *model,
              DynamicsExpansion<typename KP::base_type> *D, const KP &z) {
  jacobian(sig, diff, static_cast<const DiscretizedDynamics<KP, RK4> *>(model),
           D->Df, D->f, z);
}

// error_expansion
template <typename M, typename P, typename Q>
void errstate_jacobian(const std::vector<M> &model, std::vector<P> &G,
                       const Q &Z) {
  errstate_jacobian(model.front()->statevectortype(), model, G, Z);
}
template <typename M, typename P, typename Q>
void errstate_jacobian(StateVectorType type, const std::vector<M> &models,
                       std::vector<P> &G, const Q &Z) {
  if (StateVectorType::EuclideanState == type) {
    return;
  } else if (StateVectorType::RotationState == type) {
    const int N = length(Z);
    for (int k = 0; k < N; ++k) {
      const auto model = models[std::min(k, N - 2)];
      G[k].setZero();
      // TODO: Need to be implemented for RotationState.
      // errstate_jacobian(model->statevectortype(), model, G[k], Z[k]);
    }
  }
}
template <typename M, typename E, typename P>
void error_expansion(const std::vector<M> model, std::vector<E> &D,
                     const std::vector<P> &G) {
  _error_expansion(model.front()->statevectortype(), D, model, G);
}
template <typename M, typename E, typename P>
void _error_expansion(StateVectorType type, std::vector<E> &D,
                      const std::vector<M> &model, const std::vector<P> &G) {
  if (StateVectorType::EuclideanState == type) {
    return;
  } else if (StateVectorType::RotationState == type) {
    for (int k = 0; k < D.size(); ++k) {
      _error_expansion(D[k], G[k], G[k + 1]);
    }
  }
}
template <typename E, typename P>
void _error_expansion(E &D, const P &G1, const P &G2) {
  D.tmp = D.A * G1;
  D.fx = G2.adjoint() * D.tmp;
  D.fu = G2.adjoint() * D.B;
}

#endif
