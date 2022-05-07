#ifndef DYNAMICS_EXPANSION_H
#define DYNAMICS_EXPANSION_H

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
  VectorX<T> f;
  MatrixX<T> Df;
  Ref<MatrixX<T>> A;
  Ref<MatrixX<T>> B;
  MatrixX<T> De;
  Ref<MatrixX<T>> fx;
  Ref<MatrixX<T>> fu;
  MatrixX<T> tmp;
  static auto init(int n, int e, int m) {
    auto f = VectorX<T>::Zero(n);
    auto Df = MatrixX<T>::Zero(n, n + m);
    MatrixX<T> De;
    if (n != e) {
      De = MatrixX<T>::Zero(e, e + m);
    } else {
      De = Df;
    }
    auto tmp = MatrixX<T>::Zero(n, e);
    return DynamicsExpansion(f, Df, De, tmp, n, e, m);
  }
};

#endif
