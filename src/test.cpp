#include <glog/logging.h>
#include <gtest/gtest.h>

#include "ilqr/cost_expansion.h"
#include "ilqr/ilqr_solve.h"
#include "ilqr/ilqr_solver.h"
#include "robot_dynamics/car_model.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "solver_opts.h"
#include "trajectory_optimization/cost_functions.h"
#include "trajectory_optimization/problem.h"

using namespace google;

TEST(EigenTest, Cholesky) {
  Eigen::MatrixXd A(3, 3);
  A << 4.0, 12.0, -16.0, 12.0, 37.0, -43.0, -16.0, -43.0, 98.0;
  Eigen::MatrixXd L(A.llt().matrixL());
  Eigen::MatrixXd L_T = L.adjoint(); // conjugate transpose
  LOG(INFO) << L;
  LOG(INFO) << L_T;
  CHECK_EQ(L * L_T, A);
}

TEST(DynamicsExpansionTest, constructor) {
  std::vector<DynamicsExpansion<double>> vec;
  vec.push_back(DynamicsExpansion<double>(6, 6, 2));
  auto func = [](DynamicsExpansion<double> *in) { in->Df->setRandom(); };
  auto &val = vec.front();
  func(&val);
  CHECK_EQ(val.A, (*val.Df)(all, Eigen::seq(0, val.A.rows() - 1)));
  CHECK_EQ(val.B, (*val.Df)(all, Eigen::seqN(val.A.rows(), val.B.cols())));
  CHECK_EQ(*val.De, *val.Df);
  CHECK_EQ(val.fx, (*val.De)(all, Eigen::seq(0, val.A.rows() - 1)));
  CHECK_EQ(val.fu, (*val.De)(all, Eigen::seqN(val.A.rows(), val.B.cols())));
}

TEST(DynamicsExpansionTest, expansion) {
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  stats.parent = SolverName::iLQR;
  std::vector<double> x0({0, 0, 0, 0, 4, 0});
  std::vector<double> xf({13, -1.0, 0, 0, 1.0, 0});
  std::vector<double> uf({0, 0});
  const int N = 51;
  const double dt = 0.1;
  const double tf = 5.0;
  CarProblemSd<6, 2, DiagonalCostS> prob(x0, xf, uf, N, dt);
  iLQRSolverSd<6, 2, DiagonalCostS> solver(&prob, opts, stats, UserDefined(),
                                           Valbool<true>());
  auto &val = solver.D_vec.front();
  CHECK_EQ(val->A, (*val->Df)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->B, (*val->Df)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
  CHECK_EQ(*val->De, *val->Df);
  CHECK_EQ(val->fx, (*val->De)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->fu, (*val->De)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
  dynamics_expansion(&solver, solver.Z_dot);
  CHECK_EQ(val->A, (*val->Df)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->B, (*val->Df)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
  CHECK_EQ(*val->De, *val->Df);
  CHECK_EQ(val->fx, (*val->De)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->fu, (*val->De)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
}

TEST(DynamicsExpansionTest, jacobian) {
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  stats.parent = SolverName::iLQR;
  std::vector<double> x0({0, 0, 0, 0, 4, 0});
  std::vector<double> xf({13, -1.0, 0, 0, 1.0, 0});
  std::vector<double> uf({0, 0});
  const int N = 51;
  const double dt = 0.1;
  const double tf = 5.0;
  CarProblemSd<6, 2, DiagonalCostS> prob(x0, xf, uf, N, dt);
  iLQRSolverSd<6, 2, DiagonalCostS> solver(&prob, opts, stats, UserDefined(),
                                           Valbool<true>());
  auto &val = solver.D_vec.front();
  CHECK_EQ(val->A, (*val->Df)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->B, (*val->Df)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
  CHECK_EQ(val->De, val->Df);
  CHECK_EQ(val->fx, (*val->De)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->fu, (*val->De)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
  jacobian(dynamics_signature(solver), UserDefined(),
           solver.shared_models_.front().get(), solver.D_vec.front().get(),
           solver.Z.front());
  CHECK_EQ(val->A, (*val->Df)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->B, (*val->Df)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
  CHECK_EQ(*val->De, *val->Df);
  CHECK_EQ(val->fx, (*val->De)(all, Eigen::seq(0, val->A.rows() - 1)));
  CHECK_EQ(val->fu, (*val->De)(all, Eigen::seqN(val->A.rows(), val->B.cols())));
}
TEST(CostExpansionTest, expansion) {
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  stats.parent = SolverName::iLQR;
  std::vector<double> x0({0, 0, 0, 0, 4, 0});
  std::vector<double> xf({13, -1.0, 0, 0, 1.0, 0});
  std::vector<double> uf({0, 0});
  const int N = 51;
  const double dt = 0.1;
  const double tf = 5.0;
  CarProblemSd<6, 2, DiagonalCostS> prob(x0, xf, uf, N, dt);
  iLQRSolverSd<6, 2, DiagonalCostS> solver(&prob, opts, stats, UserDefined(),
                                           Valbool<true>());

  auto &val = solver.Efull_->front();
  CHECK_EQ(val->grad, val->data(all, last));
  CHECK_EQ(val->hess, val->data(all, seq(0, last - 1)));
  cost_expansion(solver.obj, solver.Efull_, solver.Z_dot);
  CHECK_EQ(val->grad, val->data(all, last));
  CHECK_EQ(val->hess, val->data(all, seq(0, last - 1)));
}
TEST(StateControlExpansionTest, state_control) {
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  stats.parent = SolverName::iLQR;
  std::vector<double> x0({0, 0, 0, 0, 4, 0});
  std::vector<double> xf({13, -1.0, 0, 0, 1.0, 0});
  std::vector<double> uf({0, 0});
  const int N = 51;
  const double dt = 0.1;
  const double tf = 5.0;
  CarProblemSd<6, 2, DiagonalCostS> prob(x0, xf, uf, N, dt);
  iLQRSolverSd<6, 2, DiagonalCostS> solver(&prob, opts, stats, UserDefined(),
                                           Valbool<true>());
  // StateControlExpansion<double, true> val(6, 2);
  auto &val = *solver.Qtmp;
  LOG(INFO) << val.data;
  CHECK_EQ(val.data.rows(), 8);
  CHECK_EQ(val.data.cols(), 9);
  CHECK_EQ(val.hess.rows(), 8);
  CHECK_EQ(val.hess.cols(), 8);
  CHECK_EQ(val.hess, val.data(seq(0, 7), seq(0, 7)));

  CHECK_EQ(val.grad.rows(), 8);
  CHECK_EQ(val.grad.cols(), 1);
  CHECK_EQ(val.grad, val.data(Eigen::all, Eigen::last));

  CHECK_EQ(val.xx.rows(), 6);
  CHECK_EQ(val.xx.cols(), 6);
  CHECK_EQ(val.xx, val.data(seq(0, 5), seq(0, 5)));

  CHECK_EQ(val.x.rows(), 6);
  CHECK_EQ(val.x.cols(), 1);
  CHECK_EQ(val.x, val.data(seq(0, 5), Eigen::last));

  CHECK_EQ(val.uu.rows(), 2);
  CHECK_EQ(val.uu.cols(), 2);
  CHECK_EQ(val.uu, val.data(seqN(6, 2), seqN(6, 2)));

  CHECK_EQ(val.ux.rows(), 2);
  CHECK_EQ(val.ux.cols(), 6);
  CHECK_EQ(val.ux, val.data(seqN(6, 2), seq(0, 5)));
  LOG(INFO) << val.ux;

  CHECK_EQ(val.u.rows(), 2);
  CHECK_EQ(val.u.cols(), 1);
  CHECK_EQ(val.u, val.data(seqN(6, 2), Eigen::last));
}
TEST(StateControlExpansionTest, state_only) {
  StateControlExpansion<double, true> val(6, 2);
  LOG(INFO) << val.data;
  CHECK_EQ(val.data.rows(), 8);
  CHECK_EQ(val.data.cols(), 9);
  CHECK_EQ(val.hess.rows(), 8);
  CHECK_EQ(val.hess.cols(), 8);
  CHECK_EQ(val.hess, val.data(seq(0, 7), seq(0, 7)));

  CHECK_EQ(val.grad.rows(), 8);
  CHECK_EQ(val.grad.cols(), 1);
  CHECK_EQ(val.grad, val.data(Eigen::all, Eigen::last));

  CHECK_EQ(val.xx.rows(), 6);
  CHECK_EQ(val.xx.cols(), 6);
  CHECK_EQ(val.xx, val.data(seq(0, 5), seq(0, 5)));

  CHECK_EQ(val.x.rows(), 6);
  CHECK_EQ(val.x.cols(), 1);
  CHECK_EQ(val.x, val.data(seq(0, 5), Eigen::last));
}

TEST(iLqrSolverTest, solve) {
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  stats.parent = SolverName::iLQR;
  std::vector<double> x0({0, 0, 0, 0, 4, 0});
  std::vector<double> xf({13, -1.0, 0, 0, 1.0, 0});
  std::vector<double> uf({0, 0});
  const int N = 51;
  const double dt = 0.1;
  const double tf = 5.0;
  CarProblemSd<6, 2, DiagonalCostS> prob(x0, xf, uf, N, dt);
  iLQRSolverSd<6, 2, DiagonalCostS> solver(&prob, opts, stats, UserDefined(),
                                           Valbool<true>());
  solve(&solver);
  // LOG(INFO) << solver.Z;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  LOG(INFO) << RUN_ALL_TESTS();

  google::ShutdownGoogleLogging();
  return 1;
}
