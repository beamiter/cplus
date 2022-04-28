#include <gtest/gtest.h>

#include "ilqr/cost_expansion.h"
#include "robot_dynamics/knotpoint.h"

//#include <glog/logging.h>

TEST(CostExpansionTest, StateControl) {
  auto sce1 = StateControlExpansion<double>(6, 2);
  sce1.grad(3) = 3;
  std::cout << sce1.data << std::endl;

  auto sce2 = StateControlExpansion<double, true>(6, 2);
  sce2.grad(3) = 4;
  std::cout << sce2.data << std::endl;

  auto sce3 = StateControlExpansion<double, false>(6, 2);
  sce3.grad(3) = 5;
  std::cout << sce3.data << std::endl;

  auto sce4 = StateControlExpansion<double, false>(6);
  sce4.grad(3) = 6;
  std::cout << sce4.data << std::endl;
  std::cout << sce4.state_control << std::endl;

  auto cost1 = CostExpansion<double>({1, 2, 3}, {1, 2, 3});
  auto cost2 = CostExpansion<double, false>({1, 2, 3}, {1, 2, 3});
  auto cost3 = CostExpansion<double, false>({1, 2, 3}, {0, 2, 3});
  auto cost4 = CostExpansion<double, false>({1, 2, 3});
  auto cost5 = CostExpansionHelper<double>::init(6, 2, 3);

  int model = 1;
  auto fse1 =
      FullStateExpansion<double, cost1.state_control, EuclideanState>::value(
          cost1, model);
  auto fse2 =
      FullStateExpansion<double, cost2.state_control, EuclideanState>::value(
          cost2, model);
  auto fse3 =
      FullStateExpansion<double, cost3.state_control, EuclideanState>::value(
          cost3, model);
  auto fse4 =
      FullStateExpansion<double, cost4.state_control, EuclideanState>::value(
          cost4, model);
  std::cout << fse4.data[0].data << std::endl << std::endl;
  auto fse5 =
      FullStateExpansion<double, cost5.state_control, RotationState>::value(
          cost5, model);
  std::cout << fse5[0].data << std::endl << std::endl;

  const auto &a = Eigen::VectorXd::Zero(8);
  KnotPoint<6, 2, Eigen::VectorX<double>, double> point1(a, 2.0, 3.0);
  KnotPoint<6, 2, Eigen::VectorX<double>, double> point2(6, 2, a, 2.0, 3.0);
  std::cout << point1.dt << std::endl;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // google::InitGoogleLogging(argv[0]);

  std::cout << RUN_ALL_TESTS();

  // google::ShutdownGoogleLogging();
  return 1;
}
