#include "mpc_wrapper.h"
#include <gtest/gtest.h>
using namespace PayloadMPC;
Eigen::Vector4d qref(double deg) {
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(deg * M_PI / 180., Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Quaterniond q(R);
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}
TEST(QuaternionReference, CrossesEigenSignBoundaryWithoutReverseControl) {
  for (double direction : {-1.0, 1.0}) {
    MpcWrapper w;
    w.setDynamicParams(1.817);
    Eigen::Matrix<real_t, kStateSize, 1> x =
        Eigen::Matrix<real_t, kStateSize, 1>::Zero();
    x(2) = .49;
    x.segment<4>(3) = qref(direction * 110).cast<real_t>();
    Eigen::Matrix<real_t, kInputSize, 1> u =
        Eigen::Matrix<real_t, kInputSize, 1>::Zero();
    u(0) = 1.817 * 9.795;
    Eigen::Matrix<real_t, kCostSize, kCostSize> Q =
        Eigen::Matrix<real_t, kCostSize, kCostSize>::Zero();
    Q.diagonal() << 200, 200, 200, 120, 120, 120, 120, 55, 55, 55;
    Eigen::Matrix<real_t, kInputSize, kInputSize> R =
        Eigen::Matrix<real_t, kInputSize, kInputSize>::Zero();
    R.diagonal() << 1.2, 20, 20, 1.2, 5000, 5000, 5000;
    w.initialize(Q, R, x, u, .5, .5);
    w.setExternalForce(Eigen::Vector3d::Zero());
    w.setLimits(1, 29.5, 1.5, 1, 1.5, 1, 1.5, 1);
    w.prepare();
    for (int n = 0; n < 300; n++) {
      double yaw = direction * (110 + n * .05);
      x.segment<4>(3) =
          Eigen::Vector4d(cos(yaw * M_PI / 360), 0, 0, sin(yaw * M_PI / 360))
              .cast<real_t>();
      Eigen::Matrix<real_t, kStateSize, kSamples + 1> refs =
          x.replicate(1, kSamples + 1);
      Eigen::Matrix<real_t, kInputSize, kSamples + 1> ins =
          u.replicate(1, kSamples + 1);
      ins.row(3).setConstant(direction * 5 * M_PI / 180);
      // 输入保留 Eigen 矩阵转换产生的符号跳变；模拟测量源交替使用 q/-q。
      if (n % 2)
        x.segment<4>(3) *= -1;
      for (int i = 0; i <= kSamples; i++)
        refs.col(i).segment<4>(3) =
            qref(yaw + direction * i * .05 * 5).cast<real_t>();
      w.setTrajectory(refs, ins);
      bool ok = w.update(x, false);
      Eigen::Matrix<real_t, kInputSize, 1> out;
      w.getInput(0, out);
      ASSERT_TRUE(ok) << "yaw=" << yaw;
      ASSERT_TRUE(out.allFinite());
      EXPECT_GT(direction * out(3), 0.0) << "反向偏航 yaw=" << yaw;
      EXPECT_LT(std::abs(out(3)), 0.3);
      ASSERT_TRUE(w.prepare());
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
