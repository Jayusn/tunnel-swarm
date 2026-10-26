#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <vector>

namespace fast_planner {
namespace corridor_curve {

// 地图横切面上的左右墙点；x为当前通道前向，y为横向。
struct Result {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};  // y = c + b*x + a*x*x
  bool curved{false};
  double yaw(double forward) const {
    return std::atan(center[1] + 2.0 * center[2] * forward);
  }
};

inline bool fitWall(const std::vector<Eigen::Vector2d>& points, double tolerance,
                    Eigen::Vector3d& coefficients) {
  if (points.size() < 6) return false;
  int best_count = 0;
  double best_error = 1e9;
  Eigen::Vector3d best = Eigen::Vector3d::Zero();
  for (size_t i = 0; i + 2 < points.size(); ++i) {
    for (size_t j = i + 1; j + 1 < points.size(); ++j) {
      for (size_t k = j + 1; k < points.size(); ++k) {
        if (points[k].x() - points[i].x() < 0.45) continue;
        Eigen::Matrix3d matrix;
        Eigen::Vector3d values;
        int row = 0;
        for (size_t index : {i, j, k}) {
          const double x = points[index].x();
          matrix.row(row) << 1.0, x, x*x;
          values[row++] = points[index].y();
        }
        if (std::fabs(matrix.determinant()) < 1e-6) continue;
        const Eigen::Vector3d model = matrix.inverse() * values;
        int count = 0;
        double error = 0.0;
        for (const auto& point : points) {
          const double residual = std::fabs(
              point.y() - model.dot(Eigen::Vector3d(1, point.x(), point.x()*point.x())));
          if (residual <= tolerance) { ++count; error += residual*residual; }
        }
        if (count > best_count || (count == best_count && error < best_error)) {
          best_count = count; best_error = error; best = model;
        }
      }
    }
  }
  // 短小悬挂物、柱子不能凭局部几个点替代整侧墙体。
  if (best_count < 6 || best_count < 0.70 * points.size()) return false;
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  double min_x = 1e9, max_x = -1e9;
  for (const auto& point : points) {
    const Eigen::Vector3d row(1, point.x(), point.x()*point.x());
    if (std::fabs(point.y() - best.dot(row)) > tolerance) continue;
    normal += row * row.transpose(); rhs += row * point.y();
    min_x = std::min(min_x, point.x()); max_x = std::max(max_x, point.x());
  }
  if (max_x - min_x < 0.75 - 1e-6 || min_x > 0.10 || max_x < 0.30)
    return false;
  coefficients = normal.ldlt().solve(rhs);
  return coefficients.allFinite();
}

inline bool infer(const std::vector<Eigen::Vector2d>& left,
                  const std::vector<Eigen::Vector2d>& right,
                  double tolerance, Result& result) {
  Eigen::Vector3d l, r;
  if (!fitWall(left, tolerance, l) || !fitWall(right, tolerance, r)) return false;
  const auto tangent = [](const Eigen::Vector3d& c, double x) {
    return std::atan(c[1] + 2*c[2]*x);
  };
  // 两侧持续同向渐变才是弧形通道；一侧孤立障碍和直角墙端不能接管原转弯流程。
  for (double x : {0.0, 0.35, 0.60}) {
    const double width = (l-r).dot(Eigen::Vector3d(1,x,x*x));
    if (width < 0.40 || width > 3.50)
      return false;
  }
  // 内、外弯半径不同：远方同一个x切面并不是两侧墙的同一弧长位置。
  // 只比较当前近处切线，远处使用两侧同向变化，避免把真实内弯误判为墙端。
  if (std::fabs(tangent(l,0.0)-tangent(r,0.0)) > 25.0*M_PI/180.0)
    return false;
  const double left_change = tangent(l,0.60)-tangent(l,-0.30);
  const double right_change = tangent(r,0.60)-tangent(r,-0.30);
  if (std::max(std::fabs(left_change),std::fabs(right_change)) > 55.0*M_PI/180.0)
    return false;
  result.center = 0.5*(l+r);
  result.curved = left_change*right_change > 0 &&
      std::min(std::fabs(left_change),std::fabs(right_change)) >= 3.0*M_PI/180.0;
  // 折角/只有单墙变化不能作为渐变朝向。直线可用于弧形结束后的轴线收敛。
  if (!result.curved &&
      std::max(std::fabs(left_change),std::fabs(right_change)) > 5.0*M_PI/180.0)
    return false;
  return std::fabs(result.yaw(0.35)) <= 30.0*M_PI/180.0;
}
}  // namespace corridor_curve
}  // namespace fast_planner
