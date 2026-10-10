#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mot_mapping {

// Evidence from real registered returns. Unknown points remain in the caller's
// total-point denominator; a depth envelope does not imply a visible surface.
class BidirectionalDepth {
 public:
  enum Evidence { Unknown = -1, Static = 0, Foreground = 1 };

  void build(const pcl::PointCloud<pcl::PointXYZ>& cloud,
             const Eigen::Matrix3d& rotation, const Eigen::Vector3d& origin) {
    rotation_ = rotation;
    origin_ = origin;
    bins_.clear();
    for (const auto& point : cloud) {
      const Eigen::Vector3d ray = rotation_.transpose() *
          (point.getVector3fMap().cast<double>() - origin_);
      if (ray.allFinite() && ray.squaredNorm() > 0.0)
        bins_[elevation(ray) * columns + azimuth(ray)].push_back(ray);
    }
  }

  Evidence classify(const Eigen::Vector3d& world) const {
    const Eigen::Vector3d ray = rotation_.transpose() * (world - origin_);
    const double range = ray.norm();
    if (!std::isfinite(range) || range <= 0.0) return Unknown;
    const Eigen::Vector3d direction = ray / range;
    const double angle = std::asin(std::min(1.0, 0.03 / range));
    const double latitude = std::atan2(ray.z(), ray.head<2>().norm());
    const int vertical = static_cast<int>(std::ceil(angle / step())) + 1;
    const double minimum_cos = std::cos(std::min(pi() / 2, std::abs(latitude) + angle));
    const int horizontal = minimum_cos < 1e-6 ? columns / 2 :
        std::min(int(columns / 2), static_cast<int>(std::ceil(
            angle / (step() * minimum_cos))) + 1);
    const int row = elevation(ray), column = azimuth(ray);
    double minimum = std::numeric_limits<double>::infinity();
    bool exact = false;
    for (int y = std::max(0, row - vertical); y <= std::min(int(rows - 1), row + vertical); ++y) {
      const int count = std::min(int(columns), 2 * horizontal + 1);
      for (int offset = -horizontal; offset < -horizontal + count; ++offset) {
        auto found = bins_.find(y * columns + (column + offset + columns) % columns);
        if (found == bins_.end()) continue;
        for (const auto& sample : found->second) {
          const double depth = sample.norm();
          if (direction.dot(sample) <= 0.0 ||
              range * direction.cross(sample / depth).norm() > 0.03) continue;
          minimum = std::min(minimum, depth);
          exact = exact || std::abs(depth - range) <= 0.03;
        }
      }
    }
    // A real nearer return blocks visibility of a farther surface, even if
    // another neighboring ray also returned close to the query depth.
    if (minimum < range - 0.03) return Unknown;
    if (exact) return Static;
    if (std::isfinite(minimum) && range < minimum - 0.03) return Foreground;
    return Unknown;
  }

 private:
  enum { columns = 1440, rows = 720 };
  static double pi() { return 3.14159265358979323846; }
  static double step() { return pi() / 720.0; }
  static int azimuth(const Eigen::Vector3d& ray) {
    const int bin = static_cast<int>(std::floor((std::atan2(ray.y(), ray.x()) + pi()) / step()));
    return (bin + columns) % columns;
  }
  static int elevation(const Eigen::Vector3d& ray) {
    return std::max(0, std::min(int(rows - 1), static_cast<int>(std::floor(
        (std::atan2(ray.z(), ray.head<2>().norm()) + pi() / 2) / step()))));
  }
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  std::unordered_map<int, std::vector<Eigen::Vector3d,
      Eigen::aligned_allocator<Eigen::Vector3d>>> bins_;
};

}  // namespace mot_mapping
