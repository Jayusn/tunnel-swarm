#ifndef FUEL_FAPP_VEHICLE_GEOMETRY_H_
#define FUEL_FAPP_VEHICLE_GEOMETRY_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

namespace fuel_fapp
{

// Signed translational clearance between a vertical cylinder and an AABB.
// Box dimensions are full, nonnegative extents; zero-sized axes are allowed.
// The scalar margin pads the combined configuration-space envelope once.
inline double cylinderBoxClearance(
    const Eigen::Vector3d &vehicle_position, const Eigen::Vector3d &box_center,
    const Eigen::Vector3d &box_full_size, const double radius,
    const double half_height, const double margin,
    Eigen::Vector3d *gradient = nullptr)
{
  const Eigen::Vector3d relative = vehicle_position - box_center;
  const Eigen::Vector2d rectangle_gap =
      relative.head<2>().cwiseAbs() - 0.5 * box_full_size.head<2>();
  const Eigen::Vector2d outside_rectangle = rectangle_gap.cwiseMax(0.0);
  const double outside_rectangle_distance = outside_rectangle.norm();
  const double horizontal_gap =
      outside_rectangle_distance +
      std::min(0.0, rectangle_gap.maxCoeff()) - radius;
  const double vertical_gap =
      std::abs(relative.z()) - 0.5 * box_full_size.z() - half_height;
  const double outside_horizontal = std::max(0.0, horizontal_gap);
  const double outside_vertical = std::max(0.0, vertical_gap);
  const double outside_distance =
      std::hypot(outside_horizontal, outside_vertical);

  if (gradient != nullptr)
  {
    Eigen::Vector3d horizontal_normal = Eigen::Vector3d::Zero();
    if (outside_rectangle_distance > 0.0)
    {
      horizontal_normal.x() = outside_rectangle.x() /
          outside_rectangle_distance * (relative.x() < 0.0 ? -1.0 : 1.0);
      horizontal_normal.y() = outside_rectangle.y() /
          outside_rectangle_distance * (relative.y() < 0.0 ? -1.0 : 1.0);
    }
    else
    {
      const int axis = rectangle_gap.x() >= rectangle_gap.y() ? 0 : 1;
      horizontal_normal(axis) = relative(axis) < 0.0 ? -1.0 : 1.0;
    }
    const Eigen::Vector3d vertical_normal(
        0.0, 0.0, relative.z() > 0.0 ? 1.0 : -1.0);
    if (outside_distance > 0.0)
      *gradient = (outside_horizontal * horizontal_normal +
                   outside_vertical * vertical_normal) / outside_distance;
    else
      *gradient = vertical_gap >= horizontal_gap
          ? vertical_normal : horizontal_normal;
  }

  return outside_distance + std::min(0.0, std::max(horizontal_gap, vertical_gap))
      - margin;
}

} // namespace fuel_fapp

#endif
