#ifndef EXPLORATION_MANAGER_WALL_END_DETECTION_H_
#define EXPLORATION_MANAGER_WALL_END_DETECTION_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {
namespace wall_end {

struct Config {
  double search_min = -0.15;
  double search_max = 2.0;
  double search_step = 0.10;
  double side_min = 0.25;
  double side_max = 1.75;
  double inside_length = 0.90;
  double inside_step = 0.12;
  int min_inside_support = 5;
  double wall_tolerance = 0.22;
  double outside_step = 0.25;
  int outside_count = 4;
  int min_absent_count = 3;
  double min_free_ratio = 0.75;
};

struct Result {
  double forward = 0.0;
  double lateral = 0.0;
  int left_support = 0;
  int right_support = 0;
  int absent_count = 0;
  int free_samples = 0;
  double confidence = 0.0;
};

// 查询均使用当前通道的前向/横向坐标；FREE必须来自实际观测，UNKNOWN不是墙已消失的证据。
template <typename OccupiedQuery, typename FreeQuery>
bool detect(const Config& config, OccupiedQuery occupied, FreeQuery known_free,
            Result& result) {
  const double inside_step = std::max(0.10, config.inside_step);
  const int min_support = std::max(1, config.min_inside_support);
  const int inside_count = std::max(
      min_support, static_cast<int>(std::floor(config.inside_length / inside_step)));
  const int outside_count = std::max(1, config.outside_count);
  const int min_absent = std::min(outside_count, std::max(1, config.min_absent_count));
  auto median = [](std::vector<double> values) {
    if (values.empty()) return std::numeric_limits<double>::infinity();
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  };
  auto nearest_sides = [&](double forward, double& left, double& right) {
    left = right = std::numeric_limits<double>::infinity();
    for (double side = config.side_min; side <= config.side_max + 1e-6; side += 0.05) {
      if (!std::isfinite(left) && occupied(forward, -side)) left = side;
      if (!std::isfinite(right) && occupied(forward, side)) right = side;
      if (std::isfinite(left) && std::isfinite(right)) break;
    }
  };

  bool found = false;
  double best_score = -std::numeric_limits<double>::infinity();
  for (double section = config.search_min; section <= config.search_max + 1e-6;
       section += std::max(0.05, config.search_step)) {
    std::vector<double> inside_left, inside_right;
    for (int bin = 1; bin <= inside_count; ++bin) {
      double left, right;
      nearest_sides(section - bin * inside_step, left, right);
      if (std::isfinite(left)) inside_left.push_back(left);
      if (std::isfinite(right)) inside_right.push_back(right);
    }
    if (static_cast<int>(inside_left.size()) < min_support ||
        static_cast<int>(inside_right.size()) < min_support)
      continue;
    const double wall_left = median(inside_left);
    const double wall_right = median(inside_right);
    int left_support = 0, right_support = 0;
    for (double side : inside_left)
      if (std::fabs(side - wall_left) <= config.wall_tolerance) ++left_support;
    for (double side : inside_right)
      if (std::fabs(side - wall_right) <= config.wall_tolerance) ++right_support;
    if (left_support < min_support || right_support < min_support) continue;

    const double center = 0.5 * (wall_right - wall_left);
    int absent_count = 0, free_samples = 0;
    for (int probe = 1; probe <= outside_count; ++probe) {
      const double forward = section + probe * std::max(0.10, config.outside_step);
      double left, right;
      nearest_sides(forward, left, right);
      // 只接受两侧墙都消失；仍能看到更宽的双墙时，继续按通道探索。
      if (!std::isfinite(left) && !std::isfinite(right)) ++absent_count;
      // 原墙位置及通道中心均采样，避免只清过中线却把未观测的两侧当成墙端。
      for (double lateral : {-wall_left, center, wall_right})
        if (known_free(forward, lateral)) ++free_samples;
    }
    const double free_ratio = static_cast<double>(free_samples) / (3 * outside_count);
    if (absent_count < min_absent || free_ratio + 1e-6 < config.min_free_ratio) continue;

    // 前方通行中线应连续为FREE，不能跳过转角前墙后再把墙后的空区当成出口。
    bool center_clear = true;
    const double outside_end = section + outside_count * std::max(0.10, config.outside_step);
    for (double forward = std::min(0.0, section); forward <= outside_end + 1e-6;
         forward += 0.10) {
      if (!known_free(forward, center)) {
        center_clear = false;
        break;
      }
    }
    if (!center_clear) continue;

    const double score = 0.50 * std::min(left_support, right_support) +
                         1.20 * absent_count + 2.0 * free_ratio -
                         0.10 * std::max(0.0, section);
    if (found && score <= best_score) continue;
    found = true;
    best_score = score;
    result.forward = section;
    result.lateral = center;
    result.left_support = left_support;
    result.right_support = right_support;
    result.absent_count = absent_count;
    result.free_samples = free_samples;
    result.confidence = std::min(
        0.98, 0.48 + 0.04 * std::min(left_support, right_support) +
                  0.06 * absent_count + 0.20 * free_ratio);
  }
  return found;
}

}  // namespace wall_end
}  // namespace fast_planner

#endif
