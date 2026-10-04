#ifndef EXPLORATION_MANAGER_OPEN_SPACE_EXIT_DETECTION_H_
#define EXPLORATION_MANAGER_OPEN_SPACE_EXIT_DETECTION_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {
namespace open_space_exit {

struct Config {
  double search_min = -0.15;
  double search_max = 2.0;
  double search_step = 0.10;
  double side_clearance = 2.10;
  double min_open_ratio = 0.80;
  int sector_ray_count = 11;
  double sector_half_angle_deg = 20.0;
  double sample_step = 0.10;
  double inside_length = 0.90;
  double inside_step = 0.15;
  int min_inside_support = 5;
  double outside_length = 1.20;
};

struct Result {
  double forward = 0.0;
  double lateral = 0.0;
  double left_open_ratio = 0.0;
  double right_open_ratio = 0.0;
  double outside_lateral = 0.0;
  double confidence = 0.0;
};

struct Diagnostics {
  int supported_sections = 0;
  int widened_sections = 0;
  int open_sections = 0;
  int clear_sections = 0;
};

// 每侧所有预设射线均进入分母。占据、未知、地图外都不能证明横向净空超过阈值。
template <typename OccupiedQuery, typename FreeQuery>
int countOpenRays(const Config& config, double forward, double lateral, int side,
                  OccupiedQuery occupied, FreeQuery known_free) {
  const int ray_count = std::max(1, config.sector_ray_count);
  const double half_angle = std::max(0.0, std::min(45.0, config.sector_half_angle_deg)) *
                            std::acos(-1.0) / 180.0;
  const double step = std::min(0.05, std::max(0.01, config.sample_step));
  // 检查到阈值之外，严格区分 >2.1m 和恰好2.1m；距离使用横向投影，不用斜距。
  const double end = std::max(0.0, config.side_clearance) + 1e-3;
  int passing = 0;
  for (int ray = 0; ray < ray_count; ++ray) {
    const double angle = ray_count == 1 ? 0.0 :
        -half_angle + 2.0 * half_angle * ray / (ray_count - 1);
    bool clear = true;
    const int samples = std::max(1, static_cast<int>(std::ceil(end / step)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double across = std::min(sample * step, end);
      const double x = forward + across * std::tan(angle);
      const double y = lateral + side * across;
      if (occupied(x, y) || !known_free(x, y)) {
        clear = false;
        break;
      }
    }
    if (clear) ++passing;
  }
  return passing;
}

// occupied/known_free用于空间识别；footprint_free独立检查完整机体占用范围。
template <typename OccupiedQuery, typename FreeQuery, typename FootprintQuery>
bool detect(const Config& config, OccupiedQuery occupied, FreeQuery known_free,
            FootprintQuery footprint_free, Result& result, Diagnostics* diagnostics = nullptr) {
  if (diagnostics) *diagnostics = Diagnostics{};
  const double step = std::max(0.05, config.sample_step);
  const double inside_step = std::max(0.10, config.inside_step);
  const int min_support = std::max(1, config.min_inside_support);
  const int inside_count = std::max(
      min_support, static_cast<int>(std::floor(config.inside_length / inside_step + 1e-6)));
  const int rays = std::max(1, config.sector_ray_count);
  const int required = static_cast<int>(std::ceil(
      std::max(0.0, std::min(1.0, config.min_open_ratio)) * rays - 1e-9));
  auto median = [](std::vector<double> values) {
    auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  };
  for (double section = config.search_min; section <= config.search_max + 1e-6;
       section += std::max(0.05, config.search_step)) {
    // 来路必须确有双侧受限空间，不要求边界具有墙面或门框形状。
    std::vector<double> left, right;
    for (int bin = 1; bin <= inside_count; ++bin) {
      const double x = section - bin * inside_step;
      for (int side : {-1, 1}) {
        for (double across = 0.25; across <= config.side_clearance + 1e-6;
             across += std::min(0.05, step)) {
          if (!occupied(x, side * across)) continue;
          (side < 0 ? left : right).push_back(across);
          break;
        }
      }
    }
    if (static_cast<int>(left.size()) < min_support ||
        static_cast<int>(right.size()) < min_support) continue;
    if (diagnostics) ++diagnostics->supported_sections;
    const double left_width = median(left), right_width = median(right);
    const double center = 0.5 * (right_width - left_width);
    const int outside_count = std::max(1, static_cast<int>(
        std::ceil(std::max(step, config.outside_length) / step)));

    // 开阔区可以偏在通道一侧，但来路的两侧边界都必须实际向外扩展。
    // 在旧墙位置之外检查已观测FREE，避免单侧开口靠移动开阔区中心变成出口。
    bool widened = true;
    for (int probe = 1; widened && probe <= outside_count; ++probe) {
      const double x = section + std::min(probe * step, config.outside_length);
      for (double edge : {-left_width - step, right_width + step}) {
        const int samples = std::max(1, static_cast<int>(std::ceil(
            std::fabs(edge - center) / std::min(0.05, step))));
        for (int sample = 1; sample <= samples; ++sample) {
          const double y = center + (edge - center) * sample / samples;
          if (occupied(x, y) || !known_free(x, y)) {
            widened = false;
            break;
          }
        }
        if (!widened) break;
      }
    }
    if (!widened) continue;
    if (diagnostics) ++diagnostics->widened_sections;

    // 连续检查当前位置到过渡平面，再检查门外段；不能跨过前墙、箱体或未知区域。
    const double travel = std::hypot(section, center);
    const int approach_samples = std::max(1, static_cast<int>(std::ceil(travel / step)));
    bool path_clear = true;
    for (int sample = 0; sample <= approach_samples; ++sample) {
      const double ratio = static_cast<double>(sample) / approach_samples;
      if (!footprint_free(ratio * section, ratio * center)) {
        path_clear = false;
        break;
      }
    }
    for (int sample = 1; path_clear && sample <= outside_count; ++sample)
      path_clear = footprint_free(
          section + std::min(sample * step, config.outside_length), center);
    if (!path_clear) continue;

    if (diagnostics) ++diagnostics->clear_sections;
    // 门内中心用于出口位置和通过空间；门外净空中心允许横向平移。
    // 优先原中心，再向两侧寻找最近的完整开阔区域，不降低净空阈值。
    std::vector<double> outside_centers{center};
    for (double shift = step; shift <= config.side_clearance + 1e-6; shift += step) {
      outside_centers.push_back(center - shift);
      outside_centers.push_back(center + shift);
    }
    for (double outside_center : outside_centers) {
      int min_left = rays, min_right = rays;
      bool open = true;
      for (int probe = 1; probe <= outside_count; ++probe) {
        const double x = section + std::min(probe * step, config.outside_length);
        if (occupied(x, outside_center) || !known_free(x, outside_center)) {
          open = false;
          break;
        }
        min_left = std::min(min_left,
            countOpenRays(config, x, outside_center, -1, occupied, known_free));
        min_right = std::min(min_right,
            countOpenRays(config, x, outside_center, 1, occupied, known_free));
        if (min_left < required || min_right < required) {
          open = false;
          break;
        }
      }
      if (!open) continue;
      if (diagnostics) ++diagnostics->open_sections;
      result.forward = section;
      result.lateral = center;
      result.outside_lateral = outside_center;
      result.left_open_ratio = static_cast<double>(min_left) / rays;
      result.right_open_ratio = static_cast<double>(min_right) / rays;
      result.confidence = std::min(0.98,
          0.55 + 0.40 * std::min(result.left_open_ratio, result.right_open_ratio));
      return true;  // 保留最早满足条件的过渡，不把出口拖到开阔区域深处。
    }
  }
  return false;
}

}  // namespace open_space_exit
}  // namespace fast_planner

#endif
