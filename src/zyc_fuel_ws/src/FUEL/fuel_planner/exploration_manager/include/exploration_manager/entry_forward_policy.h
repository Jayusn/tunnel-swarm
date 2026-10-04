#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_planner {
namespace entry_forward {

inline bool needsAdvance(double progress) {
  return progress >= -0.15 && progress < 1.0 - 1e-6;
}

struct Goal {
  Eigen::Vector3d position;
  double advance{0.0};
  double side{0.0};
  double clearance{0.0};
  double continuation{0.0};
};

// 净空和下一段连接性只参与评分；没有额外净空硬阈值，通行由调用方检查。
template <typename Evaluate>
bool selectGoal(const Eigen::Vector3d& start, const Eigen::Vector3d& forward,
                double remaining, Evaluate evaluate, Goal& best) {
  const Eigen::Vector3d lateral(-forward.y(), forward.x(), 0.0);
  const double minimum_step = std::min(0.20, remaining);
  double best_score = -std::numeric_limits<double>::infinity();
  for (double advance = remaining; advance >= minimum_step - 1e-6; advance -= 0.20) {
    for (double side : {0.0, -0.20, 0.20, -0.40, 0.40, -0.60, 0.60}) {
      Goal candidate;
      candidate.position = start + advance * forward + side * lateral;
      candidate.advance = advance;
      candidate.side = side;
      if (!evaluate(candidate.position, candidate.clearance, candidate.continuation))
        continue;
      const double score = 2.0 * std::min(0.60, candidate.clearance) +
                           1.5 * candidate.continuation + 0.50 * advance -
                           0.25 * std::fabs(side);
      if (score > best_score + 1e-6) {
        best = candidate;
        best_score = score;
      }
    }
  }
  return std::isfinite(best_score);
}

}  // namespace entry_forward
}  // namespace fast_planner
