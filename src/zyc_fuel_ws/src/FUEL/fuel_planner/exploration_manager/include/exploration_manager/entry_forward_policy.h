#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {
namespace entry_forward {

inline bool needsAdvance(double progress) {
  return progress >= -0.15 && progress < 1.0 - 1e-6;
}

// 入口起步允许真实起点略在门外，路径不能比起点更靠外；进门后不能再出门。
// 目标仍须在门内，起点余量不用于放宽候选点的入口边界。
inline bool pathStartsTowardInside(const std::vector<Eigen::Vector3d>& path,
                                  const Eigen::Vector3d& origin,
                                  const Eigen::Vector3d& forward,
                                  double start_back_margin) {
  if (path.empty()) return false;
  const double start_progress = (path.front() - origin).dot(forward);
  if (!(start_progress >= -std::max(0.0, start_back_margin)) ||
      !((path.back() - origin).dot(forward) >= 0.0))
    return false;
  bool entered = start_progress >= 0.0;
  for (const auto& point : path) {
    const double progress = (point - origin).dot(forward);
    if (!(progress >= (entered ? 0.0 : start_progress))) return false;
    if (progress >= 0.0) entered = true;
  }
  return true;
}

struct Goal {
  Eigen::Vector3d position;
  double advance{0.0};
  double side{0.0};
  double clearance{0.0};
  double continuation{0.0};
  // 0：普通短步；1：另一侧跨障通路；2：较宽或已选侧的跨障通路。
  int passage_priority{0};
  double passage_side{0.0};
};

// 先选能接到已观测障碍通路的候选，再比较短步净空；无通路证据时沿用原评分。
template <typename Evaluate>
bool selectGoal(const Eigen::Vector3d& start, const Eigen::Vector3d& forward,
                double remaining, Evaluate evaluate, Goal& best) {
  const Eigen::Vector3d lateral(-forward.y(), forward.x(), 0.0);
  const double minimum_step = std::min(0.20, remaining);
  double best_score = -std::numeric_limits<double>::infinity();
  int best_passage_priority = -1;
  for (double advance = remaining; advance >= minimum_step - 1e-6; advance -= 0.20) {
    for (double side : {0.0, -0.20, 0.20, -0.40, 0.40, -0.60, 0.60}) {
      Goal candidate;
      candidate.position = start + advance * forward + side * lateral;
      candidate.advance = advance;
      candidate.side = side;
      if (!evaluate(candidate))
        continue;
      const double score = 2.0 * std::min(0.60, candidate.clearance) +
                           1.5 * candidate.continuation + 0.50 * advance -
                           0.25 * std::fabs(side);
      if (candidate.passage_priority > best_passage_priority ||
          (candidate.passage_priority == best_passage_priority &&
           score > best_score + 1e-6)) {
        best_passage_priority = candidate.passage_priority;
        best = candidate;
        best_score = score;
      }
    }
  }
  return std::isfinite(best_score);
}

}  // namespace entry_forward
}  // namespace fast_planner
