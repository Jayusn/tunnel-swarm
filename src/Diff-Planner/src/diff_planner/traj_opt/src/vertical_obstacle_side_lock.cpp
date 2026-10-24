#include <optimizer/vertical_obstacle_side_lock.h>

#include <optimizer/suspended_obstacle_underpass.h>

#include <algorithm>
#include <cmath>

namespace diff_planner
{

void VerticalObstacleSideLock::setConfig(const Config &config)
{
  config_ = config;
  if (!std::isfinite(config_.corridor_half_width) ||
      config_.corridor_half_width <= 0.0)
    config_.corridor_half_width = 0.75;
  if (!std::isfinite(config_.width_similarity_tolerance) ||
      config_.width_similarity_tolerance < 0.0)
    config_.width_similarity_tolerance = 0.15;
  if (!std::isfinite(config_.min_side_width) || config_.min_side_width <= 0.0)
    config_.min_side_width = 0.20;
  if (!std::isfinite(config_.min_vertical_gap) ||
      config_.min_vertical_gap <= 0.0)
    config_.min_vertical_gap = 0.35;
  if (!std::isfinite(config_.association_distance) ||
      config_.association_distance <= 0.0)
    config_.association_distance = 0.75;
  if (!std::isfinite(config_.longitudinal_margin) ||
      config_.longitudinal_margin < 0.0)
    config_.longitudinal_margin = 0.30;
  if (!std::isfinite(config_.search_timeout) || config_.search_timeout <= 0.0)
    config_.search_timeout = 0.20;
  if (!config_.enabled)
    lock_.active = false;
}

const VerticalObstacleSideLock::Config &
VerticalObstacleSideLock::config() const
{
  return config_;
}

bool VerticalObstacleSideLock::widthsAreComparable(
    const double left_width, const double right_width,
    const double min_width, const double tolerance)
{
  return std::isfinite(left_width) && std::isfinite(right_width) &&
         std::isfinite(min_width) && std::isfinite(tolerance) &&
         left_width >= min_width && right_width >= min_width &&
         std::abs(left_width - right_width) <= tolerance;
}

Eigen::Vector3d VerticalObstacleSideLock::chooseSideNormal(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &obstacle_center,
    const Eigen::Vector3d &lateral,
    const double left_width, const double right_width,
    const double width_similarity_tolerance)
{
  if (left_width > right_width + width_similarity_tolerance + 1.0e-6)
    return lateral;
  if (right_width > left_width + width_similarity_tolerance + 1.0e-6)
    return -lateral;

  double strongest_offset = 0.0;
  for (const Eigen::Vector3d &point : path)
  {
    if (!point.allFinite())
      continue;
    const double offset = (point - obstacle_center).dot(lateral);
    if (std::abs(offset) > std::abs(strongest_offset))
      strongest_offset = offset;
  }

  if (std::abs(strongest_offset) <= 1.0e-6)
    strongest_offset = left_width >= right_width ? 1.0 : -1.0;
  return strongest_offset >= 0.0 ? lateral : -lateral;
}

double VerticalObstacleSideLock::computePassageCenter(
    const double obstacle_boundary_offset, const double wall_boundary_offset)
{
  return 0.5 * (obstacle_boundary_offset + wall_boundary_offset);
}

bool VerticalObstacleSideLock::isSameObstacle(
    const Eigen::Vector3d &locked_center, const Eigen::Vector3d &locked_axis,
    const Eigen::Vector3d &locked_normal, const double locked_half_length,
    const Eigen::Vector3d &candidate_center,
    const double candidate_half_length, const double association_distance,
    const double longitudinal_margin)
{
  if (!locked_center.allFinite() || !locked_axis.allFinite() ||
      !locked_normal.allFinite() || !candidate_center.allFinite() ||
      locked_axis.norm() <= 1.0e-6 || locked_normal.norm() <= 1.0e-6)
    return false;

  const Eigen::Vector3d delta = candidate_center - locked_center;
  const double lateral_distance = std::abs(delta.dot(locked_normal.normalized()));
  const double longitudinal_distance = std::abs(delta.dot(locked_axis.normalized()));
  return lateral_distance <= association_distance + 1.0e-6 &&
         longitudinal_distance <= locked_half_length + candidate_half_length +
                                      longitudinal_margin + 1.0e-6;
}

bool VerticalObstacleSideLock::isGroundConnected(
    const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &point) const
{
  const int sample_count =
      std::max(1, static_cast<int>(std::ceil(2.0 / step_size)));
  std::vector<int> downward_occupancy;
  downward_occupancy.reserve(sample_count);
  for (int index = 1; index <= sample_count; ++index)
  {
    Eigen::Vector3d below = point;
    below.z() -= index * step_size;
    downward_occupancy.push_back(grid_map->getInflateOccupancy(below));
  }
  return !SuspendedObstacleUnderpass::hasVerticalGap(
      downward_occupancy, step_size, config_.min_vertical_gap);
}

std::vector<VerticalObstacleSideLock::Passage>
VerticalObstacleSideLock::findSplitPassages(
    const std::vector<std::vector<int>> &occupancy,
    const double step_size, const double min_width,
    std::pair<int, int> *divider_rows)
{
  if (divider_rows) *divider_rows = {-1, -1};
  if (occupancy.size() < 3 || occupancy.front().size() < 3 ||
      !std::isfinite(step_size) || step_size <= 0.0 ||
      !std::isfinite(min_width) || min_width <= 0.0)
    return {};
  const int rows = occupancy.size(), cols = occupancy.front().size();
  for (const auto &row : occupancy)
    if (static_cast<int>(row.size()) != cols)
      return {};

  // 障碍范围由目标连线上实际被挡住的区段确定，而不是占据连通块。
  // 膨胀障碍即使与侧墙相连，另一侧仍可能存在完整通路。
  int first_row = rows, last_row = -1;
  for (int r = 0; r < rows; ++r)
  {
    if (occupancy[r][cols / 2] < 0) return {};
    if (occupancy[r][cols / 2] > 0)
    {
      first_row = std::min(first_row, r);
      last_row = r;
    }
  }
  // 目标连线上的障碍前后必须已观测完整，不能把截断的墙端当作出口。
  if (last_row < first_row || first_row == 0 || last_row == rows - 1)
    return {};
  if (divider_rows) *divider_rows = {first_row, last_row};

  // 检查包含纵向余量的整个窗口，局部空洞不能充当贯通通路。
  std::vector<bool> through(cols, true);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      through[c] = through[c] && occupancy[r][c] == 0;

  std::vector<Passage> passages;
  for (int c = 0; c < cols;)
  {
    if (!through[c])
    {
      ++c;
      continue;
    }
    const int begin = c;
    while (c < cols && through[c])
      ++c;
    // 贯通自由带是各截面的交集，其两端的限制可来自不同纵向位置。
    // 必须各有真实占据边界；未知格和扫描上限不能假装成墙。
    if (begin == 0 || c == cols) continue;
    bool bounded_low = false, bounded_high = false;
    for (int r = 0; r < rows; ++r)
    {
      bounded_low = bounded_low || occupancy[r][begin - 1] > 0;
      bounded_high = bounded_high || occupancy[r][c] > 0;
    }
    if (!bounded_low || !bounded_high) continue;
    if ((c - begin) * step_size + 1.0e-6 >= min_width)
      passages.push_back({(begin - cols / 2 - 0.5) * step_size,
                          (c - cols / 2 - 0.5) * step_size});
  }
  return passages;
}

bool VerticalObstacleSideLock::detectCandidate(
    const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    Candidate &candidate) const
{
  const Eigen::Vector3d horizontal(end.x() - start.x(), end.y() - start.y(), 0.0);
  const double horizontal_length = horizontal.norm();
  if (!grid_map || horizontal_length <= 1.0e-3)
    return false;

  candidate.axis = horizontal / horizontal_length;
  candidate.lateral =
      Eigen::Vector3d(-candidate.axis.y(), candidate.axis.x(), 0.0);

  const Eigen::Vector3d segment = end - start;
  const int sample_count = std::max(
      1, static_cast<int>(std::ceil(segment.norm() / step_size)));
  std::vector<Eigen::Vector3d> occupied_points;
  for (int index = 0; index <= sample_count; ++index)
  {
    const Eigen::Vector3d point =
        start + segment * (static_cast<double>(index) / sample_count);
    if (grid_map->getInflateOccupancy(point) != 0)
      occupied_points.push_back(point);
  }
  if (occupied_points.empty())
  {
    ROS_INFO_THROTTLE(1.0, "[side_lock_diag] candidate=0 reason=clear_line");
    return false;
  }

  for (const Eigen::Vector3d &point : occupied_points)
  {
    if (!isGroundConnected(grid_map, step_size, point))
    {
      ROS_INFO_THROTTLE(1.0, "[side_lock_diag] candidate=0 reason=not_ground_connected");
      return false;
    }
  }

  candidate.center.setZero();
  for (const Eigen::Vector3d &point : occupied_points)
    candidate.center += point;
  candidate.center /= occupied_points.size();
  candidate.half_length = 0.0;
  for (const Eigen::Vector3d &point : occupied_points)
  {
    candidate.half_length = std::max(
        candidate.half_length,
        std::abs((point - candidate.center).dot(candidate.axis)));
  }
  // 横向覆盖完整的 1.5m 通道，包括柱子偏心时较远的一侧外墙。
  const int lateral_cells = static_cast<int>(
      std::ceil(2.0 * config_.corridor_half_width / step_size));
  const int longitudinal_cells = static_cast<int>(std::ceil(
      (candidate.half_length + config_.longitudinal_margin) / step_size)) + 2;
  std::vector<std::vector<int>> occupancy(
      2 * longitudinal_cells + 1, std::vector<int>(2 * lateral_cells + 1));
  for (int r = -longitudinal_cells; r <= longitudinal_cells; ++r)
    for (int c = -lateral_cells; c <= lateral_cells; ++c)
      occupancy[r + longitudinal_cells][c + lateral_cells] =
          grid_map->getInflateOccupancy(candidate.center +
              r * step_size * candidate.axis + c * step_size * candidate.lateral);
  std::pair<int, int> divider_rows;
  candidate.passages = findSplitPassages(
      occupancy, step_size, config_.min_side_width, &divider_rows);
  if (divider_rows.first >= 0)
  {
    candidate.center += (0.5 * (divider_rows.first + divider_rows.second) -
                         longitudinal_cells) * step_size * candidate.axis;
    candidate.half_length =
        0.5 * (divider_rows.second - divider_rows.first + 1) * step_size;
  }
  double left_width = 0.0, right_width = 0.0;
  for (const auto &passage : candidate.passages)
  {
    double &width = passage.min_offset + passage.max_offset > 0.0
        ? left_width : right_width;
    width = std::max(width, passage.max_offset - passage.min_offset);
  }
  ROS_INFO_THROTTLE(1.0,
      "[side_lock_diag] candidate=%d reason=%s left_width=%.2f right_width=%.2f passages=%zu",
      divider_rows.first >= 0, divider_rows.first >= 0 ? "measured" : "incomplete_line_span",
      left_width, right_width, candidate.passages.size());
  return divider_rows.first >= 0;
}

bool VerticalObstacleSideLock::trySearch(
    AStar &a_star, const GridMap::Ptr &grid_map, const double step_size,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const std::vector<Eigen::Vector3d> &unconstrained_path,
    std::vector<Eigen::Vector3d> &path)
{
  if (!config_.enabled || !std::isfinite(step_size) || step_size <= 0.0 ||
      !start.allFinite() || !end.allFinite() || unconstrained_path.empty())
    return false;

  path.clear();
  Candidate candidate;
  const bool detected = detectCandidate(grid_map, step_size, start, end, candidate);
  const bool same_obstacle = detected && lock_.active && isSameObstacle(
      lock_.center, lock_.axis, lock_.normal, lock_.half_length,
      candidate.center, candidate.half_length, config_.association_distance,
      config_.longitudinal_margin);

  auto search_passage = [&](const Candidate &geometry, const Passage &passage,
                            bool &exhausted) {
    AStarSearchRegion region;
    region.enabled = region.limit_passage = region.prefer_side_center = true;
    region.corridor_half_width = config_.corridor_half_width;
    region.side_half_length = geometry.half_length;
    region.time_limit = config_.search_timeout;
    region.line_start = start;
    region.line_end = end;
    region.side_origin = geometry.center;
    region.side_normal = geometry.lateral;
    region.side_axis = geometry.axis;
    region.side_center_offset = 0.5 * (passage.min_offset + passage.max_offset);
    region.passage_min_offset = passage.min_offset;
    region.passage_max_offset = passage.max_offset;
    ASTAR_RET result = a_star.AstarSearch(step_size, start, end, &region);
    if (result == ASTAR_RET::SEARCH_ERR)
    {
      // 沿用原来的中心偏好关闭重试，不改变通路边界。
      region.prefer_side_center = false;
      result = a_star.AstarSearch(step_size, start, end, &region);
    }
    exhausted = result == ASTAR_RET::SEARCH_ERR && a_star.lastSearchExhausted();
    if (result != ASTAR_RET::SUCCESS) return false;
    path = a_star.getPath();
    return !path.empty();
  };
  auto commit_passage = [&](const Passage &passage, const char *reason) {
    // 只有当前地图上的约束搜索成功，才替换已经确认的通路。
    lock_.active = true;
    lock_.center = candidate.center;
    lock_.axis = candidate.axis;
    lock_.normal = candidate.lateral;
    lock_.half_length = candidate.half_length;
    lock_.passage_center_offset = 0.5 * (passage.min_offset + passage.max_offset);
    lock_.passage_min_offset = passage.min_offset;
    lock_.passage_max_offset = passage.max_offset;
    lock_.blocked_observations = 0;
    lock_.last_blocked_revision = 0;
    ROS_INFO("[通道分流侧锁] %s：净宽 %.2fm，中心偏移 %.2fm（正左负右）。",
             reason, passage.max_offset - passage.min_offset,
             lock_.passage_center_offset);
  };

  const bool lock_relevant = lockAppliesToSegment(start, end);

  if (lock_relevant)
  {
    // 旧边界可能变窄或平移，先尝试同一通路的新边界。
    double low = (lock_.center + lock_.passage_min_offset * lock_.normal -
                   candidate.center).dot(candidate.lateral);
    double high = (lock_.center + lock_.passage_max_offset * lock_.normal -
                    candidate.center).dot(candidate.lateral);
    if (low > high) std::swap(low, high);
    int matching = -1;
    double best_overlap = 0.0;
    for (size_t i = 0; i < candidate.passages.size(); ++i)
    {
      const auto &passage = candidate.passages[i];
      const double overlap = std::min(high, passage.max_offset) -
                             std::max(low, passage.min_offset);
      if (overlap > best_overlap + 1.0e-6)
      {
        matching = static_cast<int>(i);
        best_overlap = overlap;
      }
    }
    Candidate previous;
    previous.center = lock_.center;
    previous.axis = lock_.axis;
    previous.lateral = lock_.normal;
    previous.half_length = lock_.half_length;
    bool exhausted = false;
    // 完整通道测量已经确认原侧没有足够净宽，无需等待三维搜索超时。
    const bool measured_unavailable = same_obstacle && matching < 0;
    const bool previous_passed = !measured_unavailable && search_passage(previous,
        {lock_.passage_min_offset, lock_.passage_max_offset}, exhausted);
    if (previous_passed)
    {
      lock_.blocked_observations = 0;
      lock_.last_blocked_revision = 0;
      ROS_INFO_THROTTLE(1.0, "[通道分流侧锁] 原通路搜索通过，保持已确认选择。");
      return true;
    }
    path.clear();
    if (!same_obstacle || (!measured_unavailable && !exhausted))
    {
      lock_.blocked_observations = 0;
      ROS_WARN_THROTTLE(1.0,
          "[通道分流侧锁] 原通路未通过复核，但观测不完整或搜索未穷尽；保留选择，本轮不换边。");
      return true;
    }

    if (matching >= 0)
    {
      if (search_passage(candidate, candidate.passages[matching], exhausted))
      {
        commit_passage(candidate.passages[matching], "沿用原侧并更新通路边界");
        return true;
      }
      if (!exhausted)
      {
        lock_.blocked_observations = 0;
        return true;
      }
    }

    // 同一帧地图中优化器可多次调用搜索，不能把重试次数当作堵塞帧数。
    const uint64_t revision = grid_map->getObservationRevision();
    if (revision != lock_.last_blocked_revision)
    {
      lock_.last_blocked_revision = revision;
      ++lock_.blocked_observations;
    }
    if (lock_.blocked_observations < 3)
    {
      ROS_WARN_THROTTLE(1.0,
          "[通道分流侧锁] 原通路连续不可用 %d/3 次地图观测，暂不换边。",
          lock_.blocked_observations);
      return true;
    }
    ROS_WARN_THROTTLE(1.0,
        "[通道分流侧锁] 原通路连续三次地图观测确认不可用，尝试其他贯通通路。");
  }
  else if (!detected)
  {
    return false;
  }

  if (!detected) return false;
  if (candidate.passages.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[通道分流侧锁] 已识别挡路障碍，但无有效贯通通路。");
    return true;
  }

  double reference_offset = 0.0;
  for (const auto &point : unconstrained_path)
  {
    const double offset = (point - candidate.center).dot(candidate.lateral);
    if (std::abs(offset) > std::abs(reference_offset)) reference_offset = offset;
  }
  // 首次选择/确认原路无解后，按净宽优先；相近宽度沿用原A*的方向。
  while (!candidate.passages.empty())
  {
    auto selected = std::max_element(candidate.passages.begin(), candidate.passages.end(),
        [](const Passage &a, const Passage &b) {
          return a.max_offset - a.min_offset < b.max_offset - b.min_offset;
        });
    const double widest = selected->max_offset - selected->min_offset;
    for (auto it = candidate.passages.begin(); it != candidate.passages.end(); ++it)
      if (widest - (it->max_offset - it->min_offset) <=
              config_.width_similarity_tolerance + 1.0e-6 &&
          std::abs(0.5 * (it->min_offset + it->max_offset) - reference_offset) <
              std::abs(0.5 * (selected->min_offset + selected->max_offset) - reference_offset))
        selected = it;
    bool exhausted = false;
    if (search_passage(candidate, *selected, exhausted))
    {
      commit_passage(*selected, lock_relevant ? "原路无解后更换通路" : "首次确认通路");
      return true;
    }
    if (!exhausted) return true;
    candidate.passages.erase(selected);
  }
  return true;
}

bool VerticalObstacleSideLock::lockAppliesToSegment(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end) const
{
  if (!lock_.active) return false;
  const Eigen::Vector3d segment = end - start;
  const double fraction = segment.squaredNorm() > 1.0e-6
      ? std::max(0.0, std::min(1.0, (lock_.center - start).dot(segment) /
                                     segment.squaredNorm())) : 0.0;
  const double start_along = (start - lock_.center).dot(lock_.axis);
  const double end_along = (end - lock_.center).dot(lock_.axis);
  return
      std::min(start_along, end_along) <= lock_.half_length &&
      std::max(start_along, end_along) >= -lock_.half_length &&
      (start + fraction * segment - lock_.center).head<2>().norm() <=
          config_.association_distance + lock_.half_length + config_.longitudinal_margin;
}

bool VerticalObstacleSideLock::allowsDirectRoute(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end) const
{
  if (!lockAppliesToSegment(start, end)) return true;
  const double along = (start - lock_.center).dot(lock_.axis);
  const Eigen::Vector3d delta = end - start;
  const double span = delta.dot(lock_.axis);
  double first = 0.0, last = 1.0;
  if (std::abs(span) > 1.0e-6)
  {
    first = (-lock_.half_length - along) / span;
    last = (lock_.half_length - along) / span;
    if (first > last) std::swap(first, last);
    first = std::max(0.0, first);
    last = std::min(1.0, last);
  }
  // 直线在障碍区段内的横向偏移也是线性的，检查入、出口即可。
  for (double fraction : {first, last})
  {
    const double offset = (start + fraction * delta - lock_.center).dot(lock_.normal);
    if (offset < lock_.passage_min_offset - 1.0e-6 ||
        offset > lock_.passage_max_offset + 1.0e-6) return false;
  }
  return true;
}

bool VerticalObstacleSideLock::passageCenterError(
    const Eigen::Vector3d &position, double &error, Eigen::Vector3d &direction) const
{
  if (!lockAppliesToSegment(position, position))
    return false;
  error = (position - lock_.center).dot(lock_.normal) - lock_.passage_center_offset;
  direction = lock_.normal;
  return true;
}

} // namespace diff_planner
