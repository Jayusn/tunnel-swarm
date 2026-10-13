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
  if (occupancy.size() < 3 || occupancy.front().size() < 3 ||
      !std::isfinite(step_size) || step_size <= 0.0 ||
      !std::isfinite(min_width) || min_width <= 0.0)
    return {};
  const int rows = occupancy.size(), cols = occupancy.front().size();
  for (const auto &row : occupancy)
    if (static_cast<int>(row.size()) != cols)
      return {};

  // 接触采样边界的占据连通块属于外墙或尚未完整观测的障碍，不能当作中间分隔物。
  std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
  int first_row = rows, last_row = -1;
  bool intersects_search_line = false;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
    {
      if (visited[r][c] || occupancy[r][c] <= 0)
        continue;
      std::vector<std::pair<int, int>> component{{r, c}};
      visited[r][c] = true;
      bool bounded = true, intersects = false;
      int low = r, high = r;
      for (size_t i = 0; i < component.size(); ++i)
      {
        const int x = component[i].first, y = component[i].second;
        low = std::min(low, x);
        high = std::max(high, x);
        intersects = intersects || y == cols / 2;
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
          {
            if (dx == 0 && dy == 0)
              continue;
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols ||
                occupancy[nx][ny] < 0)
            {
              bounded = false;
              continue;
            }
            if (occupancy[nx][ny] > 0 && !visited[nx][ny])
            {
              visited[nx][ny] = true;
              component.emplace_back(nx, ny);
            }
          }
      }
      if (intersects && !bounded)
        return {};
      if (bounded)
      {
        first_row = std::min(first_row, low);
        last_row = std::max(last_row, high);
        intersects_search_line = intersects_search_line || intersects;
      }
    }
  if (!intersects_search_line || last_row < first_row)
    return {};

  // 在完整障碍段及其前后各一格上求自由空间交集，避免把局部空洞当成通路。
  std::vector<bool> through(cols, true);
  for (int r = first_row - 1; r <= last_row + 1; ++r)
    for (int c = 0; c < cols; ++c)
      through[c] = through[c] && occupancy[r][c] == 0;

  std::vector<Passage> passages;
  const auto &cross_section = occupancy[(first_row + last_row) / 2];
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
    // 两端必须确实碰到墙或中间分隔物；扫描上限不能假装成墙。
    if (begin == 0 || c == cols ||
        cross_section[begin - 1] <= 0 || cross_section[c] <= 0)
      continue;
    if ((c - begin) * step_size + 1.0e-6 >= min_width)
      passages.push_back({(begin - cols / 2 - 0.5) * step_size,
                          (c - cols / 2 - 0.5) * step_size});
  }
  if (passages.size() < 2)
    return {};
  if (divider_rows)
    *divider_rows = {first_row, last_row};
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
    return false;

  for (const Eigen::Vector3d &point : occupied_points)
  {
    if (!isGroundConnected(grid_map, step_size, point))
      return false;
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
  if (!candidate.passages.empty())
  {
    candidate.center += (0.5 * (divider_rows.first + divider_rows.second) -
                         longitudinal_cells) * step_size * candidate.axis;
    candidate.half_length =
        0.5 * (divider_rows.second - divider_rows.first + 1) * step_size;
  }
  return !candidate.passages.empty();
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

  Candidate candidate;
  if (!detectCandidate(grid_map, step_size, start, end, candidate))
  {
    lock_.active = false;
    return false;
  }

  const bool same_obstacle = lock_.active && isSameObstacle(
      lock_.center, lock_.axis, lock_.normal, lock_.half_length,
      candidate.center, candidate.half_length, config_.association_distance,
      config_.longitudinal_margin);
  Passage selected = candidate.passages.front();
  for (const auto &passage : candidate.passages)
  {
    const double width = passage.max_offset - passage.min_offset;
    const double selected_width = selected.max_offset - selected.min_offset;
    const double center = 0.5 * (passage.min_offset + passage.max_offset);
    const double selected_center = 0.5 * (selected.min_offset + selected.max_offset);
    if (width > selected_width + 1.0e-6 ||
        (same_obstacle && std::abs(width - selected_width) <= 1.0e-6 &&
         std::abs(center - lock_.passage_center_offset) <
             std::abs(selected_center - lock_.passage_center_offset)))
      selected = passage;
  }
  const double passage_center = 0.5 * (selected.min_offset + selected.max_offset);
  if (!same_obstacle ||
      std::abs(passage_center - lock_.passage_center_offset) > step_size)
    ROS_INFO("[通道分流侧锁] 确认 %zu 条贯通通路，选择最宽 %.2fm，中心偏移 %.2fm。",
             candidate.passages.size(), selected.max_offset - selected.min_offset,
             passage_center);
  lock_.active = true;
  lock_.center = candidate.center;
  lock_.axis = candidate.axis;
  lock_.normal = passage_center >= 0.0 ? candidate.lateral : -candidate.lateral;
  lock_.half_length = candidate.half_length;
  lock_.passage_center_offset = passage_center;

  AStarSearchRegion region;
  region.enabled = true;
  region.limit_passage = true;
  region.prefer_side_center = true;
  region.corridor_half_width = config_.corridor_half_width;
  region.side_half_length = candidate.half_length;
  region.time_limit = config_.search_timeout;
  region.report_timeout = true;
  region.line_start = start;
  region.line_end = end;
  region.side_origin = lock_.center;
  region.side_normal = candidate.lateral;
  region.side_axis = lock_.axis;
  region.side_center_offset = passage_center;
  region.passage_min_offset = selected.min_offset;
  region.passage_max_offset = selected.max_offset;

  path.clear();
  if (a_star.AstarSearch(step_size, start, end, &region) != ASTAR_RET::SUCCESS)
  {
    region.prefer_side_center = false;
    ROS_WARN_THROTTLE(1.0,
                      "[通道分流侧锁] 中心参考搜索失败，保持最宽通路重试。");
    if (a_star.AstarSearch(step_size, start, end, &region) != ASTAR_RET::SUCCESS)
      return true;
  }
  path = a_star.getPath();
  return true;
}

} // namespace diff_planner
