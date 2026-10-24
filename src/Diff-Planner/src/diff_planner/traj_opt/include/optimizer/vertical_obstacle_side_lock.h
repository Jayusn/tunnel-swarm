#ifndef DIFF_PLANNER_VERTICAL_OBSTACLE_SIDE_LOCK_H_
#define DIFF_PLANNER_VERTICAL_OBSTACLE_SIDE_LOCK_H_

#include <Eigen/Eigen>

#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

#include <vector>

namespace diff_planner
{

class VerticalObstacleSideLock
{
public:
  struct Config
  {
    bool enabled{false};
    double corridor_half_width{0.75};
    double width_similarity_tolerance{0.15};
    double min_side_width{0.20};
    double min_vertical_gap{0.35};
    double association_distance{0.75};
    double longitudinal_margin{0.30};
    double search_timeout{0.20};
  };

  void setConfig(const Config &config);
  const Config &config() const;

  struct Passage
  {
    double min_offset;
    double max_offset;
  };

  // 平面采样的行沿通道方向，列沿横向；允许挡路障碍与侧墙相连。
  static std::vector<Passage> findSplitPassages(
      const std::vector<std::vector<int>> &occupancy,
      double step_size, double min_width,
      std::pair<int, int> *divider_rows = nullptr);

  bool trySearch(AStar &a_star, const GridMap::Ptr &grid_map, double step_size,
                 const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                 const std::vector<Eigen::Vector3d> &unconstrained_path,
                 std::vector<Eigen::Vector3d> &path);
  bool passageCenterError(const Eigen::Vector3d &position,
                          double &error, Eigen::Vector3d &direction) const;
  bool allowsDirectRoute(const Eigen::Vector3d &start, const Eigen::Vector3d &end) const;

  static bool widthsAreComparable(double left_width, double right_width,
                                  double min_width, double tolerance);
  static Eigen::Vector3d chooseSideNormal(
      const std::vector<Eigen::Vector3d> &path,
      const Eigen::Vector3d &obstacle_center,
      const Eigen::Vector3d &lateral,
      double left_width, double right_width,
      double width_similarity_tolerance);
  static double computePassageCenter(double obstacle_boundary_offset,
                                     double wall_boundary_offset);
  static bool isSameObstacle(const Eigen::Vector3d &locked_center,
                             const Eigen::Vector3d &locked_axis,
                             const Eigen::Vector3d &locked_normal,
                             double locked_half_length,
                             const Eigen::Vector3d &candidate_center,
                             double candidate_half_length,
                             double association_distance,
                             double longitudinal_margin);

private:
  struct Candidate
  {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    Eigen::Vector3d lateral{Eigen::Vector3d::Zero()};
    double half_length{0.0};
    std::vector<Passage> passages;
  };

  struct Lock
  {
    bool active{false};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    double half_length{0.0};
    double passage_center_offset{0.0};
    double passage_min_offset{0.0};
    double passage_max_offset{0.0};
    int blocked_observations{0};
    uint64_t last_blocked_revision{0};
  };

  bool lockAppliesToSegment(const Eigen::Vector3d &start, const Eigen::Vector3d &end) const;

  bool detectCandidate(const GridMap::Ptr &grid_map, double step_size,
                       const Eigen::Vector3d &start,
                       const Eigen::Vector3d &end,
                       Candidate &candidate) const;
  bool isGroundConnected(const GridMap::Ptr &grid_map, double step_size,
                         const Eigen::Vector3d &point) const;

  Config config_;
  Lock lock_;
};

} // namespace diff_planner

#endif
