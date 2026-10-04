#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <exploration_manager/task_search_manager.h>
#include <exploration_manager/wall_end_detection.h>

namespace {
void check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << message << std::endl;
  std::exit(1);
}

struct Wall {
  double begin, end, lateral;
};

struct Scene {
  std::vector<Wall> walls;
  bool unknown_outside = false;
  bool only_center_observed = false;
  bool front_wall = false;

  bool occupied(double forward, double lateral) const {
    if (front_wall && forward >= 0.50 && forward <= 0.65) return true;
    for (const auto& wall : walls)
      if (forward >= wall.begin && forward < wall.end &&
          std::fabs(lateral - wall.lateral) <= 0.04)
        return true;
    return false;
  }
  bool free(double forward, double lateral) const {
    if (occupied(forward, lateral)) return false;
    if (forward >= 0.50 && unknown_outside) return false;
    if (forward >= 0.50 && only_center_observed && std::fabs(lateral) > 0.30)
      return false;
    return true;
  }
};

bool detect(const Scene& scene, fast_planner::wall_end::Result& result) {
  return fast_planner::wall_end::detect(
      fast_planner::wall_end::Config{},
      [&](double forward, double lateral) { return scene.occupied(forward, lateral); },
      [&](double forward, double lateral) { return scene.free(forward, lateral); }, result);
}
}  // namespace

namespace fast_planner {
class TaskSearchManagerTest {
public:
  static void run() {
    ros::Time::init();
    TaskSearchManager manager;
    manager.require_stage2_detections_ = false;
    manager.exit_detection_enabled_ = true;
    manager.exit_candidate_confirmed_ = true;
    manager.exit_portal_locked_ = true;
    manager.exit_portal_outside_verified_ = true;
    manager.exit_portal_center_ = Eigen::Vector3d(1.0, 0.0, 0.60);
    manager.exit_candidate_ = Eigen::Vector3d(0.55, 0.0, 0.60);
    manager.exit_outward_direction_ = Eigen::Vector2d(1.0, 0.0);
    // 没有入口拓扑、累计里程为0、frontier仍有覆盖，确认出口也必须切入穿越。
    manager.exit_geodesic_distance_ = 0.0;
    Eigen::Vector3d goal;
    double yaw = 0.0;
    manager.exit_detection_enabled_ = false;
    manager.buildStage3Goal(manager.exit_candidate_, 0.0, false, goal, yaw);
    check(!manager.stage3Active(), "disabled automatic detection changed mission stage");
    manager.exit_detection_enabled_ = true;
    manager.buildStage3Goal(manager.exit_candidate_, 0.0, false, goal, yaw);
    check(manager.mission_stage_ == TaskSearchManager::CROSS_EXIT,
          "confirmed wall end waited for frontier exhaustion or entry distance");

    TaskSearchManager pending;
    pending.require_stage2_detections_ = false;
    pending.exit_detection_enabled_ = true;
    pending.buildStage3Goal(Eigen::Vector3d::Zero(), 0.0, true, goal, yaw);
    check(!pending.stage3Active(), "frontier exhaustion alone authorized an exit");
  }
};
}  // namespace fast_planner

int main() {
  fast_planner::wall_end::Result result;
  Scene exit{{{-3.0, 0.50, -0.65}, {-3.0, 0.50, 0.65}}};
  check(detect(exit, result), "bilateral wall disappearance was not detected");
  check(std::fabs(result.forward - 0.50) <= 0.30, "exit plane is far from wall ends");
  check(std::fabs(result.lateral) < 0.06, "centered corridor produced a lateral bias");

  Scene asymmetric{{{-3.0, 0.50, -0.55}, {-3.0, 0.50, 0.95}}};
  check(detect(asymmetric, result), "asymmetric walls required a frame shape");
  check(std::fabs(result.lateral - 0.20) < 0.06, "asymmetric corridor center is wrong");

  Scene corridor{{{-3.0, 5.0, -0.65}, {-3.0, 5.0, 0.65}}};
  check(!detect(corridor, result), "continuing bilateral walls were treated as an exit");
  Scene one_side{{{-3.0, 0.50, -0.65}, {-3.0, 5.0, 0.65}}};
  check(!detect(one_side, result), "one disappearing wall was treated as an exit");
  Scene wider{{{-3.0, 0.50, -0.65}, {-3.0, 0.50, 0.65},
               {0.50, 5.0, -1.20}, {0.50, 5.0, 1.20}}};
  check(!detect(wider, result), "wider continuing walls were treated as disappeared");
  Scene gap{{{-3.0, 0.50, -0.65}, {-3.0, 0.50, 0.65},
             {0.75, 5.0, -0.65}, {0.75, 5.0, 0.65}}};
  check(!detect(gap, result), "short bilateral wall gaps were treated as an exit");
  Scene unseen = exit;
  unseen.unknown_outside = true;
  check(!detect(unseen, result), "unknown outside space was treated as free");
  unseen = exit;
  unseen.only_center_observed = true;
  check(!detect(unseen, result), "unobserved wall positions were treated as disappeared");
  Scene corner = exit;
  corner.front_wall = true;
  check(!detect(corner, result), "blocked forward corner was treated as an exit");
  Scene posts{{{0.0, 0.10, -0.65}, {0.0, 0.10, 0.65}}};
  check(!detect(posts, result), "isolated posts replaced the corridor wall history");
  fast_planner::TaskSearchManagerTest::run();
  return 0;
}
