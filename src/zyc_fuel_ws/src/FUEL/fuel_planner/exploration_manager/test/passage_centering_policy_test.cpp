#include <exploration_manager/passage_centering_policy.h>
#include <iostream>
#include <stdexcept>

using Eigen::Vector3d;
using namespace fast_planner::passage_centering;
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
int main() {
  Config cfg;
  cfg.step = 0.025;
  // These bounds represent feasible VEHICLE centres after footprint checks.
  // A low roof/ground narrows Z, without changing the desired lateral centre.
  auto corridor = [](const Vector3d& p) {
    return p.y() > -0.10 && p.y() < 0.90 && p.z() >= 0.10 && p.z() <= 0.12;
  };
  const Vector3d point(0, 0, 0.11);
  const auto centre = sectionCentre(point, Vector3d::UnitX(), 0.45, cfg, corridor);
  require(std::fabs(centre.y() - 0.40) <= cfg.step, "asymmetric lane should move towards its centre");
  require(centre.z() == point.z(), "ground and roof must not shift the selected height");
  auto divided = [](const Vector3d& p) {
    return p.y() > -0.1 && p.y() < 1.5 && !(p.y() >= 0.3 && p.y() <= 0.35);
  };
  require(sectionCentre(point, Vector3d::UnitX(), 0.45, cfg, divided).y() < 0.15,
          "never centre across a separating obstacle or unknown strip");
  auto open = [](const Vector3d&) { return true; };
  require((sectionCentre(point, Vector3d::UnitX(), 0.45, cfg, open) - point).norm() < 1e-9,
          "unbounded symmetric open space must not cause lateral drift");
  require((sectionCentre(point, Vector3d::UnitZ(), 0.45, cfg, corridor) - point).norm() < 1e-9,
          "fixed-XY vertical motion must not be centred horizontally");
  std::vector<Vector3d> path{point, Vector3d(2, 0, 0.11)};
  auto safe = [&](const std::vector<Vector3d>& route) {
    for (std::size_t i = 1; i < route.size(); ++i)
      if (!knownSegment(route[i - 1], route[i], 0.01, corridor)) return false;
    return true;
  };
  require(centrePath(path, 3.0, cfg, corridor, safe), "a straight edge-hugging route must acquire central guides");
  require(path.front() == point && path.back() == Vector3d(2, 0, 0.11), "preserve actual start and fixed goal");
  double best_y = 0.0;
  for (const auto& p : path) { best_y = std::max(best_y, p.y()); require(p.z() == 0.11, "preserve height"); }
  require(best_y >= 0.375 && safe(path), "central route must remain connected and inside the lane");
  const std::vector<Vector3d> original{point, Vector3d(2, 0, 0.11)};
  path = original;
  auto reject = [](const std::vector<Vector3d>&) { return false; };
  require(!centrePath(path, 3.0, cfg, corridor, reject) && path == original,
          "failed footprint/mission validation must retain the original route");
  auto unknown = [](const Vector3d&) { return false; };
  require(!centrePath(path, 3.0, cfg, unknown, safe) && path == original,
          "unknown space must not invent a central route");
  // Rotate the world: centering follows the path's normal, not world Y.
  auto rotated = [&](const Vector3d& p) { return corridor(Vector3d(p.y(), -p.x(), p.z())); };
  const auto turned = sectionCentre(point, Vector3d::UnitY(), 0.45, cfg, rotated);
  require(std::fabs(turned.x() + 0.4) <= cfg.step, "centering must follow travel direction");
  std::cout << "PASS passage centering: asymmetric/low/divided/open lanes, vertical motion, connected guides\n";
}
