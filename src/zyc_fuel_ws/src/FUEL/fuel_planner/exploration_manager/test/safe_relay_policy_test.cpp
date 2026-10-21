#include <exploration_manager/safe_relay_policy.h>
#include <iostream>
#include <stdexcept>
using namespace fast_planner::safe_relay;
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
std::vector<Sample> straight(int last = 60) {
  std::vector<Sample> route;
  for (int i = 0; i <= last; ++i) {
    Sample s;
    s.position = Eigen::Vector3d(i * 0.1, 0.0, 0.8);
    s.progress = i * 0.1;
    s.clearance = 1.0;
    s.hold_free = s.confirmed = true;
    route.push_back(s);
  }
  return route;
}
void checkHorizontalSpacing() {
  // Same bent measured route for all destination types. B has >1m of
  // accumulated travel but is only 0.65m from the latest issued point A.
  // C is exactly 0.70m away. A is deliberately not at the world origin.
  const Eigen::Vector3d origin(3.0, -2.0, 1.0);
  for (const auto kind : {Kind::NORMAL, Kind::AFTER_OBSTACLE, Kind::BEFORE_OBSTACLE}) {
    const std::vector<Eigen::Vector3d> offsets{
        {0.0, 0.0, 0.0}, {0.4, 0.0, 0.0}, {0.4, 0.5, 0.0},
        {0.65, 0.0, 0.0}, {0.70, 0.0, 0.0}, {2.5, 0.0, 0.0}};
    std::vector<Sample> route;
    for (std::size_t i = 0; i < offsets.size(); ++i) {
      Sample p;
      p.position = origin + offsets[i];
      p.progress = i == 0 ? 0.0 : route.back().progress +
          (p.position - route.back().position).norm();
      p.hold_free = p.confirmed = i == 0 || i == 3 || i == 4 ||
          (kind == Kind::BEFORE_OBSTACLE && i < 5);
      p.clearance = 1.0;
      route.push_back(p);
    }
    Config cfg;
    cfg.minimum_progress = 0.7;
    cfg.normal_spacing = kind == Kind::NORMAL ? 1.0 : 10.0;
    require(cfg.minimum_xy_spacing == 0.7, "default horizontal inter-goal spacing must be 0.7m");
    auto selected = select(route, 0, route.back().position, cfg);
    require(selected.kind == kind && selected.index == 4,
            "all goal types must skip nearby B and accept C at the 0.7m XY boundary");

    route[4].position.x() = origin.x() + 0.6999;
    require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
            "just below 0.7m XY must wait even though cumulative progress is sufficient");
    route[4].hold_free = route[4].confirmed = false;
    require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
            "no horizontally separated endpoint must not fall back to a closer point");

    // Extra altitude/3D separation cannot satisfy a horizontal-only gate.
    route[3].position.z() += 1.0;
    for (std::size_t i = 1; i < route.size(); ++i)
      route[i].progress = route[i-1].progress + (route[i].position-route[i-1].position).norm();
    require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
            "height changes do not substitute for XY separation");

    // The publisher explicitly disables only this gate for the first goal.
    cfg.minimum_xy_spacing = 0.0;
    selected = select(route, 0, route.back().position, cfg);
    require(selected.kind == kind && selected.index == 3,
            "entrance exception leaves the existing candidate and progress checks intact");
  }
}
int main() {
  checkHorizontalSpacing();
  Config cfg;
  // Legacy cases below exercise the ordinary leader-clearance gate separately
  // from the new long-gap exception.
  cfg.maximum_gap = 100.0;
  Config gap_cfg;
  auto gap_route = straight(14);
  require(select(gap_route, 0, gap_route.back().position, gap_cfg).kind == Kind::WAIT,
          "current-position fallback must not activate before 1.5m of route progress");
  gap_route = straight(15);
  for (auto& p : gap_route) p.clearance = 0.31 + 0.01 * p.progress;
  auto gap_pick = select(gap_route, 0, gap_route.back().position, gap_cfg);
  require(gap_pick.kind == Kind::GAP_FALLBACK && gap_pick.index == 15,
          "1.5m gap must allow the qualified current endpoint without waiting another 1m");
  for (auto& p : gap_route) p.confirmed = false;
  require(select(gap_route, 0, gap_route.back().position, gap_cfg).kind == Kind::WAIT,
          "long-gap fallback must retain map confirmation");
  for (auto& p : gap_route) { p.confirmed = true; p.hold_free = false; }
  require(select(gap_route, 0, gap_route.back().position, gap_cfg).kind == Kind::WAIT,
          "1.5m gap must never fabricate an unsafe endpoint");
  for (auto& p : gap_route) { p.hold_free = true; p.position.x() = 0.65; }
  gap_route[0].position.x() = 0.0;
  require(select(gap_route, 0, gap_route.back().position, gap_cfg).kind == Kind::WAIT,
          "long-gap fallback still requires 0.7m XY separation from the last published point");
  // Prefer a local clearance peak even when a weaker candidate is already
  // eligible for release. Merely crossing the spacing gate is insufficient.
  auto ranked = straight();
  for (auto& p : ranked) p.clearance = 0.31;
  ranked[14].clearance = 0.55;
  ranked[16].clearance = 0.60;
  require(select(ranked, 0, ranked.back().position, cfg).index == 16,
          "ordinary point must wait for the better nearby clearance peak");
  ranked[16].confirmed = false;
  require(select(ranked, 0, ranked.back().position, cfg).index == 14,
          "unconfirmed wider space must not veto a confirmed waiting point");
  ranked[16].confirmed = true;
  Config wait_cfg = cfg;
  wait_cfg.normal_spacing = 10.0;
  ranked[16].clearance = 0.31;
  for (std::size_t i = 20; i < ranked.size(); ++i) ranked[i].hold_free = false;
  require(select(ranked, 0, ranked.back().position, wait_cfg).index == 14,
          "pre-obstacle fallback also compares clearance within its window");
  ranked = straight();
  for (auto& p : ranked) p.clearance = 0.31;
  ranked[5].hold_free = false;
  ranked[11].clearance = 0.55;
  ranked[13].clearance = 0.60;
  Config after_cfg = wait_cfg;
  after_cfg.minimum_xy_spacing = 0.0;
  require(select(ranked, 0, ranked.back().position, after_cfg).index == 13,
          "post-obstacle fallback must maximize clearance within its local window");
  // A broad previous pocket -> weak pre-obstacle point -> weak post-obstacle
  // point -> broad new pocket. Model the geometry behind recorded points 9-12,
  // without hard-coding their coordinates or requiring a larger global radius.
  auto pockets = straight();
  const std::vector<double> gaps{0.55, 0.55, 0.55, 0.55, 0.55, 0.55, 0.55,
      0.47, 0.40, 0.31, 0.0, 0.33, 0.35, 0.42, 0.50, 0.58, 0.59,
      0.58, 0.50, 0.45, 0.40, 0.35};
  for (std::size_t i = 0; i < pockets.size(); ++i) {
    pockets[i].hold_free = pockets[i].confirmed = i < gaps.size() && i != 10;
    pockets[i].clearance = i < gaps.size() ? gaps[i] : 0.0;
  }
  require(select(pockets, 0, pockets.back().position, cfg).index == 16 &&
          select(pockets, 0, pockets.back().position, wait_cfg).index == 16,
          "ordinary and fallback goals must skip both weak transition points for the wide pocket");
  for (std::size_t i = 10; i < pockets.size(); ++i)
    pockets[i].hold_free = pockets[i].confirmed = false;
  require(select(pockets, 0, pockets.back().position, wait_cfg).kind == Kind::WAIT,
          "pre-obstacle fallback must retain the existing pocket instead of publishing a weaker point");
  auto route = straight(100);
  auto pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 10, "normal route must release at 1m progress");
  auto next = select(route, pick.index, route.back().position, cfg);
  require(next.kind == Kind::NORMAL && next.index == 20, "normal route must continue at 1m spacing");
  require(route[pick.index].position.z() == 0.8, "keep measured height");

  route = straight();
  for (int i = 20; i <= 30; ++i) route[i].hold_free = route[i].confirmed = false;
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 10,
          "a distant transit-only segment must not suppress the ordinary 1m point");
  pick = select(route, 10, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 31,
          "normal endpoints remain eligible beyond unqualified historical samples");
  // When ordinary spacing is not yet available, retain the existing B/C/W fallback.
  Config fallback_cfg = cfg;
  fallback_cfg.normal_spacing = 6.0;
  pick = select(route, 0, route.back().position, fallback_cfg);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index == 31, "first qualified post-obstacle sample is eligible");
  const auto rejected_b = pick.index;
  route[rejected_b].hold_free = route[rejected_b].confirmed = false;
  pick = select(route, 0, route.back().position, fallback_cfg);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index != rejected_b, "B fails: try qualified C");
  for (std::size_t i = 20; i < route.size(); ++i)
    route[i].hold_free = route[i].confirmed = false;
  pick = select(route, 0, route.back().position, fallback_cfg);
  require(pick.kind == Kind::BEFORE_OBSTACLE && pick.index == 19,
          "B/C fail: release forward pre-obstacle waiting point");
  pick = select(route, 17, route.back().position, cfg);
  require(pick.kind == Kind::WAIT, "waiting fallback must not command rollback or duplicate anchor");
  for (auto& sample : route) sample.hold_free = sample.confirmed = false;
  require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
          "all endpoints fail: do not fabricate a target");

  route = straight(19);
  require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
          "do not publish a point the leader has not cleared by 1m");
  route = straight(20);
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 10,
          "1m normal progress plus 1m leader clearance is sufficient");

  route = straight();
  route[9].hold_free = route[9].confirmed = false;
  route[11].hold_free = route[11].confirmed = false;
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 10,
          "ordinary B needs its own stopping space, not its neighbours' stopping space");
  route[10].confirmed = false;
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 12,
          "independent ordinary selection must retain map confirmation");

  Config entrance_cfg = cfg;
  entrance_cfg.minimum_xy_spacing = 0.0;  // No previously issued point at entrance.
  route = straight(19);
  for (auto& s : route) s.hold_free = s.confirmed = false;
  route[0].hold_free = route[0].confirmed = true;
  route[6].hold_free = route[6].confirmed = true;
  route[7].hold_free = route[7].confirmed = true;
  pick = select(route, 0, route.back().position, entrance_cfg);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index == 6,
          "default 1m policy retains post-obstacle fallback below normal spacing");
  route[6].hold_free = route[6].confirmed = false;
  pick = select(route, 0, route.back().position, entrance_cfg);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index == 7,
          "default fallback still tries C after B itself fails");
  route = straight(19);
  for (int i = 15; i <= 19; ++i) route[i].hold_free = route[i].confirmed = false;
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::BEFORE_OBSTACLE && pick.index == 9,
          "default 1m policy retains a checked pre-obstacle waiting fallback");

  // B is independently eligible even if the anchor and all intervening
  // samples are unqualified. The follower plans its own connection.
  route = straight();
  for (auto& s : route) s.hold_free = s.confirmed = false;
  route[30].hold_free = route[30].confirmed = true;
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 30,
          "isolated qualified B must not require neighbouring stopping space");
  route[30].confirmed = false;
  require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
          "isolated B still requires map confirmation");
  route[30].confirmed = true;
  route[30].hold_free = false;
  require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
          "isolated B itself must have safe stopping space");
  route = straight();
  for (auto& s : route) s.confirmed = false;
  require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
          "single map confirmation is insufficient; no dwell is needed");
  for (auto& s : route) s.hold_free = false;
  require(select(route, 0, route.back().position, cfg).kind == Kind::WAIT,
          "no qualified stopping space must produce no target");
  route = straight();
  for (int i = 12; i <= 40; ++i) route[i].hold_free = route[i].confirmed = false;
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index == 10,
          "a safe ordinary point before the 1.2m endpoint remains eligible");
  pick = select(route, 10, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index > 40,
          "unsafe 1.2m planning endpoint must never become stopping goal");
  // A bend uses route order, not world X or vehicle yaw.
  for (auto& s : route) s.position = Eigen::Vector3d(0.0, -s.progress, 1.1);
  pick = select(route, 10, route.back().position, cfg);
  require(pick.kind == Kind::NORMAL && pick.index > 40,
          "post-obstacle means along actual route, not world X");
  auto leader_near = route[pick.index].position;
  auto released = select(route, 0, leader_near, cfg);
  require(released.kind == Kind::WAIT ||
              (route[released.index].position.head<2>() - leader_near.head<2>()).norm() >= 1.0 - 1e-6,
          "arc clearance alone is insufficient on a looping route");
  require(select({}, 0, Eigen::Vector3d::Zero(), cfg).kind == Kind::WAIT,
          "empty actual route cannot produce predicted waypoints");
  std::cout << "PASS safe relay: normal progress, B/C/W/no-update, passage, confirmation, curves, XY inter-goal spacing, XYZ\n";
}
