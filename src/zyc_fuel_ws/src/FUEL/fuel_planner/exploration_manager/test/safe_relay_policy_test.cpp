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
int main() {
  Config cfg;
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

  route = straight(19);
  for (auto& s : route) s.hold_free = s.confirmed = false;
  route[0].hold_free = route[0].confirmed = true;
  route[6].hold_free = route[6].confirmed = true;
  route[7].hold_free = route[7].confirmed = true;
  pick = select(route, 0, route.back().position, cfg);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index == 6,
          "default 1m policy retains post-obstacle fallback below normal spacing");
  route[6].hold_free = route[6].confirmed = false;
  pick = select(route, 0, route.back().position, cfg);
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
  std::cout << "PASS safe relay: normal progress, B/C/W/no-update, passage, confirmation, curves, XYZ\n";
}
