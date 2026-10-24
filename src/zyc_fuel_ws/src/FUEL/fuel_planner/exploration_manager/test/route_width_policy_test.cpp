#include <exploration_manager/route_width_policy.h>
#include <cstdlib>
#include <iostream>

using Eigen::Vector3d;
using namespace fast_planner;

static void require(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}

int main() {
  const Vector3d forward(1, 0, 0);
  const Vector3d west(-1, 0, 0);
  require(!route_width::sameAlternative({4.43,-3.13,.52}, -1, west,
                                      {4.45,-3.45,.52}, 0, west),
          "recorded right-to-center switch cannot count as two agreeing observations");
  require(route_width::sameAlternative({4.43,-3.13,.52}, -1, west,
                                     {4.40,-3.10,.52}, -1, west),
          "same-side map updates should confirm a passage");
  require(!route_width::sameAlternative({4.43,-3.13,.52}, -1, west,
                                      {4.43,-3.13,.52}, -1, forward),
          "a changed corridor axis cannot reuse old side confirmation");
  route_width::SplitRoute lane;
  lane.side = -1; lane.entry = .5; lane.exit = 1.1; lane.offset = .3;
  require(route_width::followsSplit({{0,0,.5},{.3,-.5,.5},{1.2,-.5,.5}}, lane),
          "right route must retain the lane through the obstacle rear");
  require(!route_width::followsSplit({{0,0,.5},{.3,-.5,.5},{1.2,0,.5}}, lane),
          "centering before the obstacle rear must not be accepted");
  require(!route_width::followsSplit({{0,0,.5},{.3,-.5,.5},{.9,-.5,.5}}, lane),
          "truncating before the rear cannot count as a complete bypass");
  // A central obstacle separates a narrow straight lane and a wider right
  // lane. Both routes are feasible; enumeration order must not pick straight.
  const auto free = [](const Vector3d& p) {
    if (p.x() < 0.25) return p.y() > -1.3 && p.y() < 0.4;
    return (p.y() > -0.2 && p.y() < 0.4) || (p.y() > -1.3 && p.y() < -0.4);
  };
  const std::vector<Vector3d> straight{{0, 0, .6}, {1, 0, .6}};
  const std::vector<Vector3d> right{{0, 0, .6}, {.2, -.8, .6}, {1, -.8, .6}};
  const auto a = route_width::measure(straight, forward, 1.0, .9, .05, free);
  const auto b = route_width::measure(right, forward, 1.0, .9, .05, free);
  require(a.complete && b.complete, "both known routes must be measured");
  require(route_width::prefer(b, a), "wider right lane must beat feasible straight");
  require(!route_width::prefer(a, b), "enumeration order must not restore straight priority");
  require(route_width::materiallyWider(b, a, .10), "substantial width gain should permit replan");
  const auto short_step = route_width::measure(
      std::vector<Vector3d>{{0, 0, .6}, {.2, -.8, .6}}, forward, 1.0, .9, .05, free);
  require(!short_step.complete && !route_width::prefer(short_step, a),
          "short sideways endpoint cannot masquerade as a complete route");
  const auto narrow_tail = [&](const Vector3d& p) {
    return free(p) && (p.x() < .85 || (p.y() > -.95 && p.y() < -.65));
  };
  const auto c = route_width::measure(right, forward, 1.0, .9, .05, narrow_tail);
  require(c.complete && c.minimum < a.minimum,
          "late bottleneck must defeat an initially wider lane");
  const auto unknown_middle = [&](const Vector3d& p) {
    return free(p) && !(p.x() >= .5 && p.x() <= .7 && p.y() < -.4);
  };
  require(!route_width::measure(right, forward, 1.0, .9, .05, unknown_middle).complete,
          "unknown middle cannot be skipped");
  const auto mirrored = [&](const Vector3d& p) {
    return free(Vector3d(p.x(), -p.y(), p.z()));
  };
  std::vector<Vector3d> left = right;
  for (auto& p : left) p.y() *= -1;
  const auto mirrored_score = route_width::measure(left, forward, 1., .9, .05, mirrored);
  require(std::fabs(mirrored_score.minimum - b.minimum) < 1e-6,
          "left and right must obey the same policy");
  auto fluctuation = a;
  fluctuation.minimum += .04;
  require(!route_width::materiallyWider(fluctuation, a, .10),
          "small map fluctuations must not force switching");
  require(!route_width::materiallyWider(b, route_width::Score{}, .10),
          "missing baseline evidence cannot trigger quality-only switching");
  std::cout << "route width policy: all checks passed\n";
}
