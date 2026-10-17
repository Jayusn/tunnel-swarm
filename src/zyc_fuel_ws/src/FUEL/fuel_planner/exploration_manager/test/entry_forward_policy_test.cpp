#include <cstdlib>
#include <iostream>
#include "exploration_manager/entry_forward_policy.h"

void check(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}

int main() {
  using namespace fast_planner::entry_forward;
  const Eigen::Vector3d start(0.0, 0.0, 0.45), forward(1.0, 0.0, 0.0);
  Goal goal;
  check(needsAdvance(0.99), "不足一米继续推进");
  check(!needsAdvance(1.0) && !needsAdvance(1.0214667), "实际一米后不追补1.05米");

  const auto inward = [&](std::initializer_list<double> progress) {
    std::vector<Eigen::Vector3d> path;
    for (double p : progress) path.push_back(start + p * forward);
    return pathStartsTowardInside(path, start, forward, 0.10);
  };
  check(inward({-0.038, -0.02, 0.0, 0.65}), "录包中回摆3.8厘米仍允许向门内起步");
  check(inward({-0.10, -0.05, 0.65}), "十厘米起点边界允许向内推进");
  check(!inward({-0.101, 0.65}), "超出起点余量不能豁免入口边界");
  check(!inward({-0.038, -0.06, 0.65}), "不能从回摆位置继续向门外绕路");
  check(!inward({-0.038, -0.01}), "终点仍在门外不能发布");
  check(!inward({-0.038, 0.1, -0.01, 0.65}), "已经进门后不能再出门");
  check(!inward({0.02, -0.01, 0.65}), "门内起点不能使用余量规划到门外");
  check(inward({0.02, 0.01, 0.65}), "门内路径沿用原入口边界");
  const Eigen::Vector3d rotated_origin(3.0, 5.0, 0.5), rotated_forward(0.0, 1.0, 0.0);
  check(pathStartsTowardInside({Eigen::Vector3d(3.0, 4.962, 0.5),
                               Eigen::Vector3d(3.2, 5.65, 0.5)},
                              rotated_origin, rotated_forward, 0.10),
        "任意入口位置与航向均支持斜向前起步");

  const auto open = [](Goal& goal) {
    double& clearance = goal.clearance; double& continuation = goal.continuation;
    clearance = 0.6; continuation = 0.4; return true;
  };
  check(selectGoal(start, forward, 1.05, open, goal), "开放通道有目标");
  check(std::fabs(goal.advance - 1.05) < 1e-6 && goal.side == 0.0,
        "开放通道保持直行并优先长步");

  // 正前方终点仍可到达，但前方被障碍封住；右侧有净空和下一段连接。
  const auto bypass = [](Goal& goal) {
    const auto& p = goal.position;
    double& clearance = goal.clearance; double& continuation = goal.continuation;
    if (p.y() < -0.1 || p.y() > 0.45) return false; // 两侧墙壁
    clearance = p.y() >= 0.19 ? 0.4 : 0.15;
    continuation = p.y() >= 0.19 ? 0.4 : 0.0;
    return true;
  };
  check(selectGoal(start, forward, 1.05, bypass, goal), "允许斜前方绕障入口");
  check(goal.advance > 0.5 && goal.side > 0.0 && goal.position.z() == start.z(),
        "净空与后续连接评分选择斜前方且保持高度");

  const auto connected = [](Goal& goal) {
    const auto& p = goal.position;
    double& clearance = goal.clearance; double& continuation = goal.continuation;
    clearance = 0.2;
    continuation = p.y() >= 0.19 ? 0.4 : 0.0;
    return std::fabs(p.y()) < 0.3;
  };
  check(selectGoal(start, forward, 1.05, connected, goal) && goal.side > 0.0,
        "相同净空时优先有下一段连接的斜前方");
  const auto clear = [](Goal& goal) {
    const auto& p = goal.position;
    double& clearance = goal.clearance; double& continuation = goal.continuation;
    clearance = p.y() >= 0.19 ? 0.4 : 0.15;
    continuation = 0.4;
    return std::fabs(p.y()) < 0.3;
  };
  check(selectGoal(start, forward, 1.05, clear, goal) && goal.side > 0.0,
        "相同后续连接时优先净空更好的斜前方");

  // 复现“右侧短步净空更大，但只有左侧能接到障碍后方”的评分冲突。
  const auto passage = [](Goal& candidate) {
    if (std::fabs(candidate.side) > 0.21) return false;
    candidate.clearance = candidate.side < -0.1 ? 0.60 : 0.20;
    candidate.continuation = 0.40;
    candidate.passage_priority = candidate.side > 0.1 ? 2 : 0;
    return true;
  };
  check(selectGoal(start, forward, 0.62, passage, goal) && goal.side > 0.0,
        "跨障通路优先于错误侧的短步大净空");
  const auto widths = [](Goal& candidate) {
    if (std::fabs(candidate.side) < 0.1 || std::fabs(candidate.side) > 0.21) return false;
    candidate.passage_priority = candidate.side > 0.0 ? 2 : 1;
    candidate.clearance = candidate.side > 0.0 ? 0.15 : 0.60;
    candidate.continuation = 0.40;
    return true;
  };
  check(selectGoal(start, forward, 0.62, widths, goal) && goal.side > 0.0,
        "两侧均可跨障时先选间距更宽侧，不能由窄侧大净空覆盖");
  const auto equal_widths = [](Goal& candidate) {
    if (std::fabs(candidate.side) < 0.1 || std::fabs(candidate.side) > 0.21) return false;
    candidate.passage_priority = 2;
    candidate.clearance = candidate.side < 0.0 ? 0.60 : 0.20;
    candidate.continuation = candidate.side < 0.0 ? 0.0 : 0.40;
    return true;
  };
  check(selectGoal(start, forward, 0.62, equal_widths, goal) && goal.side < 0.0,
        "相同间距的可行跨障通路再比较净空");
  const auto alternate = [](Goal& candidate) {
    candidate.clearance = 0.20;
    candidate.continuation = 0.40;
    // 原选左侧已不可用，右侧仍能跨过障碍；不能被旧侧偏好锁住。
    candidate.passage_priority = candidate.side < -0.1 ? 1 : 0;
    return true;
  };
  check(selectGoal(start, forward, 0.62, alternate, goal) && goal.side < 0.0,
        "原侧跨障不可用时允许另一侧可行通路");

  const auto narrow = [](Goal& goal) {
    const auto& p = goal.position;
    double& clearance = goal.clearance; double& continuation = goal.continuation;
    clearance = 0.10; continuation = 0.0;
    return std::fabs(p.y()) < 0.1 && p.x() < 0.7;
  };
  check(selectGoal(start, forward, 1.05, narrow, goal), "短可通行段仍可前进");
  check(goal.advance >= 0.2 && goal.advance < 0.7 && goal.side == 0.0,
        "不额外要求大净空或下一段必须通行");
  const auto blocked = [](Goal&) { return false; };
  check(!selectGoal(start, forward, 1.05, blocked, goal), "没有可行路径时不造目标");
  check(selectGoal(start, Eigen::Vector3d(0.0, 1.0, 0.0), 0.15, open, goal),
        "小于步长的剩余距离支持旋转入口方向");
  check(std::fabs(goal.position.y() - 0.15) < 1e-6 && goal.position.x() == 0.0,
        "推进按入口法向而非固定世界X");
  return 0;
}
