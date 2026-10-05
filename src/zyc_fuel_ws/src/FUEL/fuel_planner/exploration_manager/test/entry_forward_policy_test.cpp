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

  const auto open = [](const Eigen::Vector3d&, double& clearance, double& continuation) {
    clearance = 0.6; continuation = 0.4; return true;
  };
  check(selectGoal(start, forward, 1.05, open, goal), "开放通道有目标");
  check(std::fabs(goal.advance - 1.05) < 1e-6 && goal.side == 0.0,
        "开放通道保持直行并优先长步");

  // 正前方终点仍可到达，但前方被障碍封住；右侧有净空和下一段连接。
  const auto bypass = [](const Eigen::Vector3d& p, double& clearance, double& continuation) {
    if (p.y() < -0.1 || p.y() > 0.45) return false; // 两侧墙壁
    clearance = p.y() >= 0.19 ? 0.4 : 0.15;
    continuation = p.y() >= 0.19 ? 0.4 : 0.0;
    return true;
  };
  check(selectGoal(start, forward, 1.05, bypass, goal), "允许斜前方绕障入口");
  check(goal.advance > 0.5 && goal.side > 0.0 && goal.position.z() == start.z(),
        "净空与后续连接评分选择斜前方且保持高度");

  const auto connected = [](const Eigen::Vector3d& p, double& clearance, double& continuation) {
    clearance = 0.2;
    continuation = p.y() >= 0.19 ? 0.4 : 0.0;
    return std::fabs(p.y()) < 0.3;
  };
  check(selectGoal(start, forward, 1.05, connected, goal) && goal.side > 0.0,
        "相同净空时优先有下一段连接的斜前方");
  const auto clear = [](const Eigen::Vector3d& p, double& clearance, double& continuation) {
    clearance = p.y() >= 0.19 ? 0.4 : 0.15;
    continuation = 0.4;
    return std::fabs(p.y()) < 0.3;
  };
  check(selectGoal(start, forward, 1.05, clear, goal) && goal.side > 0.0,
        "相同后续连接时优先净空更好的斜前方");

  const auto narrow = [](const Eigen::Vector3d& p, double& clearance, double& continuation) {
    clearance = 0.10; continuation = 0.0;
    return std::fabs(p.y()) < 0.1 && p.x() < 0.7;
  };
  check(selectGoal(start, forward, 1.05, narrow, goal), "短可通行段仍可前进");
  check(goal.advance >= 0.2 && goal.advance < 0.7 && goal.side == 0.0,
        "不额外要求大净空或下一段必须通行");
  const auto blocked = [](const Eigen::Vector3d&, double&, double&) { return false; };
  check(!selectGoal(start, forward, 1.05, blocked, goal), "没有可行路径时不造目标");
  check(selectGoal(start, Eigen::Vector3d(0.0, 1.0, 0.0), 0.15, open, goal),
        "小于步长的剩余距离支持旋转入口方向");
  check(std::fabs(goal.position.y() - 0.15) < 1e-6 && goal.position.x() == 0.0,
        "推进按入口法向而非固定世界X");
  return 0;
}
