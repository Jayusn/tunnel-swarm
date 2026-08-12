#include "entry_frame_rules.h"

#include <cassert>
#include <cmath>

int main() {
  using diff_planner::entry_frame::lintelHeightThreshold;
  using diff_planner::entry_frame::lintelSupportAccepted;
  using diff_planner::entry_frame::oneSidedInferenceAllowed;
  using diff_planner::entry_frame::lintelHeightConsistent;
  using diff_planner::entry_frame::modeBShouldReplaceDoor;
  using diff_planner::entry_frame::modeCShouldReplaceDoor;
  using diff_planner::entry_frame::doorSafetyCheckDistance;
  using diff_planner::entry_frame::portalExitRequired;

  // 横梁阈值必须固定参考起飞前地面高度，不能随飞行高度升高。
  assert(std::fabs(lintelHeightThreshold(0.01, 0.65) - 0.66) < 1e-9);

  // 横梁只统计门柱内侧，稀疏的两格横梁可通过，单格和比例不足仍拒绝。
  assert(lintelSupportAccepted(2, 10, 2, 0.15));
  assert(!lintelSupportAccepted(1, 10, 2, 0.15));
  assert(!lintelSupportAccepted(2, 20, 2, 0.15));
  assert(lintelHeightConsistent(1.05, 1.20, 0.15 + 1e-9));
  assert(!lintelHeightConsistent(1.04, 1.20, 0.15));

  // 方案B必须观测到双边门柱；仅方案A允许使用旧的单边推断兼容逻辑。
  assert(oneSidedInferenceAllowed(false));
  assert(!oneSidedInferenceAllowed(true));

  // 方案B在不同门平面间必须保留第一道门，不能被更深处的高分截面替换。
  assert(!modeBShouldReplaceDoor(20.0, 2.0, 10.0, 1.0, 0.10));
  assert(modeBShouldReplaceDoor(5.0, 0.8, 10.0, 1.0, 0.10));

  // 同一门平面内才允许用结构分挑选更完整的观测。
  assert(modeBShouldReplaceDoor(11.0, 1.05, 10.0, 1.0, 0.10));
  assert(!modeBShouldReplaceDoor(9.0, 1.05, 10.0, 1.0, 0.10));

  // 方案C严格选最近截面，远处候选分数再高也不能替换；同一0.1m栅格行内才比较结构分。
  assert(!modeCShouldReplaceDoor(100.0, 1.20, 1.0, 1.00, 0.05));
  assert(modeCShouldReplaceDoor(1.0, 0.80, 100.0, 1.00, 0.05));
  assert(modeCShouldReplaceDoor(11.0, 1.03, 10.0, 1.00, 0.05));

  // 方案B在机体越过门平面时立即交给FUEL，不能因0.9m后的地图尚未建全而刷掉第一道门。
  assert(std::fabs(doorSafetyCheckDistance(true, 0.90, 0.0)) < 1e-9);
  assert(std::fabs(doorSafetyCheckDistance(false, 0.90, 0.25) - 0.90) < 1e-9);

  // 门后通道支撑在方案B中只影响质量分，不再是进入候选列表的硬条件；方案A维持旧行为。
  assert(!portalExitRequired(true));
  assert(portalExitRequired(false));
  return 0;
}
