#include "entry_frame_rules.h"

#include <cassert>
#include <cmath>

int main() {
  using diff_planner::entry_frame::lintelHeightThreshold;
  using diff_planner::entry_frame::lintelSupportAccepted;
  using diff_planner::entry_frame::oneSidedInferenceAllowed;
  using diff_planner::entry_frame::lintelHeightConsistent;

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
  return 0;
}
