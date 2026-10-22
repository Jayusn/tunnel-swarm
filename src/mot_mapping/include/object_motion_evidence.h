#pragma once
#include <cstdint>
#include <cmath>

namespace mot_mapping {
// Two adjacent transitions in three valid observations. No inferred free space:
// the caller supplies measured bidirectional depth support, including unknowns
// in its denominator. A missing/invalid observation cannot bridge confirmation.
struct ObjectMotionEvidence {
  uint64_t last_stamp = 0;
  bool last_supported = false;
  void clear() { last_stamp = 0; last_supported = false; }
  bool update(uint64_t stamp, uint64_t previous_stamp, bool valid,
              bool transition_valid, double moving, double stationary, double depth,
              double minimum_match, double minimum_advantage, double minimum_depth) {
    if (!valid || stamp == 0 || stamp <= last_stamp) { clear(); return false; }
    const bool adjacent = last_stamp != 0 && previous_stamp == last_stamp;
    const bool supported = adjacent && transition_valid &&
        std::isfinite(moving) && std::isfinite(stationary) && std::isfinite(depth) &&
        moving >= minimum_match && moving - stationary >= minimum_advantage &&
        depth >= minimum_depth;
    const bool confirmed = supported && last_supported;
    last_stamp = stamp;
    last_supported = supported;
    return confirmed;
  }
};
}  // namespace mot_mapping
