#pragma once
#include <cstdint>
#include <cmath>

namespace mot_mapping {
// Two measured adjacent transitions. An optional short window tolerates one
// unsupported observation between them; no motion is inferred across a gap.
// Background/shape rejection still clears the evidence.
struct ObjectMotionEvidence {
  uint64_t last_stamp = 0;
  bool last_supported = false;
  uint64_t supported_stamp = 0;
  unsigned unsupported_observations = 0;
  void clear() {
    last_stamp = supported_stamp = 0;
    last_supported = false;
    unsupported_observations = 0;
  }
  bool update(uint64_t stamp, uint64_t previous_stamp, bool valid,
              bool transition_valid, double moving, double stationary, double depth,
              double minimum_match, double minimum_advantage, double minimum_depth,
              uint64_t evidence_window_ns = 0) {
    if (!valid || stamp == 0 || stamp <= last_stamp) { clear(); return false; }
    const bool adjacent = last_stamp != 0 && previous_stamp == last_stamp &&
        (evidence_window_ns == 0 || stamp - last_stamp <= evidence_window_ns);
    const bool supported = adjacent && transition_valid &&
        std::isfinite(moving) && std::isfinite(stationary) && std::isfinite(depth) &&
        moving >= minimum_match && moving - stationary >= minimum_advantage &&
        depth >= minimum_depth;
    const bool retained = evidence_window_ns > 0 && supported_stamp != 0 &&
        stamp > supported_stamp && stamp - supported_stamp <= evidence_window_ns &&
        unsupported_observations <= 1;
    const bool confirmed = supported && (evidence_window_ns == 0 ? last_supported : retained);
    if (supported) {
      supported_stamp = stamp;
      unsupported_observations = 0;
    } else if (++unsupported_observations > 1) {
      supported_stamp = 0;
    }
    last_stamp = stamp;
    last_supported = supported;
    return confirmed;
  }
};
}  // namespace mot_mapping
