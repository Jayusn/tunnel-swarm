#pragma once

#include <algorithm>

namespace diff_planner {
namespace entry_frame {

inline double lintelHeightThreshold(double ground_reference_z, double height_offset) {
  return ground_reference_z + height_offset;
}

inline bool lintelSupportAccepted(int occupied_bins, int total_bins,
                                  int minimum_bins, double minimum_ratio) {
  const double ratio = static_cast<double>(occupied_bins) / std::max(1, total_bins);
  return occupied_bins >= minimum_bins && ratio >= minimum_ratio;
}

inline bool oneSidedInferenceAllowed(bool soft_frame_mode) {
  return !soft_frame_mode;
}

inline bool lintelHeightConsistent(double minimum_z, double maximum_z,
                                   double maximum_spread) {
  return maximum_z >= minimum_z && maximum_z - minimum_z <= maximum_spread;
}

}  // namespace entry_frame
}  // namespace diff_planner
