#pragma once

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace exploration_constraint {

struct Gate {
  std::uint32_t sequence{0};
  Eigen::Vector2d center{0.0, 0.0};
  Eigen::Vector2d normal{1.0, 0.0};
  double left_extent{0.0};
  double right_extent{0.0};
  double thickness{0.0};
};

struct State {
  bool enabled{false};
  double exploration_initial_yaw{0.0};
  double entry_inside_yaw{0.0};

  bool entry_plane_active{false};
  Eigen::Vector2d entry_origin{0.0, 0.0};
  Eigen::Vector2d entry_direction{1.0, 0.0};
  double entry_back_margin{0.0};

  Eigen::Vector2d segment_origin{0.0, 0.0};
  Eigen::Vector2d segment_direction{1.0, 0.0};
  double segment_high_water{0.0};
  double segment_regression_margin{0.25};

  bool transition_active{false};
  Eigen::Vector2d transition_anchor{0.0, 0.0};
  Eigen::Vector2d transition_incoming_direction{1.0, 0.0};
  Eigen::Vector2d transition_outgoing_direction{1.0, 0.0};
  double transition_old_high_water{0.0};
  double transition_regression_margin{0.45};
  double transition_radius{0.80};
  double transition_corridor_half_width{1.25};

  std::vector<Gate> completed_gates;
};

inline bool normalizeDirection(Eigen::Vector2d& direction) {
  if (!direction.allFinite() || direction.norm() < 1e-6) return false;
  direction.normalize();
  return true;
}

inline bool validGate(const Gate& gate) {
  Eigen::Vector2d normal = gate.normal;
  return gate.center.allFinite() && normalizeDirection(normal) &&
         std::isfinite(gate.left_extent) && gate.left_extent >= 0.0 &&
         std::isfinite(gate.right_extent) && gate.right_extent >= 0.0 &&
         std::isfinite(gate.thickness) && gate.thickness > 0.0;
}

inline bool validState(const State& state) {
  if (!state.enabled) return true;
  Eigen::Vector2d segment = state.segment_direction;
  if (!state.segment_origin.allFinite() || !normalizeDirection(segment) ||
      !std::isfinite(state.segment_high_water) ||
      !std::isfinite(state.segment_regression_margin) ||
      state.segment_regression_margin < 0.0)
    return false;
  if (state.entry_plane_active) {
    Eigen::Vector2d entry = state.entry_direction;
    if (!state.entry_origin.allFinite() || !normalizeDirection(entry) ||
        !std::isfinite(state.entry_back_margin) || state.entry_back_margin < 0.0)
      return false;
  }
  if (state.transition_active) {
    Eigen::Vector2d incoming = state.transition_incoming_direction;
    Eigen::Vector2d outgoing = state.transition_outgoing_direction;
    if (!state.transition_anchor.allFinite() || !normalizeDirection(incoming) ||
        !normalizeDirection(outgoing) ||
        !std::isfinite(state.transition_old_high_water) ||
        !std::isfinite(state.transition_regression_margin) ||
        state.transition_regression_margin < 0.0 ||
        !std::isfinite(state.transition_radius) || state.transition_radius <= 0.0 ||
        !std::isfinite(state.transition_corridor_half_width) ||
        state.transition_corridor_half_width <= 0.0)
      return false;
  }
  for (const auto& gate : state.completed_gates)
    if (!validGate(gate)) return false;
  return true;
}

inline bool pointInsideGate(const Gate& gate, const Eigen::Vector2d& point) {
  if (!validGate(gate) || !point.allFinite()) return false;
  const Eigen::Vector2d normal = gate.normal.normalized();
  const Eigen::Vector2d left(-normal.y(), normal.x());
  const Eigen::Vector2d relative = point - gate.center;
  const double lateral = relative.dot(left);
  return std::fabs(relative.dot(normal)) <= 0.5 * gate.thickness + 1e-6 &&
         lateral <= gate.left_extent + 1e-6 &&
         lateral >= -gate.right_extent - 1e-6;
}

inline bool segmentCrossesGate(const Gate& gate, const Eigen::Vector2d& start,
                               const Eigen::Vector2d& end) {
  if (!validGate(gate) || !start.allFinite() || !end.allFinite()) return true;
  if (pointInsideGate(gate, start) || pointInsideGate(gate, end)) return true;
  const Eigen::Vector2d normal = gate.normal.normalized();
  const Eigen::Vector2d left(-normal.y(), normal.x());
  const double start_side = (start - gate.center).dot(normal);
  const double end_side = (end - gate.center).dot(normal);
  const double denominator = start_side - end_side;
  if (std::fabs(denominator) < 1e-9 || start_side * end_side > 0.0) return false;
  const double ratio = std::max(0.0, std::min(1.0, start_side / denominator));
  const Eigen::Vector2d intersection = start + ratio * (end - start);
  const double lateral = (intersection - gate.center).dot(left);
  return lateral <= gate.left_extent + 1e-6 &&
         lateral >= -gate.right_extent - 1e-6;
}

inline bool insideCorridor(const Eigen::Vector2d& point,
                           const Eigen::Vector2d& origin,
                           const Eigen::Vector2d& direction,
                           double minimum_progress, double half_width) {
  if (direction.norm() < 1e-6) return false;
  const Eigen::Vector2d forward = direction.normalized();
  const Eigen::Vector2d left(-forward.y(), forward.x());
  const Eigen::Vector2d relative = point - origin;
  return relative.dot(forward) + 1e-6 >= minimum_progress &&
         std::fabs(relative.dot(left)) <= half_width + 1e-6;
}

inline bool pointAllowed(const State& state, const Eigen::Vector2d& point) {
  if (!state.enabled) return true;
  if (!validState(state) || !point.allFinite()) return false;
  // The confirmed turn anchor is deliberately inside the just-completed gate;
  // allow the short transition bubble to leave that anchor before applying the
  // historical-gate no-return rule.
  if (state.transition_active &&
      (point - state.transition_anchor).norm() <= state.transition_radius + 1e-6)
    return true;
  for (const auto& gate : state.completed_gates)
    if (pointInsideGate(gate, point)) return false;
  if (state.entry_plane_active &&
      (point - state.entry_origin).dot(state.entry_direction.normalized()) <
          -state.entry_back_margin - 1e-6)
    return false;

  if (!state.transition_active) {
    const double progress =
        (point - state.segment_origin).dot(state.segment_direction.normalized());
    return progress + 1e-6 >=
           state.segment_high_water - state.segment_regression_margin;
  }

  if ((point - state.transition_anchor).norm() <= state.transition_radius + 1e-6)
    return true;
  const double incoming_floor =
      state.transition_old_high_water - state.transition_regression_margin;
  if (insideCorridor(point, state.segment_origin,
                     state.transition_incoming_direction, incoming_floor,
                     state.transition_corridor_half_width))
    return true;
  return insideCorridor(point, state.transition_anchor,
                        state.transition_outgoing_direction,
                        -state.segment_regression_margin,
                        state.transition_corridor_half_width);
}

inline bool segmentAllowed(const State& state, const Eigen::Vector2d& start,
                           const Eigen::Vector2d& end) {
  if (!pointAllowed(state, start) || !pointAllowed(state, end)) return false;
  if (state.transition_active) return true;
  for (const auto& gate : state.completed_gates)
    if (segmentCrossesGate(gate, start, end)) return false;
  return true;
}

}  // namespace exploration_constraint
