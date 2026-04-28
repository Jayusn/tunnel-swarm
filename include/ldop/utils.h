#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdint>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>

namespace ldopcore {

struct BoundingBox3D {
  geometry_msgs::Point center;
  geometry_msgs::Vector3 size;
  double yaw{0.0};
};

struct DynamicObjectDetection {
  // detection_id 仅在当前帧内有效，后续跟踪阶段会再分配稳定 track id。
  std::uint32_t detection_id{0U};
  BoundingBox3D bbox;
  ros::Time stamp;
};

struct TrackHistorySample {
  // 轨迹历史只保留最小跟踪快照，避免在公共类型里提前固化更多状态。
  ros::Time stamp;
  geometry_msgs::Point position;
  geometry_msgs::Vector3 velocity;
  bool matched{false};
};

// 统一把 steady_clock 的时间差转换成毫秒，避免各模块重复写样板代码。
inline double elapsedMs(const std::chrono::steady_clock::time_point& start,
                        const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// 统一毫秒日志输出精度，保证不同模块的 timing 日志格式一致。
inline std::string formatFloatMs(const double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

}  // namespace ldopcore
