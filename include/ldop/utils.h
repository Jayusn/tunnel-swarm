#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ldop/motion_model.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

namespace ldopcore {

enum class ObjectClass : std::uint8_t {
  Unknown = 0U,
  Human = 1U,
  Vehicle = 2U,
  Uav = 3U,
  Other = 4U,
};

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
  // 当前 detection 的点数只服务于 tracker 的合并/分离保护；
  // 不进入对外 ROS 消息，避免把聚类实现细节固化成公共接口。
  std::size_t point_count{0U};
};

struct TrackHistorySample {
  // 轨迹历史保存模型原生状态；未来 predictor 按 motion_model_type 解释状态布局，
  // 避免 tracker 同时维护公共投影和模型向量两套状态来源。
  ros::Time stamp;
  ObjectClass object_class{ObjectClass::Unknown};
  MotionModelType motion_model_type{MotionModelType::CV3D};
  Eigen::VectorXd model_state;
  Eigen::MatrixXd model_covariance;
  bool matched{false};
};

// 几何点构造在 marker、跟踪和后续调试输出里都很常见，集中到这里避免各模块各写一份临时小函数。
inline geometry_msgs::Point makePoint(const double x, const double y, const double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

// 三维中心距离是几何语义，不属于 tracker 私有状态；放在通用层能减少模块内薄 helper。
inline double distance3D(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

// ROS 点类型和 Eigen 向量的转换属于跨模块边界小操作，统一放在通用层避免各模块保留同义 helper。
inline Eigen::Vector3d pointToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

inline geometry_msgs::Vector3 eigenToVector(const Eigen::Vector3d& vector) {
  geometry_msgs::Vector3 ros_vector;
  ros_vector.x = vector.x();
  ros_vector.y = vector.y();
  ros_vector.z = vector.z();
  return ros_vector;
}

// Vector3 的分量级比较属于通用几何小操作，集中后 tracker 不需要保留同义薄 helper。
inline double maxComponent(const geometry_msgs::Vector3& vector) {
  return std::max({vector.x, vector.y, vector.z});
}

inline geometry_msgs::Vector3 componentMax(const geometry_msgs::Vector3& lhs,
                                           const geometry_msgs::Vector3& rhs) {
  geometry_msgs::Vector3 result;
  result.x = std::max(lhs.x, rhs.x);
  result.y = std::max(lhs.y, rhs.y);
  result.z = std::max(lhs.z, rhs.z);
  return result;
}

// RViz DELETEALL marker 只依赖 header 和可选 namespace，各可视化模块共用同一份构造规则。
inline visualization_msgs::Marker makeDeleteAllMarker(const std_msgs::Header& header,
                                                      const std::string& marker_namespace = "") {
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = marker_namespace;
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  return marker;
}

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
