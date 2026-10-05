#include <exploration_manager/task_search_manager.h>
#include <plan_env/sdf_map.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <utility>

namespace {
visualization_msgs::MarkerArray exitHistoryMarkers(const geometry_msgs::PoseStamped& exit) {
  visualization_msgs::Marker frame;
  frame.header = exit.header;
  frame.ns = "confirmed_exit_history";
  frame.id = 0;
  frame.type = visualization_msgs::Marker::LINE_STRIP;
  frame.action = visualization_msgs::Marker::ADD;
  frame.pose = exit.pose;
  frame.scale.x = 0.08;
  frame.color.g = 0.55;
  frame.color.b = frame.color.a = 1.0;
  // 与实时出口矩形使用同样的范围，坐标在出口位姿的局部截面中。
  const double bottom = std::max(0.05, exit.pose.position.z - 0.55) - exit.pose.position.z;
  for (const auto& corner : {std::pair<double, double>{-0.55, bottom},
                            {-0.55, 0.75}, {0.55, 0.75}, {0.55, bottom}}) {
    geometry_msgs::Point point;
    point.y = corner.first;
    point.z = corner.second;
    frame.points.push_back(point);
  }
  frame.points.push_back(frame.points.front());
  visualization_msgs::Marker text = frame;
  text.id = 1;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.points.clear();
  text.pose.position.z += 0.95;
  text.pose.orientation = geometry_msgs::Quaternion();
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.20;
  text.text = "CONFIRMED EXIT HISTORY / 5 MAP OBSERVATIONS";
  visualization_msgs::MarkerArray markers;
  markers.markers = {frame, text};
  return markers;
}
}  // namespace

// 只实例化累计地图和检测器；不实例化探索FSM、轨迹服务器或飞控客户端。
int main(int argc, char** argv) {
  ros::init(argc, argv, "exit_detection_node");
  ros::NodeHandle nh("~");
  auto map = std::make_shared<fast_planner::SDFMap>();
  map->initMap(nh);
  fast_planner::TaskSearchManager detector;
  detector.initialize(nh);
  detector.setMap(map);

  auto confirmed_pub = nh.advertise<std_msgs::Bool>("confirmed", 1, true);
  auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("detected_exit", 1);
  auto history_pub = nh.advertise<visualization_msgs::MarkerArray>("exit_history_markers", 1, true);
  // 重启时清除上一次测试的历史标记；完整数组锁存，晚打开RViz也能收到全部标记。
  visualization_msgs::MarkerArray clear_history;
  for (int id : {0, 1}) {
    visualization_msgs::Marker marker;
    nh.param("mission/task_search/world_frame", marker.header.frame_id, std::string("world"));
    marker.ns = "confirmed_exit_history";
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE;
    clear_history.markers.push_back(marker);
  }
  history_pub.publish(clear_history);
  bool history_saved = false;
  std_msgs::Bool confirmed;
  confirmed.data = false;
  confirmed_pub.publish(confirmed);
  auto odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "odom", 50, [&](const nav_msgs::Odometry::ConstPtr& odom) {
        const auto& position = odom->pose.pose.position;
        geometry_msgs::PoseStamped exit;
        confirmed.data = detector.updateExitDetectionOnly(
            Eigen::Vector3d(position.x, position.y, position.z),
            tf::getYaw(odom->pose.pose.orientation), exit);
        confirmed_pub.publish(confirmed);
        if (confirmed.data) pose_pub.publish(exit);
        // confirmed已包含原有净宽、稳定性、机体通过和1米近场检查。
        // 第五次有效新地图证据才保存一次；实时候选撤销不会删除历史标记。
        if (!history_saved && confirmed.data && detector.exitEvidenceCount() >= 5) {
          history_pub.publish(exitHistoryMarkers(exit));
          history_saved = true;
          ROS_INFO("[exit_detection_test] saved exit history after %d map observations "
                   "at (%.2f,%.2f,%.2f); retained until restart, detection only.",
                   detector.exitEvidenceCount(), exit.pose.position.x,
                   exit.pose.position.y, exit.pose.position.z);
        }
      });
  ROS_INFO("[exit_detection_test] waiting for world-frame cloud and odometry; "
           "face forward along the starting corridor; map-confirmed turns update direction.");
  ros::spin();
  return 0;
}
