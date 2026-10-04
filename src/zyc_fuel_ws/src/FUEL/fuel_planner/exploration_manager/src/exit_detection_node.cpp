#include <exploration_manager/task_search_manager.h>
#include <plan_env/sdf_map.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

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
      });
  ROS_INFO("[exit_detection_test] waiting for world-frame cloud and odometry; "
           "face forward along the starting corridor; map-confirmed turns update direction.");
  ros::spin();
  return 0;
}
