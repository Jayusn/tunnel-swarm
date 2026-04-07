#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <nav_msgs/Odometry.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

namespace ldopcore {

struct OctomapPublishData {
  octomap_msgs::Octomap full_msg;
  std::uint64_t update_count{0};
  std::size_t node_count{0};
  bool has_full_msg{false};
};

struct OctomapFrameResult {
  sensor_msgs::PointCloud2 static_cloud_msg;
  OctomapPublishData publish_data;
};

class OctomapMapper {
 public:
  // 独立负责提取世界系静态点云、使用世界系里程计更新 OctoMap 并发布标准地图消息。
  OctomapMapper(ros::NodeHandle& nh,
                ros::NodeHandle& pnh,
                const std::string& world_frame,
                bool verbose);

  OctomapFrameResult processStaticCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                        const nav_msgs::Odometry& odom_msg);
  void publish(const OctomapPublishData& publish_data);

 private:
  void loadParameters();
  void configureOctomap();
  void setupPublishers();
  sensor_msgs::PointCloud2 buildStaticCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                            octomap::Pointcloud* octomap_cloud) const;
  octomap::point3d buildSensorOrigin(const nav_msgs::Odometry& odom_msg) const;
  void updateOctomap(const octomap::Pointcloud& static_cloud,
                     const octomap::point3d& sensor_origin,
                     const std_msgs::Header& header,
                     OctomapPublishData* publish_data);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher octomap_full_pub_;

  std::string world_frame_;
  std::string octomap_full_topic_;
  bool verbose_;
  double octomap_resolution_;
  double octomap_max_range_;
  double octomap_hit_probability_;
  double octomap_miss_probability_;
  double octomap_clamping_min_probability_;
  double octomap_clamping_max_probability_;
  double octomap_occupancy_threshold_;

  std::atomic<std::uint64_t> octomap_update_count_;
  std::unique_ptr<octomap::OcTree> octree_;
};

}  // namespace ldopcore
