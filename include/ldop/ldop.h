#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <ldop/DynamicObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

namespace ldop {

class LDOP {
 public:
  LDOP(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~LDOP();

  int spinnerThreadCount() const;

 private:
  struct InputFrame {
    sensor_msgs::PointCloud2ConstPtr cloud_msg;
    nav_msgs::Odometry odom_msg;
    bool has_odom{false};
    ros::Time received_time;
    std::uint64_t sequence{0};
  };

  struct ProcessedFrame {
    sensor_msgs::PointCloud2 static_cloud_msg;
    sensor_msgs::PointCloud2 dynamic_cloud_msg;
    ldop::DynamicObjectArray dynamic_objects_msg;
    visualization_msgs::MarkerArray marker_array_msg;
    nav_msgs::Odometry odom_msg;
    bool has_odom{false};
    ros::Time processed_time;
    std::uint64_t sequence{0};
    std::size_t skipped_input_frames{0};
  };

  void loadParameters();
  void setupRosInterfaces();
  void startWorkers();
  void stopWorkers();

  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  void processingLoop();
  void publishingLoop();

  ProcessedFrame processFrame(const InputFrame& frame, std::size_t skipped_frames);
  void publishFrame(const ProcessedFrame& frame);
  ldop::DynamicObjectArray buildDynamicObjectArray(const ProcessedFrame& frame) const;
  visualization_msgs::MarkerArray buildMarkerArray(const ProcessedFrame& frame) const;
  sensor_msgs::PointCloud2 buildEmptyCloud(const std_msgs::Header& header) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher static_cloud_pub_;
  ros::Publisher dynamic_cloud_pub_;
  ros::Publisher dynamic_objects_pub_;
  ros::Publisher marker_pub_;

  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string static_cloud_topic_;
  std::string dynamic_cloud_topic_;
  std::string dynamic_objects_topic_;
  std::string marker_topic_;
  std::string world_frame_;
  bool verbose_;
  int input_queue_size_;
  int output_queue_size_;
  int spinner_thread_count_;

  std::atomic<bool> running_;
  std::atomic<std::uint64_t> received_cloud_count_;
  std::atomic<std::uint64_t> processed_cloud_count_;
  std::atomic<std::uint64_t> dropped_cloud_count_;

  mutable std::mutex odom_mutex_;
  std::mutex input_mutex_;
  std::mutex output_mutex_;
  std::condition_variable input_cv_;
  std::condition_variable output_cv_;

  nav_msgs::Odometry latest_odom_;
  bool has_latest_odom_;
  std::deque<InputFrame> input_queue_;
  std::deque<ProcessedFrame> output_queue_;

  std::thread processing_thread_;
  std::thread publishing_thread_;
};

}  // namespace ldop
