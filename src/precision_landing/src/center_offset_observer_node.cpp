#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32.h>
#include <XmlRpcValue.h>

#include "precision_landing/aruco_tracker.hpp"

// Observation only: no flight state, trigger, setpoint or mode-service interface.
class CenterOffsetObserver {
 public:
  CenterOffsetObserver() : nh_("~"), transport_(nh_) {
    precision_landing::ArucoTrackerConfig config;
    nh_.param("marker/size_m", config.marker_size_m, 0.50);
    nh_.param("marker/stable_frames", config.stable_frames, 5);
    nh_.param("marker/max_reprojection_error_px", config.max_reprojection_error_px, 4.0);
    nh_.param("marker/min_distance_m", config.min_distance_m, 0.15);
    nh_.param("marker/max_distance_m", config.max_distance_m, 6.0);
    nh_.param("marker/max_position_jump_m", config.max_position_jump_m, 0.60);
    nh_.param("marker/max_tilt_deg", config.max_tilt_deg, 60.0);
    nh_.param("marker/requested_id", requested_id_, -1);
    nh_.param<std::string>("body_frame", body_frame_, "UAV0/body");
    tracker_.reset(new precision_landing::ArucoTracker(config));
    XmlRpc::XmlRpcValue matrix;
    std::vector<double> translation;
    if (!nh_.getParam("camera_to_body/rotation", matrix) || matrix.size() != 3 ||
        !nh_.getParam("camera_to_body/translation_m", translation) || translation.size() != 3)
      throw std::runtime_error("Missing camera-to-body calibration");
    for (int row = 0; row < 3; ++row) {
      if (matrix[row].size() != 3) throw std::runtime_error("Invalid rotation matrix");
      translation_[row] = translation[row];
      for (int col = 0; col < 3; ++col) {
        auto& value = matrix[row][col];
        rotation_(row, col) = value.getType() == XmlRpc::XmlRpcValue::TypeInt
            ? static_cast<int>(value) : static_cast<double>(value);
      }
    }
    if (!rotation_.allFinite() || !translation_.allFinite())
      throw std::runtime_error("Nonfinite calibration");
    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("target_body", 1);
    id_pub_ = nh_.advertise<std_msgs::Int32>("locked_id", 1, true);
    debug_pub_ = transport_.advertise("debug_image", 1);
    info_sub_ = nh_.subscribe("camera_info", 1, &CenterOffsetObserver::cameraInfo, this);
    image_sub_ = transport_.subscribe("image", 1, &CenterOffsetObserver::image, this);
    ROS_INFO("CENTER OBSERVER ONLY: X forward, Y left, Z up; physical centering must be measured by operator");
  }

 private:
  void cameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_.release();
    if (msg->width == 0 || msg->height == 0 || !std::isfinite(msg->K[0]) ||
        !std::isfinite(msg->K[4]) || msg->K[0] <= 0 || msg->K[4] <= 0) return;
    for (double v : msg->K) if (!std::isfinite(v)) return;
    for (double v : msg->D) if (!std::isfinite(v)) return;
    camera_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) camera_.at<double>(i / 3, i % 3) = msg->K[i];
    distortion_ = cv::Mat(msg->D, true).reshape(1, 1);
    width_ = msg->width;
    height_ = msg->height;
  }

  void image(const sensor_msgs::ImageConstPtr& msg) {
    const double age = (ros::Time::now() - msg->header.stamp).toSec();
    if (camera_.empty() || msg->width != width_ || msg->height != height_ ||
        msg->header.stamp.isZero() || age < -0.05 || age > 0.30) {
      ROS_WARN_THROTTLE(2.0, "Waiting for fresh image and matching calibrated CameraInfo");
      return;
    }
    try {
      const auto image = cv_bridge::toCvShare(msg, "bgr8");
      const auto observation = tracker_->process(image->image, camera_, distortion_,
                                                msg->header.stamp.toSec(), requested_id_);
      std_msgs::Int32 id;
      id.data = tracker_->lockedId();
      id_pub_.publish(id);
      debug_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", tracker_->debugImage()).toImageMsg());
      if (!observation.valid) {
        ROS_WARN_THROTTLE(2.0, "No valid locked ArUco; no center samples published");
        return;
      }
      const Eigen::Vector3d body = rotation_ * observation.position_camera + translation_;
      if (!body.allFinite() || body.z() >= 0) return;
      geometry_msgs::PointStamped point;
      point.header = msg->header;
      point.header.frame_id = body_frame_;
      point.point.x = body.x(); point.point.y = body.y(); point.point.z = body.z();
      point_pub_.publish(point);
      ROS_INFO_THROTTLE(1.0, "id=%d X(front)=%+.1f cm Y(left)=%+.1f cm height=%.3f m",
                        observation.id, body.x() * 100, body.y() * 100, -body.z());
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(2.0, "Observation failed: %s", error.what());
    }
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport transport_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_pub_;
  ros::Subscriber info_sub_;
  ros::Publisher point_pub_, id_pub_;
  std::unique_ptr<precision_landing::ArucoTracker> tracker_;
  Eigen::Matrix3d rotation_;
  Eigen::Vector3d translation_;
  cv::Mat camera_, distortion_;
  unsigned int width_{0}, height_{0};
  int requested_id_{-1};
  std::string body_frame_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "center_offset_observer");
  try {
    CenterOffsetObserver observer;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Center observer failed: %s", error.what());
    return 1;
  }
  return 0;
}
