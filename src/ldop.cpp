#include <algorithm>
#include <sstream>
#include <ldop/ldop.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace ldop {

LDOP::LDOP(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      verbose_(false),
      input_queue_size_(3),
      output_queue_size_(3),
      spinner_thread_count_(0),
      running_(true),
      received_cloud_count_(0U),
      processed_cloud_count_(0U),
      dropped_cloud_count_(0U),
      has_latest_odom_(false) {
  loadParameters();
  setupRosInterfaces();
  startWorkers();

  ROS_INFO_STREAM("LDOP is ready. Input cloud: " << input_cloud_topic_ << ", odom: " << odom_topic_
                                                 << ", static output: " << static_cloud_topic_
                                                 << ", dynamic output: " << dynamic_cloud_topic_
                                                 << ", dynamic objects: " << dynamic_objects_topic_
                                                 << ", marker output: " << marker_topic_);
}

LDOP::~LDOP() {
  stopWorkers();
}

int LDOP::spinnerThreadCount() const {
  return spinner_thread_count_;
}

void LDOP::loadParameters() {
  pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/points_raw");
  pnh_.param<std::string>("odom_topic", odom_topic_, "/lidar_slam/odom");
  pnh_.param<std::string>("static_cloud_topic", static_cloud_topic_, "/ldop/static_cloud");
  pnh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_, "/ldop/dynamic_cloud");
  pnh_.param<std::string>("dynamic_objects_topic", dynamic_objects_topic_, "/ldop/dynamic_objects");
  pnh_.param<std::string>("marker_topic", marker_topic_, "/ldop/debug_markers");
  pnh_.param<std::string>("world_frame", world_frame_, "map");
  pnh_.param("verbose", verbose_, false);
  pnh_.param("input_queue_size", input_queue_size_, 3);
  pnh_.param("output_queue_size", output_queue_size_, 3);
  pnh_.param("spinner_thread_count", spinner_thread_count_, 0);

  input_queue_size_ = std::max(1, input_queue_size_);
  output_queue_size_ = std::max(1, output_queue_size_);
}

void LDOP::setupRosInterfaces() {
  cloud_sub_ = nh_.subscribe(input_cloud_topic_, input_queue_size_, &LDOP::pointCloudCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 50, &LDOP::odomCallback, this);

  static_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_cloud_topic_, 1);
  dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_cloud_topic_, 1);
  dynamic_objects_pub_ = nh_.advertise<ldop::DynamicObjectArray>(dynamic_objects_topic_, 1);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
}

void LDOP::startWorkers() {
  processing_thread_ = std::thread(&LDOP::processingLoop, this);
  publishing_thread_ = std::thread(&LDOP::publishingLoop, this);
}

void LDOP::stopWorkers() {
  running_.store(false);
  input_cv_.notify_all();
  output_cv_.notify_all();

  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  if (publishing_thread_.joinable()) {
    publishing_thread_.join();
  }
}

void LDOP::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *odom_msg;
  has_latest_odom_ = true;
}

void LDOP::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  InputFrame frame;
  frame.cloud_msg = cloud_msg;
  frame.received_time = ros::Time::now();
  frame.sequence = received_cloud_count_.fetch_add(1U) + 1U;

  {
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    frame.has_odom = has_latest_odom_;
    if (frame.has_odom) {
      frame.odom_msg = latest_odom_;
    }
  }

  std::size_t buffered_frames = 0U;
  {
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    while (static_cast<int>(input_queue_.size()) >= input_queue_size_) {
      input_queue_.pop_front();
      dropped_cloud_count_.fetch_add(1U);
    }
    input_queue_.push_back(std::move(frame));
    buffered_frames = input_queue_.size();
  }
  input_cv_.notify_one();

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "LDOP queued cloud #" << received_cloud_count_.load()
                                                        << ", buffered frames: " << buffered_frames);
  }
}

void LDOP::processingLoop() {
  while (true) {
    InputFrame frame;
    std::size_t skipped_frames = 0U;

    {
      std::unique_lock<std::mutex> input_lock(input_mutex_);
      input_cv_.wait(input_lock, [this] { return !running_.load() || !input_queue_.empty(); });

      if (!running_.load() && input_queue_.empty()) {
        return;
      }

      frame = input_queue_.back();
      if (input_queue_.size() > 1) {
        skipped_frames = input_queue_.size() - 1;
        dropped_cloud_count_.fetch_add(skipped_frames);
      }
      input_queue_.clear();
    }

    ProcessedFrame processed = processFrame(frame, skipped_frames);
    processed_cloud_count_.fetch_add(1U);

    {
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      while (static_cast<int>(output_queue_.size()) >= output_queue_size_) {
        output_queue_.pop_front();
      }
      output_queue_.push_back(std::move(processed));
    }
    output_cv_.notify_one();
  }
}

void LDOP::publishingLoop() {
  while (true) {
    ProcessedFrame frame;

    {
      std::unique_lock<std::mutex> output_lock(output_mutex_);
      output_cv_.wait(output_lock, [this] { return !running_.load() || !output_queue_.empty(); });

      if (!running_.load() && output_queue_.empty()) {
        return;
      }

      frame = output_queue_.back();
      output_queue_.clear();
    }

    publishFrame(frame);
  }
}

LDOP::ProcessedFrame LDOP::processFrame(const InputFrame& frame, std::size_t skipped_frames) {
  ProcessedFrame processed;
  processed.static_cloud_msg = *frame.cloud_msg;
  processed.dynamic_cloud_msg = buildEmptyCloud(frame.cloud_msg->header);
  processed.has_odom = frame.has_odom;
  processed.processed_time = ros::Time::now();
  processed.sequence = frame.sequence;
  processed.skipped_input_frames = skipped_frames;

  if (frame.has_odom) {
    processed.odom_msg = frame.odom_msg;
  }

  processed.dynamic_objects_msg = buildDynamicObjectArray(processed);
  processed.marker_array_msg = buildMarkerArray(processed);
  return processed;
}

void LDOP::publishFrame(const ProcessedFrame& frame) {
  static_cloud_pub_.publish(frame.static_cloud_msg);
  dynamic_cloud_pub_.publish(frame.dynamic_cloud_msg);
  dynamic_objects_pub_.publish(frame.dynamic_objects_msg);
  marker_pub_.publish(frame.marker_array_msg);

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "LDOP published frame #" << frame.sequence
                                                           << ", processed: "
                                                           << processed_cloud_count_.load()
                                                           << ", dropped: "
                                                           << dropped_cloud_count_.load());
  }
}

ldop::DynamicObjectArray LDOP::buildDynamicObjectArray(const ProcessedFrame& frame) const {
  ldop::DynamicObjectArray object_array;
  object_array.header = frame.static_cloud_msg.header;
  if (object_array.header.frame_id.empty()) {
    object_array.header.frame_id = world_frame_;
  }

  // The real detector / tracker / predictor will fill this array later.
  // Keeping the topic alive now lets future modules integrate against a stable interface.
  object_array.objects.clear();
  return object_array;
}

visualization_msgs::MarkerArray LDOP::buildMarkerArray(const ProcessedFrame& frame) const {
  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker text_marker;
  text_marker.header = frame.static_cloud_msg.header;
  if (text_marker.header.frame_id.empty()) {
    text_marker.header.frame_id = world_frame_;
  }
  text_marker.ns = "ldop_status";
  text_marker.id = 0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.orientation.w = 1.0;
  text_marker.pose.position.z = 1.0;
  text_marker.scale.z = 0.45;
  text_marker.color.r = 0.10f;
  text_marker.color.g = 0.85f;
  text_marker.color.b = 0.35f;
  text_marker.color.a = 0.95f;
  text_marker.lifetime = ros::Duration(0.3);

  std::ostringstream text;
  text << "LDOP frame " << frame.sequence << " | dropped " << dropped_cloud_count_.load();
  if (frame.has_odom) {
    text << " | odom ok";
  } else {
    text << " | odom missing";
  }
  text_marker.text = text.str();
  markers.markers.push_back(text_marker);

  if (frame.has_odom) {
    const auto& pose = frame.odom_msg.pose.pose;
    const auto& twist = frame.odom_msg.twist.twist;
    const std::string odom_frame =
        frame.odom_msg.header.frame_id.empty() ? world_frame_ : frame.odom_msg.header.frame_id;

    visualization_msgs::Marker ego_marker;
    ego_marker.header.frame_id = odom_frame;
    ego_marker.header.stamp = frame.processed_time;
    ego_marker.ns = "ldop_risk";
    ego_marker.id = 1;
    ego_marker.type = visualization_msgs::Marker::SPHERE;
    ego_marker.action = visualization_msgs::Marker::ADD;
    ego_marker.pose = pose;
    ego_marker.scale.x = 0.6;
    ego_marker.scale.y = 0.6;
    ego_marker.scale.z = 0.6;
    ego_marker.color.r = 0.95f;
    ego_marker.color.g = 0.55f;
    ego_marker.color.b = 0.10f;
    ego_marker.color.a = 0.85f;
    ego_marker.lifetime = ros::Duration(0.3);
    markers.markers.push_back(ego_marker);

    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.frame_id = odom_frame;
    trajectory_marker.header.stamp = frame.processed_time;
    trajectory_marker.ns = "ldop_risk";
    trajectory_marker.id = 2;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.scale.x = 0.12;
    trajectory_marker.color.r = 1.0f;
    trajectory_marker.color.g = 0.25f;
    trajectory_marker.color.b = 0.25f;
    trajectory_marker.color.a = 0.95f;
    trajectory_marker.lifetime = ros::Duration(0.3);

    for (int i = 0; i < 6; ++i) {
      const double dt = 0.25 * static_cast<double>(i);
      geometry_msgs::Point point;
      point.x = pose.position.x + twist.linear.x * dt;
      point.y = pose.position.y + twist.linear.y * dt;
      point.z = pose.position.z + twist.linear.z * dt;
      trajectory_marker.points.push_back(point);
    }
    markers.markers.push_back(trajectory_marker);
  }

  return markers;
}

sensor_msgs::PointCloud2 LDOP::buildEmptyCloud(const std_msgs::Header& header) const {
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  if (cloud.header.frame_id.empty()) {
    cloud.header.frame_id = world_frame_;
  }
  cloud.height = 1;
  cloud.width = 0;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.point_step = 0;
  cloud.row_step = 0;
  return cloud;
}

}  // namespace ldop
