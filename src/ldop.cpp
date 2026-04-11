#include <ldop/ldop.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include <sensor_msgs/point_cloud2_iterator.h>

namespace ldopcore {

double elapsedMs(const std::chrono::steady_clock::time_point& start,
                 const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string formatFloatMs(const double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

Ldop::Ldop(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      running_(true),
      received_cloud_count_(0U),
      received_odom_count_(0U),
      matched_odom_count_(0U),
      missing_odom_count_(0U),
      processed_cloud_count_(0U),
      dropped_cloud_count_(0U),
      has_latest_odom_(false) {
  loadParameters();
  setupRosInterfaces();
  ufomap_mapper_ = std::make_unique<UfomapMapper>(nh_, pnh_, world_frame_, verbose_);
  startWorkers();

  ROS_INFO_STREAM("  LDOP is ready.");
  ROS_INFO_STREAM("  input cloud: " << input_cloud_topic_);
  ROS_INFO_STREAM("  odom: " << odom_topic_);
  ROS_INFO_STREAM("  raw cloud output: " << raw_cloud_output_topic_);
  ROS_INFO_STREAM("  odom output: " << odom_output_topic_);
  ROS_INFO_STREAM("  static output: " << static_cloud_topic_);
  ROS_INFO_STREAM("  dynamic output: " << dynamic_cloud_topic_);
  ROS_INFO_STREAM("  dynamic objects: " << dynamic_objects_topic_);
}

Ldop::~Ldop() {
  stopWorkers();
}

void Ldop::loadParameters() {
  // 从参数服务器加载话题名与坐标系配置。
  pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/points_raw");
  pnh_.param<std::string>("odom_topic", odom_topic_, "/lidar_slam/odom");
  pnh_.param<std::string>("raw_cloud_output_topic", raw_cloud_output_topic_, "/ldop/raw_cloud");
  pnh_.param<std::string>("odom_output_topic", odom_output_topic_, "/ldop/odom");
  pnh_.param<std::string>("static_cloud_topic", static_cloud_topic_, "/ldop/static_cloud");
  pnh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_, "/ldop/dynamic_cloud");
  pnh_.param<std::string>("dynamic_objects_topic", dynamic_objects_topic_, "/ldop/dynamic_objects");
  pnh_.param<std::string>("world_frame", world_frame_, "camera_init");
  pnh_.param("verbose", verbose_, false);
}

void Ldop::setupRosInterfaces() {
  // 订阅输入，并提前创建 LDOP 自己负责的输出通道。
  cloud_sub_ = nh_.subscribe(input_cloud_topic_, 50, &Ldop::pointCloudCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 200, &Ldop::odomCallback, this);

  raw_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(raw_cloud_output_topic_, 10);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_output_topic_, 10);
  static_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_cloud_topic_, 10);
  dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_cloud_topic_, 10);
  dynamic_objects_pub_ = nh_.advertise<::ldop::DynamicObjectArray>(dynamic_objects_topic_, 10);
}

void Ldop::startWorkers() {
  // 将计算和发布拆到后台线程，避免在 ROS 回调里做重活。
  processing_thread_ = std::thread(&Ldop::processingLoop, this);
  publishing_thread_ = std::thread(&Ldop::publishingLoop, this);
}

void Ldop::stopWorkers() {
  // 通知线程退出，并等待它们安全收尾。
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

void Ldop::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  if (odom_msg == nullptr) {
    return;
  }

  // 极简版本只缓存最新一帧里程计，供点云回调快速取快照。
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *odom_msg;
  has_latest_odom_ = true;
  received_odom_count_.fetch_add(1U);
}

void Ldop::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  if (cloud_msg == nullptr) {
    return;
  }

  const auto callback_start = std::chrono::steady_clock::now();
  InputFrame frame;
  const std::uint64_t cloud_sequence = received_cloud_count_.fetch_add(1U) + 1U;

  {
    // 点云入队前先拷贝一份最新里程计，后续处理线程不再访问共享 odom。
    std::lock_guard<std::mutex> odom_lock(odom_mutex_);
    if (!has_latest_odom_) {
      missing_odom_count_.fetch_add(1U);
      if (verbose_) {
        ROS_WARN_STREAM_THROTTLE(1.0, "skipped cloud #" << cloud_sequence
                                                             << " because no odom is available yet");
      }
      return;
    }
    frame.odom_msg = latest_odom_;
  }
  // 点云进入系统时先编号；只有拿到最新 odom 快照，才允许进入处理队列。
  matched_odom_count_.fetch_add(1U);

  frame.cloud_msg = *cloud_msg;
  frame.sequence = cloud_sequence;
  frame.odom_cloud_dt_ms =
      (frame.odom_msg.header.stamp - cloud_msg->header.stamp).toSec() * 1000.0;

  {
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    // 只有输入队列触顶时才丢掉最旧点云，其余情况由处理线程按先进先出顺序消费。
    while (static_cast<int>(input_queue_.size()) >= 10) {
      input_queue_.pop_front();
      dropped_cloud_count_.fetch_add(1U);
    }
    input_queue_.push_back(std::move(frame));
    input_queue_.back().point_cloud_callback_ms =
        elapsedMs(callback_start, std::chrono::steady_clock::now());
  }
  input_cv_.notify_one();
}

void Ldop::processingLoop() {
  while (true) {
    InputFrame frame;

    {
      std::unique_lock<std::mutex> input_lock(input_mutex_);
      input_cv_.wait(input_lock, [this] { return !running_.load() || !input_queue_.empty(); });

      if (!running_.load() && input_queue_.empty()) {
        return;
      }

      // 队列为空时才会等待；只要有完整数据对且写线程已经释放互斥锁，就按先进先出顺序处理最旧一帧。
      frame = std::move(input_queue_.front());
      input_queue_.pop_front();
    }

    const auto loop_start = std::chrono::steady_clock::now();
    ProcessedFrame processed = processFrame(frame);
    processed.timing.processing_loop_ms = elapsedMs(loop_start, std::chrono::steady_clock::now());
    processed_cloud_count_.fetch_add(1U);

    {
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      // 只有输出队列触顶时才丢掉最旧结果，其余情况由发布线程按先进先出顺序发送。
      while (static_cast<int>(output_queue_.size()) >= 10) {
        output_queue_.pop_front();
      }
      output_queue_.push_back(std::move(processed));
    }
    output_cv_.notify_one();
  }
}

void Ldop::publishingLoop() {
  while (true) {
    ProcessedFrame frame;

    {
      std::unique_lock<std::mutex> output_lock(output_mutex_);
      output_cv_.wait(output_lock, [this] { return !running_.load() || !output_queue_.empty(); });

      if (!running_.load() && output_queue_.empty()) {
        return;
      }

      // 队列为空时才会等待；只要有可发布结果且写线程已经释放互斥锁，就按先进先出顺序发布最旧一帧。
      frame = std::move(output_queue_.front());
      output_queue_.pop_front();
    }

    const auto loop_start = std::chrono::steady_clock::now();
    publishFrame(frame);
    frame.timing.publishing_loop_ms = elapsedMs(loop_start, std::chrono::steady_clock::now());

    if (verbose_) {
      ROS_INFO_STREAM_THROTTLE(1.0, "LDOP timing frame #" << frame.sequence
                                    << ": odomCloudDt="
                                    << formatFloatMs(frame.odom_cloud_dt_ms)
                                    << "ms, pointCloudCallback="
                                    << formatFloatMs(frame.timing.point_cloud_callback_ms)
                                    << "ms, processingLoop="
                                    << formatFloatMs(frame.timing.processing_loop_ms)
                                    << "ms, processFrame="
                                    << formatFloatMs(frame.timing.process_frame_ms)
                                    << "ms, processInputCloud="
                                    << formatFloatMs(frame.ufomap_timing.process_input_cloud_ms)
                                    << "ms, publishFrame="
                                    << formatFloatMs(frame.timing.publish_frame_ms)
                                    << "ms, publishingLoop="
                                    << formatFloatMs(frame.timing.publishing_loop_ms)
                                    << "ms");
      ROS_INFO_STREAM_THROTTLE(1.0, "LDOP status frame #" << frame.sequence
                                    << ": received=" << received_cloud_count_.load()
                                    << ", processed=" << processed_cloud_count_.load()
                                    << ", backlog="
                                    << (received_cloud_count_.load() > processed_cloud_count_.load()
                                            ? received_cloud_count_.load() -processed_cloud_count_.load(): 0U)
                                    << ", matched=" << matched_odom_count_.load()
                                    << ", missing=" << missing_odom_count_.load()
                                    << ", dropped=" << dropped_cloud_count_.load()
                                    << ", ufomapNodes=" << frame.ufomap_runtime_stats.node_count);
    }
  }
}

Ldop::ProcessedFrame Ldop::processFrame(const InputFrame& frame) {
  // 主流程只做调度，把世界系点云和里程计委托给独立的 UFOMap 模块。
  const auto processing_start = std::chrono::steady_clock::now();
  ProcessedFrame processed;
  processed.raw_cloud_msg = frame.cloud_msg;
  processed.odom_msg = frame.odom_msg;
  processed.sequence = frame.sequence;
  processed.odom_cloud_dt_ms = frame.odom_cloud_dt_ms;
  processed.timing.point_cloud_callback_ms = frame.point_cloud_callback_ms;

  UfomapFrameResult ufomap_result =
      ufomap_mapper_->processInputCloud(frame.cloud_msg, frame.odom_msg);
  processed.static_map_cloud_msg = std::move(ufomap_result.static_map_cloud_msg);
  processed.ufomap_runtime_stats = ufomap_result.runtime_stats;
  processed.ufomap_publish_data = std::move(ufomap_result.publish_data);
  processed.ufomap_timing = ufomap_result.timing;
  processed.dynamic_cloud_msg = std::move(ufomap_result.dynamic_cloud_msg);

  processed.dynamic_objects_msg = buildDynamicObjectArray(processed);
  processed.timing.process_frame_ms =
      elapsedMs(processing_start, std::chrono::steady_clock::now());
  return processed;
}

void Ldop::publishFrame(ProcessedFrame& frame) {
  // 统一从这里发出 LDOP 输出，UFOMap 统计信息则交给独立模块维护。
  const auto publish_start = std::chrono::steady_clock::now();
  raw_cloud_pub_.publish(frame.raw_cloud_msg);
  odom_pub_.publish(frame.odom_msg);
  static_cloud_pub_.publish(frame.static_map_cloud_msg);
  dynamic_cloud_pub_.publish(frame.dynamic_cloud_msg);
  dynamic_objects_pub_.publish(frame.dynamic_objects_msg);
  ufomap_mapper_->publish(frame.ufomap_publish_data);
  frame.timing.publish_frame_ms =
      elapsedMs(publish_start, std::chrono::steady_clock::now());
}

::ldop::DynamicObjectArray Ldop::buildDynamicObjectArray(const ProcessedFrame& frame) const {
  ::ldop::DynamicObjectArray object_array;
  object_array.header = frame.dynamic_cloud_msg.header;
  if (object_array.header.frame_id.empty()) {
    object_array.header.frame_id = world_frame_;
  }

  object_array.objects.clear();
  if (frame.dynamic_cloud_msg.data.empty() || frame.dynamic_cloud_msg.width == 0U ||
      frame.dynamic_cloud_msg.height == 0U) {
    return object_array;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float min_z = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  std::size_t finite_count = 0U;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(frame.dynamic_cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(frame.dynamic_cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(frame.dynamic_cloud_msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      min_z = std::min(min_z, z);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
      max_z = std::max(max_z, z);
      ++finite_count;
    }
  } catch (const std::runtime_error& error) {
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "failed to build dynamic objects from cloud: " << error.what());
    return object_array;
  }

  if (finite_count == 0U) {
    return object_array;
  }

  ::ldop::DynamicObject object;
  object.id = static_cast<std::uint32_t>(frame.sequence);
  object.center.x = (static_cast<double>(min_x) + static_cast<double>(max_x)) * 0.5;
  object.center.y = (static_cast<double>(min_y) + static_cast<double>(max_y)) * 0.5;
  object.center.z = (static_cast<double>(min_z) + static_cast<double>(max_z)) * 0.5;
  object.size.x = std::max(static_cast<double>(max_x - min_x), 0.0);
  object.size.y = std::max(static_cast<double>(max_y - min_y), 0.0);
  object.size.z = std::max(static_cast<double>(max_z - min_z), 0.0);
  object.velocity.x = 0.0;
  object.velocity.y = 0.0;
  object.velocity.z = 0.0;
  object_array.objects.push_back(object);
  return object_array;
}

sensor_msgs::PointCloud2 Ldop::buildEmptyCloud(const std_msgs::Header& header) const {
  // 构造一个合法但没有点的 PointCloud2，供动态点云占位输出。
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  if (cloud.header.frame_id.empty()) {
    cloud.header.frame_id = world_frame_;
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  cloud.is_dense = false;
  return cloud;
}

}  // namespace ldopcore
