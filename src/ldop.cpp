#include <ldop/ldop.h>
#include <ldop/utils.h>

#include <chrono>

namespace ldopcore {

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
  dynamic_object_clusterer_ = std::make_unique<DynamicObjectClusterer>(pnh_, verbose_);
  startWorkers();

  ROS_INFO_STREAM("  LDOP is ready.");
  ROS_INFO_STREAM("  input cloud: " << input_cloud_topic_);
  ROS_INFO_STREAM("  odom: " << odom_topic_);
  ROS_INFO_STREAM("  static output: " << static_cloud_topic_);
  ROS_INFO_STREAM("  dynamic output: " << dynamic_cloud_topic_);
  ROS_INFO_STREAM("  dynamic objects: " << dynamic_objects_topic_);
  ROS_INFO_STREAM("  dynamic object markers: " << dynamic_object_markers_topic_);
  ROS_INFO_STREAM("  static voxel markers: " << static_map_markers_topic_);
}

Ldop::~Ldop() {
  stopWorkers();
}

void Ldop::loadParameters() {
  // 从参数服务器加载话题名与坐标系配置。
  pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/points_raw");
  pnh_.param<std::string>("odom_topic", odom_topic_, "/lidar_slam/odom");
  pnh_.param<std::string>("static_cloud_topic", static_cloud_topic_, "/ldop/static_cloud");
  pnh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_, "/ldop/dynamic_cloud");
  pnh_.param<std::string>("dynamic_objects_topic", dynamic_objects_topic_, "/ldop/dynamic_objects");
  pnh_.param<std::string>("dynamic_object_markers_topic",
                          dynamic_object_markers_topic_,
                          "/ldop/dynamic_object_markers");
  pnh_.param<std::string>("static_map_markers_topic", static_map_markers_topic_, "/ldop/static_map_markers");
  pnh_.param<std::string>("world_frame", world_frame_, "camera_init");
  pnh_.param("verbose", verbose_, false);
}

void Ldop::setupRosInterfaces() {
  // 订阅输入，并提前创建 LDOP 自己负责的输出通道。
  cloud_sub_ = nh_.subscribe(input_cloud_topic_, 50, &Ldop::pointCloudCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic_, 200, &Ldop::odomCallback, this);

  static_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_cloud_topic_, 10);
  dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_cloud_topic_, 10);
  dynamic_objects_pub_ = nh_.advertise<ldop::DynamicObjectArray>(dynamic_objects_topic_, 10);
  dynamic_object_markers_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(dynamic_object_markers_topic_, 10);
  static_map_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(static_map_markers_topic_, 10, true);
}

void Ldop::startWorkers() {
  // 将处理与发布统一放到后台线程，避免在 ROS 回调里做重活。
  processing_thread_ = std::thread(&Ldop::processingLoop, this);
}

void Ldop::stopWorkers() {
  // 通知线程退出，并等待它们安全收尾。
  running_.store(false);
  input_cv_.notify_all();

  if (processing_thread_.joinable()) {
    processing_thread_.join();
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
    processed_cloud_count_.fetch_add(1U);
    publishFrame(processed);
    processed.timing.processing_loop_ms = elapsedMs(loop_start, std::chrono::steady_clock::now());

    if (verbose_) {
      ROS_INFO_STREAM_THROTTLE(1.0, "LDOP timing frame #" << processed.sequence
                                    << ": odomCloudDt="
                                    << formatFloatMs(processed.odom_cloud_dt_ms)
                                    << "ms, pointCloudCallback="
                                    << formatFloatMs(processed.timing.point_cloud_callback_ms)
                                    << "ms, processingLoop="
                                    << formatFloatMs(processed.timing.processing_loop_ms)
                                    << "ms, processFrame="
                                    << formatFloatMs(processed.timing.process_frame_ms)
                                    << "ms, publishFrame="
                                    << formatFloatMs(processed.timing.publish_frame_ms)
                                    << "ms");
      ROS_INFO_STREAM_THROTTLE(1.0, "LDOP status frame #" << processed.sequence
                                    << ": received=" << received_cloud_count_.load()
                                    << ", processed=" << processed_cloud_count_.load()
                                    << ", backlog="
                                    << (received_cloud_count_.load() > processed_cloud_count_.load()
                                            ? received_cloud_count_.load() -processed_cloud_count_.load(): 0U)
                                    << ", matched=" << matched_odom_count_.load()
                                    << ", missing=" << missing_odom_count_.load()
                                    << ", dropped=" << dropped_cloud_count_.load()
                                    << ", ufomapNodes=" << processed.ufomap_runtime_stats.node_count);
    }
  }
}

Ldop::ProcessedFrame Ldop::processFrame(const InputFrame& frame) {
  // 主流程只做调度，把世界系点云和里程计委托给独立的 UFOMap 模块。
  const auto processing_start = std::chrono::steady_clock::now();
  ProcessedFrame processed;
  processed.sequence = frame.sequence;
  processed.odom_cloud_dt_ms = frame.odom_cloud_dt_ms;
  processed.timing.point_cloud_callback_ms = frame.point_cloud_callback_ms;

  UfomapFrameResult ufomap_result =
      ufomap_mapper_->processInputCloud(frame.cloud_msg, frame.odom_msg);
  processed.static_map_cloud_msg = std::move(ufomap_result.static_map_cloud_msg);
  processed.static_map_markers_msg = std::move(ufomap_result.static_map_markers);
  processed.ufomap_runtime_stats = ufomap_result.runtime_stats;
  processed.dynamic_cloud_msg = std::move(ufomap_result.dynamic_cloud_msg);
  processed.dynamic_cluster_points = std::move(ufomap_result.classification.dynamic_cluster_points);
  DynamicObjectClustererFrameResult cluster_result =
      dynamic_object_clusterer_->processDynamicObjects(processed.dynamic_cloud_msg.header,
                                                      processed.dynamic_cluster_points);
  processed.dynamic_objects_msg = std::move(cluster_result.dynamic_objects_msg);
  processed.dynamic_object_markers_msg = std::move(cluster_result.dynamic_object_markers_msg);
  processed.timing.process_frame_ms =
      elapsedMs(processing_start, std::chrono::steady_clock::now());
  return processed;
}

void Ldop::publishFrame(ProcessedFrame& frame) {
  // 统一从这里发出 LDOP 输出；静态体素 marker 已在处理阶段异步构建完成。
  const auto publish_start = std::chrono::steady_clock::now();
  static_cloud_pub_.publish(frame.static_map_cloud_msg);
  dynamic_cloud_pub_.publish(frame.dynamic_cloud_msg);
  dynamic_objects_pub_.publish(frame.dynamic_objects_msg);
  dynamic_object_markers_pub_.publish(frame.dynamic_object_markers_msg);
  static_map_markers_pub_.publish(frame.static_map_markers_msg);
  frame.timing.publish_frame_ms =
      elapsedMs(publish_start, std::chrono::steady_clock::now());
}

}  // namespace ldopcore
