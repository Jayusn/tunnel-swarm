#include <ldop/ldop.h>

namespace ldopcore {

namespace {

constexpr int kInputQueueSize = 10;
constexpr int kOutputQueueSize = 10;

}  // namespace

Ldop::Ldop(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      verbose_(false),
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
  startWorkers();

  ROS_INFO_STREAM("LDOP is ready. Input cloud: " << input_cloud_topic_ << "\n odom: " << odom_topic_
                                  << "\n         raw cloud output: "<< raw_cloud_output_topic_
                                  << "\n         odom output: " << odom_output_topic_
                                  << "\n         static output: " << static_cloud_topic_
                                  << "\n         dynamic output: " << dynamic_cloud_topic_
                                  << "\n         dynamic objects: " << dynamic_objects_topic_);
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
  pnh_.param<std::string>("sensor_frame", sensor_frame_, "body");
  pnh_.param("verbose", verbose_, false);
}

void Ldop::setupRosInterfaces() {
  // 订阅输入，并提前创建所有输出通道。
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

  // std::size_t buffered_frames = 0U; 一次回调的现有输入队列长度
  // std::size_t dropped_on_enqueue = 0U; 一次回调中丢弃的输入队列长度
  {
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    // 只有输入队列触顶时才丢掉最旧点云，其余情况由处理线程按先进先出顺序消费。
    while (static_cast<int>(input_queue_.size()) >= kInputQueueSize) {
      input_queue_.pop_front();
      dropped_cloud_count_.fetch_add(1U);
      // ++dropped_on_enqueue;
    }
    input_queue_.push_back(std::move(frame));
    // buffered_frames = input_queue_.size();
  }
  input_cv_.notify_one(); // 通知“input_cv_.wait”处理线程现在可以干活了

  const double odom_cloud_time_offset_sec = (frame.odom_msg.header.stamp - cloud_msg->header.stamp).toSec();
  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "received cloud count: " << received_cloud_count_.load()
                                  << ", odom matched: " << matched_odom_count_.load()
                                  // << ", odom missing: " << missing_odom_count_.load()                              
                                  << ", input queue : " << input_queue_.size()
                                  << ", dropped on enqueue: " << dropped_cloud_count_.load()
                                  << ", odom-cloud dt: " << odom_cloud_time_offset_sec);
  }
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

    ProcessedFrame processed = processFrame(frame);
    processed_cloud_count_.fetch_add(1U);

    {
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      // 只有输出队列触顶时才丢掉最旧结果，其余情况由发布线程按先进先出顺序发送。
      while (static_cast<int>(output_queue_.size()) >= kOutputQueueSize) {
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

    publishFrame(frame);
  }
}

Ldop::ProcessedFrame Ldop::processFrame(const InputFrame& frame) {
  // 当前实现仍是骨架：静态点云直接透传，动态点云先构造为空。
  ProcessedFrame processed;
  processed.raw_cloud_msg = frame.cloud_msg;
  processed.static_cloud_msg = frame.cloud_msg;
  processed.dynamic_cloud_msg = buildEmptyCloud(frame.cloud_msg.header);
  processed.odom_msg = frame.odom_msg;
  processed.sequence = frame.sequence;
  processed.dynamic_objects_msg = buildDynamicObjectArray(processed);
  return processed;
}

void Ldop::publishFrame(const ProcessedFrame& frame) {
  // 统一从这里发出所有结果，方便后续补充更多输出。
  raw_cloud_pub_.publish(frame.raw_cloud_msg);
  odom_pub_.publish(frame.odom_msg);
  static_cloud_pub_.publish(frame.static_cloud_msg);
  dynamic_cloud_pub_.publish(frame.dynamic_cloud_msg);
  dynamic_objects_pub_.publish(frame.dynamic_objects_msg);

  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "published frame #" << frame.sequence
                                  << ", processed: "<< processed_cloud_count_.load()
                                  << ", dropped: "<< dropped_cloud_count_.load());
  }
}

::ldop::DynamicObjectArray Ldop::buildDynamicObjectArray(const ProcessedFrame& frame) const {
  // 先发布空数组，占住接口，方便后续接入检测/跟踪结果。
  ::ldop::DynamicObjectArray object_array;
  object_array.header = frame.static_cloud_msg.header;
  if (object_array.header.frame_id.empty()) {
    object_array.header.frame_id = world_frame_;
  }

  // 真正的检测、跟踪、预测结果后续会在这里填充。
  // 现在先保持话题和消息类型稳定，方便后续模块联调。
  object_array.objects.clear();
  return object_array;
}

sensor_msgs::PointCloud2 Ldop::buildEmptyCloud(const std_msgs::Header& header) const {
  // 构造一个合法但没有点的 PointCloud2，供动态点云占位输出。
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

}  // namespace ldopcore
