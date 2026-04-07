#include <ldop/octomap_mapper.h>

#include <cmath>
#include <stdexcept>

#include <octomap_msgs/conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace ldopcore {

namespace {

sensor_msgs::PointCloud2 buildEmptyCloud(const std_msgs::Header& header,
                                         const std::string& world_frame) {
  // 构造一个合法但没有点的 PointCloud2，作为异常时的兜底输出。
  sensor_msgs::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = world_frame;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  cloud.is_dense = false;
  return cloud;
}

}  // namespace

OctomapMapper::OctomapMapper(ros::NodeHandle& nh,
                             ros::NodeHandle& pnh,
                             const std::string& world_frame,
                             const bool verbose)
    : nh_(nh),
      pnh_(pnh),
      world_frame_(world_frame),
      verbose_(verbose),
      octomap_update_count_(0U) {
  loadParameters();
  configureOctomap();
  setupPublishers();

  ROS_INFO_STREAM("OctoMap module is ready. resolution: " << octomap_resolution_
                  << ", max range: " << octomap_max_range_
                  << ", world frame: " << world_frame_
                  << ", topic: " << octomap_full_topic_);
}

void OctomapMapper::loadParameters() {
  // OctoMap 模块单独读取自己的话题和建图参数，避免继续挤在 ldop 主文件里。
  pnh_.param<std::string>("octomap_full_topic", octomap_full_topic_, "/ldop/octomap_full");
  pnh_.param("octomap_resolution", octomap_resolution_, 0.2);
  pnh_.param("octomap_max_range", octomap_max_range_, 50.0);
  pnh_.param("octomap_hit_probability", octomap_hit_probability_, 0.7);
  pnh_.param("octomap_miss_probability", octomap_miss_probability_, 0.4);
  pnh_.param("octomap_clamping_min_probability",
             octomap_clamping_min_probability_, 0.12);
  pnh_.param("octomap_clamping_max_probability",
             octomap_clamping_max_probability_, 0.97);
  pnh_.param("octomap_occupancy_threshold",
             octomap_occupancy_threshold_, 0.5);
}

void OctomapMapper::configureOctomap() {
  octree_.reset();

  octree_ = std::make_unique<octomap::OcTree>(octomap_resolution_);
  octree_->setProbHit(octomap_hit_probability_);
  octree_->setProbMiss(octomap_miss_probability_);
  octree_->setClampingThresMin(octomap_clamping_min_probability_);
  octree_->setClampingThresMax(octomap_clamping_max_probability_);
  octree_->setOccupancyThres(octomap_occupancy_threshold_);
}

void OctomapMapper::setupPublishers() {
  // 使用 latch 发布最近一张地图，晚订阅的可视化工具也能立即拿到当前状态。
  octomap_full_pub_ = nh_.advertise<octomap_msgs::Octomap>(octomap_full_topic_, 10, true);
}

OctomapFrameResult OctomapMapper::processStaticCloud(const sensor_msgs::PointCloud2& cloud_msg,
                                                     const nav_msgs::Odometry& odom_msg) {
  OctomapFrameResult result;
  octomap::Pointcloud octomap_cloud;
  result.static_cloud_msg = buildStaticCloud(cloud_msg, &octomap_cloud);
  const octomap::point3d sensor_origin = buildSensorOrigin(odom_msg);
  updateOctomap(octomap_cloud, sensor_origin, result.static_cloud_msg.header, &result.publish_data);
  return result;
}

void OctomapMapper::publish(const OctomapPublishData& publish_data) {
  if (publish_data.has_full_msg) {
    octomap_full_pub_.publish(publish_data.full_msg);
  }
}

sensor_msgs::PointCloud2 OctomapMapper::buildStaticCloud(
    const sensor_msgs::PointCloud2& cloud_msg,
    octomap::Pointcloud* octomap_cloud) const {
  // 当前约定输入点云已经在世界系，因此这里只做消息整理和有效点提取。
  sensor_msgs::PointCloud2 static_cloud = cloud_msg;

  if (octomap_cloud != nullptr) {
    // OctoMap 插入前先预留容量，避免逐点 push_back 时频繁扩容。
    octomap_cloud->clear();
    octomap_cloud->reserve(static_cast<std::size_t>(cloud_msg.width) *
                           static_cast<std::size_t>(cloud_msg.height));
  }

  if (static_cloud.data.empty()) {
    return static_cloud;
  }

  try {
    // 直接读取 xyz 字段，静态点云发布和 OctoMap 插图共用同一份世界系数据。
    sensor_msgs::PointCloud2Iterator<float> iter_x(static_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(static_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(static_cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float point_x = *iter_x;
      const float point_y = *iter_y;
      const float point_z = *iter_z;

      if (!std::isfinite(point_x) || !std::isfinite(point_y) || !std::isfinite(point_z)) {
        static_cloud.is_dense = false;
        continue;
      }

      if (octomap_cloud != nullptr) {
        octomap_cloud->push_back(point_x, point_y, point_z);
      }
    }
  } catch (const std::runtime_error& error) {
    // 输入消息缺少 xyz 字段时，返回空云并放弃当前帧更新。
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "failed to access xyz fields from cloud: " << error.what());
    return buildEmptyCloud(cloud_msg.header, world_frame_);
  }

  return static_cloud;
}

octomap::point3d OctomapMapper::buildSensorOrigin(const nav_msgs::Odometry& odom_msg) const {
  if (!odom_msg.header.frame_id.empty() && odom_msg.header.frame_id != world_frame_) {
    ROS_WARN_STREAM_THROTTLE(1.0,
                             "octomap expects world-frame odom, but got "
                                 << odom_msg.header.frame_id << ", configured world frame: "
                                 << world_frame_);
  }

  // 当前约定 odom pose 已经表达在世界系下，因此位置可直接作为 ray 原点。
  return octomap::point3d(odom_msg.pose.pose.position.x,
                          odom_msg.pose.pose.position.y,
                          odom_msg.pose.pose.position.z);
}

void OctomapMapper::updateOctomap(const octomap::Pointcloud& static_cloud,
                                  const octomap::point3d& sensor_origin,
                                  const std_msgs::Header& header,
                                  OctomapPublishData* publish_data) {
  if (publish_data == nullptr || octree_ == nullptr ||
      static_cloud.size() == 0U) {
    return;
  }

  const double max_range = octomap_max_range_ > 0.0 ? octomap_max_range_ : -1.0;
  octree_->insertPointCloud(static_cloud, sensor_origin, max_range, false, true);

  // 每次更新后都做一次压缩，降低消息体积和常驻内存。
  octree_->prune();

  publish_data->update_count = octomap_update_count_.fetch_add(1U) + 1U;
  publish_data->node_count = octree_->size();

  if (octomap_msgs::fullMapToMsg(*octree_, publish_data->full_msg)) {
    publish_data->full_msg.header = header;
    publish_data->has_full_msg = true;
  } else {
    ROS_WARN_STREAM_THROTTLE(1.0, "failed to serialize full octomap message");
  }
}

}  // namespace ldopcore
