#include "mapping_manager.h"
#include "density_partition.h"
#include "bidirectional_depth.h"
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace {

struct CallbackTimer {
  uint64_t stamp;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  ~CallbackTimer() {
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "FAPP_CALLBACK_TIME " << stamp << " " << milliseconds << std::endl;
  }
};

// Frame-level ray evidence. Unobserved rays and returns behind an old
// foreground are unknown, so they retain the original nearest-point test.
struct DepthView {
  static constexpr double step = 3.14159265358979323846 / 720.0;
  static constexpr int columns = 1440, rows = 720;
  MapData pose;
  std::unordered_map<int, PointVector> bins;

  static int azimuth(const Eigen::Vector3d& ray) {
    int bin = std::floor((std::atan2(ray.y(), ray.x()) +
                         3.14159265358979323846) / step);
    return (bin + columns) % columns;
  }

  static int elevation(const Eigen::Vector3d& ray) {
    return std::max(0, std::min(rows - 1, static_cast<int>(std::floor(
        (std::atan2(ray.z(), ray.head<2>().norm()) +
         3.14159265358979323846 / 2) / step))));
  }

  void build(const Points& points, const MapData& sensor_pose) {
    pose = sensor_pose;
    for (const auto& point : points) {
      Eigen::Vector3d ray = pose.R.transpose() *
          (point.getVector3fMap().cast<double>() - pose.T);
      if (!ray.allFinite() || ray.squaredNorm() == 0.0) continue;
      PointType sample;
      sample.x = ray.x(); sample.y = ray.y(); sample.z = ray.z();
      bins[elevation(ray) * columns + azimuth(ray)].push_back(sample);
    }
  }

  // -1: unknown, 0: old surface support, 1: foreground occlusion.
  int classify(const PointType& point) const {
    Eigen::Vector3d ray = pose.R.transpose() *
        (point.getVector3fMap().cast<double>() - pose.T);
    double range = ray.norm();
    if (!std::isfinite(range) || range == 0.0) return -1;
    Eigen::Vector3d direction = ray / range;
    double angle = std::asin(std::min(1.0, 0.03 / range));
    double latitude = std::atan2(ray.z(), ray.head<2>().norm());
    int vertical = static_cast<int>(std::ceil(angle / step)) + 1;
    double minimum_cos = std::cos(std::min(3.14159265358979323846 / 2,
                                         std::abs(latitude) + angle));
    int horizontal = minimum_cos < 1e-6 ? columns / 2 :
        std::min(columns / 2, static_cast<int>(std::ceil(
            angle / (step * minimum_cos))) + 1);
    int row = elevation(ray), column = azimuth(ray);
    double minimum = std::numeric_limits<double>::infinity(), maximum = 0.0;
    for (int y = std::max(0, row - vertical); y <= std::min(rows - 1, row + vertical); ++y) {
      int count = std::min(columns, 2 * horizontal + 1);
      for (int offset = -horizontal; offset < -horizontal + count; ++offset) {
        auto found = bins.find(y * columns + (column + offset + columns) % columns);
        if (found == bins.end()) continue;
        for (const auto& sample : found->second) {
          Eigen::Vector3d previous = sample.getVector3fMap().cast<double>();
          double depth = previous.norm();
          if (direction.dot(previous) <= 0.0 ||
              range * direction.cross(previous / depth).norm() > 0.03) continue;
          minimum = std::min(minimum, depth);
          maximum = std::max(maximum, depth);
        }
      }
    }
    if (!std::isfinite(minimum)) return -1;
    if (range < minimum - 0.03) return 1;
    if (range <= maximum + 0.03) return 0;
    return -1;
  }
};

constexpr double DepthView::step;
constexpr int DepthView::columns;
constexpr int DepthView::rows;

}  // namespace

namespace mot_mapping {

MappingRos::MappingRos(ros::NodeHandle &nh):nh(nh) {}

MappingRos::~MappingRos() {std::cout << "Exit Tracker" << std::endl;}

void MappingRos::init() {
  t_start = ros::Time::now();
  frame_num = 1;
  id = 0;

  Remaining_Points.reset(new Points);
  ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));

  mp_ = std::make_unique<MapParam>();
  md_ = std::make_unique<MapData>();

  nh.param("dbscan/core_pts", mp_->core_pts, 4);
  nh.param("dbscan/tolerance", mp_->tolerance, 0.02);
  nh.param("dbscan/min_cluster", mp_->min_cluster, 20);
  nh.param("dbscan/max_cluster", mp_->max_cluster, 800);
  nh.param("clustering/voxel_size", mp_->cluster_voxel_size, 0.05);
  nh.param("clustering/density_split_enabled", mp_->density_split_enabled, false);
  nh.param("clustering/candidate_partition_enabled", mp_->candidate_partition_enabled, false);
  nh.param("clustering/object_candidate_split_enabled", mp_->object_candidate_split_enabled, false);
  nh.param("association/object_candidate_isolation_enabled", mp_->object_candidate_isolation_enabled, false);
  nh.param("clustering/core_radius", mp_->core_radius, 0.08);
  nh.param("clustering/core_density_fraction", mp_->core_density_fraction, 0.7);
  nh.param("association/global_assignment_enabled", mp_->global_assignment_enabled, false);
  nh.param("tracking/position_measurement_variance", mp_->position_measurement_variance, 0.09);
  nh.param("detection/translated_startup_enabled", mp_->translated_startup_enabled, false);
  nh.param("detection/translated_startup_fraction", mp_->translated_startup_fraction, 0.8);
  if (!std::isfinite(mp_->cluster_voxel_size) || mp_->cluster_voxel_size <= 0.0 ||
      !std::isfinite(mp_->core_radius) || mp_->core_radius <= 0.0 ||
      !std::isfinite(mp_->core_density_fraction) || mp_->core_density_fraction <= 0.0 ||
      mp_->core_density_fraction > 1.0 ||
      !std::isfinite(mp_->position_measurement_variance) || mp_->position_measurement_variance <= 0.0 ||
      !std::isfinite(mp_->translated_startup_fraction) || mp_->translated_startup_fraction <= 0.0 ||
      mp_->translated_startup_fraction > 1.0)
    throw std::runtime_error("invalid clustering, tracking or translated-startup parameter");
  nh.param("detection/thresh_dist", mp_->thresh_dist, 0.08);
  nh.param("detection/thresh_var", mp_->thresh_var, 0.28);
  nh.param("detection/motion_displacement", mp_->motion_displacement, 0.06);
  nh.param("detection/motion_confirmation_frames", mp_->motion_confirmation_frames, 3);
  if (mp_->motion_confirmation_frames < 3 || mp_->motion_confirmation_frames > 5)
    throw std::runtime_error("motion_confirmation_frames must be 3, 4 or 5");
  std::string dynamic_mode;
  nh.param<std::string>("detection/dynamic_mode", dynamic_mode, "general");
  if (dynamic_mode != "general" && dynamic_mode != "swing-only")
    throw std::runtime_error("detection/dynamic_mode must be general or swing-only");
  mp_->swing_only = dynamic_mode == "swing-only";
  nh.param("detection/object_motion_evidence_enabled", mp_->object_motion_evidence_enabled, false);
  nh.param("detection/object_motion_match_fraction", mp_->object_motion_match_fraction, 0.45);
  nh.param("detection/object_motion_match_advantage", mp_->object_motion_match_advantage, 0.30);
  nh.param("detection/object_motion_depth_fraction", mp_->object_motion_depth_fraction, 0.40);
  for (double value : {mp_->object_motion_match_fraction, mp_->object_motion_match_advantage,
                       mp_->object_motion_depth_fraction})
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
      throw std::runtime_error("object motion evidence fractions must be in (0, 1]");
  nh.param("detection/time_evidence_enabled", mp_->time_evidence_enabled, false);
  nh.param("detection/motion_window", mp_->motion_window, 0.6);
  nh.param("detection/confirmed_reacquisition_enabled", mp_->confirmed_reacquisition_enabled, false);
  nh.param("detection/reacquisition_max_gap", mp_->reacquisition_max_gap, 0.6);
  if (!std::isfinite(mp_->motion_window) || mp_->motion_window <= 0.0 ||
      !std::isfinite(mp_->reacquisition_max_gap) || mp_->reacquisition_max_gap <= 0.0)
    throw std::runtime_error("motion and reacquisition windows must be positive");
  nh.param("detection/min_motion_speed", mp_->min_motion_speed, 0.15);
  nh.param("detection/min_motion_novel_fraction", mp_->min_motion_novel_fraction, 0.3);
  nh.param("detection/tracked_motion_novel_fraction", mp_->tracked_motion_novel_fraction,
           mp_->min_motion_novel_fraction);
  nh.param("detection/max_above_uav", mp_->max_above_uav, 1.5);
  nh.param("association/geometry_enabled", mp_->geometry_association_enabled, false);
  nh.param("association/competition_enabled", mp_->competition_association_enabled, false);
  nh.param("association/point_source_competition_enabled", mp_->point_source_competition_enabled, false);
  nh.param("detection/publish_measured_body_position_enabled",
           mp_->publish_measured_body_position_enabled, false);
  nh.param("detection/dense_change_evidence_enabled", mp_->dense_change_evidence_enabled, false);
  nh.param("detection/published_startup_source_enabled", mp_->published_startup_source_enabled, false);
  nh.param("detection/bidirectional_ray_evidence_enabled",
           mp_->bidirectional_ray_evidence_enabled, false);
  nh.param("detection/bidirectional_static_suppression_enabled",
           mp_->bidirectional_static_suppression_enabled, false);
  nh.param("detection/strict_depth_evidence_enabled", mp_->strict_depth_evidence_enabled, false);
  nh.param("detection/depth_evidence_enabled", mp_->depth_evidence_enabled, false);
  nh.param("detection/occlusion_startup_enabled", mp_->occlusion_startup_enabled, false);
  nh.param("detection/shape_consistency_enabled", mp_->shape_consistency_enabled, false);
  nh.param("detection/shape_consistency_distance", mp_->shape_consistency_distance, 0.05);
  nh.param("detection/shape_consistency_fraction", mp_->shape_consistency_fraction, 0.5);
  nh.param("background_online/enabled", mp_->online_background_enabled, false);
  nh.param("detection/reference_strong_two_frame_enabled", mp_->reference_strong_two_frame_enabled, false);
  if (mp_->depth_evidence_enabled || mp_->occlusion_startup_enabled || mp_->shape_consistency_enabled ||
      mp_->time_evidence_enabled || mp_->strict_depth_evidence_enabled || mp_->online_background_enabled ||
      mp_->reference_strong_two_frame_enabled || mp_->bidirectional_ray_evidence_enabled ||
      mp_->object_motion_evidence_enabled ||
      mp_->bidirectional_static_suppression_enabled || mp_->candidate_partition_enabled ||
      (mp_->translated_startup_enabled && !mp_->time_evidence_enabled)) {
    std::vector<double> offset;
    // Same MID360 offset as FAST-LIO's mid360.yaml; use its live parameter
    // when the sensor stack is running. Replay supplies that parameter too.
    nh.param("/UAV1/mapping/extrinsic_T", offset,
             std::vector<double>{-0.011, -0.02329, 0.04412});
    if (offset.size() != 3) throw std::runtime_error("FAST-LIO extrinsic_T requires three values");
    mp_->lidar_offset = Eigen::Vector3d(offset[0], offset[1], offset[2]);
    ROS_INFO_STREAM("FAPP lidar offset: " << mp_->lidar_offset.transpose());
  }
  nh.param("background_surface/enabled", mp_->background_surface_enabled, true);
  nh.param("background_plane/enabled", mp_->background_plane_enabled, true);
  nh.param("background_plane/distance", mp_->background_plane_distance, 0.03);
  nh.param("background_plane/min_points", mp_->background_plane_min_points, 100);
  nh.param("background_plane/max_planes", mp_->background_plane_max_planes, 6);
  nh.param("background_consistency/enabled", mp_->background_consistency_enabled, false);
  nh.param("background_consistency/local_planes_enabled", mp_->background_local_planes_enabled, false);
  nh.param("background_consistency/local_post_association", mp_->background_local_post_association, true);
  nh.param("background_consistency/distance", mp_->background_consistency_distance, 0.02);
  nh.param("background_consistency/fraction", mp_->background_consistency_fraction, 0.8);
  nh.param("background_consistency/min_frames", mp_->background_consistency_frames, 4);
  nh.param("background_consistency/post_association", mp_->background_consistency_post_association, true);
  nh.param("background_reference/enabled", mp_->background_reference_enabled, false);
  nh.param("detection/reference_motion_enabled", mp_->reference_motion_enabled, false);
  nh.param("background_reference/capture_duration", mp_->reference_capture_duration, 5.0);
  nh.param("background_reference/distance", mp_->reference_distance, 0.03);
  nh.param("background_reference/fraction", mp_->reference_fraction, 0.5);
  nh.param("background_online/rotation_rate", mp_->online_rotation_rate, 0.2);
  nh.param("background_online/jump_distance", mp_->online_jump_distance, 0.5);
  nh.param("background_online/scene_overlap", mp_->online_scene_overlap, 0.2);
  if (mp_->online_background_enabled) {
    OnlineBackgroundOptions options;
    nh.param("background_online/voxel_size", options.voxel_size, 0.05);
    nh.param("background_online/residual", options.residual, 0.03);
    nh.param("background_online/min_observations", options.min_observations, 5);
    nh.param("background_online/min_duration", options.min_duration, 0.5);
    nh.param("background_online/update_interval", options.update_interval, 0.3);
    nh.param("background_online/local_radius", options.local_radius, 8.0);
    mp_->online_voxel_size = options.voxel_size;
    if (!std::isfinite(mp_->online_rotation_rate) || mp_->online_rotation_rate <= 0.0 ||
        !std::isfinite(mp_->online_jump_distance) || mp_->online_jump_distance <= 0.0 ||
        !std::isfinite(mp_->online_scene_overlap) || mp_->online_scene_overlap < 0.0 ||
        mp_->online_scene_overlap > 1.0)
      throw std::runtime_error("invalid online background motion or scene threshold");
    online_background_.reset(new OnlineBackground(options));
  }
  if (mp_->background_reference_enabled || mp_->reference_motion_enabled || mp_->online_background_enabled)
    reference_points_.reset(new Points);
  nh.param("lidar/range_x", mp_->Range[0], 15.0);
  nh.param("lidar/range_y", mp_->Range[1], 15.0);
  nh.param("lidar/range_z", mp_->Range[2], 1.0);
  nh.param("ros/odom_topic", mp_->odom_topic, string("/odom"));
  nh.param("ros/lidar_topic", mp_->lidar_topic, string("/livox/lidar"));
  nh.param("ros/frame_id", frame_id_, string("UAV1/camera_init"));
  nh.param("synchronization/nearest_available_odom_enabled", mp_->nearest_available_odom_enabled, false);

  ros::NodeHandle output_nh("/UAV1/fapp");
  cloudPub = output_nh.advertise<sensor_msgs::PointCloud2>("dynamic_cloud", 10);
  mapPub = output_nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
  staticMapPub = output_nh.advertise<sensor_msgs::PointCloud2>("static_map", 10);
  edgePub = output_nh.advertise<visualization_msgs::MarkerArray>("dynamic_object_markers", 10);
  objectPosePub = output_nh.advertise<visualization_msgs::MarkerArray>("dynamic_track_markers", 10);
  statesPub = output_nh.advertise<obj_state_msgs::ObjectsStates>("states", 10);

  std::cout << "INIT!" << std::endl;

  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, mp_->lidar_topic, 50));
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, mp_->odom_topic, 25));
  if (mp_->nearest_available_odom_enabled) {
    odom_cache_.reset(new message_filters::Cache<nav_msgs::Odometry>(*odom_sub_, 100));
    // Cache receives each real message before forwarding it to the unchanged
    // synchronizer. Selection never waits for an odometry message to arrive.
    sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
        MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_cache_));
  } else {
    sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
        MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
  }
  sync_cloud_odom_->registerCallback(boost::bind(&MappingRos::cloudOdomCallback, this, _1, _2));

  map_pub_timer_ = nh.createTimer(ros::Duration(0.05), &MappingRos::mapPubCallback, this);
}


float MappingRos::calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void MappingRos::generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths) {
    float &x_dist = box_lengths[0];
    float &y_dist = box_lengths[1];
    float &z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}

void MappingRos::pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w) {
  int num = p_b->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_b->points[i].x,
         p_b->points[i].y,
         p_b->points[i].z;
    // p = md_->R * p + md_->T;
    if ((p - md_->T).head(2).norm() < 5.0) {
        PointType pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];

        p_w->points.push_back(pt);
    }
  }
}

void MappingRos::pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T) {
  int num = p_w->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_w->points[i].x,
         p_w->points[i].y,
         p_w->points[i].z;
    p = (p - md_->T);

    PointType pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    p_b->points.push_back(pt);
  }
}


void MappingRos::removeStaticPlanes(PointsPtr& points, const PointsPtr& history) {
  if (!mp_->background_plane_enabled || history->empty()) return;
  pcl::search::KdTree<PointType> history_tree;
  history_tree.setInputCloud(history);
  PointsPtr remaining(new Points(*points));
  PointsPtr foreground(new Points);
  pcl::SACSegmentation<PointType> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(100);
  segmentation.setDistanceThreshold(mp_->background_plane_distance);
  for (int plane = 0; plane < mp_->background_plane_max_planes &&
       remaining->size() >= static_cast<size_t>(mp_->background_plane_min_points); ++plane) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients coefficients;
    segmentation.setInputCloud(remaining);
    segmentation.segment(*inliers, coefficients);
    if (inliers->indices.size() < static_cast<size_t>(mp_->background_plane_min_points)) break;
    PointsPtr surface(new Points);
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.filter(*surface);
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance;
    pcl::compute3DCentroid(*surface, centroid);
    pcl::computeCovarianceMatrixNormalized(*surface, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    int supported = 0;
    std::vector<int> nearest(1);
    std::vector<float> squared_distance(1);
    for (const auto& point : *surface) {
      if (history_tree.nearestKSearch(point, 1, nearest, squared_distance) > 0 &&
          squared_distance[0] <= 0.12 * 0.12) ++supported;
    }
    // Require a broad surface in two directions and support in the previous
    // registered cloud. A small curved return is not a background plane.
    bool background = solver.eigenvalues()[1] >= 0.5 * 0.5 / 12.0 &&
                      supported >= 0.8 * surface->size();
    if (!background) *foreground += *surface;
    ROS_DEBUG_STREAM("FAPP_PLANE " << cloud_stamp_.toNSec() << " "
        << surface->size() << " " << background << " " << coefficients);
    PointsPtr next(new Points);
    extract.setNegative(true);
    extract.filter(*next);
    remaining = next;
  }
  *foreground += *remaining;
  points = foreground;
}

void MappingRos::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                                   const nav_msgs::OdometryConstPtr& synchronized_odom) {
  cloud_stamp_ = msg->header.stamp;
  CallbackTimer callback_timer{cloud_stamp_.toNSec()};
  nav_msgs::OdometryConstPtr odom = synchronized_odom;
  if (odom_cache_) {
    const auto received = odom_cache_->getInterval(odom_cache_->getOldestTime(),
                                                  odom_cache_->getLatestTime());
    const uint64_t cloud_ns = cloud_stamp_.toNSec();
    auto distance = [&](const nav_msgs::OdometryConstPtr& value) {
      const uint64_t stamp = value->header.stamp.toNSec();
      return stamp >= cloud_ns ? stamp - cloud_ns : cloud_ns - stamp;
    };
    for (const auto& value : received) {
      if (distance(value) < distance(odom) ||
          (distance(value) == distance(odom) && value->header.stamp < odom->header.stamp))
        odom = value;
    }
    ROS_DEBUG_STREAM("FAPP_SYNC_SELECTION " << cloud_ns << " "
        << synchronized_odom->header.stamp.toNSec() << " " << odom->header.stamp.toNSec()
        << " " << received.size());
  }
  ROS_DEBUG_STREAM("FAPP_SYNC_POSE " << cloud_stamp_.toNSec() << " "
      << odom->header.stamp.toNSec() << " "
      << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " "
      << odom->pose.pose.position.z << " " << odom->pose.pose.orientation.x << " "
      << odom->pose.pose.orientation.y << " " << odom->pose.pose.orientation.z << " "
      << odom->pose.pose.orientation.w);
  const double frame_dt = previous_cloud_stamp_.isZero() ? 0.0 :
      (cloud_stamp_ - previous_cloud_stamp_).toSec();
  previous_cloud_stamp_ = cloud_stamp_;
  ++frame_num;
  if (frame_dt > 0.0) {
    for (auto& tracker : trackers) {
      tracker->predict(frame_dt);
      tracker->age++;
    }
  }
  PointsPtr latest_cloud(new Points);
  pcl::fromROSMsg(*msg, *latest_cloud);

  md_->T << odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z;

  md_->R = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();

  std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
  double compTime;
  // FAST-LIO registered clouds are already in the odometry frame.

  // Pointcloud Filter
  PointsPtr cloud_filtered(new Points);
  vox.setInputCloud(latest_cloud);
  vox.setLeafSize(0.1, 0.1, 0.1);
  vox.filter(*cloud_filtered);

  PointsPtr cloud_world(new Points);
  PointsPtr nonfilter_pts(new Points);
  PointsPtr PointToAdd(new Points);
  PointsPtr ClusterPoints(new Points);
  pointsBodyToWorld(cloud_filtered, cloud_world);
  pointsBodyToWorld(latest_cloud, nonfilter_pts);

  // Capture the initial static scene in the registered world frame. Once
  // frozen, this reference never learns the trajectory of a moving object.
  // Dynamic publication is suppressed during capture; the raw cloud path remains active.
  bool reference_just_frozen = false;
  if (reference_points_ && !reference_tree_) {
    if (reference_start_stamp_.isZero()) reference_start_stamp_ = cloud_stamp_;
    if ((cloud_stamp_ - reference_start_stamp_).toSec() < mp_->reference_capture_duration) {
      for (const auto& point : *latest_cloud)
        if (point.getVector3fMap().allFinite()) reference_points_->push_back(point);
    } else if (!reference_points_->empty()) {
      reference_tree_ = std::make_shared<pcl::search::KdTree<PointType>>();
      reference_tree_->setInputCloud(reference_points_);
      reference_just_frozen = true;
      ROS_INFO_STREAM("FAPP_REFERENCE_READY " << cloud_stamp_.toNSec() << " "
          << reference_points_->size() << " " << reference_start_stamp_.toNSec());
    }
  }

  // A track created during calibration must not carry its motion confirmation
  // into the first scene after the reference is frozen. The raw observation
  // history is also reset so the first post-capture candidate starts fresh.
  if (reference_just_frozen) {
    trackers.clear();
    buffer.clear();
    observed_cloud_stamps_.clear();
    observed_sensor_poses.clear();
    timed_observations_.clear();
    bidirectional_previous_observation_.points.reset();
    next_background_sample_stamp_ = ros::Time();
    last_background_sample_stamp_ = ros::Time();
    if (online_background_) {
      online_background_->clearPending();
      online_has_previous_pose_ = false;
      online_previous_voxels_.clear();
    }
    ROS_INFO_STREAM("FAPP_REFERENCE_RESET " << cloud_stamp_.toNSec());
  }

  // Compare novelty against the two preceding raw observations.
  PointsPtr observed_history[2];
  MapData sensor_history[2];
  ros::Time history_stamps[2];
  if (mp_->time_evidence_enabled) {
    while (!timed_observations_.empty() &&
           (cloud_stamp_ - timed_observations_.front().stamp).toSec() > 4.0)
      timed_observations_.pop_front();
    int selected[2] = {-1, -1};
    for (int history = 0; history < 2; ++history) {
      const double lag = 0.3 * (history + 1);
      double best_error = 0.15 + 1e-9;
      for (int raw = 0; raw < timed_observations_.size(); ++raw) {
        const double age = (cloud_stamp_ - timed_observations_[raw].stamp).toSec();
        if (age <= 0.0 || (history > 0 && selected[0] >= 0 &&
            timed_observations_[raw].stamp == timed_observations_[selected[0]].stamp)) continue;
        const double error = std::abs(age - lag);
        if (error < best_error) {
          selected[history] = raw;
          best_error = error;
        }
      }
      if (selected[history] >= 0) {
        const auto& sample = timed_observations_[selected[history]];
        observed_history[history] = sample.points;
        sensor_history[history] = sample.sensor_pose;
        history_stamps[history] = sample.stamp;
      }
    }
    MapData sensor_pose;
    sensor_pose.R = md_->R;
    sensor_pose.T = md_->T + md_->R * mp_->lidar_offset;
    timed_observations_.push_back({cloud_stamp_, nonfilter_pts, sensor_pose});
    if (timed_observations_.size() > 64) timed_observations_.pop_front();
    ROS_DEBUG_STREAM("FAPP_TIME_EVIDENCE " << cloud_stamp_.toNSec() << " "
        << history_stamps[0].toNSec() << " " << history_stamps[1].toNSec() << " "
        << timed_observations_.size());
  } else {
    if (!buffer.empty()) {
      observed_history[0] = buffer.front();
      history_stamps[0] = observed_cloud_stamps_.front();
    }
    if (buffer.size() > 1) {
      observed_history[1] = buffer.back();
      history_stamps[1] = observed_cloud_stamps_.back();
    }
  }
  if (!mp_->time_evidence_enabled &&
      (mp_->depth_evidence_enabled || mp_->occlusion_startup_enabled ||
       mp_->strict_depth_evidence_enabled || mp_->online_background_enabled ||
       mp_->reference_strong_two_frame_enabled)) {
    if (!observed_sensor_poses.empty()) sensor_history[0] = observed_sensor_poses.front();
    if (observed_sensor_poses.size() > 1) sensor_history[1] = observed_sensor_poses.back();
    MapData sensor_pose;
    sensor_pose.R = md_->R;
    sensor_pose.T = md_->T + md_->R * mp_->lidar_offset;
    observed_sensor_poses.push_back(sensor_pose);
    if (observed_sensor_poses.size() > 2) observed_sensor_poses.pop_front();
  }
  buffer.push_back(nonfilter_pts);
  observed_cloud_stamps_.push_back(cloud_stamp_);
  if (buffer.size() > 2) {
    buffer.pop_front();
    observed_cloud_stamps_.pop_front();
  }
  *ClusterPoints += *nonfilter_pts;
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(mp_->cluster_voxel_size, mp_->cluster_voxel_size, mp_->cluster_voxel_size);
  vox.filter(*ClusterPoints);

  // Retain bounded, registered observations, including foreground returns.
  // Keeping foreground prevents a moving object's old position becoming a
  // clean background plane merely because it was previously classified moving.
  auto background_history = background_history_;
  if (mp_->background_consistency_enabled) {
    PointsPtr background_sample = ClusterPoints;
    bool append_sample = true;
    if (mp_->time_evidence_enabled) {
      append_sample = false;
      if (next_background_sample_stamp_.isZero()) next_background_sample_stamp_ = cloud_stamp_;
      if (cloud_stamp_ >= next_background_sample_stamp_) {
        // Keep the sampling phase across callback jitter and skip missed ticks.
        const double elapsed = (cloud_stamp_ - next_background_sample_stamp_).toSec();
        const ros::Time deadline = next_background_sample_stamp_ +
            ros::Duration(0.3 * std::floor((elapsed + 1e-9) / 0.3));
        for (auto sample = timed_observations_.rbegin(); sample != timed_observations_.rend(); ++sample) {
          if (sample->stamp > deadline) continue;
          if (last_background_sample_stamp_.isZero() || sample->stamp > last_background_sample_stamp_) {
            background_sample.reset(new Points);
            vox.setInputCloud(sample->points);
            vox.setLeafSize(0.05, 0.05, 0.05);
            vox.filter(*background_sample);
            last_background_sample_stamp_ = sample->stamp;
            append_sample = true;
          }
          break;
        }
        next_background_sample_stamp_ = deadline + ros::Duration(0.3);
      }
    }
    if (append_sample) {
      std::shared_ptr<pcl::search::KdTree<PointType>> tree;
      if (!background_sample->empty()) {
        tree = std::make_shared<pcl::search::KdTree<PointType>>();
        tree->setInputCloud(background_sample);
      }
      background_history_.push_back(tree);
      if (background_history_.size() > 12) background_history_.pop_front();
    }
  }

  Remaining_Points = ClusterPoints;

  PointsPtr All_Points(new Points);

  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Filter Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  std::cout << "Input Size:" << ClusterPoints->size() << std::endl;

  sensor_msgs::PointCloud2 map_ros;
  PointsPtr OutputPoints(new Points);
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*OutputPoints);

  pcl::toROSMsg(*OutputPoints, map_ros);
  map_ros.header.frame_id = frame_id_;
  map_ros.header.stamp = cloud_stamp_;
  mapPub.publish(map_ros);

  // DBSCAN Cluster
  if (!previous_points.empty()) removeStaticPlanes(ClusterPoints, previous_points.back());
  std::vector<pcl::PointIndices> cluster_indices;
  dbscan.setCorePointMinPts(mp_->core_pts);
  dbscan.setClusterTolerance(mp_->tolerance);
  dbscan.setMinClusterSize(mp_->min_cluster);
  dbscan.setMaxClusterSize(mp_->max_cluster);
  dbscan.setInputCloud(ClusterPoints);
  dbscan.setSearchMethod();
  dbscan.extractNano(cluster_indices);

  std::cout << "Cluster size: " << cluster_indices.size() << std::endl;
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Cluster Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  if (frame_num == 2){
    previous_points.push_back(cloud_world);
    return;
  }

  // During the static-reference capture window, keep the four dynamic-output
  // topics alive with empty, timestamped messages. Dynamic avoidance starts
  // only after the reference is frozen; no calibration fragment is published.
  if (reference_points_ && !reference_tree_) {
    // Keep the registered-map bootstrap valid while the dynamic detector is
    // intentionally bypassed. The frame-2 compatibility early return above
    // remains unchanged, so existing output timestamp sets stay aligned.
    previous_points.clear();
    previous_points.push_back(cloud_world);
    detections.clear();
    deleted_indices.clear();
    sensor_msgs::PointCloud2 dynamic_pts;
    dynamic_pts.header.frame_id = frame_id_;
    dynamic_pts.header.stamp = cloud_stamp_;
    cloudPub.publish(dynamic_pts);
    obj_state_msgs::ObjectsStates states;
    states.header.stamp = cloud_stamp_;
    states.header.frame_id = frame_id_;
    statesPub.publish(states);
    visualizeFunction({});
    trackers.clear();
    std::cout << "Tracking Time Cost (ms)： 0" << std::endl;
    return;
  }

  int pt_size = previous_points[0]->points.size();
  //ikd-tree
  if (ikdtree_ptr->Root_Node == nullptr) {
    ikdtree_ptr->Build(previous_points[0]->points);
    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  else {
    PointVector PointNoNeedDownsample;
    bool need_add = true;

    for (int i = 0; i < pt_size; ++i) {
      PointType mid_point;
      mid_point.x = floor(previous_points[0]->points[i].x/0.1)*0.1 + 0.5 * 0.1;
      mid_point.y = floor(previous_points[0]->points[i].y/0.1)*0.1 + 0.5 * 0.1;
      mid_point.z = floor(previous_points[0]->points[i].z/0.1)*0.1 + 0.5 * 0.1;
      // float dist  = calc_dist(cloud_filtered->points[i],mid_point);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(mid_point, 1, points_near, pointSearchSqDis);

      if (points_near.empty()) continue;

      if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.1 ||
          fabs(points_near[0].y - mid_point.y) > 0.5 * 0.1 ||
          fabs(points_near[0].z - mid_point.z) > 0.5 * 0.1) {
        PointNoNeedDownsample.push_back(previous_points[0]->points[i]);
        PointToAdd->points.push_back(previous_points[0]->points[i]);
      }
    }
    // int add_point_size = ikdtree_ptr->Add_Points(PointToAdd, true);
    ikdtree_ptr->Add_Points(PointNoNeedDownsample, false);
    input_point.push_back(PointToAdd);
    if (input_point.size() > 25) {
      PointVector PointDelete;
      for (auto& delet_pt: input_point[0]->points) {
        PointDelete.push_back(delet_pt);
      }
      ikdtree_ptr->Delete_Points(PointDelete);
      input_point.pop_front();
    }

    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "ikd-Tree Time Cost (ms)： " << compTime << std::endl;
  tic = std::chrono::high_resolution_clock::now();
  std::cout << "ikd-Tree Size: " << ikdtree_ptr->size() << std::endl;
  // ikdtree_ptr->flatten(ikdtree_ptr->Root_Node, ikdtree_ptr->PCL_Storage, NOT_RECORD);
  // All_Points->points = ikdtree_ptr->PCL_Storage;
  // All_Points->width = All_Points->points.size();
  // All_Points->height = 1;
  // All_Points->is_dense = true;

  // sensor_msgs::PointCloud2 previous_cloud;
  // pcl::toROSMsg(*All_Points, previous_cloud);
  // previous_cloud.header.frame_id = "world";
  // cloudPub.publish(previous_cloud);

  PointsPtr dynamic_points(new Points);

  struct Candidate {
    ObjectState state;
    pcl::PointIndices indices;
    pcl::PointIndices dynamic_indices;
    bool supplemental;
    bool object_component;
    int parent_key;
    bool changed;
    bool static_background;
    bool reference_foreground;
    double novel_fraction;
    double occlusion_fraction;
    double bilateral_foreground_fraction;
    double independent_motion_fraction;
    double static_plane_fraction;
    double bidirectional_forward_fraction = 0.0;
    double bidirectional_forward_static_fraction = 0.0;
    bool bidirectional_forward_valid = false;
    size_t dense_count;
    int background_supported_frames;
    Eigen::Vector3d dense_size;
    int dense_point_index;
    std::vector<Eigen::Vector3d> dense_points;
  };
  DepthView depth_views[2];
  BidirectionalDepth current_depth_view, previous_depth_view;
  const RawObservation previous_ray = bidirectional_previous_observation_;
  const bool need_bidirectional = mp_->bidirectional_ray_evidence_enabled ||
      mp_->object_motion_evidence_enabled ||
      mp_->bidirectional_static_suppression_enabled || mp_->candidate_partition_enabled ||
      (mp_->translated_startup_enabled && !mp_->time_evidence_enabled);
  pcl::search::KdTree<PointType> observation_trees[2];
  bool has_observed_history[2];
  for (int history = 0; history < 2; ++history) {
    has_observed_history[history] = observed_history[history] &&
                                   !observed_history[history]->empty();
    if (has_observed_history[history])
      observation_trees[history].setInputCloud(observed_history[history]);
    if ((mp_->depth_evidence_enabled || mp_->occlusion_startup_enabled ||
         mp_->strict_depth_evidence_enabled || mp_->online_background_enabled ||
         mp_->reference_strong_two_frame_enabled) && has_observed_history[history])
      depth_views[history].build(*observed_history[history], sensor_history[history]);
  }
  const bool have_previous_ray = previous_ray.points && !previous_ray.points->empty() &&
      previous_ray.stamp < cloud_stamp_;
  pcl::search::KdTree<PointType> object_motion_previous_tree;
  if (mp_->object_motion_evidence_enabled && have_previous_ray)
    object_motion_previous_tree.setInputCloud(previous_ray.points);
  if (need_bidirectional) {
    MapData current_sensor_pose;
    current_sensor_pose.R = md_->R;
    current_sensor_pose.T = md_->T + md_->R * mp_->lidar_offset;
    current_depth_view.build(*latest_cloud, current_sensor_pose.R, current_sensor_pose.T);
    if (have_previous_ray)
      previous_depth_view.build(*previous_ray.points, previous_ray.sensor_pose.R, previous_ray.sensor_pose.T);
    bidirectional_previous_observation_ = {cloud_stamp_, latest_cloud, current_sensor_pose};
  }
  std::vector<Candidate> candidates;
  std::vector<bool> object_components(cluster_indices.size(), false);
  if (mp_->object_candidate_split_enabled) {
    // Preserve each original candidate and its measured center. Propose only
    // other measured components, each subject to the ordinary pipeline with
    // its own evidence; no component inherits the parent's confirmation.
    DensityPartitionOptions options;
    options.radius = mp_->core_radius;
    options.alpha = mp_->core_density_fraction;
    options.min_cluster = mp_->min_cluster;
    options.max_cluster = mp_->max_cluster;
    options.core_pts = mp_->core_pts;
    options.squared_tolerance = mp_->tolerance;
    const auto parents = cluster_indices;
    for (const auto& parent : parents) {
      const auto partition = densityPartition(*ClusterPoints, parent, options);
      if (!partition.split) continue;
      int dense_anchor = parent.indices.front();
      size_t dense_count = 0;
      for (int anchor : parent.indices) {
        size_t count = 0;
        for (int index : parent.indices)
          if ((ClusterPoints->points[index].getVector3fMap() -
               ClusterPoints->points[anchor].getVector3fMap()).squaredNorm() <= mp_->tolerance)
            ++count;
        if (count > dense_count) { dense_count = count; dense_anchor = anchor; }
      }
      size_t added = 0;
      for (const auto& component : partition.clusters) {
        if (std::find(component.indices.begin(), component.indices.end(), dense_anchor) !=
            component.indices.end()) continue;
        cluster_indices.push_back(component);
        object_components.push_back(true);
        ++added;
      }
      ROS_DEBUG_STREAM("FAPP_OBJECT_CANDIDATES " << cloud_stamp_.toNSec() << " "
          << parent.indices.size() << " " << partition.clusters.size() << " "
          << partition.leftovers.indices.size() << " " << added);
    }
  }
  detections.clear();
  deleted_indices.clear();
  std::vector<pcl::PointIndices> dynamic_parents;
  std::vector<int> candidate_parent_keys;
  if (mp_->candidate_partition_enabled) {
    // Keep every original parent candidate. Additional measured components
    // build their own observation history and never inherit confirmation.
    DensityPartitionOptions options;
    options.radius = mp_->core_radius;
    options.alpha = mp_->core_density_fraction;
    options.min_cluster = mp_->min_cluster;
    options.max_cluster = mp_->max_cluster;
    options.core_pts = mp_->core_pts;
    options.squared_tolerance = mp_->tolerance;
    const auto parents = cluster_indices;
    dynamic_parents = parents;
    for (size_t parent_index = 0; parent_index < parents.size(); ++parent_index)
      candidate_parent_keys.push_back(parent_index);
    for (size_t parent_index = 0; parent_index < parents.size(); ++parent_index) {
      const auto& parent = parents[parent_index];
      const auto partition = densityPartition(*ClusterPoints, parent, options);
      if (partition.split) {
        // Use the same first densest-neighborhood anchor as the original
        // parent measurement. Its component is already represented by that
        // parent and must not be proposed a second time.
        int dense_anchor = parent.indices.front();
        size_t dense_count = 0;
        for (int anchor : parent.indices) {
          size_t count = 0;
          for (int index : parent.indices)
            if ((ClusterPoints->points[index].getVector3fMap() -
                 ClusterPoints->points[anchor].getVector3fMap()).squaredNorm() <= mp_->tolerance)
              ++count;
          if (count > dense_count) {
            dense_count = count;
            dense_anchor = anchor;
          }
        }
        size_t supplemental_count = 0;
        for (const auto& component : partition.clusters) {
          if (std::find(component.indices.begin(), component.indices.end(), dense_anchor) !=
              component.indices.end()) continue;
          cluster_indices.push_back(component);
          object_components.push_back(object_components[parent_index]);
          dynamic_parents.push_back(parent);
          candidate_parent_keys.push_back(parent_index);
          ++supplemental_count;
        }
        ROS_DEBUG_STREAM("FAPP_CANDIDATE_PARTITION " << cloud_stamp_.toNSec() << " "
            << parent.indices.size() << " " << partition.clusters.size() << " "
            << partition.leftovers.indices.size() << " " << supplemental_count);
      }
    }
  }
  size_t candidate_cluster_index = 0;
  for (const auto& indices : cluster_indices) {
    const auto& dynamic_indices = mp_->candidate_partition_enabled ?
        dynamic_parents[candidate_cluster_index] : indices;
    const int parent_key = mp_->candidate_partition_enabled ?
        candidate_parent_keys[candidate_cluster_index] : -1;
    const bool object_component = object_components[candidate_cluster_index++];
    const bool supplemental = mp_->candidate_partition_enabled &&
        indices.indices != dynamic_indices.indices;
    const bool candidate_ray_enabled = mp_->object_motion_evidence_enabled ||
        mp_->bidirectional_ray_evidence_enabled ||
        mp_->bidirectional_static_suppression_enabled || supplemental ||
        (mp_->translated_startup_enabled && !mp_->time_evidence_enabled);
    Points cluster;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    std::vector<double> distances;
    std::vector<double> reference_distances;
    if (mp_->dense_change_evidence_enabled) reference_distances.reserve(indices.indices.size());
    for (int index : indices.indices) {
      const auto& point = ClusterPoints->points[index];
      cluster.push_back(point);
      center += Eigen::Vector3d(point.x, point.y, point.z);
      PointVector nearest;
      std::vector<float> squared_distances(3);
      ikdtree_ptr->Nearest_Search(point, 3, nearest, squared_distances);
      const double distance = nearest.empty() ? std::numeric_limits<double>::quiet_NaN() :
          std::sqrt(squared_distances[0]);
      if (!nearest.empty()) distances.push_back(distance);
      if (mp_->dense_change_evidence_enabled) reference_distances.push_back(distance);
    }
    center /= cluster.size();
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cluster, min_pt, max_pt);
    Eigen::Vector3d size(max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);

    // Use the densest existing DBSCAN neighborhood for the measured center.
    // Sparse attached rope/edge points still contribute to the output cloud
    // and box, but cannot pull the motion measurement away from its dense part.
    size_t dense_index = 0, dense_count = 0;
    for (size_t a = 0; a < cluster.size(); ++a) {
      size_t count = 0;
      for (const auto& point : cluster) {
        if ((point.getVector3fMap() - cluster[a].getVector3fMap()).squaredNorm() <=
            mp_->tolerance) ++count;
      }
      if (count > dense_count) {
        dense_count = count;
        dense_index = a;
      }
    }
    Eigen::Vector3d dense_size;
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<float> coordinates;
      coordinates.reserve(dense_count);
      for (const auto& point : cluster) {
        if ((point.getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() <=
            mp_->tolerance) coordinates.push_back(point.data[axis]);
      }
      std::sort(coordinates.begin(), coordinates.end());
      dense_size[axis] = coordinates.back() - coordinates.front();
      size_t middle = coordinates.size() / 2;
      center[axis] = coordinates.size() % 2 ? coordinates[middle] :
          0.5 * (coordinates[middle - 1] + coordinates[middle]);
    }

    // Use the same bottom as the published box; keep boxes crossing the limit.
    if (mp_->max_above_uav >= 0.0 &&
        center.z() - 0.5 * size.z() > md_->T.z() + mp_->max_above_uav) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " height");
      continue;
    }

    // Judge the same dense surface used for motion. An attached sparse rope
    // must not make a curved moving return look like a long planar patch.
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    size_t novel_points[2] = {0, 0};
    size_t independent_points[2] = {0, 0};
    size_t bilateral_foreground = 0, online_supported = 0;
    size_t reference_supported = 0;
    size_t ray_supported[2] = {0, 0}, ray_foreground[2] = {0, 0};
    size_t bidirectional_forward = 0, bidirectional_forward_static = 0;
    std::vector<int> observed_nearest(1);
    std::vector<float> observed_squared_distance(1);
    for (const auto& point : cluster) {
      if ((point.getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() >
          mp_->tolerance) continue;
      Eigen::Vector3d delta = Eigen::Vector3d(point.x, point.y, point.z) - center;
      covariance += delta * delta.transpose();
      if (reference_tree_ &&
          reference_tree_->nearestKSearch(point, 1, observed_nearest, observed_squared_distance) > 0 &&
          observed_squared_distance[0] <= mp_->reference_distance * mp_->reference_distance)
        ++reference_supported;
      if (online_background_tree_ &&
          online_background_tree_->nearestKSearch(point, 1, observed_nearest,
                                                  observed_squared_distance) > 0 &&
          observed_squared_distance[0] <= mp_->reference_distance * mp_->reference_distance)
        ++online_supported;
      int ray_evidence[2] = {-1, -1};
      for (int history = 0; history < 2; ++history) {
        if (!has_observed_history[history]) continue;
        int evidence = (mp_->depth_evidence_enabled || mp_->occlusion_startup_enabled ||
                        mp_->strict_depth_evidence_enabled || mp_->reference_strong_two_frame_enabled) ?
            depth_views[history].classify(point) : -1;
        ray_evidence[history] = evidence;
        if (evidence == 0) ++ray_supported[history];
        if (evidence == 1) ++ray_foreground[history];
        const int novelty_evidence = (mp_->depth_evidence_enabled || mp_->strict_depth_evidence_enabled) ? evidence : -1;
        const bool independent_enabled = mp_->point_source_competition_enabled ||
                                         mp_->bidirectional_static_suppression_enabled;
        // Both statistics query the same point in the same immutable raw
        // observation. Reuse the result without changing either criterion.
        bool nearest_novel = false;
        if (independent_enabled || novelty_evidence < 0)
          nearest_novel = observation_trees[history].nearestKSearch(point, 1, observed_nearest,
              observed_squared_distance) == 0 ||
              observed_squared_distance[0] > mp_->thresh_dist * mp_->thresh_dist;
        if (independent_enabled && nearest_novel)
          ++independent_points[history];
        bool novel = novelty_evidence == 1 || (novelty_evidence < 0 && nearest_novel);
        if (novel) ++novel_points[history];
      }
      if (candidate_ray_enabled && have_previous_ray) {
        const auto evidence = previous_depth_view.classify(point.getVector3fMap().cast<double>());
        if (evidence == BidirectionalDepth::Foreground) ++bidirectional_forward;
        if (evidence == BidirectionalDepth::Static) ++bidirectional_forward_static;
      }
      if (ray_evidence[0] == 1 && ray_evidence[1] == 1) ++bilateral_foreground;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    const auto& eigenvalues = solver.eigenvalues();
    if (size.maxCoeff() >= 0.5 && eigenvalues[0] < 0.01 * eigenvalues[2]) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " large_plane");
      continue;
    }

    // Small sampled fragments can move along an unchanged surface. Require
    // support in both raw observations, and only reject a thin dense return.
    bool static_surface = mp_->background_surface_enabled &&
                          eigenvalues[0] < 0.01 * eigenvalues[2];
    PointType query;
    query.x = center.x();
    query.y = center.y();
    query.z = center.z();
    for (int history = 0; static_surface && history < 2; ++history) {
      std::vector<int> local_indices;
      std::vector<float> local_squared_distances;
      if (!has_observed_history[history] ||
          observation_trees[history].radiusSearch(query, 0.35, local_indices,
                                                  local_squared_distances) < 15) {
        static_surface = false;
        break;
      }
      Eigen::Vector3d local_center = Eigen::Vector3d::Zero();
      for (int index : local_indices)
        local_center += observed_history[history]->points[index].getVector3fMap().cast<double>();
      local_center /= local_indices.size();
      Eigen::Matrix3d local_covariance = Eigen::Matrix3d::Zero();
      for (int index : local_indices) {
        Eigen::Vector3d delta = observed_history[history]->points[index].getVector3fMap().cast<double>() - local_center;
        local_covariance += delta * delta.transpose();
      }
      local_covariance /= local_indices.size();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> local_solver(local_covariance);
      const auto& local_eigenvalues = local_solver.eigenvalues();
      if (local_eigenvalues[0] >= 0.01 * local_eigenvalues[2] ||
          local_eigenvalues[1] <= 0.001) {
        static_surface = false;
        break;
      }
      Eigen::Vector3d normal = local_solver.eigenvectors().col(0);
      size_t supported = 0;
      for (const auto& point : cluster) {
        if ((point.getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() >
            mp_->tolerance) continue;
        Eigen::Vector3d delta = point.getVector3fMap().cast<double>() - local_center;
        if (std::abs(normal.dot(delta)) <= 0.03) ++supported;
      }
      if (supported < 0.8 * dense_count) static_surface = false;
    }
    if (static_surface) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " static_surface");
      continue;
    }

    // A changing sample on a broad, repeatedly observed plane is background
    // even if the small current fragment alone does not look planar. Only
    // actual historical neighborhoods qualify; no unbounded plane extension.
    bool stable_background = false;
    bool local_background = false;
    int supported_frames = 0;
    double static_plane_fraction = 0.0;
    if (mp_->background_consistency_enabled) {
      int local_plane_frames = 0;
      for (const auto& tree : background_history) {
        if (!tree) continue;
        std::vector<int> local_indices;
        std::vector<float> local_distances;
        if (tree->radiusSearch(query, 0.35, local_indices, local_distances) < 20) continue;
        auto points = tree->getInputCloud();
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (int index : local_indices) mean += points->points[index].getVector3fMap().cast<double>();
        mean /= local_indices.size();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int index : local_indices) {
          Eigen::Vector3d d = points->points[index].getVector3fMap().cast<double>() - mean;
          cov += d * d.transpose();
        }
        cov /= local_indices.size();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> fit(cov);
        bool broad_plane = fit.eigenvalues()[0] < 0.01 * fit.eigenvalues()[2] &&
                           fit.eigenvalues()[1] >= 0.005;
        auto dense_supported = [&](const Eigen::Vector3d& normal,
                                   const Eigen::Vector3d& anchor) {
          size_t supported = 0;
          for (const auto& point : cluster) {
            if ((point.getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() >
                mp_->tolerance) continue;
            if (std::abs(normal.dot(point.getVector3fMap().cast<double>() - anchor)) <=
                mp_->background_consistency_distance) ++supported;
          }
          if (mp_->point_source_competition_enabled ||
              mp_->bidirectional_static_suppression_enabled)
            static_plane_fraction = std::max(static_plane_fraction,
                static_cast<double>(supported) / dense_count);
          return supported >= mp_->background_consistency_fraction * dense_count;
        };
        if (broad_plane) {
          if (dense_supported(fit.eigenvectors().col(0), mean)) ++supported_frames;
          continue;
        }
        if (!mp_->background_local_planes_enabled) continue;

        // Separate at most two historical surfaces at corners. This fit only
        // supplies background evidence; it never clips current target points.
        PointsPtr remaining(new Points);
        for (int index : local_indices) remaining->push_back(points->points[index]);
        const PointsPtr neighborhood = remaining;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> surfaces;
        pcl::SACSegmentation<PointType> segmentation;
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(50);
        segmentation.setDistanceThreshold(mp_->background_consistency_distance);
        for (int plane = 0; plane < 2 && remaining->size() >= 6; ++plane) {
          pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
          pcl::ModelCoefficients coefficients;
          segmentation.setInputCloud(remaining);
          segmentation.segment(*inliers, coefficients);
          if (inliers->indices.size() < 6 || coefficients.values.size() != 4) break;
          Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
          double scale = normal.norm();
          if (scale == 0.0) break;
          normal /= scale;
          // The first extraction removes the corner's shared edge too. Count
          // each plane's support in the original neighborhood, not the residue.
          std::vector<int> support;
          for (size_t index = 0; index < neighborhood->size(); ++index)
            if (std::abs(normal.dot(neighborhood->points[index].getVector3fMap().cast<double>()) +
                         coefficients.values[3] / scale) <= mp_->background_consistency_distance)
              support.push_back(index);
          Eigen::Vector3d plane_mean = Eigen::Vector3d::Zero();
          Eigen::Matrix3d plane_cov = Eigen::Matrix3d::Zero();
          if (support.size() >= 20) {
            for (int index : support)
              plane_mean += neighborhood->points[index].getVector3fMap().cast<double>();
            plane_mean /= support.size();
            for (int index : support) {
              Eigen::Vector3d d = neighborhood->points[index].getVector3fMap().cast<double>() - plane_mean;
              plane_cov += d * d.transpose();
            }
            plane_cov /= support.size();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> plane_fit(plane_cov);
            if (plane_fit.eigenvalues()[0] < 0.01 * plane_fit.eigenvalues()[2] &&
                plane_fit.eigenvalues()[1] >= 0.005)
              surfaces.push_back({plane_fit.eigenvectors().col(0), plane_mean});
          }
          pcl::ExtractIndices<PointType> extract;
          extract.setInputCloud(remaining);
          extract.setIndices(inliers);
          extract.setNegative(true);
          PointsPtr next(new Points);
          extract.filter(*next);
          remaining = next;
        }
        size_t supported = 0;
        if (mp_->point_source_competition_enabled ||
            mp_->bidirectional_static_suppression_enabled)
          for (const auto& surface : surfaces) dense_supported(surface.first, surface.second);
        for (const auto& point : cluster) {
          if ((point.getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() >
              mp_->tolerance) continue;
          if (std::any_of(surfaces.begin(), surfaces.end(), [&](const auto& surface) {
                return std::abs(surface.first.dot(point.getVector3fMap().cast<double>() - surface.second)) <=
                       mp_->background_consistency_distance;
              })) ++supported;
        }
        if (supported >= mp_->background_consistency_fraction * dense_count) {
          ++supported_frames;
          ++local_plane_frames;
        }
      }
      if (mp_->background_local_planes_enabled)
        ROS_DEBUG_STREAM("FAPP_LOCAL_BACKGROUND " << cloud_stamp_.toNSec() << " "
            << center.transpose() << " " << local_plane_frames);
      ROS_DEBUG_STREAM("FAPP_BACKGROUND " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " " << supported_frames << " " << background_history.size());
      if (supported_frames >= mp_->background_consistency_frames) {
        ROS_DEBUG_STREAM("FAPP_BACKGROUND_MATCH " << cloud_stamp_.toNSec() << " "
            << center.transpose() << " stable_background");
        stable_background = true;
        local_background = supported_frames - local_plane_frames < mp_->background_consistency_frames;
      }
    }

    if (stable_background && !mp_->background_consistency_post_association &&
        !(local_background && mp_->background_local_post_association)) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " stable_background_before_association");
      continue;
    }
    double avg_dist = 0.0, var_dist = 0.0;
    for (double distance : distances) avg_dist += distance;
    if (!distances.empty()) avg_dist /= distances.size();
    if (avg_dist > 0.0) {
      for (double distance : distances)
        var_dist += (distance - avg_dist) * (distance - avg_dist) / (avg_dist * avg_dist);
      var_dist /= distances.size();
    }
    const bool parent_changed = avg_dist > mp_->thresh_dist && avg_dist < 5.0 &&
                                var_dist < mp_->thresh_var;
    if (mp_->dense_change_evidence_enabled) {
      const double parent_mean = avg_dist, parent_variance = var_dist;
      size_t dense_reference_count = 0;
      avg_dist = 0.0;
      var_dist = 0.0;
      // Reuse the same real nearest-map queries. Motion center and novelty
      // already use this dense neighborhood; a sparse parent tail must not
      // supply a different source for the changed/startup decision.
      for (size_t point = 0; point < cluster.size(); ++point) {
        if ((cluster[point].getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() <=
            mp_->tolerance && std::isfinite(reference_distances[point])) {
          avg_dist += reference_distances[point];
          ++dense_reference_count;
        }
      }
      if (dense_reference_count > 0) avg_dist /= dense_reference_count;
      if (avg_dist > 0.0) {
        for (size_t point = 0; point < cluster.size(); ++point) {
          if ((cluster[point].getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() <=
              mp_->tolerance && std::isfinite(reference_distances[point])) {
            const double difference = reference_distances[point] - avg_dist;
            var_dist += difference * difference / (avg_dist * avg_dist);
          }
        }
        var_dist /= dense_reference_count;
      }
      ROS_DEBUG_STREAM("FAPP_DENSE_CHANGE " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " " << distances.size() << " " << dense_reference_count
          << " " << parent_mean << " " << parent_variance << " " << avg_dist << " " << var_dist);
    }
    Candidate candidate;
    candidate.state.position = center;
    candidate.static_background = stable_background;
    candidate.background_supported_frames = supported_frames;
    const double reference_fraction = static_cast<double>(reference_supported) / dense_count;
    const bool reference_background = reference_tree_ && reference_fraction >= mp_->reference_fraction;
    candidate.reference_foreground = reference_tree_ && !reference_background;
    if (mp_->background_reference_enabled && reference_background)
      candidate.static_background = true;
    const bool online_background_match = online_background_tree_ &&
        static_cast<double>(online_supported) / dense_count >= mp_->reference_fraction;
    if (online_background_match) {
      candidate.static_background = true;
      candidate.reference_foreground = false;
    }
    if (mp_->online_background_enabled)
      ROS_DEBUG_STREAM("FAPP_ONLINE_MATCH " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " " << online_supported << " " << dense_count << " "
          << online_background_match);
    if (reference_tree_)
      ROS_DEBUG_STREAM("FAPP_REFERENCE " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " " << dense_count << " " << reference_supported << " "
          << reference_background);
    candidate.state.size = size;
    candidate.indices = indices;
    if (mp_->candidate_partition_enabled) candidate.dynamic_indices = dynamic_indices;
    candidate.supplemental = supplemental;
    candidate.object_component = object_component;
    candidate.parent_key = parent_key;
    candidate.dense_size = dense_size;
    candidate.dense_count = dense_count;
    candidate.dense_point_index = indices.indices[dense_index];
    candidate.occlusion_fraction = static_cast<double>(std::min(ray_foreground[0], ray_foreground[1])) /
                                   dense_count;
    candidate.bilateral_foreground_fraction = static_cast<double>(bilateral_foreground) / dense_count;
    candidate.independent_motion_fraction = static_cast<double>(std::min(independent_points[0], independent_points[1])) / dense_count;
    candidate.static_plane_fraction = static_plane_fraction;
    candidate.bidirectional_forward_valid = candidate_ray_enabled && have_previous_ray && dense_count > 0;
    if (candidate.bidirectional_forward_valid) {
      candidate.bidirectional_forward_fraction = static_cast<double>(bidirectional_forward) / dense_count;
      candidate.bidirectional_forward_static_fraction =
          static_cast<double>(bidirectional_forward_static) / dense_count;
    }
    if (mp_->shape_consistency_enabled || mp_->translated_startup_enabled ||
        mp_->reference_strong_two_frame_enabled || candidate_ray_enabled) {
      for (const auto& point : cluster) {
        if ((point.getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() <=
            mp_->tolerance)
          candidate.dense_points.push_back(point.getVector3fMap().cast<double>() - center);
      }
    }
    candidate.novel_fraction = static_cast<double>(std::max(novel_points[0],
                                                          novel_points[1])) / dense_count;
    candidate.changed = avg_dist > mp_->thresh_dist && avg_dist < 5.0 &&
                        var_dist < mp_->thresh_var;
    // Dense evidence only verifies the original parent change. It cannot
    // authorize an additional startup that the original parent rejected.
    if (mp_->dense_change_evidence_enabled) candidate.changed = candidate.changed && parent_changed;
    ROS_DEBUG_STREAM("FAPP_CANDIDATE " << cloud_stamp_.toNSec() << " "
        << center.x() << " " << center.y() << " " << center.z() << " "
        << size.x() << " " << size.y() << " " << size.z() << " "
        << cluster.size() << " " << avg_dist << " " << var_dist);
    ROS_DEBUG_STREAM("FAPP_EVIDENCE " << cloud_stamp_.toNSec() << " "
        << center.transpose() << " " << dense_count << " "
        << novel_points[0] << " " << novel_points[1] << " "
        << ray_supported[0] << " " << ray_supported[1] << " "
        << ray_foreground[0] << " " << ray_foreground[1]);
    if (mp_->point_source_competition_enabled)
      ROS_DEBUG_STREAM("FAPP_POINT_SOURCE " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " " << candidate.independent_motion_fraction << " "
          << candidate.static_plane_fraction);
    if (candidate_ray_enabled)
      ROS_DEBUG_STREAM("FAPP_BIDIRECTIONAL_CANDIDATE " << cloud_stamp_.toNSec() << " "
          << center.transpose() << " " << candidate.bidirectional_forward_valid << " "
          << candidate.bidirectional_forward_fraction << " "
          << candidate.bidirectional_forward_static_fraction);
    candidates.push_back(candidate);
  }

  if (online_background_) {
    auto key = [](const PointType& point, double size) {
      return std::to_string(static_cast<long long>(std::floor(point.x / size))) + "," +
             std::to_string(static_cast<long long>(std::floor(point.y / size))) + "," +
             std::to_string(static_cast<long long>(std::floor(point.z / size)));
    };
    // Learn the unmodified pre-plane cloud. Current candidate voxels never
    // become background, including candidates not yet confirmed as moving.
    std::unordered_set<std::string> excluded_voxels, visible_voxels;
    for (const auto& candidate : candidates)
      for (int index : candidate.indices.indices)
        excluded_voxels.insert(key(ClusterPoints->points[index], mp_->online_voxel_size));
    std::vector<uint8_t> excluded(OutputPoints->size(), 0);
    for (size_t index = 0; index < OutputPoints->size(); ++index) {
      const auto& point = OutputPoints->points[index];
      if (!point.getVector3fMap().allFinite()) { excluded[index] = 1; continue; }
      visible_voxels.insert(key(point, 0.2));
      excluded[index] = excluded_voxels.count(key(point, mp_->online_voxel_size)) > 0;
      // Unmatched startup-reference points alone are unknown new scenery.
      // Actual foreground in both historical radar views must never be learned.
      if (!excluded[index] && has_observed_history[0] && has_observed_history[1] &&
          depth_views[0].classify(point) == 1 && depth_views[1].classify(point) == 1)
        excluded[index] = 1;
      for (const auto& tracker : trackers) {
        if (excluded[index]) break;
        if (tracker->last_published_stamp.isZero() || tracker->observations.empty() ||
            (cloud_stamp_ - tracker->last_published_stamp).toSec() > 0.6) continue;
        const Eigen::Vector3d distance = (point.getVector3fMap().cast<double>() -
            tracker->observations.back().second).cwiseAbs();
        if ((distance.array() <= (0.5 * tracker->observed_dense_size.array() + 0.15)).all())
          excluded[index] = 1;
      }
    }
    double rotation_rate = 0.0, translation = 0.0, overlap = 1.0;
    if (online_has_previous_pose_) {
      const Eigen::Matrix3d rotation = online_previous_pose_.R.transpose() * md_->R;
      rotation_rate = std::acos(std::max(-1.0, std::min(1.0, (rotation.trace() - 1.0) / 2.0))) /
          std::max(frame_dt, 1e-6);
      translation = (md_->T - online_previous_pose_.T).norm();
      size_t common = 0;
      for (const auto& voxel : visible_voxels) common += online_previous_voxels_.count(voxel);
      if (!visible_voxels.empty() && !online_previous_voxels_.empty())
        overlap = static_cast<double>(common) /
            std::min(visible_voxels.size(), online_previous_voxels_.size());
    }
    const bool paused = frame_dt <= 0.0 || frame_dt > 0.4 ||
        rotation_rate > mp_->online_rotation_rate || translation > mp_->online_jump_distance ||
        overlap < mp_->online_scene_overlap;
    const auto update = online_background_->observe(*OutputPoints, excluded,
        cloud_stamp_.toSec(), paused, md_->T);
    online_previous_pose_ = *md_;
    online_has_previous_pose_ = true;
    online_previous_voxels_ = std::move(visible_voxels);
    if (update.changed) {
      online_background_tree_.reset();
      if (!online_background_->acceptedCloud()->empty()) {
        online_background_tree_ = std::make_shared<pcl::search::KdTree<PointType>>();
        online_background_tree_->setInputCloud(online_background_->acceptedCloud());
      }
    }
    ROS_DEBUG_STREAM("FAPP_ONLINE_UPDATE " << cloud_stamp_.toNSec() << " "
        << update.added << " " << update.removed << " " << update.accepted << " "
        << update.pending << " " << paused << " " << rotation_rate << " "
        << translation << " " << overlap);
  }

  // Match closest pairs once each; multiple detections must not update one track.
  struct Match {
    bool stale;
    double distance;
    int candidate, tracker;
  };
  std::vector<Match> possible_pairs;
  std::vector<bool> association_stale(trackers.size(), true);
  for (size_t j = 0; j < trackers.size(); ++j) {
    const auto& tracker = trackers[j];
    association_stale[j] = tracker->last_seen_frame != frame_num - 1;
    if (mp_->confirmed_reacquisition_enabled && association_stale[j] &&
        !tracker->last_published_stamp.isZero()) {
      const double observed_gap = (cloud_stamp_ - tracker->observations.back().first).toSec();
      const double published_gap = (cloud_stamp_ - tracker->last_published_stamp).toSec();
      // A recently published track may compete by measured distance with
      // continuous returns. An unconfirmed wall fragment must not steal its
      // reappearing foreground merely because the foreground was occluded.
      if (observed_gap > 0.0 && observed_gap <= mp_->reacquisition_max_gap &&
          published_gap >= 0.0 && published_gap <= mp_->reacquisition_max_gap)
        association_stale[j] = false;
    }
  }
  std::vector<size_t> competing_candidates(trackers.size(), 0);
  std::vector<bool> has_weak_return(trackers.size(), false), has_motion_return(trackers.size(), false);
  for (int i = 0; i < candidates.size(); ++i) {
    for (int j = 0; j < trackers.size(); ++j) {
      if (mp_->candidate_partition_enabled &&
          candidates[i].supplemental != trackers[j]->supplemental_candidate) continue;
      if (mp_->object_candidate_isolation_enabled &&
          candidates[i].object_component != trackers[j]->object_component) continue;
      double distance = (candidates[i].state.position -
                         trackers[j]->observations.back().second).norm();
      if (distance < 0.82) {
        double cost = distance;
        if (mp_->geometry_association_enabled)
          cost = 0.5 * distance +
                 0.5 * (candidates[i].state.position - trackers[j]->pos()).norm() +
                 0.25 * (candidates[i].dense_size - trackers[j]->observed_dense_size).norm();
        possible_pairs.push_back({association_stale[j],
                                  cost, i, j});
        ++competing_candidates[j];
        const bool weak = candidates[i].static_background ||
            (candidates[i].background_supported_frames >= 2 &&
             candidates[i].novel_fraction < mp_->min_motion_novel_fraction);
        has_weak_return[j] = has_weak_return[j] || weak;
        has_motion_return[j] = has_motion_return[j] || !weak;
      }
    }
  }
  // Only disambiguate candidates competing for an actually published,
  // continuously observed track. Keep every original pair available: the
  // evidence changes sorting, never the 0.82 m gate or minimum point count.
  if (mp_->competition_association_enabled) {
    for (auto& pair : possible_pairs) {
      const auto& tracker = trackers[pair.tracker];
      if (tracker->last_seen_frame != frame_num - 1 || competing_candidates[pair.tracker] < 2 ||
          !has_weak_return[pair.tracker] || !has_motion_return[pair.tracker] ||
          tracker->last_published_stamp.isZero() ||
          (cloud_stamp_ - tracker->last_published_stamp).toSec() > 0.6) continue;
      const auto& candidate = candidates[pair.candidate];
      const bool weak_motion_return = candidate.static_background ||
          (candidate.background_supported_frames >= 2 &&
           candidate.novel_fraction < mp_->min_motion_novel_fraction);
      const double count_difference = tracker->observed_dense_count == 0 ? 0.0 :
          static_cast<double>(std::max(candidate.dense_count, tracker->observed_dense_count) -
                              std::min(candidate.dense_count, tracker->observed_dense_count)) /
          std::max(candidate.dense_count, tracker->observed_dense_count);
      pair.distance += (weak_motion_return ? 0.30 : 0.0) + 0.20 * count_difference;
      ROS_DEBUG_STREAM("FAPP_COMPETITION " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " " << tracker->id << " "
          << weak_motion_return << " " << count_difference << " " << pair.distance);
    }
  }
  if (mp_->point_source_competition_enabled) {
    std::vector<bool> strong_return(trackers.size(), false);
    auto strong = [&](const Candidate& candidate) {
      return !candidate.static_background &&
          candidate.static_plane_fraction < mp_->background_consistency_fraction &&
          candidate.independent_motion_fraction >= mp_->min_motion_novel_fraction;
    };
    for (const auto& pair : possible_pairs)
      if (strong(candidates[pair.candidate])) strong_return[pair.tracker] = true;
    // Only remove the weak pairing to an actually published, continuous
    // track when another current candidate carries independent motion.
    // Other pairs and the weak candidate's ordinary new-track path remain.
    possible_pairs.erase(std::remove_if(possible_pairs.begin(), possible_pairs.end(),
        [&](const Match& pair) {
          const auto& tracker = trackers[pair.tracker];
          const auto& candidate = candidates[pair.candidate];
          const bool blocked = tracker->last_seen_frame == frame_num - 1 &&
              competing_candidates[pair.tracker] >= 2 && strong_return[pair.tracker] &&
              !tracker->last_published_stamp.isZero() &&
              (cloud_stamp_ - tracker->last_published_stamp).toSec() <= 0.6 &&
              candidate.static_plane_fraction >= mp_->background_consistency_fraction &&
              candidate.independent_motion_fraction < mp_->min_motion_novel_fraction;
          if (blocked)
            ROS_DEBUG_STREAM("FAPP_POINT_SOURCE_BLOCK " << cloud_stamp_.toNSec() << " "
                << candidate.state.position.transpose() << " " << tracker->id << " "
                << candidate.independent_motion_fraction << " " << candidate.static_plane_fraction);
          return blocked;
        }), possible_pairs.end());
  }
  // An old position from a swing must not steal a currently observed track.
  std::sort(possible_pairs.begin(), possible_pairs.end(),
      [](const Match& a, const Match& b) {
        if (a.stale != b.stale) return a.stale < b.stale;
        if (a.distance != b.distance) return a.distance < b.distance;
        if (a.candidate != b.candidate) return a.candidate < b.candidate;
        return a.tracker < b.tracker;
      });
  std::vector<int> matched(candidates.size(), -1);
  std::vector<bool> used(trackers.size(), false);
  if (mp_->global_assignment_enabled) {
    for (bool stale : {false, true}) {
      std::vector<int> rows, columns;
      std::vector<int> row_index(candidates.size(), -1), column_index(trackers.size(), -1);
      for (int i = 0; i < candidates.size(); ++i)
        if (matched[i] < 0) { row_index[i] = rows.size(); rows.push_back(i); }
      for (int j = 0; j < trackers.size(); ++j)
        if (!used[j] && association_stale[j] == stale) {
          column_index[j] = columns.size(); columns.push_back(j);
        }
      if (rows.empty() || columns.empty()) continue;
      double maximum_cost = 0.0;
      for (const auto& pair : possible_pairs)
        if (pair.stale == stale && row_index[pair.candidate] >= 0 &&
            column_index[pair.tracker] >= 0)
          maximum_cost = std::max(maximum_cost, pair.distance);
      const double unmatched_cost = (rows.size() + 1) * (maximum_cost + 1.0);
      const double forbidden_cost = 2.0 * (rows.size() + 1) * unmatched_cost;
      std::vector<std::vector<double>> costs(rows.size(),
          std::vector<double>(columns.size() + rows.size(), unmatched_cost));
      for (auto& row : costs)
        std::fill(row.begin(), row.begin() + columns.size(), forbidden_cost);
      for (const auto& pair : possible_pairs)
        if (pair.stale == stale && row_index[pair.candidate] >= 0 &&
            column_index[pair.tracker] >= 0)
          costs[row_index[pair.candidate]][column_index[pair.tracker]] = pair.distance;
      HungarianAlgorithm solver;
      std::vector<int> assignment;
      solver.Solve(costs, assignment);
      for (int row = 0; row < rows.size(); ++row) {
        const int column = assignment[row];
        if (column >= 0 && column < columns.size() && costs[row][column] < unmatched_cost) {
          matched[rows[row]] = columns[column];
          used[columns[column]] = true;
        }
      }
    }
  } else {
    for (const auto& pair : possible_pairs) {
      int i = pair.candidate, j = pair.tracker;
      if (matched[i] < 0 && !used[j]) {
        matched[i] = j;
        used[j] = true;
      }
    }
  }

  obj_state_msgs::ObjectsStates states;
  states.header.stamp = cloud_stamp_;
  states.header.frame_id = frame_id_;
  std::vector<std::pair<int, int>> published_pairs;
  std::set<int> published_parents;
  std::vector<bool> published_dynamic_indices;
  if (mp_->candidate_partition_enabled)
    published_dynamic_indices.assign(ClusterPoints->size(), false);
  for (int i = 0; i < candidates.size(); ++i) {
    int j = matched[i];
    const auto& candidate = candidates[i];
    bool recently_published = j >= 0 && trackers[j]->last_seen_frame == frame_num - 1 &&
        !trackers[j]->last_published_stamp.isZero() &&
        (cloud_stamp_ - trackers[j]->last_published_stamp).toSec() <= 0.6;
    const bool check_observed_shape = mp_->swing_only || mp_->object_motion_evidence_enabled;
    bool shape_consistent = true;
    std::vector<Eigen::Vector3d> swing_shape_points;
    Eigen::Vector3d lidar_direction = Eigen::Vector3d::Zero();
    if (mp_->shape_consistency_enabled) {
      if (check_observed_shape) {
        swing_shape_points = candidate.dense_points;
        // A six-point cluster can have fewer than six dense-neighborhood
        // points. Use its full measured shape for this sparse-only check.
        if (candidate.indices.indices.size() >= 6 && candidate.indices.indices.size() <= 7) {
          swing_shape_points.clear();
          for (int index : candidate.indices.indices)
            swing_shape_points.push_back(ClusterPoints->points[index].getVector3fMap().cast<double>() -
                                         candidate.state.position);
        }
      }
      const auto& current_shape = check_observed_shape ? swing_shape_points : candidate.dense_points;
      lidar_direction = candidate.state.position - (md_->T + md_->R * mp_->lidar_offset);
      if (lidar_direction.norm() > 0.0) lidar_direction.normalize();
      // Swing evidence also checks sparse returns and observations collected
      // before ordinary dynamic confirmation. General mode keeps its rules.
      // A substantially different viewing direction leaves shape unknown.
      const size_t minimum_shape_points = check_observed_shape ? 6 : 8;
      if (j >= 0 && (trackers[j]->last_seen_frame == frame_num - 1 ||
          (mp_->confirmed_reacquisition_enabled && !association_stale[j])) &&
          (check_observed_shape ||
           trackers[j]->last_dynamic_stamp == trackers[j]->observations.back().first) &&
          current_shape.size() >= minimum_shape_points &&
          (check_observed_shape ? trackers[j]->swing_shape_points.size() :
                            trackers[j]->observed_dense_points.size()) >= minimum_shape_points &&
          lidar_direction.dot(trackers[j]->observed_lidar_direction) >= std::cos(0.2617993877991494)) {
        const auto& previous = check_observed_shape ? trackers[j]->swing_shape_points :
                                               trackers[j]->observed_dense_points;
        size_t forward = 0, backward = 0;
        double squared_distance = mp_->shape_consistency_distance * mp_->shape_consistency_distance;
        for (const auto& current : current_shape) {
          if (std::any_of(previous.begin(), previous.end(), [&](const Eigen::Vector3d& old) {
              return (current - old).squaredNorm() <= squared_distance;
          })) ++forward;
        }
        for (const auto& old : previous) {
          if (std::any_of(current_shape.begin(), current_shape.end(),
              [&](const Eigen::Vector3d& current) {
                return (current - old).squaredNorm() <= squared_distance;
              })) ++backward;
        }
        double fraction = std::max(static_cast<double>(forward) / current_shape.size(),
                                   static_cast<double>(backward) / previous.size());
        ROS_DEBUG_STREAM("FAPP_SHAPE_SOURCE " << cloud_stamp_.toNSec() << " "
            << trackers[j]->id << " " << trackers[j]->last_seen_frame << " " << frame_num
            << " " << previous.size() << " " << current_shape.size() << " "
            << forward << " " << backward << " " << fraction);
        shape_consistent = fraction >= mp_->shape_consistency_fraction;
        ROS_DEBUG_STREAM("FAPP_SHAPE " << cloud_stamp_.toNSec() << " "
            << candidate.state.position.transpose() << " " << previous.size() << " "
            << current_shape.size() << " " << fraction << " " << shape_consistent);
      }
    }
    bool retained_gap = false;
    bool confirmed_reacquisition = false;
    bool consecutive_observation = j >= 0 && trackers[j]->last_seen_frame == frame_num - 1;
    const bool previous_reference_foreground = consecutive_observation &&
        trackers[j]->observed_reference_foreground;
    // A newly sampled part of a fixed plane can occlude a farther wall in
    // both histories. That ray evidence alone cannot authorize the two-frame
    // exception. Test the current measured points against the preceding
    // measured plane in world coordinates; ordinary three-frame motion is
    // deliberately unaffected, including a sparse planar moving target.
    bool two_frame_static_surface = false;
    bool two_frame_geometry_known = false;
    if (mp_->reference_strong_two_frame_enabled && consecutive_observation &&
        trackers[j]->observations.size() == 1 &&
        trackers[j]->observed_dense_points.size() >= 3 && candidate.dense_points.size() >= 3) {
      const auto& previous_points = trackers[j]->observed_dense_points;
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto& point : previous_points) mean += point;
      mean /= previous_points.size();
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto& point : previous_points) {
        const Eigen::Vector3d delta = point - mean;
        covariance += delta * delta.transpose();
      }
      covariance /= previous_points.size();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> fit(covariance);
      two_frame_geometry_known = fit.info() == Eigen::Success &&
          fit.eigenvalues().allFinite() && fit.eigenvalues()[1] > 1e-6;
      // A line or a failed fit cannot exclude static surface resampling.
      // It retains ordinary confirmation rather than authorizing an exception.
      if (two_frame_geometry_known && fit.eigenvalues()[0] < 0.01 * fit.eigenvalues()[2]) {
        const Eigen::Vector3d normal = fit.eigenvectors().col(0);
        const Eigen::Vector3d anchor = trackers[j]->observations.back().second + mean;
        size_t supported = 0;
        for (const auto& point : candidate.dense_points)
          if (std::abs(normal.dot(candidate.state.position + point - anchor)) <= 0.03)
            ++supported;
        two_frame_static_surface = supported >= 0.8 * candidate.dense_points.size();
      }
    }
    if (j < 0) {
      auto tracker = std::make_shared<Ekf>(0.02);
      for (int axis = 0; axis < 3; ++axis)
        tracker->Rt(axis, axis) = mp_->position_measurement_variance;
      tracker->reset(candidate.state.position, id++);
      trackers.push_back(tracker);
      j = trackers.size() - 1;
    } else {
      auto& tracker = trackers[j];
      const auto& previous = tracker->observations.back();
      double dt = (cloud_stamp_ - previous.first).toSec();
      if (dt > 0.0) {
        Eigen::Vector3d velocity = (candidate.state.position - previous.second) / dt;
        tracker->update(candidate.state.position, 0.5 * velocity + 0.5 * tracker->vel());
      }
      if (tracker->last_seen_frame != frame_num - 1) {
        const double published_gap = tracker->last_published_stamp.isZero() ? -1.0 :
            (cloud_stamp_ - tracker->last_published_stamp).toSec();
        confirmed_reacquisition = mp_->confirmed_reacquisition_enabled &&
            dt > 0.0 && dt <= mp_->reacquisition_max_gap &&
            published_gap >= 0.0 && published_gap <= mp_->reacquisition_max_gap;
        retained_gap = confirmed_reacquisition ||
            (mp_->reference_motion_enabled && reference_tree_ &&
            tracker->last_seen_frame == frame_num - 2 &&
            !tracker->last_published_stamp.isZero() &&
            (cloud_stamp_ - tracker->last_published_stamp).toSec() <= 0.6 &&
            (cloud_stamp_ - previous.first).toSec() <= 0.6);
        if (mp_->confirmed_reacquisition_enabled)
          ROS_DEBUG_STREAM("FAPP_REACQUIRE " << cloud_stamp_.toNSec() << " "
              << candidate.state.position.transpose() << " " << tracker->id << " "
              << dt << " " << published_gap << " " << confirmed_reacquisition << " "
              << tracker->observations.size());
        if (mp_->reference_motion_enabled && reference_tree_)
          ROS_DEBUG_STREAM("FAPP_REFERENCE_GAP " << cloud_stamp_.toNSec() << " "
              << candidate.state.position.transpose() << " "
              << frame_num - tracker->last_seen_frame << " " << retained_gap << " "
              << tracker->observations.size());
        if (!retained_gap) {
          tracker->observations.clear();
          tracker->swing_confirmation.clear();
          tracker->evidence_observations.clear();
          tracker->bidirectional_evidence.clear();
          if (mp_->confirmed_reacquisition_enabled)
            tracker->last_motion_stamp = ros::Time();
        }
      }
    }
    auto& tracker = trackers[j];
    tracker->supplemental_candidate = candidate.supplemental;
    tracker->object_component = candidate.object_component;
    bool bidirectional_transition_valid = false;

    if (need_bidirectional && tracker->last_seen_frame == frame_num - 1 &&
        candidate.bidirectional_forward_valid && !tracker->observations.empty() &&
        !tracker->observed_dense_points.empty() &&
        tracker->observations.back().first == previous_ray.stamp &&
        (tracker->bidirectional_evidence.empty() ||
         tracker->bidirectional_evidence.back().first == previous_ray.stamp)) {
      size_t vacated = 0, vacated_static = 0;
      const Eigen::Vector3d previous_center = tracker->observations.back().second;
      for (const auto& relative : tracker->observed_dense_points) {
        const auto evidence = current_depth_view.classify(previous_center + relative);
        if (evidence == BidirectionalDepth::Foreground) ++vacated;
        if (evidence == BidirectionalDepth::Static) ++vacated_static;
      }
      const double bidirectional_static_forward = candidate.bidirectional_forward_static_fraction;
      const double bidirectional_static_vacated = static_cast<double>(vacated_static) /
          tracker->observed_dense_points.size();
      const double vacated_fraction = static_cast<double>(vacated) /
          tracker->observed_dense_points.size();
      const double bidirectional_motion = std::max(candidate.bidirectional_forward_fraction, vacated_fraction);
      bidirectional_transition_valid = true;
      tracker->bidirectional_evidence.push_back({cloud_stamp_, Eigen::Vector3d(
          bidirectional_motion, bidirectional_static_forward, bidirectional_static_vacated)});
      while (tracker->bidirectional_evidence.size() > 2)
        tracker->bidirectional_evidence.pop_front();
      ROS_DEBUG_STREAM("FAPP_BIDIRECTIONAL_TRANSITION " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << candidate.bidirectional_forward_fraction << " "
          << vacated_fraction << " " << bidirectional_motion << " "
          << bidirectional_static_forward << " " << bidirectional_static_vacated);
    }
    if (need_bidirectional && !bidirectional_transition_valid)
      tracker->bidirectional_evidence.clear();
    const ros::Time previous_motion_stamp = tracker->last_motion_stamp;
    bool object_motion_ready = false;
    if (mp_->object_motion_evidence_enabled) {
      const bool valid_observation = !candidate.static_background && shape_consistent &&
          !candidate.dense_points.empty();
      double moving_fraction = 0.0, static_fraction = 0.0, depth_fraction = 0.0;
      if (valid_observation && bidirectional_transition_valid && have_previous_ray) {
        size_t moving = 0, stationary = 0;
        std::vector<int> nearest(1);
        std::vector<float> squared_distance(1);
        for (const auto& relative : candidate.dense_points) {
          const Eigen::Vector3d current = candidate.state.position + relative;
          const Eigen::Vector3d moved = tracker->observations.back().second + relative;
          PointType point;
          point.x = current.x(); point.y = current.y(); point.z = current.z();
          if (object_motion_previous_tree.nearestKSearch(point, 1, nearest, squared_distance) > 0 &&
              squared_distance[0] <= 0.05 * 0.05) ++stationary;
          point.x = moved.x(); point.y = moved.y(); point.z = moved.z();
          if (object_motion_previous_tree.nearestKSearch(point, 1, nearest, squared_distance) > 0 &&
              squared_distance[0] <= 0.05 * 0.05) ++moving;
        }
        moving_fraction = double(moving) / candidate.dense_points.size();
        static_fraction = double(stationary) / candidate.dense_points.size();
        depth_fraction = tracker->bidirectional_evidence.back().second.x();
      }
      object_motion_ready = tracker->object_motion_evidence.update(
          cloud_stamp_.toNSec(), previous_ray.stamp.toNSec(), valid_observation,
          bidirectional_transition_valid, moving_fraction, static_fraction, depth_fraction,
          mp_->object_motion_match_fraction, mp_->object_motion_match_advantage,
          mp_->object_motion_depth_fraction);
      ROS_DEBUG_STREAM("FAPP_OBJECT_MOTION " << cloud_stamp_.toNSec() << " " << tracker->id
          << " " << candidate.state.position.transpose() << " " << valid_observation
          << " " << bidirectional_transition_valid << " " << moving_fraction
          << " " << static_fraction << " " << depth_fraction << " " << object_motion_ready);
    }
    // Do not seed a later swing with positions across a rejected shape change.
    if (check_observed_shape && !shape_consistent)
      tracker->observations.clear();
    tracker->observed_dense_size = candidate.dense_size;
    tracker->observed_dense_count = candidate.dense_count;
    tracker->observed_reference_foreground = candidate.reference_foreground && !candidate.static_background &&
        (!mp_->reference_strong_two_frame_enabled || candidate.bilateral_foreground_fraction >= 0.50);
    if (mp_->shape_consistency_enabled || mp_->reference_strong_two_frame_enabled || need_bidirectional) {
      tracker->observed_dense_points = candidate.dense_points;
      tracker->observed_lidar_direction = lidar_direction;
    }
    if (check_observed_shape && mp_->shape_consistency_enabled)
      tracker->swing_shape_points = std::move(swing_shape_points);
    tracker->last_seen_frame = frame_num;
    tracker->observations.push_back({cloud_stamp_, candidate.state.position});
    if (mp_->translated_startup_enabled) {
      tracker->evidence_observations.push_back({cloud_stamp_, candidate.state.position});
      while (!tracker->evidence_observations.empty() &&
             (cloud_stamp_ - tracker->evidence_observations.front().first).toSec() > 1.0)
        tracker->evidence_observations.pop_front();
      while (tracker->evidence_observations.size() > 16)
        tracker->evidence_observations.pop_front();
    }
    if (mp_->time_evidence_enabled) {
      const double observation_window = confirmed_reacquisition ?
          mp_->reacquisition_max_gap + 0.8 : mp_->motion_window;
      while (tracker->observations.size() > mp_->motion_confirmation_frames &&
             (cloud_stamp_ - tracker->observations.front().first).toSec() > observation_window)
        tracker->observations.pop_front();
      while (tracker->observations.size() > 32) tracker->observations.pop_front();
    } else if (tracker->observations.size() > mp_->motion_confirmation_frames) {
      tracker->observations.pop_front();
    }
    if (retained_gap) {
      const double maximum_span = confirmed_reacquisition ?
          mp_->reacquisition_max_gap + 0.8 : 0.8;
      while (tracker->observations.size() > 1 &&
             (cloud_stamp_ - tracker->observations.front().first).toSec() > maximum_span)
        tracker->observations.pop_front();
    }

    if (mp_->object_motion_evidence_enabled)
      while (tracker->observations.size() > 3) tracker->observations.pop_front();
    double displacement = 0.0;
    for (const auto& first : tracker->observations)
      for (const auto& second : tracker->observations)
        displacement = std::max(displacement, (first.second - second.second).norm());
    // Registered centers are already ego-motion compensated. Average the
    // measured segment speeds: reversals do not cancel, and an input gap must
    // not dilute the preceding fast observation into a slow time average.
    double observed_speed = 0.0;
    size_t speed_samples = 0;
    for (size_t observation = 1; observation < tracker->observations.size(); ++observation) {
      const auto& previous = tracker->observations[observation - 1];
      const auto& current = tracker->observations[observation];
      double dt = (current.first - previous.first).toSec();
      if (dt > 0.0) {
        observed_speed += (current.second - previous.second).norm() / dt;
        ++speed_samples;
      }
    }
    if (speed_samples > 0) observed_speed /= speed_samples;
    bool enough_observations = mp_->time_evidence_enabled ?
        tracker->observations.size() >= mp_->motion_confirmation_frames :
        tracker->observations.size() == mp_->motion_confirmation_frames;
    if (mp_->object_motion_evidence_enabled)
      enough_observations = tracker->observations.size() == 3;
    bool has_motion = enough_observations &&
                      displacement >= mp_->motion_displacement &&
                      observed_speed >= mp_->min_motion_speed;
    bool bidirectional_startup = false;
    bool supplemental_ray_motion = false;
    bool bidirectional_static_suppression = false;
    if (need_bidirectional && tracker->bidirectional_evidence.size() >= 2) {
      const auto& first = tracker->bidirectional_evidence[tracker->bidirectional_evidence.size() - 2].second;
      const auto& second = tracker->bidirectional_evidence.back().second;
      const double motion_threshold = 0.26;
      supplemental_ray_motion = first.x() >= motion_threshold && second.x() >= motion_threshold;
      bidirectional_startup = mp_->bidirectional_ray_evidence_enabled &&
          first.x() >= motion_threshold && second.x() >= motion_threshold;
      bidirectional_static_suppression =
          mp_->bidirectional_static_suppression_enabled &&
          first.x() < motion_threshold && second.x() < motion_threshold &&
          first.y() >= 0.55 && first.z() >= 0.55 &&
          second.y() >= 0.55 && second.z() >= 0.55 &&
          candidate.static_plane_fraction >= 0.55 &&
          candidate.independent_motion_fraction < motion_threshold;
      ROS_DEBUG_STREAM("FAPP_BIDIRECTIONAL_EVIDENCE " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << first.x() << " " << second.x() << " "
          << bidirectional_startup << " " << bidirectional_static_suppression);
    }
    const double two_frame_dt = tracker->observations.size() == 2 ?
        (tracker->observations.back().first - tracker->observations.front().first).toSec() : 0.0;
    const bool strong_reference_motion =
        mp_->reference_strong_two_frame_enabled && mp_->reference_motion_enabled &&
        consecutive_observation && previous_reference_foreground && candidate.reference_foreground &&
        !candidate.static_background && two_frame_geometry_known && !two_frame_static_surface &&
        tracker->observations.size() == 2 &&
        two_frame_dt > 0.0 && two_frame_dt <= 0.4 && displacement >= 0.30 && observed_speed >= 0.50 &&
        candidate.novel_fraction >= 0.50 && candidate.bilateral_foreground_fraction >= 0.50;
    if (mp_->reference_strong_two_frame_enabled && tracker->observations.size() == 2)
      ROS_DEBUG_STREAM("FAPP_TWO_FRAME_EVIDENCE " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " " << previous_reference_foreground << " "
          << candidate.bilateral_foreground_fraction << " " << strong_reference_motion << " "
          << two_frame_static_surface << " " << two_frame_geometry_known);
    if (candidate.changed) tracker->last_change_stamp = cloud_stamp_;
    bool recently_changed = tracker->last_change_stamp >= tracker->observations.front().first;
    // A tracked object must still have measured motion, not just EKF velocity.
    // Novelty may occur in the first observation, before motion confirmation.
    // Sampling can move a cluster center along an unchanged surface. Refresh
    // motion only when its dense return also changes across the raw observed
    // clouds; keep the existing brief support through swing turns.
    double required_novel_fraction = recently_published && shape_consistent ?
        std::min(mp_->min_motion_novel_fraction, mp_->tracked_motion_novel_fraction) :
        mp_->min_motion_novel_fraction;
    bool reference_startup = mp_->reference_motion_enabled && candidate.reference_foreground;
    const bool strict_foreground = mp_->strict_depth_evidence_enabled &&
        !candidate.static_background && candidate.bilateral_foreground_fraction > 0.0 &&
        candidate.bilateral_foreground_fraction >= mp_->min_motion_novel_fraction;
    const bool occlusion_startup = mp_->occlusion_startup_enabled &&
        !candidate.static_background &&
        candidate.occlusion_fraction > 0.0 &&
        candidate.occlusion_fraction >= mp_->min_motion_novel_fraction;
    bool translated_startup = mp_->translated_startup_enabled &&
        has_motion && consecutive_observation && !candidate.static_background &&
        !candidate.dense_points.empty();
    if (mp_->translated_startup_enabled && !mp_->time_evidence_enabled) {
      const bool before_ray_eligible = translated_startup;
      translated_startup = translated_startup && supplemental_ray_motion;
      ROS_DEBUG_STREAM("FAPP_TRANSLATED_RAY " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << before_ray_eligible << " " << supplemental_ray_motion << " "
          << translated_startup);
    }
    for (int history = 0; translated_startup && history < 2; ++history) {
      const auto observed = std::find_if(tracker->evidence_observations.begin(),
          tracker->evidence_observations.end(), [&](const std::pair<ros::Time, Eigen::Vector3d>& value) {
            return value.first == history_stamps[history];
          });
      if (!has_observed_history[history] || history_stamps[history].isZero() ||
          observed == tracker->evidence_observations.end()) {
        translated_startup = false;
        break;
      }
      size_t moving_supported = 0, static_supported = 0;
      std::vector<int> nearest(1);
      std::vector<float> squared_distance(1);
      for (const auto& relative : candidate.dense_points) {
        PointType current, moved;
        const Eigen::Vector3d current_position = candidate.state.position + relative;
        const Eigen::Vector3d old_position = observed->second + relative;
        current.x = current_position.x(); current.y = current_position.y(); current.z = current_position.z();
        moved.x = old_position.x(); moved.y = old_position.y(); moved.z = old_position.z();
        if (observation_trees[history].nearestKSearch(current, 1, nearest, squared_distance) > 0 &&
            squared_distance[0] <= 0.05 * 0.05) ++static_supported;
        if (observation_trees[history].nearestKSearch(moved, 1, nearest, squared_distance) > 0 &&
            squared_distance[0] <= 0.05 * 0.05) ++moving_supported;
      }
      const double moving_fraction = static_cast<double>(moving_supported) / candidate.dense_points.size();
      const double static_fraction = static_cast<double>(static_supported) / candidate.dense_points.size();
      ROS_DEBUG_STREAM("FAPP_TRANSLATED_SOURCE " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << history << " " << history_stamps[history].toNSec() << " "
          << candidate.dense_points.size() << " " << moving_supported << " " << static_supported);
      translated_startup = moving_fraction >= mp_->translated_startup_fraction &&
          moving_fraction - static_fraction >= 0.25;
      ROS_DEBUG_STREAM("FAPP_TRANSLATED_STARTUP " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " " << history << " "
          << moving_fraction << " " << static_fraction << " " << translated_startup);
    }
    if (mp_->occlusion_startup_enabled)
      ROS_DEBUG_STREAM("FAPP_OCCLUSION_STARTUP " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " " << candidate.occlusion_fraction << " "
          << has_motion << " " << occlusion_startup);
    if (mp_->strict_depth_evidence_enabled)
      ROS_DEBUG_STREAM("FAPP_STRICT_DEPTH " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " " << candidate.bilateral_foreground_fraction
          << " " << has_motion << " " << strict_foreground);
    // A subsequently rejected confirmation must not carry published history
    // into a new startup. Keep last_dynamic's existing shape/bookkeeping use.
    const bool historical_startup = mp_->published_startup_source_enabled ?
        (!tracker->last_published_stamp.isZero() &&
         tracker->last_dynamic_stamp == tracker->last_published_stamp) :
        !tracker->last_dynamic_stamp.isZero();
    bool startup_evidence = mp_->strict_depth_evidence_enabled ? strict_foreground :
        (recently_changed || historical_startup ||
         reference_startup || occlusion_startup || translated_startup);
    if (mp_->published_startup_source_enabled)
      ROS_DEBUG_STREAM("FAPP_PUBLISHED_STARTUP " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << tracker->last_dynamic_stamp.toNSec() << " "
          << tracker->last_published_stamp.toNSec() << " " << recently_changed << " "
          << historical_startup << " " << startup_evidence);
    const bool original_startup_evidence = startup_evidence;
    if (bidirectional_startup || (candidate.supplemental && supplemental_ray_motion))
      startup_evidence = true;
    if (need_bidirectional)
      ROS_DEBUG_STREAM("FAPP_BIDIRECTIONAL_DECISION " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << original_startup_evidence << " "
          << bidirectional_startup << " " << has_motion << " " << candidate.novel_fraction
          << " " << bidirectional_static_suppression);
    if (has_motion && startup_evidence &&
        candidate.novel_fraction >= required_novel_fraction)
      tracker->last_motion_stamp = cloud_stamp_;
    if (strong_reference_motion && (!mp_->strict_depth_evidence_enabled || strict_foreground)) {
      tracker->last_motion_stamp = cloud_stamp_;
      ROS_DEBUG_STREAM("FAPP_REFERENCE_STRONG_TWO " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " " << displacement << " "
          << observed_speed << " " << candidate.novel_fraction);
    }
    if (mp_->object_motion_evidence_enabled) {
      // Replace all legacy startup paths; retain the existing 0.6 s exit hold.
      tracker->last_motion_stamp = previous_motion_stamp;
      if (has_motion && object_motion_ready && !bidirectional_static_suppression &&
          candidate.novel_fraction >= required_novel_fraction)
        tracker->last_motion_stamp = cloud_stamp_;
      if (bidirectional_static_suppression) tracker->object_motion_evidence.clear();
    }
    bool confirmed = !tracker->last_motion_stamp.isZero() &&
                     (cloud_stamp_ - tracker->last_motion_stamp).toSec() <= 0.6;
    if (confirmed_reacquisition)
      confirmed = confirmed && has_motion && candidate.novel_fraction >= required_novel_fraction;
    ROS_DEBUG_STREAM("FAPP_TRACK " << cloud_stamp_.toNSec() << " "
        << candidate.state.position.transpose() << " " << tracker->id << " "
        << matched[i] << " " << tracker->observations.size() << " "
        << displacement << " " << observed_speed << " " << candidate.changed << " "
        << candidate.novel_fraction << " " << confirmed);
    if (mp_->swing_only && (!confirmed || candidate.static_background ||
        bidirectional_static_suppression || !shape_consistent))
      tracker->swing_confirmation.clear();
    if (!confirmed) continue;
    tracker->last_dynamic_stamp = cloud_stamp_;
    // Apply background evidence to publication after the original association
    // and motion bookkeeping. Removing a candidate earlier can redirect a
    // real moving object's association and erase its startup evidence.
    if (candidate.static_background) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " stable_background");
      continue;
    }
    if (bidirectional_static_suppression) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " bidirectional_static");
      continue;
    }
    if (!shape_consistent) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " incoherent_dense_shape");
      continue;
    }

    if (candidate.supplemental)
      ROS_DEBUG_STREAM("FAPP_SUPPLEMENTAL_EVIDENCE " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << candidate.parent_key << " "
          << published_parents.count(candidate.parent_key) << " " << has_motion << " "
          << candidate.novel_fraction << " " << supplemental_ray_motion);
    if (candidate.supplemental &&
        (!published_parents.count(candidate.parent_key) || !has_motion ||
         candidate.novel_fraction < mp_->min_motion_novel_fraction || !supplemental_ray_motion)) {
      ROS_DEBUG_STREAM("FAPP_REJECT " << cloud_stamp_.toNSec() << " "
          << candidate.state.position.transpose() << " supplemental_source");
      continue;
    }

    // Partition only an already confirmed, publishable candidate. Its background
    // and motion evidence must not be bypassed by splitting wall fragments.
    ObjectState published_state = candidate.state;
    pcl::PointIndices published_indices = candidate.indices;
    if (mp_->density_split_enabled || mp_->publish_measured_body_position_enabled) {
      DensityPartitionOptions options;
      options.radius = mp_->core_radius;
      options.alpha = mp_->core_density_fraction;
      options.min_cluster = mp_->min_cluster;
      options.max_cluster = mp_->max_cluster;
      options.core_pts = mp_->core_pts;
      options.squared_tolerance = mp_->tolerance;
      const auto partition = densityPartition(*ClusterPoints, candidate.indices, options);
      const auto body = std::find_if(partition.clusters.begin(), partition.clusters.end(),
          [&](const pcl::PointIndices& component) {
            return std::find(component.indices.begin(), component.indices.end(),
                             candidate.dense_point_index) != component.indices.end();
          });
      if (partition.split && body != partition.clusters.end()) {
        published_indices = *body;
        Points body_points;
        for (int index : published_indices.indices)
          body_points.push_back(ClusterPoints->points[index]);
        pcl::PointXYZ minimum, maximum;
        pcl::getMinMax3D(body_points, minimum, maximum);
        published_state.position = 0.5 * (minimum.getVector3fMap().cast<double>() +
                                          maximum.getVector3fMap().cast<double>());
        published_state.size = maximum.getVector3fMap().cast<double>() -
                               minimum.getVector3fMap().cast<double>();
        ROS_DEBUG_STREAM("FAPP_PARTITION " << cloud_stamp_.toNSec() << " "
            << candidate.indices.indices.size() << " " << published_indices.indices.size()
            << " " << partition.clusters.size() << " " << partition.leftovers.indices.size());
      }
    }
    if (mp_->publish_measured_body_position_enabled) {
      // An unsplit sparse parent still has a measured bounding box. Use its
      // actual extrema rather than attaching a parent size to a dense median.
      Points body_points;
      for (int index : published_indices.indices)
        body_points.push_back(ClusterPoints->points[index]);
      pcl::PointXYZ minimum, maximum;
      pcl::getMinMax3D(body_points, minimum, maximum);
      published_state.position = 0.5 * (minimum.getVector3fMap().cast<double>() +
                                      maximum.getVector3fMap().cast<double>());
      published_state.size = maximum.getVector3fMap().cast<double>() -
                             minimum.getVector3fMap().cast<double>();
      ROS_DEBUG_STREAM("FAPP_MEASURED_BODY " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << published_state.position.transpose() << " "
          << published_state.size.transpose() << " " << published_indices.indices.size());
    }
    // Preserve the detector's qualified-candidate feedback even before swing
    // publication, otherwise published_startup_source prevents confirmation.
    tracker->last_published_stamp = cloud_stamp_;
    if (mp_->swing_only && !mp_->object_motion_evidence_enabled) {
      // The short history has already passed the detector's motion checks.
      // Include its first observations instead of discarding the outward leg
      // while waiting for ordinary motion confirmation.
      if (tracker->swing_confirmation.empty()) {
        for (size_t k = 0; k + 1 < tracker->observations.size(); ++k) {
          const auto& observation = tracker->observations[k];
          tracker->swing_confirmation.update(observation.first.toSec(),
                                             observation.second, true);
        }
      }
      const bool swing = tracker->swing_confirmation.update(
          cloud_stamp_.toSec(), candidate.state.position, true);
      ROS_DEBUG_STREAM("FAPP_SWING " << cloud_stamp_.toNSec() << " "
          << tracker->id << " " << swing);
      if (!swing) continue;
    }
    int detection_index = detections.size();
    detections.push_back(published_state);
    if (mp_->candidate_partition_enabled && !candidate.supplemental)
      published_parents.insert(candidate.parent_key);
    published_pairs.push_back({detection_index, j});
    // Keep the complete original return in the dynamic cloud. Independent
    // confirmed components from one parent contribute its points only once.
    if (mp_->candidate_partition_enabled) {
      pcl::PointIndices unique_dynamic_indices;
      unique_dynamic_indices.header = candidate.dynamic_indices.header;
      for (int index : candidate.dynamic_indices.indices) {
        if (published_dynamic_indices[index]) continue;
        published_dynamic_indices[index] = true;
        unique_dynamic_indices.indices.push_back(index);
        dynamic_points->points.push_back(ClusterPoints->points[index]);
      }
      if (!unique_dynamic_indices.indices.empty())
        deleted_indices.push_back(unique_dynamic_indices);
    } else {
      deleted_indices.push_back(candidate.indices);
      for (int index : candidate.indices.indices)
        dynamic_points->points.push_back(ClusterPoints->points[index]);
    }

    obj_state_msgs::State state;
    state.header = states.header;
    const Eigen::Vector3d output_position = mp_->publish_measured_body_position_enabled ?
        published_state.position : tracker->pos();
    state.position.x = output_position.x();
    state.position.y = output_position.y();
    state.position.z = output_position.z();
    state.velocity.x = tracker->vel().x();
    state.velocity.y = tracker->vel().y();
    state.velocity.z = tracker->vel().z();
    state.size.x = published_state.size.x();
    state.size.y = published_state.size.y();
    state.size.z = published_state.size.z();
    states.states.push_back(state);
  }

  sensor_msgs::PointCloud2 dynamic_pts;
  pcl::toROSMsg(*dynamic_points, dynamic_pts);
  dynamic_pts.header.frame_id = frame_id_;
  dynamic_pts.header.stamp = cloud_stamp_;
  cloudPub.publish(dynamic_pts);
  statesPub.publish(states);
  visualizeFunction(published_pairs);

  trackers.erase(std::remove_if(trackers.begin(), trackers.end(),
      [this](const std::shared_ptr<Ekf>& tracker) {
        return frame_num - tracker->last_seen_frame > 20;
      }), trackers.end());
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Tracking Time Cost (ms)： " << compTime <<std::endl;
}

void MappingRos::ekfPredictCallback(const ros::TimerEvent& e) {
  if (trackers.size() == 0)
    return;

  for (int i = 0; i < trackers.size(); ++i) {
    double update_dt = (ros::Time::now() - trackers[i]->last_update_stamp_).toSec();
    trackers[i]->predict(update_dt);
  }
}

void MappingRos::mapPubCallback(const ros::TimerEvent& e) {
  if (Remaining_Points->points.size() == 0)
    return;

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // KD_TREE<PointType>::Ptr delete_tree;
  // delete_tree.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));
  // delete_tree->Build(Remaining_Points->points);
  // delete_tree->Delete_Point_Boxes(delete_boxes);

  // Points static_pts;
  // delete_tree->flatten(delete_tree->Root_Node, delete_tree->PCL_Storage, NOT_RECORD);
  // static_pts.points = delete_tree->PCL_Storage;
  // static_pts.width = static_pts.points.size();
  // static_pts.height = 1;
  // static_pts.is_dense = true;


  // delete_boxes.clear();
  // sensor_msgs::PointCloud2 map_static;
  // pcl::toROSMsg(static_pts, map_static);
  // map_static.header.frame_id = "world";
  // staticMapPub.publish(map_static);
}


double MappingRos::iou(ObjectState state1, ObjectState state2) {
  double distance = (state1.position - state2.position).norm();
  double iou = atan(distance)*2/M_PI;
  return iou;
}

void MappingRos::visualizeFunction(const std::vector<std::pair<int, int>> pairs) {
  visualization_msgs::MarkerArray poses;
  visualization_msgs::MarkerArray boxes;

  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  clear.header.frame_id = frame_id_;
  clear.header.stamp = cloud_stamp_;
  boxes.markers.push_back(clear);
  poses.markers.push_back(clear);

  for (size_t detectionIndex = 0; detectionIndex < detections.size(); ++detectionIndex) {
    pcl::PointXYZ minPt, maxPt;
    // if (detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2 > 1.7 ||
    //     detections[detectionIndex].position[1] < -2.0 ||
    //     detections[detectionIndex].position[2] > 1.8)
    //   continue;
    minPt.x = detections[detectionIndex].position[0] - detections[detectionIndex].size[0]/2;
    minPt.y = detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2;
    minPt.z = detections[detectionIndex].position[2] - detections[detectionIndex].size[2]/2;
    maxPt.x = detections[detectionIndex].position[0] + detections[detectionIndex].size[0]/2;
    maxPt.y = detections[detectionIndex].position[1] + detections[detectionIndex].size[1]/2;
    maxPt.z = detections[detectionIndex].position[2] + detections[detectionIndex].size[2]/2;
    visualization_msgs::Marker edgeMarker;
    edgeMarker.id = detectionIndex;
    edgeMarker.header.stamp = cloud_stamp_;
    edgeMarker.header.frame_id = frame_id_;
    edgeMarker.pose.orientation.w = 1.00;
    edgeMarker.lifetime = ros::Duration(1.0);
    edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 1.00;
    edgeMarker.color.g = 0.50;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 0.80;
    edgeMarker.scale.x = 0.1;
    geometry_msgs::Point point[8];
    point[0].x = minPt.x; point[0].y = maxPt.y; point[0].z = maxPt.z;
    point[1].x = minPt.x; point[1].y = minPt.y; point[1].z = maxPt.z;
    point[2].x = minPt.x; point[2].y = minPt.y; point[2].z = minPt.z;
    point[3].x = minPt.x; point[3].y = maxPt.y; point[3].z = minPt.z;
    point[4].x = maxPt.x; point[4].y = maxPt.y; point[4].z = minPt.z;
    point[5].x = maxPt.x; point[5].y = minPt.y; point[5].z = minPt.z;
    point[6].x = maxPt.x; point[6].y = minPt.y; point[6].z = maxPt.z;
    point[7].x = maxPt.x; point[7].y = maxPt.y; point[7].z = maxPt.z;
    for (int l = 0; l < 8; l++) {
      edgeMarker.points.push_back(point[l]);
    }
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[3]);
    edgeMarker.points.push_back(point[2]);
    edgeMarker.points.push_back(point[5]);
    edgeMarker.points.push_back(point[6]);
    edgeMarker.points.push_back(point[1]);
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[7]);
    edgeMarker.points.push_back(point[4]);
    boxes.markers.push_back(edgeMarker);

    auto match = std::find_if(pairs.begin(), pairs.end(),
        [detectionIndex](const std::pair<int, int>& pair) {
          return pair.first == static_cast<int>(detectionIndex);
        });
    if (match == pairs.end()) continue;
    int trackerIndex = match->second;

    visualization_msgs::Marker poseMarker;
    ObjectState state;
    state.position = mp_->publish_measured_body_position_enabled ?
        detections[detectionIndex].position : trackers[trackerIndex]->pos();
    state.velocity = trackers[trackerIndex]->vel();
    poseMarker.id = trackerIndex+100;
    poseMarker.header.stamp = cloud_stamp_;
    poseMarker.header.frame_id = frame_id_;
    poseMarker.lifetime = ros::Duration(1.0);
    poseMarker.type = visualization_msgs::Marker::ARROW;
    poseMarker.action = visualization_msgs::Marker::ADD;
    poseMarker.ns = "objectpose";
    poseMarker.color.r = 0.00;
    poseMarker.color.g = 1.00;
    poseMarker.color.b = 0.00;
    poseMarker.color.a = 1.00;
    poseMarker.scale.x = 0.10;
    poseMarker.scale.y = 0.18;
    poseMarker.scale.z = 0.30;
    poseMarker.pose.orientation.w = 1.0;
    geometry_msgs::Point arrow[2];
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();
    if (state.velocity.norm() > 0.01) direction = state.velocity.normalized();
    arrow[0].x = state.position[0]; arrow[0].y = state.position[1]; arrow[0].z = state.position[2];
    arrow[1].x = state.position[0] + direction[0];
    arrow[1].y = state.position[1] + direction[1];
    arrow[1].z = state.position[2] + direction[2];
    // std::cout << "vx:" << state.velocity[0] << std::endl;
    // std::cout << "vy:" << state.velocity[1] << std::endl;
    // std::cout << "vz:" << state.velocity[2] << std::endl;
    poseMarker.points.push_back(arrow[0]);
    poseMarker.points.push_back(arrow[1]);
    poses.markers.push_back(poseMarker);
  }
  objectPosePub.publish(poses);
  edgePub.publish(boxes);

}

}
