#include "mapping_manager.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace {

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
  nh.param("detection/thresh_dist", mp_->thresh_dist, 0.08);
  nh.param("detection/thresh_var", mp_->thresh_var, 0.28);
  nh.param("detection/motion_displacement", mp_->motion_displacement, 0.06);
  nh.param("detection/motion_confirmation_frames", mp_->motion_confirmation_frames, 3);
  if (mp_->motion_confirmation_frames < 3 || mp_->motion_confirmation_frames > 5)
    throw std::runtime_error("motion_confirmation_frames must be 3, 4 or 5");
  nh.param("detection/min_motion_speed", mp_->min_motion_speed, 0.15);
  nh.param("detection/min_motion_novel_fraction", mp_->min_motion_novel_fraction, 0.3);
  nh.param("detection/max_above_uav", mp_->max_above_uav, 1.5);
  nh.param("association/geometry_enabled", mp_->geometry_association_enabled, false);
  nh.param("detection/depth_evidence_enabled", mp_->depth_evidence_enabled, false);
  if (mp_->depth_evidence_enabled) {
    std::vector<double> offset;
    // Same MID360 offset as FAST-LIO's mid360.yaml; use its live parameter
    // when the sensor stack is running. Replay supplies that parameter too.
    nh.param("/UAV1/mapping/extrinsic_T", offset,
             std::vector<double>{-0.011, -0.02329, 0.04412});
    if (offset.size() != 3) throw std::runtime_error("FAST-LIO extrinsic_T requires three values");
    mp_->lidar_offset = Eigen::Vector3d(offset[0], offset[1], offset[2]);
    ROS_INFO_STREAM("FAPP depth evidence lidar offset: " << mp_->lidar_offset.transpose());
  }
  nh.param("background_surface/enabled", mp_->background_surface_enabled, true);
  nh.param("background_plane/enabled", mp_->background_plane_enabled, true);
  nh.param("background_plane/distance", mp_->background_plane_distance, 0.03);
  nh.param("background_plane/min_points", mp_->background_plane_min_points, 100);
  nh.param("background_plane/max_planes", mp_->background_plane_max_planes, 6);
  nh.param("lidar/range_x", mp_->Range[0], 15.0);
  nh.param("lidar/range_y", mp_->Range[1], 15.0);
  nh.param("lidar/range_z", mp_->Range[2], 1.0);
  nh.param("ros/odom_topic", mp_->odom_topic, string("/odom"));
  nh.param("ros/lidar_topic", mp_->lidar_topic, string("/livox/lidar"));
  nh.param("ros/frame_id", frame_id_, string("UAV1/camera_init"));

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
  sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
      MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
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
                                   const nav_msgs::OdometryConstPtr& odom) {
  cloud_stamp_ = msg->header.stamp;
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

  // Compare novelty against the two preceding raw observations.
  PointsPtr observed_history[2];
  MapData sensor_history[2];
  if (!buffer.empty()) observed_history[0] = buffer.front();
  if (buffer.size() > 1) observed_history[1] = buffer.back();
  if (mp_->depth_evidence_enabled) {
    if (!observed_sensor_poses.empty()) sensor_history[0] = observed_sensor_poses.front();
    if (observed_sensor_poses.size() > 1) sensor_history[1] = observed_sensor_poses.back();
    MapData sensor_pose;
    sensor_pose.R = md_->R;
    sensor_pose.T = md_->T + md_->R * mp_->lidar_offset;
    observed_sensor_poses.push_back(sensor_pose);
    if (observed_sensor_poses.size() > 2) observed_sensor_poses.pop_front();
  }
  buffer.push_back(nonfilter_pts);
  if (buffer.size() > 2) {
    buffer.pop_front();
  }
  *ClusterPoints += *nonfilter_pts;
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*ClusterPoints);

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
    bool changed;
    double novel_fraction;
    Eigen::Vector3d dense_size;
  };
  DepthView depth_views[2];
  pcl::search::KdTree<PointType> observation_trees[2];
  bool has_observed_history[2];
  for (int history = 0; history < 2; ++history) {
    has_observed_history[history] = observed_history[history] &&
                                   !observed_history[history]->empty();
    if (has_observed_history[history])
      observation_trees[history].setInputCloud(observed_history[history]);
    if (mp_->depth_evidence_enabled && has_observed_history[history])
      depth_views[history].build(*observed_history[history], sensor_history[history]);
  }
  std::vector<Candidate> candidates;
  detections.clear();
  deleted_indices.clear();
  for (const auto& indices : cluster_indices) {
    Points cluster;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    std::vector<double> distances;
    for (int index : indices.indices) {
      const auto& point = ClusterPoints->points[index];
      cluster.push_back(point);
      center += Eigen::Vector3d(point.x, point.y, point.z);
      PointVector nearest;
      std::vector<float> squared_distances(3);
      ikdtree_ptr->Nearest_Search(point, 3, nearest, squared_distances);
      if (!nearest.empty())
        distances.push_back(std::sqrt(squared_distances[0]));
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
    size_t ray_supported[2] = {0, 0}, ray_foreground[2] = {0, 0};
    std::vector<int> observed_nearest(1);
    std::vector<float> observed_squared_distance(1);
    for (const auto& point : cluster) {
      if ((point.getVector3fMap() - cluster[dense_index].getVector3fMap()).squaredNorm() >
          mp_->tolerance) continue;
      Eigen::Vector3d delta = Eigen::Vector3d(point.x, point.y, point.z) - center;
      covariance += delta * delta.transpose();
      for (int history = 0; history < 2; ++history) {
        if (!has_observed_history[history]) continue;
        int evidence = mp_->depth_evidence_enabled ? depth_views[history].classify(point) : -1;
        if (evidence == 0) ++ray_supported[history];
        if (evidence == 1) ++ray_foreground[history];
        bool novel = evidence == 1 || (evidence < 0 &&
            (observation_trees[history].nearestKSearch(point, 1, observed_nearest,
                                                     observed_squared_distance) == 0 ||
             observed_squared_distance[0] > mp_->thresh_dist * mp_->thresh_dist));
        if (novel) ++novel_points[history];
      }
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

    double avg_dist = 0.0, var_dist = 0.0;
    for (double distance : distances) avg_dist += distance;
    if (!distances.empty()) avg_dist /= distances.size();
    if (avg_dist > 0.0) {
      for (double distance : distances)
        var_dist += (distance - avg_dist) * (distance - avg_dist) / (avg_dist * avg_dist);
      var_dist /= distances.size();
    }
    Candidate candidate;
    candidate.state.position = center;
    candidate.state.size = size;
    candidate.indices = indices;
    candidate.dense_size = dense_size;
    candidate.novel_fraction = static_cast<double>(std::max(novel_points[0],
                                                          novel_points[1])) / dense_count;
    candidate.changed = avg_dist > mp_->thresh_dist && avg_dist < 5.0 &&
                        var_dist < mp_->thresh_var;
    ROS_DEBUG_STREAM("FAPP_CANDIDATE " << cloud_stamp_.toNSec() << " "
        << center.x() << " " << center.y() << " " << center.z() << " "
        << size.x() << " " << size.y() << " " << size.z() << " "
        << cluster.size() << " " << avg_dist << " " << var_dist);
    ROS_DEBUG_STREAM("FAPP_EVIDENCE " << cloud_stamp_.toNSec() << " "
        << center.transpose() << " " << dense_count << " "
        << novel_points[0] << " " << novel_points[1] << " "
        << ray_supported[0] << " " << ray_supported[1] << " "
        << ray_foreground[0] << " " << ray_foreground[1]);
    candidates.push_back(candidate);
  }

  // Match closest pairs once each; multiple detections must not update one track.
  struct Match {
    bool stale;
    double distance;
    int candidate, tracker;
  };
  std::vector<Match> possible_pairs;
  for (int i = 0; i < candidates.size(); ++i) {
    for (int j = 0; j < trackers.size(); ++j) {
      double distance = (candidates[i].state.position -
                         trackers[j]->observations.back().second).norm();
      if (distance < 0.82) {
        double cost = distance;
        if (mp_->geometry_association_enabled)
          cost = 0.5 * distance +
                 0.5 * (candidates[i].state.position - trackers[j]->pos()).norm() +
                 0.25 * (candidates[i].dense_size - trackers[j]->observed_dense_size).norm();
        possible_pairs.push_back({trackers[j]->last_seen_frame != frame_num - 1,
                                  cost, i, j});
      }
    }
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
  for (const auto& pair : possible_pairs) {
    int i = pair.candidate, j = pair.tracker;
    if (matched[i] < 0 && !used[j]) {
      matched[i] = j;
      used[j] = true;
    }
  }

  obj_state_msgs::ObjectsStates states;
  states.header.stamp = cloud_stamp_;
  states.header.frame_id = frame_id_;
  std::vector<std::pair<int, int>> published_pairs;
  for (int i = 0; i < candidates.size(); ++i) {
    int j = matched[i];
    const auto& candidate = candidates[i];
    if (j < 0) {
      auto tracker = std::make_shared<Ekf>(0.02);
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
      if (tracker->last_seen_frame != frame_num - 1)
        tracker->observations.clear();
    }
    auto& tracker = trackers[j];
    tracker->observed_dense_size = candidate.dense_size;
    tracker->last_seen_frame = frame_num;
    tracker->observations.push_back({cloud_stamp_, candidate.state.position});
    if (tracker->observations.size() > mp_->motion_confirmation_frames)
      tracker->observations.pop_front();

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
    bool has_motion = tracker->observations.size() == mp_->motion_confirmation_frames &&
                      displacement >= mp_->motion_displacement &&
                      observed_speed >= mp_->min_motion_speed;
    if (candidate.changed) tracker->last_change_stamp = cloud_stamp_;
    bool recently_changed = tracker->last_change_stamp >= tracker->observations.front().first;
    // A tracked object must still have measured motion, not just EKF velocity.
    // Novelty may occur in the first observation, before motion confirmation.
    // Sampling can move a cluster center along an unchanged surface. Refresh
    // motion only when its dense return also changes across the raw observed
    // clouds; keep the existing brief support through swing turns.
    if (has_motion && (recently_changed || !tracker->last_dynamic_stamp.isZero()) &&
        candidate.novel_fraction >= mp_->min_motion_novel_fraction)
      tracker->last_motion_stamp = cloud_stamp_;
    bool confirmed = !tracker->last_motion_stamp.isZero() &&
                     (cloud_stamp_ - tracker->last_motion_stamp).toSec() <= 0.6;
    ROS_DEBUG_STREAM("FAPP_TRACK " << cloud_stamp_.toNSec() << " "
        << candidate.state.position.transpose() << " " << tracker->id << " "
        << matched[i] << " " << tracker->observations.size() << " "
        << displacement << " " << observed_speed << " " << candidate.changed << " "
        << candidate.novel_fraction << " " << confirmed);
    if (!confirmed) continue;
    tracker->last_dynamic_stamp = cloud_stamp_;

    int detection_index = detections.size();
    detections.push_back(candidate.state);
    published_pairs.push_back({detection_index, j});
    deleted_indices.push_back(candidate.indices);
    for (int index : candidate.indices.indices)
      dynamic_points->points.push_back(ClusterPoints->points[index]);

    obj_state_msgs::State state;
    state.header = states.header;
    state.position.x = tracker->pos().x();
    state.position.y = tracker->pos().y();
    state.position.z = tracker->pos().z();
    state.velocity.x = tracker->vel().x();
    state.velocity.y = tracker->vel().y();
    state.velocity.z = tracker->vel().z();
    state.size.x = candidate.state.size.x();
    state.size.y = candidate.state.size.y();
    state.size.z = candidate.state.size.z();
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
    state.position = trackers[trackerIndex]->pos();
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
