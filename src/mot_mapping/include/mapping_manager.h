#ifndef _mot_mapping_H_
#define _mot_mapping_H_

#include <chrono>
#include <queue>
#include <unordered_set>
// ROS
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <obj_state_msgs/State.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
// Algorithm
#include "DBSCAN_kdtree.h"
#include "Hungarian.h"
#include "target_ekf.hpp"
#include "ikd_Tree.h"
#include "ikd_Tree_impl.h"
#include "online_background.h"

using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Points;
typedef pcl::PointCloud<PointType>::Ptr PointsPtr;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

struct MapParam {
  bool swing_only = false;
  bool object_motion_evidence_enabled = false;
  bool object_motion_local_bidirectional_enabled = false;
  double object_motion_local_match_fraction = 0.60;
  double object_motion_local_match_advantage = 0.20;
  double object_motion_match_fraction = 0.45;
  double object_motion_match_advantage = 0.30;
  double object_motion_depth_fraction = 0.40;
  // DBSCAN Params
  int core_pts;
  double tolerance;
  int min_cluster;
  int max_cluster;
  double cluster_voxel_size;
  bool density_split_enabled;
  bool candidate_partition_enabled;
  bool object_candidate_split_enabled = false;
  bool object_candidate_isolation_enabled = false;
  double core_radius;
  double core_density_fraction;
  // Dynamic Detection Params
  double thresh_dist;
  double thresh_var;
  double motion_displacement;
  int motion_confirmation_frames;
  bool time_evidence_enabled;
  double motion_window;
  double motion_confirmation_duration, motion_continuity_tolerance;
  bool confirmed_reacquisition_enabled;
  double reacquisition_max_gap;
  double min_motion_speed;
  double min_motion_novel_fraction;
  double tracked_motion_novel_fraction;
  double max_above_uav;
  bool geometry_association_enabled;
  bool global_assignment_enabled;
  double position_measurement_variance;
  bool translated_startup_enabled;
  double translated_startup_fraction;
  bool competition_association_enabled;
  bool point_source_competition_enabled;
  bool publish_measured_body_position_enabled;
  bool dense_change_evidence_enabled;
  bool published_startup_source_enabled;
  bool bidirectional_ray_evidence_enabled;
  bool bidirectional_static_suppression_enabled;
  bool strict_depth_evidence_enabled;
  bool depth_evidence_enabled;
  bool occlusion_startup_enabled;
  Eigen::Vector3d lidar_offset;
  bool background_surface_enabled;
  bool background_plane_enabled;
  double background_plane_distance;
  int background_plane_min_points;
  int background_plane_max_planes;
  bool background_consistency_enabled;
  bool background_local_planes_enabled;
  bool background_local_post_association;
  double background_consistency_distance;
  double background_consistency_fraction;
  int background_consistency_frames;
  bool background_consistency_post_association;
  bool shape_consistency_enabled;
  double shape_consistency_distance;
  double shape_consistency_fraction;
  bool background_reference_enabled;
  bool reference_motion_enabled;
  bool reference_strong_two_frame_enabled;
  double reference_capture_duration;
  double reference_distance;
  double reference_fraction;
  bool online_background_enabled;
  double online_voxel_size;
  double online_rotation_rate;
  double online_jump_distance;
  double online_scene_overlap;
  // Lidar Params
  Eigen::Vector3d Range;
  // ROS Params
  std::string odom_topic;
  std::string lidar_topic;
  bool nearest_available_odom_enabled;
};

struct MapData {
  Eigen::Matrix3d R; //Currnet IMU Orientation
  Eigen::Vector3d T; //Currnet IMU Positon
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ObjectState {
  int id;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d size;
};

namespace mot_mapping {

class MappingRos {

private:
  pcl::VoxelGrid<PointType> vox;
  DBSCANKdtreeCluster<PointType> dbscan;
  KD_TREE<PointType>::Ptr ikdtree_ptr;

  PointsPtr Remaining_Points;
  int frame_num;
  std::deque<PointsPtr> previous_points;
  std::deque<PointsPtr> buffer;
  std::deque<ros::Time> observed_cloud_stamps_;
  std::deque<MapData> observed_sensor_poses;
  struct RawObservation {
    ros::Time stamp;
    PointsPtr points;
    MapData sensor_pose;
  };
  std::deque<RawObservation> timed_observations_;
  RawObservation bidirectional_previous_observation_;
  ros::Time next_background_sample_stamp_, last_background_sample_stamp_;
  std::deque<PointsPtr> input_point;
  std::deque<std::shared_ptr<pcl::search::KdTree<PointType>>> background_history_;
  PointsPtr reference_points_;
  std::shared_ptr<pcl::search::KdTree<PointType>> reference_tree_;
  ros::Time reference_start_stamp_;
  std::unique_ptr<mot_mapping::OnlineBackground> online_background_;
  std::shared_ptr<pcl::search::KdTree<PointType>> online_background_tree_;
  MapData online_previous_pose_;
  bool online_has_previous_pose_ = false;
  std::unordered_set<std::string> online_previous_voxels_;
  std::vector<pcl::PointIndices> deleted_indices;
  std::vector<BoxPointType> delete_boxes;

  std::vector<std::shared_ptr<Ekf>> trackers;
  std::vector<ObjectState> detections;

  ros::Time t_start;

private:
  ros::NodeHandle &nh;
  ros::Timer ekf_predict_timer_, map_pub_timer_;
  ros::Publisher cloudPub, mapPub, staticMapPub, edgePub;
  ros::Publisher objectPosePub, statesPub;
  int id;

  std::string frame_id_;
  ros::Time cloud_stamp_, previous_cloud_stamp_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
  typedef boost::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Cache<nav_msgs::Odometry>> odom_cache_;

  SynchronizerCloudOdom sync_cloud_odom_;

  unique_ptr<MapData> md_;
  unique_ptr<MapParam> mp_;

  float calc_dist(PointType p1, PointType p2);
  void generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths);
  void pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w);
  void pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T);
  void removeStaticPlanes(PointsPtr& points, const PointsPtr& history);

  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const nav_msgs::OdometryConstPtr& odom);
  void ekfPredictCallback(const ros::TimerEvent& e);
  void mapPubCallback(const ros::TimerEvent& e);
  void visualizeFunction(const std::vector<std::pair<int, int>> pairs);

  double iou(ObjectState state1, ObjectState state2);

public:

  MappingRos(ros::NodeHandle &nh);
  ~MappingRos();

  void init();

};

}

#endif
