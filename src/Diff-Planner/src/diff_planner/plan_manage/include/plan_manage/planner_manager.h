#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <limits>
#include <functional>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>
#include <plan_manage/replan_quality_gate.h>
#include <plan_manage/swing_obstacle_guard.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <plan_manage/fapp_history.h>

namespace diff_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class DiffPlannerManager
  {
    // SECTION stable
  public:
    DiffPlannerManager();
    ~DiffPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    void setEntryMode(bool enabled);
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal,
        const bool require_improvement = false,
        bool *trajectory_replaced = nullptr);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool segmentedPlanning() const { return route_segment_count_ > 1 && !two_piece_entry_; }
    bool prepareLocalSegment(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                             Eigen::Vector3d& target, Eigen::Vector3d& velocity, bool& final);
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool OccupiedStartRecovery(const Eigen::Vector3d &start_pos,
                               const Eigen::Vector3d &target_pos,
                               double max_speed,
                               double max_initial_occupied_prefix = 0.20);
    bool OccupiedHistoryRecovery(const std::vector<Eigen::Vector3d> &reverse_path,
                                 double max_speed,
                                 double max_initial_occupied_prefix = 0.20);
    bool checkCollision(int drone_id);
    void updateDynamicObjects(const obj_state_msgs::ObjectsStates &msg,
                              const std::vector<bool>& leader_matches = {});
    void updateLeaderDynamicObjects(const std::vector<DynamicObstacleState>& observations);
    bool dynamicTrajectoryBlocked(
        const std::vector<std::pair<double, Eigen::Vector3d>> &samples,
        double now, SwingCollisionResult *result = nullptr) const;
    bool dynamicAvoidanceEnabled() const;
    bool inDynamicPassage(const Eigen::Vector3d &position, double now) const;
    void observeDynamicPosition(const Eigen::Vector3d& actual);
    bool dynamicCandidatesFailed(SwingCollisionResult *result) const {
      if (result != nullptr) *result = dynamic_candidate_failure_;
      return dynamic_candidates_failed_;
    }
    bool avoidanceTrajectoryBlocked(
        const std::vector<std::pair<double, Eigen::Vector3d>> &samples,
        double now, SwingCollisionResult *result = nullptr) const;
    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal,
                            bool check_dynamic_obstacles = true,
                            double max_initial_occupied_prefix = 0.0);
    inline double getSwarmClearance(void) { return ploy_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;
    bool two_piece_entry_{false};
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    FappHistory fapp_history_;
    FappHistory fapp_leader_history_;
    bool fapp_require_leader_confirmation_{false};
    std::vector<DynamicObstacleState> fapp_leader_observations_;
    bool fapp_memory_enabled_{false};
    void refreshDynamicHistory();
    int route_segment_count_{1};
    double route_segment_min_length_{0.60};
    double route_endpoint_clearance_{0.15};
    double route_segment_length_{0.0};
    Eigen::Vector3d route_segment_goal_{Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN())};
    std::vector<Eigen::Vector3d> route_segment_guide_;
    double normal_max_vel_{0.0}, normal_optimizer_max_vel_{0.0}, entry_max_vel_{0.0};
    double normal_max_acc_{0.0}, entry_max_acc_{0.0};
    bool setLocalTrajFromTrajectory(poly_traj::Trajectory traj, int constraint_end,
                                    bool check_dynamic_obstacles,
                                    double max_initial_occupied_prefix);
    bool optimizeDynamicCandidates(const poly_traj::MinJerkOpt &initial,
                                   const DynamicObstacleState &obstacle,
                                   double collision_time,
                                   poly_traj::MinJerkOpt &best,
                                   const std::function<bool(const poly_traj::MinJerkOpt &)> &commit);
    ReplanTrajectoryMetrics evaluateTrajectoryQuality(
        const poly_traj::Trajectory &trajectory, double start_time);
    double estimateInflatedClearance(const Eigen::Vector3d &position);
    bool endpointHasPreferredClearance(const Eigen::Vector3d &position);

    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    bool dynamic_candidates_failed_{false};
    SwingCollisionResult dynamic_candidate_failure_;

    int continous_failures_count_{0};
    bool enable_replan_quality_gate_{true};
    double replan_quality_sample_dt_{0.10};
    double replan_clearance_search_radius_{0.50};
    ReplanQualityConfig replan_quality_config_;

  public:
    typedef unique_ptr<DiffPlannerManager> Ptr;

    // !SECTION
  };
} // namespace diff_planner

#endif
