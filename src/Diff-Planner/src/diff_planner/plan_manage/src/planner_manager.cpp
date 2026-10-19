// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace diff_planner
{

  // SECTION interfaces for setup and query

  DiffPlannerManager::DiffPlannerManager() {}

  DiffPlannerManager::~DiffPlannerManager() { std::cout << "des manager" << std::endl; }

  void DiffPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    nh.param("manager/enable_replan_quality_gate", enable_replan_quality_gate_, true);
    nh.param("manager/replan_quality_sample_dt", replan_quality_sample_dt_, 0.10);
    nh.param("manager/replan_clearance_search_radius", replan_clearance_search_radius_, 0.50);
    nh.param("manager/replan_max_clearance_drop",
             replan_quality_config_.max_clearance_drop, 0.03);
    nh.param("manager/replan_min_clearance_improvement",
             replan_quality_config_.min_clearance_improvement, 0.05);
    nh.param("manager/replan_max_length_ratio",
             replan_quality_config_.max_length_ratio, 1.05);
    nh.param("manager/replan_min_length_improvement_ratio",
             replan_quality_config_.min_length_improvement_ratio, 0.05);
    nh.param("manager/replan_max_jerk_ratio",
             replan_quality_config_.max_jerk_ratio, 1.50);
    nh.param("manager/replan_min_jerk_improvement_ratio",
             replan_quality_config_.min_jerk_improvement_ratio, 0.20);
    nh.param("manager/replan_max_handoff_position_error",
             replan_quality_config_.max_handoff_position_error, 0.06);
    nh.param("manager/replan_max_handoff_velocity_error",
             replan_quality_config_.max_handoff_velocity_error, 0.10);

    if (!std::isfinite(replan_quality_sample_dt_) || replan_quality_sample_dt_ <= 0.0)
      replan_quality_sample_dt_ = 0.10;
    if (!std::isfinite(replan_clearance_search_radius_) ||
        replan_clearance_search_radius_ <= 0.0)
      replan_clearance_search_radius_ = 0.50;

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  void DiffPlannerManager::updateDynamicObjects(const obj_state_msgs::ObjectsStates &msg)
  {
    std::vector<DynamicObstacleState> obstacles;
    for (const auto &state : msg.states)
    {
      DynamicObstacleState obstacle;
      obstacle.position = Eigen::Vector3d(state.position.x, state.position.y, state.position.z);
      obstacle.velocity = Eigen::Vector3d(state.velocity.x, state.velocity.y, state.velocity.z);
      obstacle.size = Eigen::Vector3d(state.size.x, state.size.y, state.size.z);
      const ros::Time stamp = state.header.stamp.isZero() ? msg.header.stamp : state.header.stamp;
      obstacle.stamp = stamp.isZero() ? ros::Time::now().toSec() : stamp.toSec();
      obstacles.push_back(obstacle);
    }
    ploy_traj_opt_->setDynamicObstacles(obstacles);
  }

  bool DiffPlannerManager::dynamicAvoidanceEnabled() const
  {
    return ploy_traj_opt_->getDynamicObstacleConfig().enabled;
  }

  bool DiffPlannerManager::dynamicTrajectoryBlocked(
      const std::vector<std::pair<double, Eigen::Vector3d>> &samples,
      double now, SwingCollisionResult *result) const
  {
    for (const auto &sample : samples)
      for (const auto &obstacle : ploy_traj_opt_->getDynamicObstacles())
      {
        double clearance;
        Eigen::Vector3d obstacle_position;
        if (evaluateDynamicObstacle(sample.second, obstacle, now + sample.first, now,
                                    ploy_traj_opt_->getDynamicObstacleConfig(),
                                    &clearance, nullptr, &obstacle_position) && clearance < 0.0)
        {
          if (result != nullptr)
          {
            result->obstacle_id = 0;
            result->time_from_now = sample.first;
            result->obstacle_position = obstacle_position;
            result->vehicle_position = sample.second;
          }
          return true;
        }
      }
    return false;
  }

  bool DiffPlannerManager::avoidanceTrajectoryBlocked(
      const std::vector<std::pair<double, Eigen::Vector3d>> &samples,
      const double now, SwingCollisionResult *result) const
  {
    if (!dynamicAvoidanceEnabled()) return false;
    for (const auto &sample : samples)
      if (ploy_traj_opt_->staticPositionOccupied(sample.second))
      {
        if (result != nullptr)
        {
          result->time_from_now = sample.first;
          result->vehicle_position = sample.second;
          result->obstacle_position = sample.second;
        }
        return true;
      }
    return !ploy_traj_opt_->fappPlannerPriority() &&
        dynamicTrajectoryBlocked(samples, now, result);
  }

  bool DiffPlannerManager::optimizeDynamicCandidates(
      const poly_traj::MinJerkOpt &initial, const DynamicObstacleState &obstacle,
      const double collision_time, poly_traj::MinJerkOpt &best,
      const std::function<bool(const poly_traj::MinJerkOpt &)> &commit)
  {
    const auto nominal = initial.getTraj();
    const int last = nominal.getPieceNum();
    Eigen::Matrix3d head, tail;
    head << nominal.getJuncPos(0), nominal.getJuncVel(0), nominal.getJuncAcc(0);
    tail << nominal.getJuncPos(last), nominal.getJuncVel(last), nominal.getJuncAcc(last);
    const double budget = ploy_traj_opt_->planningTimeRemaining();
    const double publish_budget = 0.1 * budget;
    const double reference = ploy_traj_opt_->planningReferenceTime();
    const int directions[] = {0, 1, -1};
    const char *names[] = {"underpass", "left", "right"};
    // MinJerkOpt provides deep assignment but no owning copy constructor.
    // Keep fixed slots and sort only their costs/indices to avoid shallow
    // copies of its banded-system allocation when a vector moves elements.
    poly_traj::MinJerkOpt side_trajectories[2];
    std::vector<std::pair<double, int>> sides;
    for (int index = 0; index < 3; ++index)
    {
      // Cumulative deadlines carry unused time forward without letting a
      // failed down/left attempt consume the next candidate's allocation.
      const double remaining = ploy_traj_opt_->planningTimeRemaining();
      const double reserved = std::max(publish_budget,
          index == 0 ? 0.5 * budget : (index == 1 ? 0.25 * budget : 0.0));
      ploy_traj_opt_->beginDynamicCandidate(std::max(0.0, remaining - reserved));
      std::vector<Eigen::Vector3d> path;
      bool success = ploy_traj_opt_->searchDynamicCandidate(
          head.col(0), tail.col(0), obstacle, collision_time, directions[index], path);
      double cost = std::numeric_limits<double>::infinity();
      poly_traj::MinJerkOpt seed;
      poly_traj::MinJerkOpt candidate;
      if (success)
      {
        // Use arc-length samples of the searched route rather than reverting
        // to a straight polynomial between the original endpoints.
        std::vector<double> distance(path.size(), 0.0);
        for (size_t i = 1; i < path.size(); ++i)
          distance[i] = distance[i - 1] + (path[i] - path[i - 1]).norm();
        const double length = distance.back();
        const int pieces = std::max(2, static_cast<int>(std::ceil(
            length / std::min(0.3, pp_.polyTraj_piece_length))));
        Eigen::MatrixXd inner(3, pieces - 1);
        size_t segment = 1;
        for (int i = 1; i < pieces; ++i)
        {
          const double along = length * i / pieces;
          while (segment + 1 < path.size() && distance[segment] < along) ++segment;
          const double span = distance[segment] - distance[segment - 1];
          inner.col(i - 1) = path[segment - 1] +
              (span > 1.0e-9 ? (along - distance[segment - 1]) / span : 0.0) *
              (path[segment] - path[segment - 1]);
        }
        Eigen::VectorXd durations = Eigen::VectorXd::Constant(
            pieces, std::max(0.1, length / (pieces * pp_.max_vel_)));
        seed.reset(head, tail, pieces);
        seed.generate(inner, durations);
        std::vector<std::pair<int, int>> segments;
        success = ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, seed, true) !=
            PolyTrajOptimizer::CHK_RET::ERR;
        if (success)
        {
          ploy_traj_opt_->setUseMultitopologyTrajs(false);
          success = ploy_traj_opt_->optimizeTrajectory(head, tail, inner, durations, cost);
          if (success)
          {
            candidate = ploy_traj_opt_->getMinJerkOpt();
            const double now = ros::Time::now().toSec();
            success = ploy_traj_opt_->trajectoryCollisionFree(
                candidate.getTraj(), reference, reference) &&
                ploy_traj_opt_->trajectoryCollisionFree(candidate.getTraj(), now, now);
          }
        }
      }
      ROS_INFO("[FAPP候选] route=%s success=%d cost=%.6f remaining=%.3fs",
               names[index], success, cost, ploy_traj_opt_->planningTimeRemaining());
      ploy_traj_opt_->endDynamicCandidate();
      if (success && std::isfinite(cost))
      {
        if (index == 0)
        {
          if (commit(candidate))
          {
            best = candidate;
            ROS_INFO("[FAPP候选] selected=%s cost=%.6f", names[index], cost);
            return true;
          }
          // Final publication validation is part of the down attempt. If it
          // fails, continue with both side routes under the same deadline.
        }
        else
        {
          side_trajectories[index - 1] = candidate;
          sides.emplace_back(cost, index);
        }
      }
    }
    std::stable_sort(sides.begin(), sides.end(),
        [](const std::pair<double, int> &a, const std::pair<double, int> &b) {
          return a.first < b.first;
        });
    const double selection_time = ros::Time::now().toSec();
    for (const auto &side : sides)
      if (ploy_traj_opt_->trajectoryCollisionFree(side_trajectories[side.second - 1].getTraj(),
              selection_time, selection_time) && commit(side_trajectories[side.second - 1]))
      {
        best = side_trajectories[side.second - 1];
        ROS_INFO("[FAPP候选] selected=%s cost=%.6f", names[side.second], side.first);
        return true;
      }
    return false;
  }

  bool DiffPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal,
      const bool require_improvement, bool *trajectory_replaced)
  {
    if (trajectory_replaced != nullptr)
      *trajectory_replaced = false;
    dynamic_candidates_failed_ = false;
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;
    ploy_traj_opt_->beginPlanningCycle();
    const auto finish_planning_cycle = [this](const bool result) {
      ploy_traj_opt_->endPlanningCycle();
      return result;
    };

    bool retained_current_trajectory = false;
    const auto commit_candidate = [&](const poly_traj::MinJerkOpt &candidate_opt) {
      if (require_improvement && enable_replan_quality_gate_ &&
          traj_.local_traj.traj.getPieceNum() > 0)
      {
        const double current_elapsed = std::max(
            0.0, std::min(ros::Time::now().toSec() - traj_.local_traj.start_time,
                          traj_.local_traj.duration));
        const poly_traj::Trajectory candidate = candidate_opt.getTraj();
        const ReplanTrajectoryMetrics current_metrics =
            evaluateTrajectoryQuality(traj_.local_traj.traj, current_elapsed);
        const ReplanTrajectoryMetrics candidate_metrics =
            evaluateTrajectoryQuality(candidate, 0.0);
        const double position_error =
            (candidate.getPos(0.0) -
             traj_.local_traj.traj.getPos(current_elapsed)).norm();
        const double velocity_error =
            (candidate.getVel(0.0) -
             traj_.local_traj.traj.getVel(current_elapsed)).norm();
        const ReplanQualityDecision decision = ReplanQualityGate::decide(
            current_metrics, candidate_metrics, position_error, velocity_error,
            replan_quality_config_);

        ROS_WARN("[周期重规划验收] %s：%s；净空 old/new=%.3f/%.3f m，"
                 "长度=%.3f/%.3f m，jerk=%.3f/%.3f，交接位置/速度误差=%.3f m/%.3f m/s。",
                 decision.accept ? "接受新轨迹" : "保留旧轨迹",
                 decision.reason.c_str(), current_metrics.min_clearance,
                 candidate_metrics.min_clearance, current_metrics.length,
                 candidate_metrics.length, current_metrics.jerk_cost,
                 candidate_metrics.jerk_cost, position_error, velocity_error);
        if (!decision.accept)
        {
          retained_current_trajectory = true;
          return true;
        }
      }

      if (!setLocalTrajFromOpt(candidate_opt, touch_goal))
      {
        ROS_ERROR("Failed to store the optimized local trajectory; report planning failure.");
        return false;
      }
      if (trajectory_replaced != nullptr)
        *trajectory_replaced = true;
      return true;
    };

    static int count = 0;
    cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << count++ << "\033[0m" << endl;
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;
    // if ((start_pt - local_target_pt).norm() < 0.2)
    //   cout << "Close to goal" << endl;

    // 2026-07-06: 补充重规划入口诊断日志，直接打印起终点坐标及其在膨胀占据图中的状态，便于定位 world/odom/cloud 错位。
    const int start_occ = grid_map_->getInflateOccupancy(start_pt);
    const int goal_occ = grid_map_->getInflateOccupancy(local_target_pt);
    ROS_WARN_STREAM("[replan_diag] start_pt=" << start_pt.transpose()
                    << " start_occ=" << start_occ
                    << " goal_pt=" << local_target_pt.transpose()
                    << " goal_occ=" << goal_occ
                    << " touch_goal=" << touch_goal);

    /*** STEP 1: INIT ***/
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return finish_planning_cycle(false);
    }
    if (ploy_traj_opt_->checkPlanningTimeout("trajectory initialization"))
    {
      return finish_planning_cycle(false);
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    vector<std::pair<int, int>> segments;
    bool has_dynamic_conflict = false;
    DynamicObstacleState first_obstacle;
    double first_collision_time = 0.0;
    const auto find_dynamic_conflict = [&](const poly_traj::Trajectory &nominal) {
      // In planner priority mode the normal optimizer still includes FAPP cost,
      // without requiring a dynamic-clear candidate before it can run.
      if (!dynamicAvoidanceEnabled() || ploy_traj_opt_->fappPlannerPriority()) return false;
      const auto samples = sampleCollisionTrajectory(nominal, 0.0,
          nominal.getTotalDuration(), 0.5 * grid_map_->getResolution());
      const double reference = ploy_traj_opt_->planningReferenceTime();
      for (const auto &sample : samples)
      {
        for (const auto &obstacle : ploy_traj_opt_->getDynamicObstacles())
        {
          double clearance;
          if (evaluateDynamicObstacle(sample.second, obstacle, reference + sample.first,
                                      reference, ploy_traj_opt_->getDynamicObstacleConfig(),
                                      &clearance) && clearance < 0.0)
          {
            first_obstacle = obstacle;
            first_collision_time = sample.first;
            return true;
          }
        }
      }
      return false;
    };
    has_dynamic_conflict = find_dynamic_conflict(initMJO.getTraj());
    if (!has_dynamic_conflict &&
        ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return finish_planning_cycle(false);
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;

    // ROS_ERROR("BBBB");

    if (has_dynamic_conflict)
    {
      flag_success = optimizeDynamicCandidates(initMJO, first_obstacle,
                                               first_collision_time, best_MJO, commit_candidate);
      t_opt = ros::Time::now() - t_start;
    }
    else if (pp_.use_multitopology_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;

      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        ploy_traj_opt_->setUseMultitopologyTrajs(true);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(), final_cost))
        {
          success[i] = true;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
      }

      t_opt = ros::Time::now() - t_start;

      if (trajs.size() > 1)
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajs.size() << ",\033[1;0m"
             << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << endl;
      }

      visualization_->displayMultiOptimalPathList(vis_trajs, 0.1); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(), final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();

      t_opt = ros::Time::now() - t_start;
    }

    /*** STEP 3: Store and display results ***/
    // A static detour may introduce a dynamic conflict that was absent from
    // the nominal initialization. Apply the same candidate selection then.
    if (flag_success && !has_dynamic_conflict && find_dynamic_conflict(best_MJO.getTraj()))
    {
      has_dynamic_conflict = true;
      poly_traj::MinJerkOpt detour;
      detour = best_MJO;
      flag_success = optimizeDynamicCandidates(detour, first_obstacle,
                                               first_collision_time, best_MJO, commit_candidate);
      t_opt = ros::Time::now() - t_start;
    }
    if (flag_success && !has_dynamic_conflict)
      flag_success = commit_candidate(best_MJO);
    cout << "Success=" << (flag_success ? "yes" : "no") << endl;
    if (has_dynamic_conflict && !flag_success)
    {
      dynamic_candidates_failed_ = true;
      dynamic_candidate_failure_.time_from_now = first_collision_time;
      dynamic_candidate_failure_.vehicle_position = start_pt;
      dynamic_candidate_failure_.obstacle_position = first_obstacle.position;
    }
    if (flag_success)
    {
      if (retained_current_trajectory)
      {
        continous_failures_count_ = 0;
        return finish_planning_cycle(true);
      }
      static double sum_time = 0;
      static int count_success = 0;
      sum_time += (t_init + t_opt).toSec();
      count_success++;
      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);
      // cout << "total time:\033[42m" << (t_init + t_opt).toSec()
      //      << "\033[0m,init:" << t_init.toSec()
      //      << ",optimize:" << t_opt.toSec()
      //      << ",avg_time=" << sum_time / count_success << endl;

      cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayOptimalList(cstr_pts, 0);

      continous_failures_count_ = 0;
    }
    else
    {
      cstr_pts = has_dynamic_conflict ? initMJO.getInitConstraintPoints(getCpsNumPrePiece())
          : ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(getCpsNumPrePiece());
      visualization_->displayFailedList(cstr_pts, 0);

      continous_failures_count_++;
    }

    return finish_planning_cycle(flag_success);
  }

  double DiffPlannerManager::estimateInflatedClearance(
      const Eigen::Vector3d &position)
  {
    if (grid_map_->getInflateOccupancy(position) != 0)
      return 0.0;

    const double resolution = grid_map_->getResolution();
    const int max_cells = std::max(
        1, static_cast<int>(std::ceil(replan_clearance_search_radius_ /
                                     resolution)));
    double nearest = replan_clearance_search_radius_;
    for (int shell = 1; shell <= max_cells; ++shell)
    {
      bool found = false;
      for (int x = -shell; x <= shell; ++x)
      {
        for (int y = -shell; y <= shell; ++y)
        {
          for (int z = -shell; z <= shell; ++z)
          {
            if (std::max(std::max(std::abs(x), std::abs(y)), std::abs(z)) != shell)
              continue;
            const Eigen::Vector3d offset(x * resolution, y * resolution,
                                         z * resolution);
            if (offset.norm() >= nearest)
              continue;
            if (grid_map_->getInflateOccupancy(position + offset) != 0)
            {
              nearest = offset.norm();
              found = true;
            }
          }
        }
      }
      if (found)
        return nearest;
    }
    return nearest;
  }

  ReplanTrajectoryMetrics DiffPlannerManager::evaluateTrajectoryQuality(
      const poly_traj::Trajectory &trajectory, const double start_time)
  {
    ReplanTrajectoryMetrics metrics;
    const double duration = trajectory.getTotalDuration();
    if (trajectory.getPieceNum() <= 0 || !std::isfinite(duration) ||
        duration <= 0.0 || !std::isfinite(start_time))
      return metrics;

    const double begin = std::max(0.0, std::min(start_time, duration));
    const double remaining = duration - begin;
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(remaining / replan_quality_sample_dt_)));
    Eigen::Vector3d previous = trajectory.getPos(begin);
    if (!previous.allFinite())
      return metrics;

    metrics.valid = true;
    metrics.min_clearance = std::numeric_limits<double>::infinity();
    for (int index = 0; index <= sample_count; ++index)
    {
      const double ratio = static_cast<double>(index) / sample_count;
      const double time = begin + ratio * remaining;
      const Eigen::Vector3d position = trajectory.getPos(time);
      const Eigen::Vector3d jerk = trajectory.getJer(time);
      if (!position.allFinite() || !jerk.allFinite())
      {
        metrics.valid = false;
        return metrics;
      }

      if (index > 0)
      {
        metrics.length += (position - previous).norm();
        metrics.jerk_cost += jerk.squaredNorm() * (remaining / sample_count);
      }
      previous = position;
      const double clearance = estimateInflatedClearance(position);
      metrics.min_clearance = std::min(metrics.min_clearance, clearance);
      metrics.colliding = metrics.colliding || clearance <= 0.0;
    }
    return metrics;
  }

  bool DiffPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {
    if (two_piece_entry_)
    {
      // 参考UAV0 e61be99入口段：按速度/加速度共同分配两段时间，
      // 中间点联合求解导数，不把入口拆成两次独立目标或中途停稳。
      const double distance = (local_target_pt - start_pt).norm();
      const double half_duration = std::max(0.01, std::max(
          distance / pp_.max_vel_, std::sqrt(1.5 * distance / pp_.max_acc_)));
      Eigen::Matrix3d head, tail;
      head << start_pt, start_vel, start_acc;
      tail << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();
      Eigen::MatrixXd midpoint(3, 1);
      midpoint.col(0) = 0.5 * (start_pt + local_target_pt);
      initMJO.reset(head, tail, 2);
      initMJO.generate(midpoint, Eigen::Vector2d::Constant(half_duration));
      return true;
    }

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if (t_to_lc_end < 0)
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void DiffPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool DiffPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal,
                                             const bool check_dynamic_obstacles,
                                             const double max_initial_occupied_prefix)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    return setLocalTrajFromTrajectory(std::move(traj),
        ConstraintPoints::two_thirds_id(cps, touch_goal), check_dynamic_obstacles,
        max_initial_occupied_prefix);
  }

  bool DiffPlannerManager::setLocalTrajFromTrajectory(
      poly_traj::Trajectory traj, int constraint_end,
      bool check_dynamic_obstacles, double max_initial_occupied_prefix)
  {
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, constraint_end, pts_to_check);
    if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
    {
      const double start_time = ros::Time::now().toSec();
      if (check_dynamic_obstacles && dynamicAvoidanceEnabled())
      {
        if (!ploy_traj_opt_->trajectoryCollisionFree(
                traj, start_time, start_time, max_initial_occupied_prefix))
        {
          ROS_WARN("[FAPP动态避障] 轨迹未通过静态或动态整机检查，不发布该轨迹。");
          return false;
        }
      }
      traj_.setLocalTraj(traj, pts_to_check, start_time);
    }

    return ret;
  }

  bool DiffPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    // The existing stop command must still replace the old moving trajectory
    // when the current location itself lies inside a predicted ball envelope.
    setLocalTrajFromOpt(stopMJO, false, false);

    return true;
  }

  bool DiffPlannerManager::OccupiedStartRecovery(const Eigen::Vector3d &start_pos,
                                                 const Eigen::Vector3d &target_pos,
                                                 double max_speed,
                                                 double max_initial_occupied_prefix)
  {
    const double distance = (target_pos - start_pos).norm();
    if (!start_pos.allFinite() || !target_pos.allFinite() ||
        !std::isfinite(max_speed) || max_speed <= 0.0 || distance < 1.0e-3)
    {
      ROS_ERROR("Cannot generate occupied-start recovery trajectory: invalid start, target, or speed.");
      return false;
    }

    // A zero-endpoint-velocity quintic has a peak speed of 1.875 * distance / duration.
    // Size the duration from that analytical bound so recovery remains deliberately slow.
    const double duration = std::max(1.0, 1.875 * distance / max_speed);
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> head_state, tail_state;
    head_state << start_pos, zero, zero;
    tail_state << target_pos, zero, zero;

    poly_traj::MinJerkOpt recovery_mjo;
    recovery_mjo.reset(head_state, tail_state, 1);
    Eigen::MatrixXd inner_points(3, 0);
    Eigen::VectorXd durations(1);
    durations(0) = duration;
    recovery_mjo.generate(inner_points, durations);

    if (!setLocalTrajFromOpt(recovery_mjo, false, true, max_initial_occupied_prefix))
    {
      ROS_ERROR("Failed to store occupied-start recovery trajectory.");
      return false;
    }

    ROS_WARN("Generated occupied-start recovery trajectory: distance=%.3f m, duration=%.3f s, peak_speed<=%.3f m/s.",
             distance, duration, max_speed);
    return true;
  }

  bool DiffPlannerManager::OccupiedHistoryRecovery(
      const std::vector<Eigen::Vector3d> &reverse_path, double max_speed,
      double max_initial_occupied_prefix)
  {
    if (!std::isfinite(max_speed) || max_speed <= 0.0 ||
        !std::isfinite(pp_.max_acc_) || pp_.max_acc_ <= 0.0)
      return false;

    std::vector<Eigen::Vector3d> path;
    for (const auto &point : reverse_path)
    {
      if (!point.allFinite())
        return false;
      // 先删除数值上重复的点和同向共线点。
      if (!path.empty() && (point - path.back()).norm() <= 1.0e-6)
        continue;
      path.push_back(point);
      while (path.size() >= 3)
      {
        const std::size_t n = path.size();
        const Eigen::Vector3d before = path[n - 2] - path[n - 3];
        const Eigen::Vector3d after = path[n - 1] - path[n - 2];
        // 此处仅合并同向共线点，后续按整段偏差简化细碎转折。
        if (before.dot(after) <= 0.0 ||
            before.cross(after).norm() > 1.0e-6 * before.norm() * after.norm())
          break;
        path.erase(path.end() - 2);
      }
    }
    if (path.size() < 2)
      return false;

    // 合并厘米以内的细碎转折，避免每个历史采样点都起停一次。
    // 对整段中所有原始顶点检查偏差，防止逐点合并后累计切向拐角。
    const double tolerance = std::min(0.01, 0.2 * grid_map_->getResolution());
    std::vector<bool> keep(path.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::pair<std::size_t, std::size_t>> ranges{{0, path.size() - 1}};
    while (!ranges.empty())
    {
      const auto range = ranges.back();
      ranges.pop_back();
      const Eigen::Vector3d chord = path[range.second] - path[range.first];
      const double squared_length = chord.squaredNorm();
      double max_deviation = tolerance;
      std::size_t split = range.first;
      for (std::size_t i = range.first + 1; i < range.second; ++i)
      {
        // 投影限制在线段内，保留超出起终点的转折。
        const double ratio = squared_length > 1.0e-12 ? std::max(0.0, std::min(1.0,
            (path[i] - path[range.first]).dot(chord) / squared_length)) : 0.0;
        const double deviation = (path[i] - path[range.first] - ratio * chord).norm();
        if (deviation > max_deviation)
        {
          max_deviation = deviation;
          split = i;
        }
      }
      if (split == range.first && squared_length > 1.0e-12)
      {
        // 共线段内部也可能先退后进，保留超过容差的回折顺序。
        const double length = std::sqrt(squared_length);
        double furthest = 0.0;
        std::size_t furthest_index = range.first;
        for (std::size_t i = range.first + 1; i < range.second; ++i)
        {
          const double progress = (path[i] - path[range.first]).dot(chord) / length;
          if (furthest - progress > tolerance)
          {
            split = furthest_index;
            break;
          }
          if (progress > furthest)
          {
            furthest = progress;
            furthest_index = i;
          }
        }
      }
      if (split != range.first)
      {
        keep[split] = true;
        ranges.emplace_back(range.first, split);
        ranges.emplace_back(split, range.second);
      }
    }
    std::vector<Eigen::Vector3d> simplified;
    for (std::size_t i = 0; i < path.size(); ++i)
      if (keep[i] && (simplified.empty() || (path[i] - simplified.back()).norm() > 1.0e-6))
        simplified.push_back(path[i]);
    path.swap(simplified);
    if (path.size() < 2)
      return false;

    poly_traj::Trajectory recovery;
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    double path_length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
      const double distance = (path[i] - path[i - 1]).norm();
      // 零端速五次曲线的峰速为1.875*d/T，峰加速度为(10/sqrt(3))*d/T^2。
      // 复用配置的速度/加速度上限分配时长，不对每个细历史点强加1s停转时长。
      const double duration = std::max(1.875 * distance / max_speed,
          std::sqrt((10.0 / std::sqrt(3.0)) * distance / pp_.max_acc_));
      Eigen::Matrix3d head, tail;
      head << path[i - 1], zero, zero;
      tail << path[i], zero, zero;
      poly_traj::MinJerkOpt segment;
      segment.reset(head, tail, 1);
      segment.generate(Eigen::MatrixXd(3, 0), Eigen::VectorXd::Constant(1, duration));
      recovery.append(segment.getTraj());
      path_length += distance;
    }

    // 与普通发布共用采样/占据起点规则；整条折线只登记一次轨迹编号和起始时间。
    const int constraint_count = recovery.getPieceNum() * getCpsNumPrePiece();
    if (!setLocalTrajFromTrajectory(std::move(recovery),
            constraint_count - (constraint_count - 1) / 3,
            true, max_initial_occupied_prefix))
      return false;

    ROS_WARN("Generated history-polyline recovery: pieces=%d, path_length=%.3f m, "
             "duration=%.3f s, peak_speed<=%.3f m/s.",
             traj_.local_traj.traj.getPieceNum(), path_length,
             traj_.local_traj.duration, max_speed);
    return true;
  }

  bool DiffPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;
    if (traj_.swarm_traj[drone_id].drone_id != drone_id) // The trajectory is invalid
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < (getSwarmClearance() + traj_.swarm_traj[drone_id].des_clearance) )
      {
        return true;
      }
    }

    return false;
  }

  bool DiffPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace diff_planner
