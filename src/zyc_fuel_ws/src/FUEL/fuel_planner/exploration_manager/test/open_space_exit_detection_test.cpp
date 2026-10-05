#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <boost/make_shared.hpp>

#include <plan_env/sdf_map.h>
#include <exploration_manager/task_search_manager.h>
#include <exploration_manager/open_space_exit_detection.h>

namespace {
void check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << message << std::endl;
  std::exit(1);
}
struct Boundary { double begin, end, lateral; };
struct Scene {
  std::vector<Boundary> boundaries;
  bool unknown_outside = false;
  bool only_center_observed = false;
  bool front_blocked = false;
  bool footprint_blocked = false;
  bool occupied(double forward, double lateral) const {
    if (front_blocked && forward >= 0.80 && forward <= 0.95) return true;
    for (const auto& boundary : boundaries)
      if (forward >= boundary.begin && forward < boundary.end &&
          std::fabs(lateral - boundary.lateral) <= 0.04) return true;
    return false;
  }
  bool free(double forward, double lateral) const {
    if (occupied(forward, lateral)) return false;
    if (forward >= 0.50 && unknown_outside) return false;
    if (forward >= 0.50 && only_center_observed && std::fabs(lateral) > 0.30)
      return false;
    return true;
  }
  bool footprint(double forward, double lateral) const {
    if (footprint_blocked && forward >= 0.75 && forward <= 0.95) return false;
    for (double dx : {-0.20, 0.0, 0.20})
      for (double dy : {-0.20, 0.0, 0.20})
        if (!free(forward + dx, lateral + dy)) return false;
    return true;
  }
};
bool detect(const Scene& scene, fast_planner::open_space_exit::Result& result) {
  return fast_planner::open_space_exit::detect(
      fast_planner::open_space_exit::Config{},
      [&](double x, double y) { return scene.occupied(x, y); },
      [&](double x, double y) { return scene.free(x, y); },
      [&](double x, double y) { return scene.footprint(x, y); }, result);
}
}  // namespace

namespace fast_planner {
class TaskSearchManagerTest {
public:
  static void runMapIntegration() {
    ros::NodeHandle nh("~");
    nh.setParam("mission/task_search/exit/auto_crossing_enabled", false);
    nh.setParam("sdf_map/resolution", 0.10);
    nh.setParam("sdf_map/map_size_x", 8.0);
    nh.setParam("sdf_map/map_size_y", 8.0);
    nh.setParam("sdf_map/map_size_z", 3.0);
    nh.setParam("sdf_map/ground_height", 0.0);
    nh.setParam("sdf_map/obstacles_inflation", 0.15);
    nh.setParam("sdf_map/max_ray_length", 6.0);
    auto map = std::make_shared<SDFMap>();
    map->initMap(nh);
    TaskSearchManager manager;
    manager.initialize(nh);
    check(!manager.exit_auto_crossing_enabled_, "observation parameter did not disable automatic takeover");
    manager.setMap(map);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const Eigen::Vector3d camera(-1,0,1);
    map->inputPointCloud(cloud,0,camera);
    check(map->getUpdateSequence()==0, "empty cloud advanced map evidence");
    cloud.push_back(pcl::PointXYZ(1,0,1));
    map->inputPointCloud(cloud,1,camera);
    check(map->getUpdateSequence()==1, "real cloud did not advance map evidence");
    check(manager.consumeExitMapUpdate(map->getUpdateSequence(),ros::Time(10)),
          "real map revision was not consumed");
    check(!manager.consumeExitMapUpdate(map->getUpdateSequence(),ros::Time(11)),
          "same real map gained another vote");
    check(map->getOccupancy(Eigen::Vector3d(0.4,0,1))==SDFMap::FREE,
          "ray fusion did not establish observed center clearance");
    check(!manager.mapExitFootprintFree(Eigen::Vector3d(0.4,0,1)),
          "a single free center ray hid unknown body volume");
    // 覆盖多个高度和横向体素，证明完整机体范围的FREE与仅中心线FREE不同。
    for (int y=-6;y<=6;++y) {
      for (int z=-4;z<=4;++z) {
        const double row=0.05*y, height=1.0+0.05*z;
        cloud.clear(); cloud.push_back(pcl::PointXYZ(1,row,height));
        map->inputPointCloud(cloud,1,Eigen::Vector3d(-1,row,height));
      }
    }
    check(manager.mapExitFootprintFree(Eigen::Vector3d(0.4,0,1)),
          "fully observed free body volume was rejected");
    cloud.clear(); cloud.push_back(pcl::PointXYZ(0.4,0.1,1));
    for (int n=0;n<4;++n) map->inputPointCloud(cloud,1,Eigen::Vector3d(-1,0.1,1));
    check(!manager.mapExitFootprintFree(Eigen::Vector3d(0.4,0,1)),
          "an occupied body voxel bypassed clearance check");
    TaskSearchManager blending;
    blending.initialize(nh);
    blending.setMap(map);
    const Eigen::Vector3d first_inside(0.1,0,1), next_inside(0.7,0,1);
    check(blending.mapExitFootprintFree(first_inside) &&
          blending.mapExitFootprintFree(next_inside), "inside test anchors are not free");
    check(!blending.mapExitFootprintFree(0.65*first_inside + 0.35*next_inside),
          "inside test anchor blend missed its obstacle");
    TaskSearchManager::OpenExitResult blend_exit;
    blend_exit.portal_center = Eigen::Vector3d(0.8,0,1);
    blend_exit.outward_direction = Eigen::Vector2d::UnitX();
    blending.recordExitEvidence(blend_exit, first_inside, first_inside, ros::Time(20));
    blending.recordExitEvidence(blend_exit, next_inside, first_inside, ros::Time(21));
    check(blending.exit_candidate_hits_ == 2 &&
          blending.mapExitFootprintFree(blending.pending_exit_candidate_) &&
          (blending.pending_exit_candidate_ - next_inside).norm() < 1e-6,
          "occupied blend replaced a validated inside observation point");
    manager.corridor_frame_received_=true;
    manager.corridor_entry_crossed_=true;
    manager.transition_active_=true;
    manager.exit_candidate_hits_=2;
    manager.updateExitCandidate(Eigen::Vector3d(0,0,1));
    check(manager.exit_candidate_hits_==0, "active turn retained exit votes on real map");
    TaskSearchManager observation;
    observation.initialize(nh);
    TaskSearchManager::OpenExitResult detection;
    geometry_msgs::PoseStamped observed_exit;
    const Eigen::Vector3d start(2.0, 3.0, 0.6);
    check(!observation.updateExitDetectionOnly(start, M_PI / 2, observed_exit),
          "observation confirmed without map evidence");
    check(observation.corridor_entry_crossed_ &&
          observation.stableProgressDirection().dot(Eigen::Vector2d::UnitY()) > 0.99,
          "observation did not start inside with first odometry heading");
    observation.updateExitDetectionOnly(start, 0.0, observed_exit);
    check(observation.stableProgressDirection().dot(Eigen::Vector2d::UnitY()) > 0.99,
          "later heading replaced observation forward axis");
    detection.portal_center = Eigen::Vector3d(2.0, 3.5, 0.6);
    detection.outward_direction = Eigen::Vector2d::UnitY();
    for (int n = 0; n < 3; ++n)
      observation.recordExitEvidence(detection, start, start, ros::Time(30 + n));
    check(observation.updateExitDetectionOnly(start, 0.0, observed_exit),
          "observation did not expose confirmed exit");
    check(std::fabs(observed_exit.pose.position.y - 3.5) < 1e-6 &&
          std::fabs(observed_exit.pose.orientation.z - std::sin(M_PI / 4)) < 1e-6,
          "observation exposed wrong exit position or direction");
    check(!observation.stage3Active() && !observation.exit_portal_locked_ &&
          !observation.landingRequested(),
          "observation started crossing or landing after confirmation");
    observation.clearExitEvidence();
    check(!observation.updateExitDetectionOnly(start, 0.0, observed_exit),
          "observation retained revoked detection");
    observation.commitCorridorTurn(-Eigen::Vector3d::UnitX());
    Eigen::Vector3d mapped_direction;
    check(!observation.mappedCorridorDirection(0.0, mapped_direction),
          "normal mission unexpectedly bypassed entry forward phase");
    check(observation.mappedCorridorDirection(0.0, mapped_direction, false),
          "inside-corridor observation did not expose the confirmed turn");
    check(!observation.updateExitDetectionOnly(start, 0.0, observed_exit) &&
          observation.turnYawAlignmentPending(),
          "observation completed turn before handheld yaw aligned");
    observation.updateExitDetectionOnly(start + Eigen::Vector3d(-0.3, 0, 0),
                                         M_PI, observed_exit);
    check(!observation.turnYawAlignmentPending() && observation.transition_active_,
          "aligned handheld yaw did not release latch or ended transition too soon");
    observation.updateExitDetectionOnly(start + Eigen::Vector3d(-0.7, 0, 0),
                                         M_PI, observed_exit);
    check(!observation.transition_active_ &&
          observation.stableProgressDirection().dot(-Eigen::Vector2d::UnitX()) > 0.99,
          "observation did not retain new corridor direction after walking through turn");
    check(!observation.stage3Active() && !observation.landingRequested(),
          "walking through turn started flight task");
    // 完整流程观察开关不影响原CH9低->高人工入口；全部在隔离master中验证。
    nh.setParam("mission/task_search/exit/rc_trigger/enabled", true);
    TaskSearchManager manual_mission;
    manual_mission.initialize(nh);
    manual_mission.updateRobotPose(start, 0.0);
    auto rc = boost::make_shared<mavros_msgs::RCIn>();
    rc->channels.resize(9, 1000);
    rc->header.stamp = ros::Time(100);
    manual_mission.rcSearchLandingCallback(rc);
    rc->channels[8] = 1900;
    rc->header.stamp = ros::Time(101);
    manual_mission.rcSearchLandingCallback(rc);
    check(!manual_mission.stage3Active(), "RC high bypassed the existing hold period");
    rc->header.stamp = ros::Time(102);
    manual_mission.rcSearchLandingCallback(rc);
    check(manual_mission.mission_stage_ == TaskSearchManager::SEARCH_OUTSIDE_LANDING &&
          manual_mission.exit_portal_locked_ &&
          manual_mission.consumeRcSearchLandingStartRequest() &&
          !manual_mission.consumeRcSearchLandingStartRequest(),
          "observation mode blocked or repeated the existing RC handoff");
    // 有确认出口时，CH9先穿越，即使自动接管关闭、阶段二目标尚未完成。
    TaskSearchManager manual_crossing;
    manual_crossing.initialize(nh);
    manual_crossing.updateRobotPose(start, 0.0);
    manual_crossing.exit_candidate_confirmed_ = true;
    manual_crossing.exit_portal_outside_verified_ = true;
    manual_crossing.exit_candidate_ = start;
    manual_crossing.exit_portal_center_ = start + Eigen::Vector3d(0.5, 0, 0);
    manual_crossing.exit_outward_direction_ = Eigen::Vector2d::UnitX();
    rc->channels[8] = 1000;
    rc->header.stamp = ros::Time(110);
    manual_crossing.rcSearchLandingCallback(rc);
    rc->channels[8] = 1900;
    rc->header.stamp = ros::Time(111);
    manual_crossing.rcSearchLandingCallback(rc);
    check(!manual_crossing.stage3Active(), "RC crossing bypassed hold period");
    rc->header.stamp = ros::Time(112);
    manual_crossing.rcSearchLandingCallback(rc);
    const Eigen::Vector3d manual_portal = manual_crossing.exit_portal_center_;
    check(manual_crossing.mission_stage_ == TaskSearchManager::EXIT_APPROACH_INSIDE &&
          manual_crossing.exit_portal_locked_ &&
          manual_crossing.consumeRcSearchLandingStartRequest() &&
          !manual_crossing.consumeRcSearchLandingStartRequest(),
          "RC did not start crossing independently of the automatic switch");
    Eigen::Vector3d manual_goal;
    double manual_yaw = 0;
    manual_crossing.buildStage3Goal(start, 0, false, manual_goal, manual_yaw);
    check(manual_crossing.mission_stage_ == TaskSearchManager::CROSS_EXIT &&
          !manual_crossing.landingRequested(), "RC skipped crossing before landing search");
    auto rc_land = boost::make_shared<std_msgs::Bool>();
    rc_land->data = true;
    manager.rcLandActiveCallback(rc_land);
    check(manager.rc_land_active_ && !manager.consumeRcSearchLandingStartRequest(),
          "CH10 did not latch or clear pending CH9 request");
    manager.mission_stage_ = TaskSearchManager::CROSS_EXIT;
    Eigen::Vector3d cancelled_goal(7, 8, 9);
    double cancelled_yaw = 0;
    check(!manager.buildStage3Goal(start, 0, false, cancelled_goal, cancelled_yaw) &&
          (cancelled_goal - Eigen::Vector3d(7, 8, 9)).norm() == 0,
          "CH10 did not cancel crossing goal output");
    manager.rc_search_landing_triggered_ = false;
    rc->header.stamp = ros::Time(120);
    manager.rcSearchLandingCallback(rc);
    check(!manager.rc_search_landing_triggered_, "CH9 restarted a CH10-cancelled mission");
    rc_land->data = false;
    manager.rcLandActiveCallback(rc_land);
    check(manager.rc_land_active_, "false message cleared CH10 landing latch");
    manager.mission_stage_ = TaskSearchManager::SEARCH_OUTSIDE_LANDING;
    check(!manager.buildStage3Goal(start, 0, false, cancelled_goal, cancelled_yaw),
          "CH10 did not cancel landing search goal output");
    rc->header.stamp = ros::Time(113);
    manual_crossing.rcSearchLandingCallback(rc);
    check(manual_crossing.mission_stage_ == TaskSearchManager::CROSS_EXIT &&
          (manual_crossing.exit_portal_center_ - manual_portal).norm() < 1e-6,
          "repeated RC trigger restarted or replaced the locked exit");
    manual_crossing.external_landing_planner_ = true;
    check(!manual_crossing.buildStage3Goal(manual_portal + Eigen::Vector3d(0.65, 0, 0),
                                         0, false, manual_goal, manual_yaw) &&
          manual_crossing.mission_stage_ == TaskSearchManager::SEARCH_OUTSIDE_LANDING &&
          !manual_crossing.landingRequested(),
          "RC crossing did not hand off outside search to the external planner");
    TaskSearchManager already_outside;
    already_outside.initialize(nh);
    const Eigen::Vector3d outside_pose = manual_portal + Eigen::Vector3d(0.8,0,0);
    already_outside.updateRobotPose(outside_pose, 0.0);
    already_outside.exit_candidate_confirmed_ = true;
    already_outside.exit_portal_outside_verified_ = true;
    already_outside.exit_candidate_ = start;
    already_outside.exit_portal_center_ = manual_portal;
    already_outside.exit_outward_direction_ = Eigen::Vector2d::UnitX();
    already_outside.external_landing_planner_ = true;
    already_outside.activateRcSearchLanding(ros::Time(114));
    manual_goal = Eigen::Vector3d(7,8,9);
    check(!already_outside.buildStage3Goal(outside_pose, 0, false, manual_goal, manual_yaw) &&
          already_outside.mission_stage_ == TaskSearchManager::SEARCH_OUTSIDE_LANDING &&
          (manual_goal - Eigen::Vector3d(7,8,9)).norm() == 0 &&
          (already_outside.exit_portal_center_ - manual_portal).norm() < 1e-6,
          "RC at an already-crossed confirmed exit commanded a return inside");
    // 用真实射线融合的L形地图复现：地图支路朝向尚未与手持设备一致，不能先锁住。
    auto corner_map = std::make_shared<SDFMap>();
    corner_map->initMap(nh);
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> walls = {
        {{-3,-0.65},{1,-0.65}}, {{1,-0.65},{1,3}},
        {{1,3},{0.4,3}}, {{0.4,3},{0.4,0.65}},
        {{0.4,0.65},{-3,0.65}}, {{-3,0.65},{-3,-0.65}}};
    for (int repeat = 0; repeat < 4; ++repeat) {
      for (double camera_x : {-0.6, 0.0, 0.6}) {
        cloud.clear();
        const Eigen::Vector2d eye(camera_x, 0.0);
        for (int ray = 0; ray < 720; ++ray) {
          const double angle = ray * M_PI / 360;
          const Eigen::Vector2d direction(std::cos(angle), std::sin(angle));
          double nearest = 10.0;
          for (const auto& wall : walls) {
            const Eigen::Vector2d edge = wall.second - wall.first;
            const Eigen::Vector2d delta = wall.first - eye;
            const double denominator = direction.x()*edge.y() - direction.y()*edge.x();
            if (std::fabs(denominator) < 1e-9) continue;
            const double distance = (delta.x()*edge.y() - delta.y()*edge.x()) / denominator;
            const double fraction = (delta.x()*direction.y() - delta.y()*direction.x()) / denominator;
            if (distance > 0 && fraction >= 0 && fraction <= 1)
              nearest = std::min(nearest, distance);
          }
          const Eigen::Vector2d hit = eye + nearest * direction;
          for (int layer = -3; layer <= 3; ++layer)
            cloud.push_back(pcl::PointXYZ(hit.x(), hit.y(), 1.0 + layer*0.10));
        }
        corner_map->inputPointCloud(cloud, cloud.size(), Eigen::Vector3d(camera_x,0,1));
      }
    }
    TaskSearchManager handheld;
    handheld.initialize(nh);
    handheld.setMap(corner_map);
    handheld.setCorridorFrame(Eigen::Vector3d(0,0,1), Eigen::Vector3d::UnitX());
    handheld.corridor_entry_crossed_ = true;
    handheld.visited_positions_.push_back(Eigen::Vector3d(0,0,1));
    double old_free, branch_free;
    check(handheld.inferOccupancyTurnDirection(Eigen::Vector3d::UnitX(),
          mapped_direction, old_free, branch_free), "L-shaped map did not expose a turn");
    const double branch_yaw = std::atan2(mapped_direction.y(), mapped_direction.x());
    check(!handheld.mappedCorridorDirection(0.0, mapped_direction, false, true) &&
          !handheld.turnYawAlignmentPending() &&
          handheld.stableProgressDirection().dot(Eigen::Vector2d::UnitX()) > 0.99,
          "handheld committed a branch before actual yaw agreed");
    check(handheld.mappedCorridorDirection(branch_yaw, mapped_direction, false, true),
          "handheld rejected the mapped branch after actual yaw aligned");
    TaskSearchManager flight;
    flight.initialize(nh);
    flight.setMap(corner_map);
    flight.setCorridorFrame(Eigen::Vector3d(0,0,1), Eigen::Vector3d::UnitX());
    flight.corridor_entry_crossed_ = true;
    flight.visited_positions_.push_back(Eigen::Vector3d(0,0,1));
    check(flight.mappedCorridorDirection(0.0, mapped_direction, false) &&
          flight.turnYawAlignmentPending(),
          "formal flight lost map-first turn confirmation");
    // 真实FUEL射线建图：第三次稳定观察即锁存；后续地图不能替换出口。
    auto exit_map = std::make_shared<SDFMap>();
    exit_map->initMap(nh);
    nh.setParam("mission/task_search/exit/auto_crossing_enabled", true);
    nh.setParam("mission/task_search/require_stage2_detections", false);
    nh.setParam("mission/task_search/exit/external_landing_planner", true);
    nh.setParam("mission/task_search/exit/cross_confirm_distance", 0.60);
    nh.setParam("mission/task_search/cruise_height", 1.0);
    TaskSearchManager automatic;
    automatic.initialize(nh);
    automatic.setMap(exit_map);
    automatic.setCorridorFrame(Eigen::Vector3d(-1,0,1), Eigen::Vector3d::UnitX());
    automatic.corridor_entry_crossed_ = true;
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> exit_walls = {
        {{-3,-0.65},{0.5,-0.65}}, {{0.5,-0.65},{0.5,-2.6}},
        {{0.5,-2.6},{3,-2.6}}, {{3,-2.6},{3,2.6}},
        {{3,2.6},{0.5,2.6}}, {{0.5,2.6},{0.5,0.65}},
        {{0.5,0.65},{-3,0.65}}, {{-3,0.65},{-3,-0.65}}};
    const Eigen::Vector3d exit_eye(0.1,0,1);
    for (double camera_x : {-0.4, 0.1, 0.3}) {
      cloud.clear();
      const Eigen::Vector3d scan_eye(camera_x,0,1);
      for (int ray = 0; ray < 720; ++ray) {
        const double angle = ray * M_PI / 360;
        const Eigen::Vector2d direction(std::cos(angle), std::sin(angle));
        double nearest = 6.0;
        for (const auto& wall : exit_walls) {
          const Eigen::Vector2d edge = wall.second - wall.first;
          const Eigen::Vector2d delta = wall.first - scan_eye.head<2>();
          const double denominator = direction.x()*edge.y() - direction.y()*edge.x();
          if (std::fabs(denominator) < 1e-9) continue;
          const double distance = (delta.x()*edge.y() - delta.y()*edge.x()) / denominator;
          const double fraction = (delta.x()*direction.y() - delta.y()*direction.x()) / denominator;
          if (distance > 0 && fraction >= 0 && fraction <= 1)
            nearest = std::min(nearest, distance);
        }
        const Eigen::Vector2d hit = scan_eye.head<2>() + nearest * direction;
        for (int layer = -3; layer <= 3; ++layer)
          cloud.push_back(pcl::PointXYZ(hit.x(), hit.y(), 1.0 + layer*0.10));
      }
      for (int repeat = 0; repeat < 4; ++repeat) {
        for (int layer = -3; layer <= 3; ++layer)
          exit_map->inputPointCloud(cloud, cloud.size(),
                                   scan_eye + Eigen::Vector3d(0,0,layer*0.10));
      }
    }
    check(automatic.mapExitFootprintFree(exit_eye),
          "synthetic exit scan did not observe the current body volume");
    for (int vote = 0; vote < 3; ++vote) {
      ros::Time::setNow(ros::Time(200 + vote));
      exit_map->inputPointCloud(cloud, cloud.size(), exit_eye);
      automatic.updateRobotPose(exit_eye, 0.0);
      if (vote < 2)
        check(!automatic.exit_portal_locked_ && !automatic.stage3Active(),
              "automatic crossing started before three valid map observations");
    }
    check(automatic.exit_portal_locked_ && automatic.exit_candidate_confirmed_ &&
          automatic.mission_stage_ == TaskSearchManager::EXIT_APPROACH_INSIDE,
          "third real-map confirmation did not immediately latch and start exit approach");
    const Eigen::Vector3d locked_portal = automatic.exit_portal_center_;
    Eigen::Vector3d crossing_goal;
    double crossing_yaw = 0;
    check(automatic.buildStage3Goal(exit_eye, 0, false, crossing_goal, crossing_yaw) &&
          automatic.mission_stage_ == TaskSearchManager::CROSS_EXIT &&
          crossing_goal.x() > locked_portal.x() && !automatic.landingRequested(),
          "confirmed exit did not produce a crossing goal before landing search");
    cloud.clear();
    cloud.push_back(pcl::PointXYZ(0.3,0,1));
    exit_map->inputPointCloud(cloud, cloud.size(), exit_eye);
    ros::Time::setNow(ros::Time(204));
    automatic.updateRobotPose(exit_eye, 0.0);
    check(automatic.exit_candidate_confirmed_ && automatic.exit_portal_locked_ &&
          (automatic.exit_portal_center_ - locked_portal).norm() < 1e-6,
          "later map changed the latched exit");
    automatic.buildStage3Goal(locked_portal + Eigen::Vector3d(0.59,0,0),
                              0, false, crossing_goal, crossing_yaw);
    check(automatic.mission_stage_ == TaskSearchManager::CROSS_EXIT,
          "outside search began before crossing completed");
    check(!automatic.buildStage3Goal(locked_portal + Eigen::Vector3d(0.65,0,0),
                                    0, false, crossing_goal, crossing_yaw) &&
          automatic.mission_stage_ == TaskSearchManager::SEARCH_OUTSIDE_LANDING &&
          !automatic.landingRequested() && automatic.exit_portal_locked_,
          "automatic crossing did not hand off outside search to the external planner");
    std::cout << "real map freshness, observed volume, obstacle and turn checks passed" << std::endl;
  }
  static void run() {
    ros::Time::init();
    TaskSearchManager manager;
    check(!manager.consumeExitMapUpdate(0, ros::Time(10)), "unobserved map counted as fresh");
    check(manager.consumeExitMapUpdate(1, ros::Time(10)), "first map update rejected");
    check(!manager.consumeExitMapUpdate(1, ros::Time(20)), "unchanged map gained another vote");
    check(!manager.consumeExitMapUpdate(2, ros::Time(10, 100000000)), "confirmation period ignored");
    check(manager.consumeExitMapUpdate(2, ros::Time(10, 500000000)), "new map did not pass period gate");
    TaskSearchManager::OpenExitResult detection;
    detection.portal_center = Eigen::Vector3d(1.0, 0.0, 0.60);
    detection.outward_direction = Eigen::Vector2d(1.0, 0.0);
    const Eigen::Vector3d inside(0.55, 0.0, 0.60), pos(0.1, 0.0, 0.60);
    check(!manager.recordExitEvidence(detection, inside, pos, ros::Time(11)), "first vote confirmed exit");
    check(!manager.recordExitEvidence(detection, inside, pos, ros::Time(12)), "second vote confirmed exit");
    check(manager.recordExitEvidence(detection, inside, pos, ros::Time(13)), "third stable vote did not confirm");
    check(!manager.finalExitFrontierGuardActive(), "detection froze frontier before takeover");
    detection.portal_center.x() = 1.5;
    check(!manager.recordExitEvidence(detection, inside, pos, ros::Time(14)) &&
          manager.exit_candidate_hits_ == 1, "position jump did not restart confirmation");
    detection.outward_direction = Eigen::Vector2d(std::cos(M_PI/4), std::sin(M_PI/4));
    manager.recordExitEvidence(detection, inside, pos, ros::Time(15));
    check(manager.exit_candidate_hits_ == 1, "direction jump retained old confirmation votes");
    // 15度是允许的方向校正边界，浮点点积不能把第三票重新计为第一票。
    for (double initial_deg : {0.0, 30.0, 75.0394105, 90.0}) {
      TaskSearchManager boundary;
      detection.portal_center = Eigen::Vector3d(1.0, 0.0, 0.60);
      const double angle = initial_deg * M_PI / 180.0;
      detection.outward_direction = Eigen::Vector2d(std::cos(angle), std::sin(angle));
      boundary.recordExitEvidence(detection, inside, pos, ros::Time(40));
      boundary.recordExitEvidence(detection, inside, pos, ros::Time(41));
      const Eigen::Vector2d old = detection.outward_direction;
      const double correction = 15.0 * M_PI / 180.0;
      detection.outward_direction = Eigen::Vector2d(
          std::cos(correction)*old.x() - std::sin(correction)*old.y(),
          std::sin(correction)*old.x() + std::cos(correction)*old.y());
      check(boundary.recordExitEvidence(detection, inside, pos, ros::Time(42)) &&
            boundary.exit_candidate_hits_ == 3,
            "allowed 15-degree correction discarded the third exit vote");
    }
    detection.outward_direction = Eigen::Vector2d(1.0, 0.0);
    detection.portal_center.x() = 1.0;
    manager.clearExitEvidence();
    for (int n = 0; n < 3; ++n)
      manager.recordExitEvidence(detection, inside, Eigen::Vector3d(-1,0,0.6), ros::Time(16+n));
    check(!manager.exit_candidate_confirmed_, "far candidate locked outside one metre");
    manager.recordExitEvidence(detection, inside, pos, ros::Time(20));
    check(manager.exit_candidate_confirmed_, "near candidate did not confirm");
    manager.commitCorridorTurn(Eigen::Vector3d(0,1,0));
    check(!manager.exit_candidate_confirmed_ && manager.exit_candidate_hits_ == 0,
          "confirmed turn retained old exit evidence");

    TaskSearchManager mission;
    mission.exit_candidate_confirmed_ = true;
    mission.exit_portal_outside_verified_ = true;
    mission.exit_portal_center_ = detection.portal_center;
    mission.exit_candidate_ = inside;
    mission.exit_outward_direction_ = detection.outward_direction;
    Eigen::Vector3d goal;
    double yaw = 0;
    check(!mission.buildStage3Goal(inside, 0, false, goal, yaw), "unfinished targets allowed takeover");
    check(!mission.finalExitFrontierGuardActive(), "unfinished targets froze frontier");
    mission.require_stage2_detections_ = false;
    mission.exit_detection_enabled_ = false;
    mission.buildStage3Goal(inside, 0, false, goal, yaw);
    check(!mission.stage3Active() && !mission.exit_portal_locked_, "disabled exit detection changed stage");
    mission.exit_detection_enabled_ = true;
    mission.exit_auto_crossing_enabled_ = false;
    mission.exit_candidate_hits_ = 5;
    goal = Eigen::Vector3d(7,8,9);
    check(!mission.buildStage3Goal(inside, 0, true, goal, yaw) &&
          !mission.stage3Active() && !mission.exit_portal_locked_ &&
          mission.exit_candidate_confirmed_ && mission.exit_candidate_hits_ == 5 &&
          (goal - Eigen::Vector3d(7,8,9)).norm() == 0 &&
          !mission.finalExitFrontierGuardActive() && !mission.exitVerificationActive() &&
          !mission.landingRequested(),
          "detection-only mission lost evidence or changed flight after confirmation");
    mission.exit_auto_crossing_enabled_ = true;
    mission.buildStage3Goal(inside, 0, false, goal, yaw);
    check(mission.mission_stage_ == TaskSearchManager::CROSS_EXIT && mission.exit_portal_locked_,
          "confirmed open exit waited for frontier exhaustion or entry mileage");
    TaskSearchManager pending;
    pending.require_stage2_detections_ = false;
    pending.buildStage3Goal(Eigen::Vector3d::Zero(), 0, true, goal, yaw);
    check(!pending.stage3Active(), "frontier exhaustion alone triggered exit");
  }
};
}  // namespace fast_planner

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--map-integration") {
    ros::init(argc, argv, "open_space_exit_map_test", ros::init_options::AnonymousName);
    fast_planner::TaskSearchManagerTest::runMapIntegration();
    return 0;
  }
  fast_planner::open_space_exit::Result result;
  Scene exit{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.5,6,-2.60},{0.5,6,2.60}}};
  check(detect(exit,result), "bilateral open space with visible boundaries was not detected");
  check(std::fabs(result.forward-0.5) <= 0.3, "exit plane drifted into the open region");
  check(result.left_open_ratio >= 0.80 && result.right_open_ratio >= 0.80, "side ratios below threshold");
  Scene limitless{{{-3,0.5,-0.65},{-3,0.5,0.65}}};
  check(detect(limitless,result), "observed open space required a far wall return");
  Scene asymmetric{{{-3,0.5,-0.55},{-3,0.5,0.95},{0.5,6,-2.80},{0.5,6,2.80}}};
  check(detect(asymmetric,result) && std::fabs(result.lateral-0.20) < 0.06,
        "asymmetric passage produced wrong center");
  // 左右净空可以不对称，不需要把探测点平移到房间中心。
  Scene offset_room{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.5,6,-3.0},{0.5,6,1.80}}};
  check(detect(offset_room,result) && std::fabs(result.lateral) < 0.06 &&
        result.open_width > 2.0,
        "offset room was rejected or room center moved the exit portal");
  Scene mirrored_room{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.5,6,-1.80},{0.5,6,3.0}}};
  check(detect(mirrored_room,result) && std::fabs(result.lateral) < 0.06 &&
        result.open_width > 2.0,
        "mirrored offset room was rejected or moved the exit portal");
  Scene shifted_one_side{{{-3,0.5,-0.65},{-3,6,0.65},{0.5,6,-4.0}}};
  check(!detect(shifted_one_side,result),
        "moving the room center turned a one-sided opening into an exit");
  Scene shifted_unknown = offset_room; shifted_unknown.unknown_outside = true;
  check(!detect(shifted_unknown,result), "width sum accepted unknown space");
  Scene continuing{{{-3,6,-0.65},{-3,6,0.65}}};
  check(!detect(continuing,result), "continuing passage became exit");
  Scene one_side{{{-3,0.5,-0.65},{-3,6,0.65}}};
  check(!detect(one_side,result), "one-sided opening became exit");
  Scene wider{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.5,6,-1.80},{0.5,6,1.80}}};
  check(detect(wider,result), "total width above 2m still required 2.1m per side");
  // 墙厚为0.08m，外墙内表面间距恰好2m时不能通过。
  Scene threshold{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.5,6,-1.04},{0.5,6,1.04}}};
  check(!detect(threshold,result), "exactly 2m passed strict greater-than threshold");
  Scene below{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.5,6,-1.03},{0.5,6,1.03}}};
  check(!detect(below,result), "total width below 2m became exit");
  Scene above{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.5,6,-1.045},{0.5,6,1.045}}};
  check(detect(above,result), "total width of 2.01m was rejected");
  // 来路宽0.72m，前方左0.6m加右1.5m；当前位置仍在窄道内就应识别。
  Scene ahead{{{-3,1.2,-0.40},{-3,1.2,0.40},{1.2,6,-0.64},{1.2,6,1.54}}};
  check(detect(ahead,result) && result.forward > 1.0 && result.forward < 1.5 &&
        result.open_width > 2.0 && ahead.occupied(0,0.4),
        "asymmetric forward exit required current-position widening or 1m per side");
  Scene gap{{{-3,0.5,-0.65},{-3,0.5,0.65},{0.9,6,-0.65},{0.9,6,0.65}}};
  check(!detect(gap,result), "short bilateral gap became exit");
  Scene unseen = limitless; unseen.unknown_outside = true;
  check(!detect(unseen,result), "unobserved outside space became exit");
  unseen = limitless; unseen.only_center_observed = true;
  check(!detect(unseen,result), "observed center hid unknown sides");
  Scene corner = limitless; corner.front_blocked = true;
  check(!detect(corner,result), "blocked forward turn became exit");
  Scene blocked = limitless; blocked.footprint_blocked = true;
  check(!detect(blocked,result), "side openness bypassed body clearance");
  Scene posts{{{0,0.1,-0.65},{0,0.1,0.65}}};
  check(!detect(posts,result), "isolated posts supplied passage history");
  Scene already_open{{{-3,6,-3},{-3,6,3}}};
  check(!detect(already_open,result), "already open room supplied passage-to-open transition");
  // 扇区比例直接测试：未知也占分母，斜距不能冒充横向距离。
  fast_planner::open_space_exit::Config config;
  config.sector_ray_count = 10;
  auto none = [](double,double) { return false; };
  auto eight_observed = [](double x,double y) {
    return std::atan2(x,std::fabs(y))*180/M_PI > -12;
  };
  check(fast_planner::open_space_exit::countOpenWidthRays(config,0,0,none,eight_observed)==8,
        "unknown ray pairs were dropped from the 80 percent denominator");
  auto two_observed = [](double x,double y) {
    return std::atan2(x,std::fabs(y))*180/M_PI > 12;
  };
  check(fast_planner::open_space_exit::countOpenWidthRays(config,0,0,none,two_observed)==2,
        "two observed ray pairs became a fully open sector");
  check(fast_planner::open_space_exit::countOpenWidthRays(config,0,0,none,
      [](double,double y) { return std::fabs(y) < 1.0; })==0,
      "diagonal range was used instead of transverse clearance");
  fast_planner::TaskSearchManagerTest::run();
  std::cout << "open-space geometry and mission policy checks passed" << std::endl;
  return 0;
}
