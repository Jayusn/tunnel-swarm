#include "mpc_fsm.h"
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <atomic>
#include <memory>
#include <vector>

namespace PayloadMPC {
class AutoLandRequestTestAccess {
 public:
  static void prepare(MPCFSM& f, const ros::Time& now) {
    f.fsm_state = MPCFSM::AUTO_LAND;
    f.exec_traj_state_ = MPCFSM::HOVER;
    f.hover_pose_ = f.odom_data.p;
    f.hover_yaw_ = 0.;
    f.last_set_hover_pose_time = now;
    f.land_start_time_ = now;
  }
  static bool landing(const MPCFSM& f) { return f.fsm_state == MPCFSM::AUTO_LAND; }
  static bool idle(const MPCFSM& f) { return f.fsm_state == MPCFSM::MANUAL_CTRL; }
  static bool pending(MPCFSM& f) {
    return f.auto_land_request_future_.valid() &&
        f.auto_land_request_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
  }
  static void directRequest(MPCFSM& f) { f.processDirectAutoLand(ros::Time::now()); }
};
}

using namespace PayloadMPC;

class AutoLandRequest : public testing::Test {
 protected:
  void SetUp() override {
    ros::Time::init();
    params.config_from_ros_handle(nh);
    params.use_simulation_ = false;
    params.force_estimator_param_.enable_force_estimation = false;
    controller.reset(new MpcController(params));
    fsm.reset(new MPCFSM(nh, params, *controller));
    controller->waitForPreparation();
    fsm->odom_data.p = Eigen::Vector3d(1., 2., params.land_.min_target_z);
    fsm->odom_data.v.setZero();
    fsm->odom_data.q = Eigen::Quaterniond::Identity();
    fsm->force_attitude_odom_data.q = Eigen::Quaterniond::Identity();
    fsm->imu_data.a = Eigen::Vector3d(0., 0., params.gravity_);
    fsm->imu_data.filtered_a = fsm->imu_data.a;
    fsm->bat_data.volt = 16.;
    fsm->state_data.current_state.mode = "OFFBOARD";
    fsm->state_data.current_state.armed = true;
    fsm->rc_data.mode_valid = true;
    fsm->rc_data.is_takeoff_mode = false;
    fsm->rc_data.enter_land_mode = false;
    AutoLandRequestTestAccess::prepare(*fsm, ros::Time::now());

    // 独立回调队列只服务模拟 MAVROS，延迟不会阻塞测试本身的订阅回调。
    service_nh.setCallbackQueue(&service_queue);
    server = service_nh.advertiseService("delayed_set_mode", &AutoLandRequest::setMode, this);
    spinner.reset(new ros::AsyncSpinner(1, &service_queue));
    spinner->start();
    fsm->set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("delayed_set_mode");
    ASSERT_TRUE(fsm->set_FCU_mode_srv.waitForExistence(ros::Duration(3.)));
    fsm->ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("landing_output", 100);
    output_sub = nh.subscribe("landing_output", 100, &AutoLandRequest::output, this);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(3.);
    while (!fsm->ctrl_FCU_pub.getNumSubscribers() && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      ros::WallDuration(.01).sleep();
    }
    ASSERT_GT(fsm->ctrl_FCU_pub.getNumSubscribers(), 0U);
  }

  void TearDown() override {
    // 后台请求只持有 client 副本；回收结束前模拟服务仍存活。
    fsm.reset();
    spinner->stop();
    controller.reset();
    ros::Time::init();
  }

  bool setMode(mavros_msgs::SetMode::Request& request, mavros_msgs::SetMode::Response& response) {
    ++calls;
    if (request.custom_mode != "AUTO.LAND") wrong_mode = true;
    ros::WallDuration(delay).sleep();
    response.mode_sent = accepted;
    ++completed;
    return transport_ok;
  }

  void output(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    received.push_back(ros::WallTime::now().toSec());
    stamps.push_back(msg->header.stamp.toSec());
    EXPECT_TRUE(std::isfinite(msg->thrust));
    EXPECT_TRUE(std::isfinite(msg->body_rate.x));
  }

  void tick() {
    const auto now = ros::Time::now();
    fsm->state_data.rcv_stamp = now;
    fsm->odom_data.rcv_stamp = now;
    fsm->force_attitude_odom_data.rcv_stamp = now;
    fsm->imu_data.rcv_stamp = now;
    const auto start = ros::WallTime::now();
    fsm->process();
    max_process = std::max(max_process, (ros::WallTime::now() - start).toSec());
    ros::spinOnce();
  }

  void run(double duration) {
    const auto end = ros::WallTime::now() + ros::WallDuration(duration);
    ros::WallRate rate(100.);
    while (ros::WallTime::now() < end) { tick(); rate.sleep(); }
    ros::spinOnce();
  }

  void expectContinuous() {
    ASSERT_GT(received.size(), 60U);
    double max_gap = 0.;
    for (size_t i = 1; i < received.size(); ++i) {
      max_gap = std::max(max_gap, received[i] - received[i-1]);
      EXPECT_GT(stamps[i], stamps[i-1]);
    }
    EXPECT_LT(max_process, .15);
    EXPECT_LT(max_gap, .15);
    EXPECT_FALSE(wrong_mode.load());
    ROS_INFO("[AUTO_LAND_TEST] outputs=%zu max_process=%.3fms max_gap=%.3fms calls=%d",
             received.size(), max_process * 1000., max_gap * 1000., calls.load());
  }

  ros::NodeHandle nh{"~"}, service_nh{"~"};
  ros::CallbackQueue service_queue;
  ros::ServiceServer server;
  ros::Subscriber output_sub;
  std::unique_ptr<ros::AsyncSpinner> spinner;
  MpcParams params;
  std::unique_ptr<MpcController> controller;
  std::unique_ptr<MPCFSM> fsm;
  std::atomic<int> calls{0}, completed{0};
  std::atomic<bool> wrong_mode{false};
  double delay{1.2}, max_process{0.};
  bool accepted{true}, transport_ok{true};
  std::vector<double> received, stamps;
};

TEST_F(AutoLandRequest, SlowServiceKeepsPublishingWithoutDuplicateRequests) {
  // 延迟跨过原有 1s 重试间隔；同一调用等待中不能再发送第二个请求。
  run(1.1);
  expectContinuous();
  EXPECT_EQ(calls.load(), 1);
  EXPECT_EQ(completed.load(), 0);
  EXPECT_TRUE(AutoLandRequestTestAccess::pending(*fsm));
  EXPECT_TRUE(AutoLandRequestTestAccess::landing(*fsm));
  EXPECT_NEAR(fsm->hover_pose_.x(), 1., 1e-9);
  EXPECT_NEAR(fsm->hover_pose_.y(), 2., 1e-9);
}

TEST_F(AutoLandRequest, Px4ConfirmationStopsOutputWhileServiceStillPending) {
  run(.2);
  ASSERT_EQ(calls.load(), 1);
  ASSERT_TRUE(AutoLandRequestTestAccess::pending(*fsm));
  fsm->state_data.current_state.mode = "AUTO.LAND";
  run(.05);  // 排空确认前已经发布的消息。
  ASSERT_TRUE(AutoLandRequestTestAccess::idle(*fsm));
  const auto count = received.size();
  run(1.1);  // 响应晚到也不得重新切回控制或恢复输出。
  EXPECT_EQ(received.size(), count);
  EXPECT_EQ(calls.load(), 1);
  EXPECT_EQ(completed.load(), 1);
  EXPECT_TRUE(AutoLandRequestTestAccess::idle(*fsm));
  EXPECT_LT(max_process, .15);
}

TEST_F(AutoLandRequest, RejectedRequestKeepsControllingAndRetries) {
  delay = .02;
  accepted = false;
  run(1.2);
  expectContinuous();
  EXPECT_EQ(calls.load(), 2);
  EXPECT_TRUE(AutoLandRequestTestAccess::landing(*fsm));
}

TEST_F(AutoLandRequest, TransportFailureKeepsControllingAndRetries) {
  delay = .02;
  transport_ok = false;
  run(1.2);
  expectContinuous();
  EXPECT_EQ(calls.load(), 2);
  EXPECT_TRUE(AutoLandRequestTestAccess::landing(*fsm));
}

TEST_F(AutoLandRequest, RecoveryLandingRequestAlsoReturnsWithoutWaiting) {
  const auto start = ros::WallTime::now();
  AutoLandRequestTestAccess::directRequest(*fsm);
  EXPECT_LT((ros::WallTime::now() - start).toSec(), .15);
  EXPECT_TRUE(AutoLandRequestTestAccess::pending(*fsm));
}
