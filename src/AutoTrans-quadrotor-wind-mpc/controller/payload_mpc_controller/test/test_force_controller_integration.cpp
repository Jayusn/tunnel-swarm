#include "mpc_fsm.h"
#include <gtest/gtest.h>
namespace PayloadMPC {
// Test-only access to the actual production call site; no runtime test switches.
class ForceEstimatorTestAccess {
 public:
  static void prepare(MPCFSM& f,const ros::Time& now) {
    f.fsm_state=MPCFSM::CMD_CTRL;f.airborne_since_=now-ros::Duration(5);
    f.state_data.rcv_stamp=now;f.state_data.current_state.armed=true;f.state_data.current_state.mode="OFFBOARD";
    f.extended_state_data.rcv_stamp=now;
    f.extended_state_data.current_extended_state.landed_state=mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    f.imu_data.rcv_stamp=now;f.odom_data.rcv_stamp=now;f.force_attitude_odom_data.rcv_stamp=now;f.rpm_data.rcv_stamp=now;
    f.imu_data.filtered_a=Eigen::Vector3d(0,0,9.795);f.force_attitude_odom_data.q=Eigen::Quaterniond::Identity();
    f.rpm_data.filtered_rpm=Eigen::Vector4d::Constant(16000);
  }
  static void sample(MPCFSM& f,const Eigen::Vector3d& a,const Eigen::Vector4d& rpm) {
    f.force_estimator_.setSystemState(a,Eigen::Quaterniond::Identity(),rpm);f.force_observer_input_valid_=true;
  }
  static void reset(MPCFSM& f) {f.clearForceObserverState();}
  static void calculate(MPCFSM& f) {f.setForceEstimation();}
  static Eigen::Vector3d estimate(const MPCFSM& f){return f.fq_estimated_;}
  static Eigen::Vector3d applied(const MPCFSM& f){return f.fq_applied_;}
  static void precision(MPCFSM& f,bool active){f.precision_landing_active_=active;}
};
}
using namespace PayloadMPC;
TEST(ForceControllerIntegration, ProductionCallSiteMatchesReplayEventsAndDownstreamGates) {
  ros::NodeHandle nh("~");
  for(double z_gain : {0.0, 0.7})
  for(const std::string method:{"window_opt","aerothrow_ndob"}) {
    MpcParams p;p.config_from_ros_handle(nh);p.force_estimator_param_.method=method;
    // 该测试验证补偿门控，与实机配置是否关闭补偿无关。
    p.force_estimator_param_.enable_disturbance_compensation=true;
    // Exercise retained unequal axis gains and norm clipping as well as direct application.
    p.force_estimator_param_.force_axis_gain_x=.5;
    p.force_estimator_param_.force_axis_gain_y=.8;
    p.force_estimator_param_.force_axis_gain_z=z_gain;
    p.force_estimator_param_.max_applied_force=.45;
    MpcController controller(p);MPCFSM fsm(nh,p,controller);controller.waitForPreparation();
    SelectedForceEstimator offline;offline.init(p);
    const Eigen::Vector4d rpm=Eigen::Vector4d::Constant(16000);
    // Samples arrive every other tick; readiness follows the configured window.
    const int first_ready_tick=2*(p.force_estimator_param_.max_queue-1);
    bool input=false;
    for(int k=0;k<350;++k) {
      const ros::Time now=ros::Time(1000)+ros::Duration(.01*k);ros::Time::setNow(now);
      ForceEstimatorTestAccess::prepare(fsm,now);
      if(k==160) {ForceEstimatorTestAccess::reset(fsm);offline.reset();input=false;}
      if(k%2==0) {
        const Eigen::Vector3d force(.6*std::sin(.07*k),-.4,z_gain==0?6.:.8);
        Eigen::Vector3d a=force/p.dyn_params_.mass_q;
        a.z()+=(p.force_estimator_param_.sqrt_kf*rpm).squaredNorm()/p.dyn_params_.mass_q;
        ForceEstimatorTestAccess::sample(fsm,a,rpm);offline.setSystemState(a,Eigen::Quaterniond::Identity(),rpm);input=true;
      }
      bool precision=k>=250;ForceEstimatorTestAccess::precision(fsm,precision);
      bool blocked=k>=90 && k<100;
      if(blocked)fsm.state_data.current_state.mode="POSCTL";
      ForceEstimatorTestAccess::calculate(fsm);
      Eigen::Vector3d fl,out=Eigen::Vector3d::Zero(),applied=Eigen::Vector3d::Zero();
      bool valid=input && offline.caculate_force(fl,out);
      if(valid && offline.windowFull() && !blocked) {
        Eigen::Vector3d target=out;
        target.x()*=precision?0:p.force_estimator_param_.force_axis_gain_x;
        target.y()*=precision?0:p.force_estimator_param_.force_axis_gain_y;
        target.z()*=p.force_estimator_param_.force_axis_gain_z;
        double cap=p.force_estimator_param_.max_applied_force;if(target.norm()>cap)target*=cap/target.norm();
        applied=target;
      }
      // The first ready sample is applied immediately, with no 0.03 N Z ramp.
      if(k==first_ready_tick && z_gain>0) { EXPECT_GT(ForceEstimatorTestAccess::applied(fsm).z(),.03); }
      if(z_gain==0) EXPECT_DOUBLE_EQ(ForceEstimatorTestAccess::applied(fsm).z(),0.0);
      EXPECT_LT((out-ForceEstimatorTestAccess::estimate(fsm)).norm(),1e-12)<<method<<" tick="<<k;
      EXPECT_LT((applied-ForceEstimatorTestAccess::applied(fsm)).norm(),1e-8)<<method<<" tick="<<k;
      if(k<first_ready_tick || blocked) { EXPECT_TRUE(ForceEstimatorTestAccess::applied(fsm).isZero()); }
    }
  }
}
