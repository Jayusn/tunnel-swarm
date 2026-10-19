// Read-only rosbag evaluator. No ROS node, publishers, services, or ROS master.
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/ESCStatus.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <rosgraph_msgs/Log.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <map>
#include <limits>
#include <algorithm>
#include <random>
#include "selected_force_estimator.h"
#include "force_observer_input_synchronizer.h"
#include "vertical_disturbance_low_pass.h"
#include "disturbance_slew_limiter.h"
#include "force_build_info.h"
using namespace PayloadMPC;
using Vec = Eigen::Vector3d;

namespace {
MpcParams parameters(const std::string& config, const std::string& model) {
  const auto c = YAML::LoadFile(config), m = YAML::LoadFile(model);
  MpcParams p{};
  p.ctrl_freq_max_ = c["ctrl_freq_max"].as<double>();
  p.dyn_params_.mass_q = m["mass_q"].as<double>();
  p.gravity_ = m["gravity"].as<double>();
  const auto f = c["force_estimator"], filters = c["filter"], timeouts = c["msg_timeout"];
#define FORCE_FIELD(name) p.force_estimator_param_.name = f[#name].as<decltype(p.force_estimator_param_.name)>()
  FORCE_FIELD(kf); FORCE_FIELD(max_force); FORCE_FIELD(max_applied_force);
  FORCE_FIELD(sample_freq_fq); FORCE_FIELD(cutoff_freq_fq); FORCE_FIELD(max_queue);
  FORCE_FIELD(var_weight); FORCE_FIELD(force_observer_freq); FORCE_FIELD(enable_force_estimation);
  FORCE_FIELD(enable_disturbance_compensation); FORCE_FIELD(enable_input_sync);
  FORCE_FIELD(force_sync_history_duration); FORCE_FIELD(force_sync_max_interp_gap);
  FORCE_FIELD(force_sync_max_age); FORCE_FIELD(force_sync_reset_backjump);
  FORCE_FIELD(force_axis_gain_x); FORCE_FIELD(force_axis_gain_y); FORCE_FIELD(force_axis_gain_z);
  FORCE_FIELD(max_applied_force_rate_xy); FORCE_FIELD(max_applied_force_rate_z);
  FORCE_FIELD(cutoff_freq_applied_force_z); FORCE_FIELD(compensation_airborne_delay);
  FORCE_FIELD(min_valid_rpm);
#undef FORCE_FIELD
  p.force_estimator_param_.sqrt_kf = std::sqrt(p.force_estimator_param_.kf);
#define FILTER_FIELD(name) p.filter_param_.name = filters[#name].as<double>()
  FILTER_FIELD(sample_freq_quad_acc); FILTER_FIELD(cutoff_freq_quad_acc);
  FILTER_FIELD(sample_freq_rpm); FILTER_FIELD(cutoff_freq_rpm);
#undef FILTER_FIELD
#define TIMEOUT(name) p.msg_timeout_.name = timeouts[#name].as<double>()
  TIMEOUT(imu); TIMEOUT(odom); TIMEOUT(force_attitude_odom); TIMEOUT(rpm); TIMEOUT(state); TIMEOUT(extended_state);
#undef TIMEOUT
  return p;
}
struct Method {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  SelectedForceEstimator estimator;
  VerticalDisturbanceLowPass z_filter;
  DisturbanceSlewLimiter slew;
  Vec estimate{Vec::Zero()}, applied{Vec::Zero()};
  double duration_us{0};
  bool valid{false}, active{false};
  Method(MpcParams p, std::string label) : name(std::move(label)) {
    estimator.init(p);
    z_filter.configure(p.ctrl_freq_max_, p.force_estimator_param_.cutoff_freq_applied_force_z);
  }
  void clearApplied() { z_filter.reset(); slew.reset(); applied.setZero(); }
  void reset() { estimator.reset(); estimate.setZero(); valid=false; active=false; clearApplied(); }
  void tick(bool input_valid, bool gate, bool precision, const MpcParams& p, double dt) {
    const auto start = std::chrono::steady_clock::now();
    Vec fl;
    estimate.setZero();
    valid = input_valid && estimator.caculate_force(fl, estimate);
    duration_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count();
    active = gate && valid && estimator.windowFull();
    if (!active) { clearApplied(); return; }
    const auto& f=p.force_estimator_param_;
    Vec target=estimate;
    const auto z=z_filter.update(target.z());
    if (!z.valid) { clearApplied(); return; }
    target.z()=z.value;
    target.x()*=precision ? 0.0 : f.force_axis_gain_x;
    target.y()*=precision ? 0.0 : f.force_axis_gain_y;
    target.z()*=f.force_axis_gain_z;
    if (target.norm()>f.max_applied_force) target*=f.max_applied_force/target.norm();
    const auto limited=slew.update(target, dt, f.max_applied_force_rate_xy,
                                  f.max_applied_force_rate_z, f.max_applied_force);
    if (!limited.valid) { clearApplied(); return; }
    applied=limited.value;
    if (precision) applied.head<2>().setZero();
  }
};
std::vector<std::unique_ptr<Method>> methods(MpcParams p) {
  std::vector<std::unique_ptr<Method>> out;
  p.force_estimator_param_.method="window_opt";
  out.emplace_back(new Method(p,"window_opt"));
  for (int rate : {5,10,20,40}) for (int cutoff : {3,5,10,20}) {
    p.force_estimator_param_.method="aerothrow_ndob";
    p.force_estimator_param_.ndob_rate=rate;
    p.force_estimator_param_.ndob_cutoff_freq=cutoff;
    out.emplace_back(new Method(p,"ndob_"+std::to_string(rate)+"_"+std::to_string(cutoff)));
  }
  return out;
}
void vectorOut(std::ostream& out, const Vec& v) { for (int j=0;j<3;++j) out<<','<<v[j]; }
void methodHeader(std::ostream& out, const std::vector<std::unique_ptr<Method>>& ms) {
  for (const auto& m:ms) {
    for (const auto& field:{"fx","fy","fz","ax","ay","az","us","valid","active"})
      out<<','<<m->name<<'_'<<field;
  }
  out<<'\n';
}
void methodOut(std::ostream& out, const std::vector<std::unique_ptr<Method>>& ms) {
  for (const auto& m:ms) {
    vectorOut(out,m->estimate); vectorOut(out,m->applied);
    out<<','<<m->duration_us<<','<<m->valid<<','<<m->active;
  }
  out<<'\n';
}
bool contains(const std::string& s, const std::string& part) { return s.find(part)!=std::string::npos; }

void synthetic(const MpcParams& p, const std::string& output) {
  std::ofstream out(output);
  auto ms=methods(p);
  out<<std::setprecision(12)<<"scenario,axis,t,truth_x,truth_y,truth_z,input_x,input_y,input_z";
  methodHeader(out,ms);
  std::mt19937 rng(20261019);
  std::normal_distribution<double> noise(0.0,0.12);
  const double dt=1.0/p.ctrl_freq_max_;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.35,Vec::UnitX()) * Eigen::AngleAxisd(-0.2,Vec::UnitY()));
  const double thrust=p.dyn_params_.mass_q*p.gravity_;
  const Eigen::Vector4d rpm=Eigen::Vector4d::Constant(std::sqrt(thrust/(4*p.force_estimator_param_.kf)));
  for (const std::string scenario:{"zero","noise","step","pulse","sine_0.5","sine_1","sine_2","sine_5","outlier","bias","step_noise"}) {
    for (int axis=0;axis<3;++axis) {
      for(auto& m:ms)m->reset();
      for(int k=0;k<static_cast<int>(10.0/dt);++k) {
        const double t=k*dt;
        Vec truth=Vec::Zero();
        if (scenario=="step" || scenario=="step_noise") truth[axis]=t>=2 ? 1.0:0.0;
        if (scenario=="pulse") truth[axis]=t>=2 && t<2.2 ? 1.0:0.0;
        if (contains(scenario,"sine_")) truth[axis]=0.5*std::sin(2*M_PI*std::stod(scenario.substr(5))*t);
        if (scenario=="bias") truth[axis]=0.4;
        Vec observed=truth;
        if(scenario=="step_noise" || scenario=="noise") for(int j=0;j<3;++j) observed[j]+=noise(rng);
        // An isolated measurement outlier is NOT a real external force.
        if(scenario=="outlier" && k==static_cast<int>(2.0/dt)) observed[axis]+=3.0;
        const Vec a=q.conjugate()*((observed+thrust*(q*Vec::UnitZ()))/p.dyn_params_.mass_q);
        for(auto& m:ms) { m->estimator.setSystemState(a,q,rpm); m->tick(true,true,false,p,dt); }
        out<<scenario<<','<<axis<<','<<t;vectorOut(out,truth);vectorOut(out,observed);methodOut(out,ms);
      }
    }
  }
}
} // namespace

int main(int argc,char** argv) {
 try {
  std::map<std::string,std::string> args;
  for(int i=1;i+1<argc;i+=2) args[argv[i]]=argv[i+1];
  if(!args.count("--config") || !args.count("--model") || !args.count("--output")) {
    std::cerr<<"force_estimator_replay --config mpc.yaml --model model.yaml --output prefix "
      "[--bag file | --synthetic true] [--imu topic --attitude topic --observer-phase seconds]\n";
    return 2;
  }
  auto p=parameters(args.at("--config"),args.at("--model"));
  if(args.count("--synthetic")) { synthetic(p,args.at("--output")); return 0; }
  if(!args.count("--bag")) throw std::runtime_error("--bag required");
  const auto& f=p.force_estimator_param_;
  const std::string prefix=args.at("--output");
  const std::string vehicle=args.count("--vehicle")?args.at("--vehicle"):"/UAV1";
  const std::string controller=vehicle+"/mpc_controller_node";
  const std::string imu_topic=args.count("--imu")?args.at("--imu"):vehicle+"/livox/imu_mps2";
  const std::string att_topic=args.count("--attitude")?args.at("--attitude"):vehicle+"/fast_lio/Odom_high_freq";
  const std::string odom_topic=vehicle+"/fast_lio/Odom_high_freq";
  const std::string rpm_topic=vehicle+"/mavros/esc_status";
  const std::string force_topic=controller+"/mpc/force", applied_topic=controller+"/mpc/force_applied";
  std::vector<std::string> topics={imu_topic,att_topic,odom_topic,rpm_topic,force_topic,applied_topic,
    vehicle+"/mavros/state",vehicle+"/mavros/extended_state",controller+"/mpc/reference_trajectory",
    vehicle+"/control/attitude_setpoint",vehicle+"/mavros/setpoint_raw/attitude","/rosout",
    controller+"/mpc/force_timing",controller+"/mpc/force_config"};
  std::sort(topics.begin(),topics.end());topics.erase(std::unique(topics.begin(),topics.end()),topics.end());
  rosbag::Bag bag(args.at("--bag"),rosbag::bagmode::Read);
  rosbag::View view(bag,rosbag::TopicQuery(topics));
  if(view.size()==0) throw std::runtime_error("no requested topics");
  const double begin=view.getBeginTime().toSec();
  const double dt=1.0/p.ctrl_freq_max_, obs_dt=1.0/f.force_observer_freq;
  // Historical force publications identify control timing, but ROS subscriber
  // receipt stamps do not identify exact timer order. This remains an approximation.
  double next_tick=begin;
  rosbag::View forces(bag,rosbag::TopicQuery({force_topic}));
  if(forces.size()) next_tick=forces.getBeginTime().toSec();
  double next_observer=next_tick+(args.count("--observer-phase")?std::stod(args.at("--observer-phase")):0.0);
  std::ofstream out(prefix+".csv"), recorded(prefix+"_recorded.csv"), events(prefix+"_events.tsv"), meta(prefix+"_metadata.yaml");
  out<<std::setprecision(15)<<"t,source_stamp,input_age,new_sample,samples,input_valid,gate,mode,precision,trajectory,recovery,direct_land,raw_x,raw_y,raw_z,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,ref_x,ref_y,ref_z,omega_x,omega_y,omega_z,cmd_x,cmd_y,cmd_z,thrust";
  auto ms=methods(p);methodHeader(out,ms);
  recorded<<std::setprecision(15)<<"t,kind,x,y,z\n";
  events<<std::setprecision(15)<<"t\tsource_t\tmessage\n";
  meta<<"build: "<<YAML::Dump(YAML::Node(FORCE_BUILD_INFO))<<"\nbag: "<<args.at("--bag")
      <<"\nconfig: "<<args.at("--config")<<"\nmodel: "<<args.at("--model")
      <<"\nimu: "<<imu_topic<<"\nattitude: "<<att_topic<<"\nrpm: "<<rpm_topic
      <<"\ncontrol_dt: "<<dt<<"\nobserver_phase: "<<(next_observer-next_tick)
      <<"\nreconstruction: bag arrival time, nominal timers, ROS log mode events; not exact historical execution\n";
  ForceObserverInputSynchronizer sync;
  sync.configure(f.force_sync_history_duration,f.force_sync_max_interp_gap,f.force_sync_max_age,f.force_sync_reset_backjump);
  LowPassFilter2p<Vec> acc_filter;
  LowPassFilter2p<Eigen::Vector4d> rpm_filter;
  acc_filter.set_cutoff_frequency(p.filter_param_.sample_freq_quad_acc,p.filter_param_.cutoff_freq_quad_acc);
  rpm_filter.set_cutoff_frequency(p.filter_param_.sample_freq_rpm,p.filter_param_.cutoff_freq_rpm);
  acc_filter.reset(Vec::Zero());
  rpm_filter.reset(Eigen::Vector4d::Zero());
  Vec acceleration=Vec::Zero(),raw=Vec::Zero(),pos=Vec::Zero(),vel=Vec::Zero(),omega=Vec::Zero(),cmd=Vec::Zero();
  Vec ref=Vec::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond attitude=Eigen::Quaterniond::Identity();
  Eigen::Vector4d rpm=Eigen::Vector4d::Zero();
  double imu_time=-1,att_time=-1,rpm_time=-1,odom_time=-1,state_time=-1,extended_time=-1;
  double airborne_since=-1,source_stamp=0,thrust=0;
  bool input_valid=false,in_air=false,was_in_air=false,armed=false,offboard=false,precision=false,recovery=false,direct_land=false;
  bool new_sample=false;
  std::string trajectory="UNKNOWN";
  std::string mode="UNKNOWN";
  size_t accepted=0,skipped=0,resets=0;
  auto reset=[&]() {for(auto& m:ms)m->reset();sync.reset();input_valid=false;source_stamp=0;raw.setZero();++resets;};
  auto fresh=[&](double now) {return imu_time>=0 && att_time>=0 && rpm_time>=0 && odom_time>=0 &&
    now-imu_time<p.msg_timeout_.imu && now-att_time<p.msg_timeout_.force_attitude_odom &&
    now-rpm_time<p.msg_timeout_.rpm && now-odom_time<p.msg_timeout_.odom;};
  auto observe=[&](double now) {
    new_sample=false;
    if(!f.enable_input_sync) input_valid=false;
    if(!f.enable_force_estimation || mode=="AUTO_TAKEOFF") {reset();return;}
    if(extended_time>=0 && now-extended_time<p.msg_timeout_.extended_state) {
      if(in_air && !was_in_air) {reset();airborne_since=now;}
      else if(!in_air) airborne_since=-1;
      was_in_air=in_air;
    }
    if(!fresh(now) || !acceleration.allFinite() || !rpm.allFinite() ||
       !attitude.coeffs().allFinite() || attitude.norm()<1e-6) {reset();return;}
    ForceObserverSynchronizedInput input;
    if(f.enable_input_sync) {
      if(sync.synchronize(now,input)!=ForceObserverSyncResult::READY) {++skipped;return;}
    } else {input.acceleration=acceleration;input.attitude=attitude;input.rpm=rpm;input.stamp=now;}
    if(input.rpm.minCoeff()<f.min_valid_rpm) {reset();return;}
    for(auto& m:ms)m->estimator.setSystemState(input.acceleration,input.attitude,input.rpm);
    source_stamp=input.stamp;
    raw=p.dyn_params_.mass_q*(input.attitude*input.acceleration)-
        (f.sqrt_kf*input.rpm).squaredNorm()*input.attitude.toRotationMatrix().col(2);
    input_valid=true;new_sample=true;++accepted;
  };
  auto control=[&](double now) {
    bool gate=f.enable_force_estimation && f.enable_disturbance_compensation &&
      (mode=="AUTO_HOVER" || mode=="CMD_CTRL") && armed && offboard && in_air &&
      now-state_time<p.msg_timeout_.state && now-extended_time<p.msg_timeout_.extended_state &&
      airborne_since>=0 && now-airborne_since>=f.compensation_airborne_delay && fresh(now) &&
      rpm.allFinite() && rpm.minCoeff()>=f.min_valid_rpm && input_valid && !recovery && !direct_land;
    for(auto& m:ms) {
      if(recovery || direct_land) {m->estimate.setZero();m->valid=false;m->active=false;m->clearApplied();}
      else m->tick(input_valid,gate,precision,p,dt);
    }
    out<<now<<','<<source_stamp<<','<<(source_stamp>0?now-source_stamp:0)<<','<<new_sample<<','<<ms[0]->estimator.sampleCount()
       <<','<<input_valid<<','<<gate<<','<<mode<<','<<precision<<','<<trajectory<<','<<recovery<<','<<direct_land;
    vectorOut(out,raw);vectorOut(out,pos);vectorOut(out,vel);vectorOut(out,ref);vectorOut(out,omega);vectorOut(out,cmd);
    out<<','<<thrust;methodOut(out,ms);new_sample=false;
  };
  for(const auto& item:view) {
    const double t=item.getTime().toSec();
    while(std::min(next_observer,next_tick)<t-1e-7) {
      if(next_observer<=next_tick+1e-7) {observe(next_observer);next_observer+=obs_dt;}
      else {control(next_tick);next_tick+=dt;}
    }
    const std::string topic=item.getTopic();
    if(topic==imu_topic) {
      auto m=item.instantiate<sensor_msgs::Imu>();if(!m)continue;
      acceleration=acc_filter.apply(Vec(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z));
      omega=Vec(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);imu_time=t;
      if(f.enable_input_sync)sync.addAcceleration(m->header.stamp.toSec(),t,acceleration);
    }
    if(topic==att_topic || topic==odom_topic) {
      auto m=item.instantiate<nav_msgs::Odometry>();if(!m)continue;
      if(topic==att_topic) {
        const auto& q=m->pose.pose.orientation;attitude=Eigen::Quaterniond(q.w,q.x,q.y,q.z);att_time=t;
        if(f.enable_input_sync)sync.addAttitude(m->header.stamp.toSec(),t,attitude);
      }
      if(topic==odom_topic) {
        const auto& v=m->twist.twist.linear;const auto& a=m->pose.pose.position;
        vel=Vec(v.x,v.y,v.z);pos=Vec(a.x,a.y,a.z);odom_time=t;
      }
    }
    if(topic==rpm_topic) {
      auto m=item.instantiate<mavros_msgs::ESCStatus>();if(!m || m->esc_status.size()<4)continue;
      Eigen::Vector4d r;for(int j=0;j<4;++j)r[j]=m->esc_status[j].rpm;
      rpm=rpm_filter.apply(r);rpm_time=t;
      if(f.enable_input_sync)sync.addRpm(m->header.stamp.toSec(),t,rpm);
    }
    if(topic==vehicle+"/mavros/state") {
      auto m=item.instantiate<mavros_msgs::State>();if(!m)continue;
      armed=m->armed;offboard=m->mode=="OFFBOARD";state_time=t;
      if(!offboard) {mode="MANUAL_CTRL";precision=false;recovery=false;direct_land=false;}
    }
    if(topic==vehicle+"/mavros/extended_state") {
      auto m=item.instantiate<mavros_msgs::ExtendedState>();if(!m)continue;
      in_air=m->landed_state==mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;extended_time=t;
    }
    if(topic==controller+"/mpc/reference_trajectory") {
      auto m=item.instantiate<nav_msgs::Path>();if(m && !m->poses.empty()) {
        const auto& r=m->poses.front().pose.position;ref=Vec(r.x,r.y,r.z);
      }
    }
    if(contains(topic,"attitude_setpoint") || contains(topic,"setpoint_raw/attitude")) {
      auto m=item.instantiate<mavros_msgs::AttitudeTarget>();if(m){cmd=Vec(m->body_rate.x,m->body_rate.y,m->body_rate.z);thrust=m->thrust;}
    }
    if(topic==force_topic || topic==applied_topic) {
      auto m=item.instantiate<geometry_msgs::Accel>();if(m)
        recorded<<t<<','<<(topic==force_topic?"estimated":"applied")<<','<<m->linear.x<<','<<m->linear.y<<','<<m->linear.z<<'\n';
    }
    if(topic==controller+"/mpc/force_timing") {
      auto m=item.instantiate<geometry_msgs::Vector3Stamped>();if(m)
        recorded<<m->header.stamp.toSec()<<",timing,"<<m->vector.x<<','<<m->vector.y<<','<<m->vector.z<<'\n';
    }
    if(topic==controller+"/mpc/force_config") {
      auto m=item.instantiate<std_msgs::String>();if(m) {std::ofstream snap(prefix+"_recorded_snapshot.txt");snap<<m->data;}
    }
    if(topic=="/rosout") {
      auto m=item.instantiate<rosgraph_msgs::Log>();if(!m || m->name!=controller)continue;
      const std::string& s=m->msg;
      if(contains(s," -> ") || contains(s,"[外力]") || contains(s,"RECOVERY") || contains(s,"[安全]") || contains(s,"恢复") || contains(s,"精降"))
        events<<t<<'\t'<<m->header.stamp.toSec()<<'\t'<<s<<'\n';
      if(contains(s," -> ") && !contains(s,"[轨迹]")) {
        for(const auto& next:{"AUTO_HOVER","CMD_CTRL","AUTO_TAKEOFF","AUTO_LAND","MANUAL_CTRL"})
          if(contains(s,std::string("-> ")+next))mode=next;
      }
      if(contains(s,"开始起飞") || contains(s,"进入 AUTO_TAKEOFF")) mode="AUTO_TAKEOFF";
      if(contains(s,"[外力] 估计输入超时") || contains(s,"已重置估计窗口"))reset();
      if(contains(s,"任务精降接管"))precision=true;
      if(contains(s,"POLY_TRAJ -> HOVER")) trajectory="HOVER";
      if(contains(s,"HOVER -> POLY_TRAJ")) trajectory="TRACKING";
      if(mode=="AUTO_HOVER") trajectory="HOVER";
      if(contains(s,"锁存 FAST-LIO 恢复点")) {recovery=true; if(mode=="AUTO_TAKEOFF")mode="AUTO_HOVER";}
      if(contains(s,"NMPC 已成功恢复")) recovery=false;
      if(contains(s,"停止规划并直接请求 PX4 AUTO.LAND")) {direct_land=true;recovery=false;mode="AUTO_LAND";}
      if(contains(s,"混合状态失效") || contains(s,"PX4 已退出 OFFBOARD")) {
        reset();precision=false;recovery=false;direct_land=false;mode="MANUAL_CTRL";
      }
    }
  }
  meta<<"accepted: "<<accepted<<"\nskipped: "<<skipped<<"\nresets: "<<resets<<'\n';
  std::cout<<prefix<<": accepted="<<accepted<<", skipped="<<skipped<<"\n";
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
 return 0;
}
