#include "selected_force_estimator.h"
#include <gtest/gtest.h>
#include <limits>
using namespace PayloadMPC;
using V = Eigen::Vector3d;
namespace {
MpcParams params() {
  MpcParams p{};
  p.ctrl_freq_max_=100;
  p.dyn_params_.mass_q=1.926;
  auto& f=p.force_estimator_param_;
  f.sqrt_kf=std::sqrt(1.835e-8); f.max_force=5; f.max_queue=20;
  f.sample_freq_fq=100; f.cutoff_freq_fq=3; f.var_weight=80;
  f.enable_force_estimation=true;
  return p;
}
const Eigen::Vector4d rpm=Eigen::Vector4d::Constant(16000);
V specific(const MpcParams& p,const V& force,const Eigen::Quaterniond& q) {
  return q.conjugate()*force/p.dyn_params_.mass_q +
    V(0,0,(p.force_estimator_param_.sqrt_kf*rpm).squaredNorm()/p.dyn_params_.mass_q);
}
}
TEST(SelectedForceEstimator, DefaultIsBitIdenticalToOriginalWithHoldsAndResets) {
  auto p=params(); MultiOptForceEstimator original; SelectedForceEstimator selected;
  original.init(p); selected.init(p);
  Eigen::Quaterniond q(Eigen::AngleAxisd(.4,V::UnitY()));
  for(int k=0;k<650;++k) {
    if(k==133 || k==305) {original.reset(); selected.reset();}
    if(k%3!=0) {
      V force(std::sin(.09*k),.3*std::cos(.17*k),k==120?12:.4);
      V a=specific(p,force,q);
      original.setSystemState(a,q,rpm); selected.setSystemState(a,q,rpm);
    }
    V a,b,c,d;
    EXPECT_EQ(original.caculate_force(a,b),selected.caculate_force(c,d));
    EXPECT_TRUE((a.array()==c.array()).all()); EXPECT_TRUE((b.array()==d.array()).all());
    EXPECT_EQ(original.windowFull(),selected.windowFull());
    EXPECT_EQ(original.sampleCount(),selected.sampleCount());
  }
}
TEST(AeroThrowNDOB, EulerStateAndButterworthOutputAreIndependent) {
  auto p=params(); AeroThrowNDOB n; n.init(p);
  const V force(1,-.7,.4); const auto q=Eigen::Quaterniond::Identity();
  n.setSystemState(specific(p,force,q),q,rpm);
  // Independent direct-form Butterworth reference, not the production filter.
  double w=std::tan(M_PI*20/100), den=1+std::sqrt(2.)*w+w*w;
  double b0=w*w/den,b1=2*b0,b2=b0,a1=2*(w*w-1)/den,a2=(1-std::sqrt(2.)*w+w*w)/den;
  V x1=V::Zero(),x2=V::Zero(),y1=V::Zero(),y2=V::Zero(),fl,out;
  for(int k=1;k<=120;++k) {
    ASSERT_TRUE(n.caculate_force(fl,out));
    V expected=force*(1-std::pow(.8,k));
    EXPECT_LT((expected-n.observerState()).norm(),1e-12);
    V filtered=b0*expected+b1*x1+b2*x2-a1*y1-a2*y2;
    // Production legacy filter uses cosf for one coefficient (float rounding).
    EXPECT_LT((out-filtered).norm(),2e-7);
    x2=x1;x1=expected;y2=y1;y1=filtered;
  }
  EXPECT_EQ(n.sampleCount(),1U); EXPECT_FALSE(n.windowFull());
}
TEST(AeroThrowNDOB, CoordinatesHoverTiltAndWorldRotation) {
  auto p=params();
  for(const auto& q:{Eigen::Quaterniond::Identity(),Eigen::Quaterniond(Eigen::AngleAxisd(.65,V(1,2,3).normalized()))}) {
    for(const auto& force:{V::Zero().eval(),V(1,0,0),V(0,-1,0),V(0,0,1)}) {
      AeroThrowNDOB n;n.init(p);V fl,out;
      n.setSystemState(specific(p,force,q),q,rpm);
      ASSERT_TRUE(n.caculate_force(fl,out));
      EXPECT_LT((n.observerState()-.2*force).norm(),1e-12);
    }
  }
}
TEST(AeroThrowNDOB, ReadinessResetAndClippingDoNotOverwriteState) {
  auto p=params(); AeroThrowNDOB n; n.init(p); V fl,out;
  EXPECT_FALSE(n.caculate_force(fl,out));
  for(int k=0;k<20;++k) {
    n.setSystemState(specific(p,V(30,0,0),Eigen::Quaterniond::Identity()),Eigen::Quaterniond::Identity(),rpm);
    ASSERT_TRUE(n.caculate_force(fl,out));
    EXPECT_LE(out.norm(),5.0+1e-12);
    EXPECT_EQ(n.windowFull(),k==19);
  }
  EXPECT_GT(n.observerState().x(),29);
  n.reset();EXPECT_EQ(n.sampleCount(),0U);EXPECT_FALSE(n.windowFull());
  EXPECT_TRUE(n.observerState().isZero());EXPECT_FALSE(n.caculate_force(fl,out));
  n.setSystemState(specific(p,V::Zero(),Eigen::Quaterniond::Identity()),Eigen::Quaterniond::Identity(),rpm);
  n.caculate_force(fl,out);EXPECT_LT(out.norm(),1e-12);
}
TEST(AeroThrowNDOB, RejectsInvalidMathAndMethod) {
  EXPECT_FALSE(AeroThrowNDOB::validParameters(0,20,100));
  EXPECT_FALSE(AeroThrowNDOB::validParameters(200,20,100));
  EXPECT_FALSE(AeroThrowNDOB::validParameters(20,50,100));
  EXPECT_FALSE(AeroThrowNDOB::validParameters(20,-1,100));
  EXPECT_FALSE(AeroThrowNDOB::validParameters(std::numeric_limits<double>::quiet_NaN(),20,100));
  for(int r:{5,10,20,40})for(int c:{3,5,10,20})EXPECT_TRUE(AeroThrowNDOB::validParameters(r,c,100));
  auto p=params();p.force_estimator_param_.method="unknown";SelectedForceEstimator n;
  EXPECT_THROW(n.init(p),std::invalid_argument);
}
TEST(SelectedForceEstimator, SelectedNdobMatchesCoreUnderTheSameEventSequence) {
  auto p=params();p.force_estimator_param_.method="aerothrow_ndob";
  SelectedForceEstimator online;AeroThrowNDOB core;online.init(p);core.init(p);
  for(int k=0;k<500;++k) {
    if(k==180){online.reset();core.reset();}
    if(k%2==0) {
      V a=specific(p,V(.5*std::sin(k*.05),.2,-.1),Eigen::Quaterniond::Identity());
      online.setSystemState(a,Eigen::Quaterniond::Identity(),rpm);core.setSystemState(a,Eigen::Quaterniond::Identity(),rpm);
    }
    if(k>=300 && k<310)continue; // Recovery skips calculations, no catch-up.
    V fl1,fl2,f1,f2;EXPECT_EQ(online.caculate_force(fl1,f1),core.caculate_force(fl2,f2));
    EXPECT_TRUE((f1.array()==f2.array()).all());EXPECT_EQ(online.windowFull(),core.windowFull());
  }
}
TEST(SelectedForceEstimator, DisabledZCannotConsumeHorizontalLimit) {
  for (const std::string method : {"window_opt", "aerothrow_ndob"}) {
    for (const V force : {V(1,0,6), V(6,8,-30)}) {
      auto p=params(); p.force_estimator_param_.method=method;
      p.force_estimator_param_.force_axis_gain_z=0;
      SelectedForceEstimator e; e.init(p); V fl,out;
      const auto q=Eigen::Quaterniond::Identity();
      for (int k=0;k<300;++k) {
        e.setSystemState(specific(p,force,q),q,rpm);
        if (!e.caculate_force(fl,out)) continue;
        EXPECT_LE(out.head<2>().norm(),5+1e-10) << method;
        EXPECT_LE(std::abs(out.z()),5+1e-10) << method;
      }
      V expected=force;
      if (expected.head<2>().norm()>5) expected.head<2>()*=5/expected.head<2>().norm();
      expected.z()=std::copysign(5.,force.z());
      EXPECT_LT((out-expected).norm(),1e-4) << method;
    }
  }
}
TEST(SelectedForceEstimator, DisabledZStepsAndResetPreserveHorizontalResponse) {
  for (const std::string method : {"window_opt", "aerothrow_ndob"}) {
    auto p=params(); p.force_estimator_param_.method=method;
    p.force_estimator_param_.force_axis_gain_z=0;
    SelectedForceEstimator clean,disturbed; clean.init(p); disturbed.init(p);
    const auto q=Eigen::Quaterniond::Identity();
    for (int k=0;k<600;++k) {
      if (k==350) {clean.reset();disturbed.reset();}
      if (k%3!=0) { // Include held input and opposite-sign large Z transients.
        V force(.8*std::sin(.03*k),-.4*std::cos(.02*k),0);
        clean.setSystemState(specific(p,force,q),q,rpm);
        force.z()=k<100?0:(k<250?30:-30);
        disturbed.setSystemState(specific(p,force,q),q,rpm);
      }
      V fl,a,b;
      EXPECT_EQ(clean.caculate_force(fl,a),disturbed.caculate_force(fl,b));
      EXPECT_LT((a-b).head<2>().norm(),1e-3) << method << " tick=" << k;
      EXPECT_LE(b.head<2>().norm(),5+1e-10);
      EXPECT_LE(std::abs(b.z()),5+1e-10);
      EXPECT_EQ(clean.sampleCount(),disturbed.sampleCount());
    }
  }
}
TEST(SelectedForceEstimator, EnabledZRetainsThreeAxisNormLimit) {
  for (const std::string method : {"window_opt", "aerothrow_ndob"}) {
    for (double gain : {.7,1.}) {
      auto p=params(); p.force_estimator_param_.method=method;
      p.force_estimator_param_.force_axis_gain_z=gain;
      SelectedForceEstimator e; e.init(p); V fl,out;
      const V force(1,0,6); const auto q=Eigen::Quaterniond::Identity();
      for (int k=0;k<300;++k) {
        e.setSystemState(specific(p,force,q),q,rpm);
        if (e.caculate_force(fl,out)) EXPECT_LE(out.norm(),5+1e-10);
      }
      EXPECT_LT((out-force.normalized()*5).norm(),1e-4) << method;
    }
  }
}
int main(int argc,char** argv) {
  testing::InitGoogleTest(&argc,argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
