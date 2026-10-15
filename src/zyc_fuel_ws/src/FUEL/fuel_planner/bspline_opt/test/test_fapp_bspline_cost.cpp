#include <gtest/gtest.h>
#include <bspline_opt/fapp_bspline_cost.h>
using namespace fuel_fapp;
TEST(FappSpline, SpatialAndTimeGradient) {
  DynamicObstacleConfig c; c.enabled=true;
  DynamicObstacleState o; o.stamp=10; o.position=Eigen::Vector3d(.4,.1,1.1);
  o.velocity=Eigen::Vector3d(.2,.1,0); o.size=Eigen::Vector3d(.2,.2,.2);
  std::vector<Eigen::Vector3d> q;
  for(int i=0;i<7;++i)q.push_back(Eigen::Vector3d(-.7+.3*i,.03,1));
  std::vector<Eigen::Vector3d> g,tmp; double gt,unused;
  const double dt=.2,eps=1e-6;
  EXPECT_GT(bsplineCost(q,dt,{o},c,10,10,g,gt),0);
  for(size_t i=0;i<q.size();++i)for(int axis=0;axis<3;++axis){
    auto plus=q,minus=q;plus[i](axis)+=eps;minus[i](axis)-=eps;
    double numeric=(bsplineCost(plus,dt,{o},c,10,10,tmp,unused)-
                    bsplineCost(minus,dt,{o},c,10,10,tmp,unused))/(2*eps);
    EXPECT_NEAR(g[i](axis),numeric,1e-5);
  }
  double numeric=(bsplineCost(q,dt+eps,{o},c,10,10,tmp,unused)-
                  bsplineCost(q,dt-eps,{o},c,10,10,tmp,unused))/(2*eps);
  EXPECT_NEAR(gt,numeric,1e-5);
  EXPECT_EQ(bsplineCost(q,dt,{o},c,11,11,tmp,unused),0);
  c.enabled=false;EXPECT_EQ(bsplineCost(q,dt,{o},c,10,10,tmp,unused),0);
}
TEST(FappSpline, LowerPassAndEmptyFrames) {
  DynamicObstacleConfig c;c.enabled=true;
  DynamicObstacleState o;o.stamp=10;o.position=Eigen::Vector3d(0,0,1);
  o.size=Eigen::Vector3d(.2,.2,.2);
  std::vector<Eigen::Vector3d> q;for(int i=0;i<10;++i)q.push_back(Eigen::Vector3d(-1+.25*i,0,1));
  std::vector<Eigen::Vector3d> g;double gt;
  EXPECT_GT(bsplineCost(q,.2,{o},c,10,10,g,gt),0);
  for(auto& p:q)p.z()=.5;
  EXPECT_EQ(bsplineCost(q,.2,{o},c,10,10,g,gt),0);
  EXPECT_EQ(bsplineCost(q,.2,{},c,10,10,g,gt),0);
}

int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
