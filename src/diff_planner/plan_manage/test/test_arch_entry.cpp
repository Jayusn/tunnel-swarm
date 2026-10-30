#include <diff_planner/arch_entry.h>
#include <gtest/gtest.h>

namespace {
using diff_planner::arch_entry::Config;
using diff_planner::arch_entry::detect;
using Points=std::vector<Eigen::Vector3d>;

Points frame(double x, double y, bool arch=true, bool left=true, bool right=true) {
  Points points;
  for (double z=.2; z<=.60; z+=.05) {
    if (left) points.emplace_back(x,y-.75,z);
    if (right) points.emplace_back(x,y+.75,z);
  }
  for (int i=0;i<=60;++i) {
    const double angle=i*M_PI/60;
    points.emplace_back(x,y+.75*std::cos(angle),arch ? .60+.75*std::sin(angle) : 1.35);
  }
  return points;
}

TEST(ArchEntry, FindsRectangleAndSemicircleWithoutSelectingAdjacentGap) {
  auto points=frame(1.8,0);
  auto adjacent=frame(1.8,-2.5);
  points.insert(points.end(),adjacent.begin(),adjacent.end());
  Config c;
  auto result=detect(points,Eigen::Vector3d(0,0,.6),0,c);
  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.center.x(),1.8,.01);
  EXPECT_NEAR(result.center.y(),0,.11);
  EXPECT_DOUBLE_EQ(result.center.z(),.6);
  EXPECT_NEAR(result.spring_z+result.radius,1.35,.16);
}

TEST(ArchEntry, FlatTopAndBarePostsAreNotArches) {
  Config c;
  EXPECT_FALSE(detect(frame(1.8,0,false),Eigen::Vector3d(0,0,.6),0,c).found);
  auto posts=frame(1.8,0); posts.resize(16);
  EXPECT_FALSE(detect(posts,Eigen::Vector3d(0,0,.6),0,c).found);
}

TEST(ArchEntry, PrefersFirstSupportedArchOverDeeperFrame) {
  auto points=frame(1.8,0);
  auto deeper=frame(3.0,0);
  points.insert(points.end(),deeper.begin(),deeper.end());
  const auto r=detect(points,Eigen::Vector3d(0,0,.6),0,Config());
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.center.x(),1.8,.01);
}

TEST(ArchEntry, MissingOnePostCannotInventTheOtherSide) {
  EXPECT_FALSE(detect(frame(1.8,0,true,false,true),Eigen::Vector3d(0,0,.6),0,Config()).found);
}

TEST(ArchEntry, SolidPanelInOpeningIsRejected) {
  auto points=frame(1.8,0);
  for(double y=-.4;y<=.4;y+=.05) for(double z=.2;z<=1.2;z+=.05)
    points.emplace_back(1.8,y,z);
  EXPECT_FALSE(detect(points,Eigen::Vector3d(0,0,.6),0,Config()).found);
}

TEST(ArchEntry, ArcFromAnotherDepthCannotCompletePosts) {
  auto points=frame(1.8,0);
  for(auto& p:points) if(p.z()>.65) p.x()+=.6;
  EXPECT_FALSE(detect(points,Eigen::Vector3d(0,0,.6),0,Config()).found);
}

TEST(ArchEntry, RotatedDoorAndGroundOffsetProduceWorldCoordinates) {
  auto points=frame(1.8,0);
  const double yaw=30*M_PI/180;
  for(auto& p:points) {
    const double x=p.x(), y=p.y();
    p.x()=x*std::cos(yaw)-y*std::sin(yaw);
    p.y()=x*std::sin(yaw)+y*std::cos(yaw); p.z()-=.15;
  }
  Config c; c.goal_z=.45; c.min_structure_z=.05;
  const auto r=detect(points,Eigen::Vector3d(0,0,.6),yaw,c);
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.center.x(),1.8*std::cos(yaw),.16);
  EXPECT_NEAR(r.center.y(),1.8*std::sin(yaw),.16);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
