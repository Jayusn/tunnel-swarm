#include "bidirectional_depth.h"
#include <iostream>
#include <stdexcept>

using mot_mapping::BidirectionalDepth;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

int main() {
  BidirectionalDepth view;
  auto build = [&](const Cloud& cloud) {
    view.build(cloud, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  };
  Cloud mixed;
  mixed.push_back(pcl::PointXYZ(2, 0, 0));
  mixed.push_back(pcl::PointXYZ(4, 0, 0));
  build(mixed);
  require(view.classify({3, 0, 0}) == BidirectionalDepth::Unknown,
          "empty space between depth layers is not a static surface");
  require(view.classify({5, 0, 0}) == BidirectionalDepth::Unknown,
          "a point behind actual returns is unknown");
  require(view.classify({4, 0, 0}) == BidirectionalDepth::Unknown,
          "a real farther return behind a nearer occluder is still unknown");
  require(view.classify({1, 0, 0}) == BidirectionalDepth::Foreground,
          "actual farther returns supply foreground evidence");
  require(view.classify({2.02, 0, 0}) == BidirectionalDepth::Static,
          "a real return within three centimetres supplies static support");
  require(view.classify({3, .05, 0}) == BidirectionalDepth::Unknown,
          "returns outside the lateral neighborhood do not supply evidence");
  build(Cloud());
  require(view.classify({1, 0, 0}) == BidirectionalDepth::Unknown,
          "no return cannot supply either motion or static support");
  require(view.classify({0, 0, 0}) == BidirectionalDepth::Unknown,
          "zero range is unknown");

  Cloud previous, current;
  previous.push_back(pcl::PointXYZ(2, 0, 0));
  current.push_back(pcl::PointXYZ(2.2, 0, 0));
  build(previous);
  require(view.classify({2.2, 0, 0}) == BidirectionalDepth::Unknown,
          "radial-away current point has no forward occlusion");
  build(current);
  require(view.classify({2, 0, 0}) == BidirectionalDepth::Foreground,
          "a radial-away object's previous location has been vacated");
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d origin(.3, -.4, .1);
  view.build(current, rotation, origin);
  require(view.classify({2.2, 0, 0}) == BidirectionalDepth::Static,
          "world-static returns remain supported under radar translation and rotation");
  Cloud seam;
  seam.push_back(pcl::PointXYZ(-2, .001, 0));
  build(seam);
  require(view.classify({-2, -.001, 0}) == BidirectionalDepth::Static,
          "azimuth wrap does not lose a neighboring return");
  std::cout << "PASS bidirectional exact depth, unknown, radial vacancy, ego pose and wrap\n";
}
