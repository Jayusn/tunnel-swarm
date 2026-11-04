#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>

#include "precision_landing/aruco_tracker.hpp"

namespace {
precision_landing::TargetObservation observe(
    const precision_landing::ArucoTrackerConfig& config, int id) {
  cv::Mat marker;
  cv::aruco::drawMarker(
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250), id, 54, marker);
  cv::Mat image(480, 640, CV_8UC1, cv::Scalar(255));
  marker.copyTo(image(cv::Rect(293, 213, 54, 54)));
  const cv::Mat camera = (cv::Mat_<double>(3, 3) <<
      600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0);
  precision_landing::ArucoTracker tracker(config);
  precision_landing::TargetObservation observation;
  for (int frame = 0; frame < config.stable_frames; ++frame) {
    observation = tracker.process(image, camera, cv::Mat::zeros(1, 5, CV_64F),
                                  0.1 * frame, -1);
  }
  return observation;
}
}  // namespace

TEST(ArucoDictionary, SmallMarkerAtTwoMetersHasCorrectScale) {
  precision_landing::ArucoTrackerConfig config;
  config.dictionary = cv::aruco::DICT_4X4_50;
  config.marker_size_m = 0.18;
  // 4X4_50 shares IDs 0..49 with 4X4_250; 54 px at fx=600 represents 2 m.
  for (int id : {0, 49}) {
    const auto observation = observe(config, id);
    ASSERT_TRUE(observation.valid) << id;
    EXPECT_EQ(observation.id, id);
    EXPECT_NEAR(observation.position_camera.z(), 2.0, 0.08);
  }
}

TEST(ArucoDictionary, SmallDictionaryRejectsIdOutsideItsRange) {
  precision_landing::ArucoTrackerConfig config;
  config.dictionary = cv::aruco::DICT_4X4_50;
  config.marker_size_m = 0.18;
  EXPECT_FALSE(observe(config, 70).valid);
}

TEST(ArucoDictionary, DefaultDictionaryStillAcceptsOriginalIds) {
  precision_landing::ArucoTrackerConfig config;
  config.marker_size_m = 0.50;
  const auto observation = observe(config, 70);
  ASSERT_TRUE(observation.valid);
  EXPECT_EQ(observation.id, 70);
}
