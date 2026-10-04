#include <cstdlib>
#include <iostream>
#include <limits>
#include <boost/make_shared.hpp>
#include <ros/callback_queue.h>
#include "exploration_manager/odometry_review.h"

void check(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}

// 模拟地图占用主队列期间到达的高频消息；优先处理独立队列后一次得到五帧。
class OdomCallback : public ros::CallbackInterface {
 public:
  OdomCallback(std::deque<fast_planner::OdomReviewSample>& samples, int index)
      : samples_(samples), index_(index) {}
  CallResult call() override {
    fast_planner::OdomReviewSample sample;
    sample.stamp = 100.0 + index_ * 0.005;
    sample.position = {{double(index_), 0.0, 0.45}};
    sample.velocity = {{0.1, 0.0, 0.0}};
    samples_.push_back(sample);
    if (samples_.size() > 5) samples_.pop_front();
    return Success;
  }
 private:
  std::deque<fast_planner::OdomReviewSample>& samples_;
  int index_;
};

int main() {
  ros::Time::init();
  std::deque<fast_planner::OdomReviewSample> samples;
  std::array<double, 3> position, velocity;
  check(!fast_planner::reviewOdometry(samples, position, velocity), "不足五帧不能复核");
  ros::CallbackQueue odom_queue;
  for (int i = 0; i < 10; ++i)
    odom_queue.addCallback(boost::make_shared<OdomCallback>(samples, i));
  odom_queue.callAvailable(ros::WallDuration(0.0));
  check(odom_queue.isEmpty() && samples.size() == 5, "一次取齐最新五帧而不是等五个主循环");
  check(fast_planner::reviewOdometry(samples, position, velocity), "不同时间戳的五帧可复核");
  check(position[0] == 7.0 && velocity[0] == 0.1, "只使用最新五帧的中值");
  samples.back().position[0] = 1000.0;
  check(fast_planner::reviewOdometry(samples, position, velocity) && position[0] == 7.0,
        "五帧中值仍抑制单帧尖峰");
  samples.back().stamp = samples.front().stamp;
  check(!fast_planner::reviewOdometry(samples, position, velocity), "重复或倒序帧不凑五帧");
  samples.back().stamp = 101.0;
  samples.back().velocity[0] = std::numeric_limits<double>::quiet_NaN();
  check(!fast_planner::reviewOdometry(samples, position, velocity), "无效速度不能通过复核");
  return 0;
}
