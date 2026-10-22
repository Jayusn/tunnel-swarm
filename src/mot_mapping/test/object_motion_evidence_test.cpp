#include "object_motion_evidence.h"
#include <cstdlib>
#include <iostream>
#include <limits>
void check(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
int main() {
  using mot_mapping::ObjectMotionEvidence;
  ObjectMotionEvidence track, other;
  auto update = [&](uint64_t t, uint64_t p, double m=.8, double s=.1, double d=.4,
                    bool valid=true, bool transition=true) {
    return track.update(t,p,valid,transition,m,s,d,.6,.2,.3);
  };
  check(!update(100,0),"first observation");
  check(!update(200,100),"one transition");
  check(update(300,200),"two transitions must confirm without reversal");
  check(!other.update(300,200,true,true,.8,.1,.4,.6,.2,.3),"independent tracks");
  check(!update(400,300,.8,.8),"static sampling jitter");
  check(!update(500,400),"jitter breaks evidence");
  check(update(600,500),"new two transitions");
  check(!update(700,600,.8,.1,0),"unobserved/occluded depth is not motion");
  check(!update(800,700),"occlusion breaks evidence");
  check(update(900,800),"reconfirmation");
  check(!update(1100,1000),"missing observation breaks evidence");
  check(!update(1200,1100),"reacquisition first transition");
  check(update(1300,1200),"reacquisition second transition");
  check(!update(1400,1300,.8,.1,.4,false),"background/shape rejection");
  check(!update(1500,1400),"invalid previous observation");
  check(!update(1600,1500),"second valid observation");
  check(update(1700,1600),"third valid observation");
  check(!update(1800,1700,std::numeric_limits<double>::quiet_NaN()),"NaN evidence");
  check(!update(1800,1700),"duplicate time");
  track.clear();
  check(!update(2000,1900),"reset");
  ObjectMotionEvidence window;
  auto short_window = [&](uint64_t t, uint64_t p, bool valid=true, double stationary=.1) {
    return window.update(t,p,valid,true,.8,stationary,.4,.6,.2,.3,400);
  };
  check(!short_window(100,0), "window first observation");
  check(!short_window(200,100), "window first supported transition");
  check(!short_window(400,300), "missing frame is not inferred motion");
  check(short_window(500,400), "two measured transitions around one missing frame");
  check(!short_window(600,500,true,.8), "static evidence itself cannot confirm");
  check(!short_window(700,600,true,.8), "two unsupported observations clear window");
  check(!short_window(800,700), "restart after two unsupported observations");
  check(short_window(900,800), "fresh supported pair confirms");
  check(!short_window(1000,900,false), "rejected background clears retained window");
  check(!short_window(1100,1000), "no carry across invalid shape/background");
  check(!short_window(1200,1100), "one new supported transition");
  check(short_window(1300,1200), "two new supported transitions");
  check(!short_window(2000,1300), "expired window cannot bridge long gap");
  check(!short_window(2100,2000), "long gap does not count as a supported transition");
  check(short_window(2200,2100), "reconfirm after long gap");
  check(!short_window(2200,2100), "duplicate time clears window");
  std::cout << "PASS adjacency, translation, static jitter, unknown depth, gaps, invalid shape, independent tracks\n";
}
