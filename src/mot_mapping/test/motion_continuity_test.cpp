#include "motion_continuity.h"
#include <iostream>
#include <cstdlib>
using namespace mot_mapping;
void require(bool ok) { if (!ok) { std::cerr << "motion continuity regression failed\n"; std::exit(1); } }
int main() {
  std::vector<MotionSample> slow, turn, jitter, jump;
  for(int i=0;i<7;++i) {
    double t=i*.1;
    slow.push_back({t,Eigen::Vector3d(.08*t,0,1)});
    turn.push_back({t,Eigen::Vector3d(.2*(t-.3)*(t-.3),0,1)});
    jitter.push_back({t,Eigen::Vector3d(i%2 ? .2 : 0, .5, .6)});
    jump.push_back({t,Eigen::Vector3d(i<2 ? 0 : .5, .5, .6)});
  }
  require(motionContinuity(slow,.5,.04).continuous);
  require(motionContinuity(turn,.5,.04).continuous);
  require(!motionContinuity(jitter,.5,.04).continuous);
  require(!motionContinuity(jump,.5,.04).continuous);
  slow.resize(5); require(!motionContinuity(slow,.5,.04).ready);
  slow={{0,{0,0,1}},{.1,{.01,0,1}},{.5,{.05,0,1}},{.6,{.06,0,1}}};
  require(!motionContinuity(slow,.5,.04).continuous);
  slow[2].time=.1; require(!motionContinuity(slow,.5,.04).continuous);
  std::cout << "PASS slow motion, reversal, wall jitter, jump, startup, gap and repeated stamp\n";
}
