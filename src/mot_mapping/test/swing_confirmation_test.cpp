#include "swing_confirmation.h"
#include <cstdlib>
#include <iostream>
using mot_mapping::SwingConfirmation;
void check(bool x, const char* why) { if (!x) { std::cerr << why << '\n'; std::exit(1); } }
int main() {
 for(double period : {2.0, 6.0}) {
  SwingConfirmation swing, line, stationary;
  bool seen=false;
  for(int i=0;i<100;++i) {
   double t=i*.1;
   seen |= swing.update(t,{.25*std::sin(2*3.141592653589793*t/period),0,1},true);
   check(!line.update(t,{.15*t,0,1},true),"linear motion");
   check(!stationary.update(t,{i%2*.005,0,1},true),"static jitter");
  }
  check(seen,"swing missed");
  check(!swing.update(10,{0,0,1},false),"stop reset");
  check(!swing.update(10.1,{0,0,1},true),"restart must confirm again");
  swing.clear(); // The detector resets the track after a lost observation history.
  check(!swing.update(11,{.2,0,1},true),"gap reset");
 }
 SwingConfirmation phase;
 bool phase_seen=false;
 int sample=0;
 for(double x : {.01,.02,.01,0.,-.02,-.04,-.06,-.08,-.06,-.04,-.02,0.})
  phase_seen |= phase.update(sample++*.1,{x,0,1},true);
 check(phase_seen,"arbitrary starting phase");
 for(double step : {.08, .5}) {
  SwingConfirmation jump;
  for(int i=0;i<30;++i)
   check(!jump.update(i*.1,{i<10?0.0:(i<20?step:0.0),0,1},true),"wall jump");
 }
 SwingConfirmation sparse_jump;
 int k=0;
 for(double x : {0.,.64,.80,.77,.57})
  check(!sparse_jump.update(k++*.3,{x,0,1},true),"sparse jump plus jitter");
 SwingConfirmation invalid;
 check(!invalid.update(1,{0,0,1},true),"startup");
 check(!invalid.update(1,{.1,0,1},true),"duplicate timestamp");
 check(!invalid.update(0,{.2,0,1},true),"backward timestamp");
 std::cout << "PASS slow/fast reversal, independent tracks, linear/static, jumps, stop and gap\n";
}
